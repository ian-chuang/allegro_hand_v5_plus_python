"""
CAN transport for the Allegro Hand V5.

`AllegroCANBus` owns the python-can socket and translates between frames and
decoded state. It holds no control logic and no threads: every CAN command the
hand understands is a method here, and `driver.AllegroHand` drives it.
"""

from __future__ import annotations

import logging
import re
import subprocess
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

import can
import numpy as np

from allegro_hand_v5 import protocol as proto
from allegro_hand_v5.exceptions import (
    AllegroCANError,
    AllegroConnectionError,
    AllegroTimeoutError,
)
from allegro_hand_v5.protocol import (
    NUM_FINGERS,
    NUM_JOINTS,
    ErrorFlag,
    HandInfo,
    JointError,
    MsgID,
)

logger = logging.getLogger(__name__)


# ==================== SocketCAN link diagnostics ====================


def link_status(channel: str = "can0") -> Dict:
    """
    SocketCAN interface state, read with iproute2. Empty dict if unavailable.

    Keys: ``up``, ``state`` ("ERROR-ACTIVE" ... "BUS-OFF"), ``bitrate``,
    ``restart_ms``, ``tx_errors``, ``rx_errors``, ``rx_packets``.
    """
    try:
        out = subprocess.run(
            ["ip", "-details", "-statistics", "link", "show", channel],
            capture_output=True, text=True, timeout=2.0,
        ).stdout
    except (OSError, subprocess.SubprocessError):
        return {}
    if not out:
        return {}

    info: Dict = {"channel": channel, "up": "state UP" in out}
    for key, pattern, cast in (
        ("state", r"can state (\S+)", str),
        ("bitrate", r"\bbitrate (\d+)", int),
        ("restart_ms", r"restart-ms (\d+)", int),
    ):
        m = re.search(pattern, out)
        if m:
            info[key] = cast(m.group(1))

    m = re.search(r"berr-counter tx (\d+) rx (\d+)", out)
    if m:
        info["tx_errors"], info["rx_errors"] = int(m.group(1)), int(m.group(2))
    m = re.search(r"RX:\s+bytes\s+packets.*\n\s*(\d+)\s+(\d+)", out)
    if m:
        info["rx_packets"] = int(m.group(2))
    return info


def describe_link(channel: str = "can0") -> str:
    """One-paragraph diagnosis of the interface, with the fix if it is unhealthy."""
    info = link_status(channel)
    if not info:
        return f"{channel}: could not read the link state (is iproute2 installed?)"

    bounce = (f"sudo ip link set {channel} down && sudo ip link set {channel} up "
              f"type can bitrate 1000000 restart-ms 100")
    text = (
        f"{channel}: {'up' if info.get('up') else 'DOWN'}, "
        f"state {info.get('state', '?')}, bitrate {info.get('bitrate', '?')}, "
        f"tx_err {info.get('tx_errors', '?')}/rx_err {info.get('rx_errors', '?')}, "
        f"rx_packets {info.get('rx_packets', '?')}"
    )

    advice = []
    state = info.get("state", "")
    if not info.get("up"):
        advice.append(f"bring it up: {bounce}")
    elif state == "BUS-OFF":
        advice.append(f"the controller is BUS-OFF and will not transmit until the "
                      f"link is bounced: {bounce}")
    elif state in ("ERROR-PASSIVE", "ERROR-WARNING"):
        advice.append("the transmit error counter is climbing, so frames are going "
                      "unacknowledged: the hand is powered off, not wired to this bus, "
                      "or at a different bitrate")
    if info.get("restart_ms") == 0:
        advice.append("restart-ms is 0, so a BUS-OFF will latch forever; bring the "
                      "link up with restart-ms 100")
    if info.get("bitrate") not in (None, proto.CAN_BITRATE):
        advice.append("the Allegro Hand V5 runs at 1 Mbps; this interface does not")
    return text + ("\n  - " + "\n  - ".join(advice) if advice else "")


# ==================== Decoded state ====================


@dataclass
class BusState:
    """Everything decoded off the bus since the last reset."""

    positions: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS))
    pressures: np.ndarray = field(default_factory=lambda: np.zeros(NUM_FINGERS))
    temperatures: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS))
    imu: np.ndarray = field(default_factory=lambda: np.zeros(3))
    errors: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS, dtype=np.int64))

    position_flags: int = 0  # bitmask of fingers seen since clear_position_flags()
    last_position_time: float = 0.0
    error_count: int = 0
    last_error: Optional[JointError] = None
    can_error_frames: int = 0  # bus-level error frames, not reports from the hand
    tx_dropped: int = 0  # frames the kernel refused, almost always a full queue

    @property
    def positions_fresh(self) -> bool:
        """True once all four fingers have reported since the last clear."""
        return self.position_flags == 0x0F

    def clear_position_flags(self) -> None:
        self.position_flags = 0

    def active_errors(self) -> List[JointError]:
        """One JointError per joint currently reporting a non-zero code."""
        return [JointError(int(i), ErrorFlag(int(c))) for i, c in enumerate(self.errors) if c]


# ==================== Transport ====================


class AllegroCANBus:
    """
    SocketCAN transport: every command the hand accepts, plus feedback decoding.

    Example:
        >>> with AllegroCANBus("can0") as bus:
        ...     bus.handshake()
        ...     bus.poll(timeout=0.01)
        ...     bus.state.positions
    """

    def __init__(
        self,
        channel: str = "can0",
        interface: str = "socketcan",
        bitrate: int = proto.CAN_BITRATE,
        bus_factory: Optional[Callable[[], "can.BusABC"]] = None,
    ):
        """
        Args:
            channel: Interface name, e.g. "can0".
            interface: python-can interface; "pcan" sets the bitrate itself.
            bitrate: Bus bitrate. SocketCAN ignores it if the link is already up.
            bus_factory: Callable returning an open bus, bypassing the above.
        """
        self.channel = channel
        self.interface = interface
        self.bitrate = bitrate
        self.bus_factory = bus_factory

        self._bus: Optional["can.BusABC"] = None
        self.state = BusState()
        self.info = HandInfo()
        self.servo_is_on = False

    @property
    def is_open(self) -> bool:
        return self._bus is not None

    # ==================== Lifecycle ====================

    def open(self) -> None:
        """Open the CAN socket and drop any stale frames."""
        if self._bus is not None:
            return
        try:
            self._bus = self.bus_factory() if self.bus_factory else can.Bus(
                channel=self.channel, interface=self.interface, bitrate=self.bitrate
            )
        except Exception as e:
            raise AllegroConnectionError(f"Could not open CAN bus {self.channel!r}: {e}") from e
        self.flush()
        logger.info("CAN bus open on %s", self.channel)

    def close(self) -> None:
        """Zero the current, servo off, then close the socket. Idempotent."""
        if self._bus is None:
            return
        try:
            self.send_currents(np.zeros(NUM_JOINTS))
            self.servo_off()
        except Exception as e:
            logger.warning("Could not send the shutdown frames: %s", e)
        try:
            self._bus.shutdown()
        except Exception as e:
            logger.warning("Error closing the CAN bus: %s", e)
        finally:
            self._bus = None
            logger.info("CAN bus closed")

    def flush(self, limit: int = 200) -> None:
        """Discard buffered frames."""
        for _ in range(limit):
            if self._bus is None or self._bus.recv(timeout=0.0) is None:
                break

    def handshake(self, timeout: float = 2.0, position_period_ms: int = 3,
                  imu_period_ms: int = 0, temperature_period_ms: int = 0) -> HandInfo:
        """
        Identify the hand and start its report streams, in WONIK's own order:
        flush, servo off, ask for identity, set the periods, servo on.

        **This engages the servos.** That is not optional — the hand only reports
        joint positions once the motor drivers are on, so leaving them off means
        no feedback at all. Nothing is driven: no current has been commanded.

        Returns:
            The decoded HandInfo. Fields stay unset if the hand did not answer;
            real firmware often ignores the Information RTR while still
            answering the serial one.
        """
        self._require_open()
        self.flush()

        self.servo_off()  # never come up with a stale current command latched
        self.request_info()
        self.request_serial()
        self.set_period(position_period_ms, imu_period_ms, temperature_period_ms)
        self.servo_on()

        self.state.clear_position_flags()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.info.complete and self.state.positions_fresh:
                break
            self.poll(timeout=0.002)

        if self.info.complete:
            logger.info("%s", self.info)
        else:
            logger.warning("Hand did not send a serial number within %.1fs", timeout)
        return self.info

    def wait_for_positions(self, timeout: float = 1.5) -> np.ndarray:
        """
        Block until every finger has reported once. Returns 16 angles in radians.

        Raises:
            AllegroTimeoutError: if some finger stayed silent.
        """
        self._require_open()
        self.state.clear_position_flags()
        deadline = time.monotonic() + timeout
        while not self.state.positions_fresh:
            if time.monotonic() > deadline:
                raise AllegroTimeoutError(
                    f"Only fingers {self.state.position_flags:04b} reported a position "
                    f"within {timeout}s.\n{self.diagnose()}"
                )
            self.poll(timeout=0.002)
        return self.state.positions.copy()

    def _require_open(self) -> None:
        if self._bus is None:
            raise AllegroCANError("CAN bus is not open")

    # ==================== Transmit ====================

    def _send(self, arb_id: int, data: bytes = b"", rtr: bool = False) -> bool:
        """
        Put one frame on the bus. Returns False if the kernel refused it.

        A full transmit queue (ENOBUFS) means nothing on the bus is
        acknowledging, so frames are not draining. Raising would kill the
        control loop and leave the motors latched; the next cycle simply sends
        again, and the drop is counted for `diagnose()`.
        """
        self._require_open()
        try:
            self._bus.send(can.Message(
                arbitration_id=arb_id, data=data, is_remote_frame=rtr, is_extended_id=False,
            ))
            return True
        except can.CanError as e:
            self.state.tx_dropped += 1
            if "No buffer space" in str(e) or getattr(e, "error_code", None) == 105:
                if self.state.tx_dropped in (1, 100) or self.state.tx_dropped % 5000 == 0:
                    logger.warning("Transmit queue full on %s (%d dropped). Nothing is "
                                   "acknowledging. %s", self.channel, self.state.tx_dropped,
                                   describe_link(self.channel))
                return False
            raise AllegroCANError(f"CAN send failed (id 0x{arb_id:03X}): {e}") from e

    def servo_on(self) -> None:
        """Engage the joint motor drivers (0x040)."""
        self._send(proto.frame_id(MsgID.SERVO_ON))
        self.servo_is_on = True
        logger.info("Servo ON")

    def servo_off(self) -> None:
        """Disable the joint motor drivers (0x041). The hand goes limp."""
        self._send(proto.frame_id(MsgID.SERVO_OFF))
        self.servo_is_on = False
        logger.info("Servo OFF")

    def send_currents(self, currents_ma) -> None:
        """
        Set Torque for all four fingers (0x060-0x063): 16 motor currents in mA.

        Values are saturated to +/-`protocol.MAX_CURRENT_MA` on the way out.
        """
        currents_ma = np.asarray(currents_ma, dtype=np.float64)
        if currents_ma.shape != (NUM_JOINTS,):
            raise ValueError(f"expected {NUM_JOINTS} currents, got {currents_ma.shape}")
        for finger in range(NUM_FINGERS):
            self._send(*proto.encode_currents(finger, currents_ma[proto.finger_slice(finger)]))

    def send_pose(self, positions_rad) -> None:
        """
        Set Pose for all four fingers (0x0E0-0x0E3): 16 joint angles in radians.

        Inherited from the V4 protocol. It is **not** in the V5 manual and the
        V5 firmware is not known to act on it — the manual states the board does
        torque control only. Provided for completeness; use `send_currents` and
        the driver's PD loop for actual position control.
        """
        positions_rad = np.asarray(positions_rad, dtype=np.float64)
        if positions_rad.shape != (NUM_JOINTS,):
            raise ValueError(f"expected {NUM_JOINTS} positions, got {positions_rad.shape}")
        for finger in range(NUM_FINGERS):
            self._send(*proto.encode_pose(finger, positions_rad[proto.finger_slice(finger)]))

    def set_period(self, position_ms: int = 3, imu_ms: int = 0, temperature_ms: int = 0) -> None:
        """Set the hand's report periods in ms (0x081). 0 disables a stream."""
        self._send(*proto.encode_period(position_ms, imu_ms, temperature_ms))

    def pick(self) -> None:
        """Pick motion command (0x011)."""
        self._send(proto.frame_id(MsgID.PICK))

    def place(self) -> None:
        """Place motion command (0x012)."""
        self._send(proto.frame_id(MsgID.PLACE))

    def start_motor_calibration(self) -> None:
        """
        Ask the hand to run its own position calibration (0x089).

        It answers with 0x092 when done. This is the hand's internal encoder
        calibration, unrelated to `allegro_hand_v5.calibration`, which records
        the travel of *your* hand on the host side.
        """
        self._send(proto.frame_id(MsgID.CALIBRATE))

    def request_info(self) -> None:
        """RTR for the Information message (0x080)."""
        self._send(proto.frame_id(MsgID.INFO), rtr=True)

    def request_serial(self) -> None:
        """RTR for the Serial Number message (0x088)."""
        self._send(proto.frame_id(MsgID.SERIAL), rtr=True)

    def request_positions(self) -> None:
        """RTR for all four Position messages (0x020-0x023)."""
        for finger in range(NUM_FINGERS):
            self._send(proto.frame_id(MsgID.POSITION + finger), rtr=True)

    def request_imu(self) -> None:
        """RTR for the IMU message (0x030)."""
        self._send(proto.frame_id(MsgID.IMU), rtr=True)

    def request_temperatures(self) -> None:
        """RTR for all four Temperature messages (0x038-0x03B)."""
        for finger in range(NUM_FINGERS):
            self._send(proto.frame_id(MsgID.TEMPERATURE + finger), rtr=True)

    # ==================== Receive ====================

    def poll(self, timeout: float = 0.0, max_frames: int = 64) -> int:
        """
        Drain waiting frames into `state`. Returns how many were decoded.

        Args:
            timeout: How long the *first* receive may block; the rest drain
                without blocking. 0.0 makes the whole call non-blocking.
            max_frames: Stop after this many, so a flood cannot stall the loop.
        """
        if self._bus is None:
            return 0
        count = 0
        for i in range(max_frames):
            try:
                msg = self._bus.recv(timeout=timeout if i == 0 else 0.0)
            except can.CanError as e:
                logger.warning("CAN receive error: %s", e)
                break
            if msg is None:
                break
            if msg.is_error_frame:
                # Bus-level trouble (usually unacknowledged transmits), not a
                # report from the hand. Log the first loudly, then just count.
                self.state.can_error_frames += 1
                if self.state.can_error_frames == 1:
                    logger.warning("CAN error frame on %s. %s",
                                   self.channel, describe_link(self.channel))
                continue
            self._decode(proto.message_id(msg.arbitration_id), bytes(msg.data))
            count += 1
        return count

    def _decode(self, msg_id: int, data: bytes) -> None:
        s = self.state
        try:
            if MsgID.POSITION <= msg_id < MsgID.POSITION + NUM_FINGERS:
                finger = msg_id - MsgID.POSITION
                s.positions[proto.finger_slice(finger)] = proto.decode_position(data)
                s.position_flags |= 1 << finger
                s.last_position_time = time.monotonic()

            elif (base := proto.PRESSURE_ID_TO_FINGER.get(msg_id)) is not None:
                first, second = proto.decode_pressure(data)
                s.pressures[base] = first
                if base + 1 < NUM_FINGERS:
                    s.pressures[base + 1] = second

            elif MsgID.TEMPERATURE <= msg_id < MsgID.TEMPERATURE + NUM_FINGERS:
                finger = msg_id - MsgID.TEMPERATURE
                s.temperatures[proto.finger_slice(finger)] = proto.decode_temperature(data)

            elif msg_id == MsgID.IMU:
                s.imu[:] = proto.decode_imu(data)

            elif msg_id == MsgID.INFO:
                self.info.hardware_version, self.info.firmware_version = proto.decode_info(data)

            elif msg_id == MsgID.SERIAL:
                self.info.serial_number = proto.decode_serial(data)

            elif msg_id == MsgID.ERROR:
                err = proto.decode_error(data)
                if 0 <= err.motor_id < NUM_JOINTS:
                    s.errors[err.motor_id] = int(err.code)
                s.error_count += 1
                s.last_error = err
                logger.error("Hand error: %s", err)

            elif msg_id in (MsgID.PICK, MsgID.PLACE, MsgID.CALIBRATE_DONE):
                logger.info("Hand reported %s", MsgID(msg_id).name)

            else:
                logger.debug("Unhandled message ID 0x%03X", msg_id)

        except ValueError as e:
            logger.warning("Malformed frame 0x%03X: %s", msg_id, e)

    def clear_errors(self) -> None:
        """Forget latched error codes. Does not clear anything on the hand."""
        self.state.errors[:] = 0
        self.state.last_error = None

    def diagnose(self) -> str:
        """Human-readable summary of the link and what the hand has said so far."""
        s = self.state
        return "\n".join([
            describe_link(self.channel),
            f"  bus open: {self.is_open}, servo: {'on' if self.servo_is_on else 'off'}",
            f"  hand info: {self.info if self.info.complete else 'no reply yet'}",
            f"  fingers reporting: {s.position_flags:04b}, hand errors: {s.error_count}"
            + (f" (last: {s.last_error})" if s.last_error else ""),
            f"  bus error frames: {s.can_error_frames}, dropped on send: {s.tx_dropped}",
        ])

    def __enter__(self) -> "AllegroCANBus":
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> bool:
        self.close()
        return False

    def __repr__(self) -> str:
        return (f"AllegroCANBus(channel={self.channel!r}, "
                f"{'open' if self.is_open else 'closed'}, "
                f"servo={'on' if self.servo_is_on else 'off'})")
