"""
CAN transport for the Allegro Hand V5.

`AllegroCANBus` owns the python-can socket and translates between frames and
the decoded state in `protocol`. It holds no control logic and no threads — the
control process in `driver` drives it.
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
from allegro_hand_v5.exceptions import AllegroCANError, AllegroConnectionError, AllegroTimeoutError
from allegro_hand_v5.protocol import (
    NUM_FINGERS,
    NUM_JOINTS,
    ErrorFlag,
    HandInfo,
    JointError,
    MsgID,
)

logger = logging.getLogger(__name__)


def link_status(channel: str = "can0") -> Dict:
    """
    SocketCAN interface state, read with iproute2. Empty dict if unavailable.

    Keys: ``up``, ``state`` ("ERROR-ACTIVE" / "ERROR-WARNING" / "ERROR-PASSIVE"
    / "BUS-OFF"), ``tx_errors``, ``rx_errors``, ``bitrate``, ``restart_ms``,
    ``bus_off_count``, ``rx_packets``.
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

    # The counter row follows the header naming its columns.
    m = re.search(
        r"re-started\s+bus-errors\s+arbit-lost\s+error-warn\s+error-pass\s+bus-off\s*\n?\s*"
        r"(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)",
        out,
    )
    if m:
        info.update(zip(
            ("restarts", "bus_errors", "arbitration_lost",
             "error_warn_count", "error_passive_count", "bus_off_count"),
            (int(g) for g in m.groups()),
        ))

    m = re.search(r"RX:\s+bytes\s+packets.*\n\s*(\d+)\s+(\d+)", out)
    if m:
        info["rx_bytes"], info["rx_packets"] = int(m.group(1)), int(m.group(2))

    return info


def describe_link(channel: str = "can0") -> str:
    """One-paragraph diagnosis of the interface, with the fix if it is unhealthy."""
    info = link_status(channel)
    if not info:
        return f"{channel}: could not read the link state (is iproute2 installed?)"

    parts = [
        f"{channel}: {'up' if info.get('up') else 'DOWN'}",
        f"state {info.get('state', '?')}",
        f"bitrate {info.get('bitrate', '?')}",
        f"tx_err {info.get('tx_errors', '?')}/rx_err {info.get('rx_errors', '?')}",
        f"rx_packets {info.get('rx_packets', '?')}",
    ]
    text = ", ".join(parts)

    advice = []
    state = info.get("state", "")
    if not info.get("up"):
        advice.append(f"bring it up: sudo ip link set {channel} up type can bitrate 1000000")
    if state == "BUS-OFF":
        advice.append(
            f"the controller is BUS-OFF and will not transmit again until the link is "
            f"bounced: sudo ip link set {channel} down && "
            f"sudo ip link set {channel} up type can bitrate 1000000 restart-ms 100"
        )
    elif state in ("ERROR-PASSIVE", "ERROR-WARNING"):
        advice.append(
            "the transmit error counter is climbing, which means frames are going "
            "unacknowledged — the hand is powered off, not wired to this bus, or at a "
            "different bitrate. Fix that, then bounce the link to clear the counters"
        )
    if info.get("restart_ms") == 0:
        advice.append(
            f"restart-ms is 0, so a BUS-OFF will latch forever; consider "
            f"'sudo ip link set {channel} up type can bitrate 1000000 restart-ms 100'"
        )
    if info.get("bitrate") not in (None, 1_000_000):
        advice.append("the Allegro Hand V5 runs at 1 Mbps; this interface is not")
    if info.get("rx_packets") == 0 and info.get("up"):
        advice.append("nothing has ever been received on this interface")

    if advice:
        text += "\n  - " + "\n  - ".join(advice)
    return text


@dataclass
class BusState:
    """Everything decoded off the bus since the last reset."""

    positions: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS))
    pressures: np.ndarray = field(default_factory=lambda: np.zeros(NUM_FINGERS))
    errors: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS, dtype=np.int64))
    position_flags: int = 0  # bitmask of fingers seen since reset_position_flags()
    last_position_time: float = 0.0
    last_pressure_time: float = 0.0
    error_count: int = 0
    last_error: Optional[JointError] = None
    can_error_frames: int = 0  # bus-level error frames, not hand error reports
    tx_dropped: int = 0  # frames the kernel refused, almost always a full queue

    def all_positions_ready(self) -> bool:
        return self.position_flags == 0x0F

    def reset_position_flags(self) -> None:
        self.position_flags = 0

    def active_errors(self) -> List[JointError]:
        """One JointError per joint currently reporting a non-zero code."""
        return [
            JointError(int(i), ErrorFlag(int(code)))
            for i, code in enumerate(self.errors)
            if code
        ]


class AllegroCANBus:
    """
    SocketCAN transport: servo on/off, torque TX, and feedback decoding.

    Example:
        >>> bus = AllegroCANBus("can0")
        >>> bus.open()
        >>> bus.servo_on()
        >>> bus.send_currents(np.zeros(16))
        >>> bus.poll()
        >>> bus.state.positions
    """

    def __init__(
        self,
        channel: str = "can0",
        bitrate: int = proto.CAN_BITRATE,
        interface: str = "socketcan",
        bus_factory: Optional[Callable[[], "can.BusABC"]] = None,
    ):
        """
        Args:
            channel: Interface name, e.g. "can0".
            bitrate: Bus bitrate. SocketCAN ignores it if the link is already up.
            interface: python-can interface; "pcan" sets the bitrate itself.
            bus_factory: Callable returning an open bus, bypassing the above.
        """
        self.channel = channel
        self.bitrate = bitrate
        self.interface = interface
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
            if self.bus_factory is not None:
                self._bus = self.bus_factory()
            else:
                self._bus = can.Bus(
                    channel=self.channel, interface=self.interface, bitrate=self.bitrate
                )
        except Exception as e:
            raise AllegroConnectionError(f"Could not open CAN bus {self.channel!r}: {e}") from e

        self.flush()
        logger.info("CAN bus open on %s", self.channel)

    def close(self) -> None:
        """Servo off, then close the socket. Safe to call more than once."""
        if self._bus is None:
            return
        try:
            self.send_currents(np.zeros(NUM_JOINTS))
            self.servo_off()
            time.sleep(0.01)
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
        if self._bus is None:
            return
        for _ in range(limit):
            if self._bus.recv(timeout=0.001) is None:
                break

    def handshake(self, timeout: float = 2.0, position_period_ms: int = 3) -> HandInfo:
        """
        Identify the hand and get its position stream running.

        **Engages the servos.** That is not optional: the hand only reports joint
        positions once the motor drivers are on, so leaving them off means no
        feedback at all. Nothing is driven — the caller has not commanded any
        current yet — and `close()` always turns them back off.

        The frames go out back to back, as the stock driver sends them; the hand
        answers at its own pace and we drain replies afterwards.

        Args:
            timeout: How long to wait for the serial reply and the first poses.
            position_period_ms: Position report period; 0 disables the stream.

        Returns:
            The decoded HandInfo. Fields stay unset if the hand did not reply;
            note that real firmware often ignores the Information RTR while
            still answering the serial one.
        """
        self._require_open()
        self.flush()

        self.servo_off()  # never come up with a stale torque command latched
        self.request_info()
        self.request_serial()
        self.set_period(position_period_ms)
        self.servo_on()

        self.state.reset_position_flags()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.info.complete and self.state.all_positions_ready():
                break
            self.poll(timeout=0.002)

        if self.info.complete:
            logger.info("%s", self.info)
        else:
            logger.warning("Hand did not send a serial number within %.1fs", timeout)
        if not self.state.all_positions_ready():
            logger.warning(
                "Only fingers %04b reported a position during the handshake",
                self.state.position_flags,
            )
        return self.info

    def _require_open(self) -> None:
        if self._bus is None:
            raise AllegroCANError("CAN bus is not open")

    # ==================== Transmit ====================

    def _send(self, arb_id: int, data: bytes = b"", rtr: bool = False) -> bool:
        """
        Put one frame on the bus. Returns False if the kernel refused it.

        A full transmit queue (ENOBUFS) is not an error worth raising on: it
        means nothing is acknowledging, so frames are not draining. That is a
        bus problem the caller cannot fix per-frame, and in the control loop the
        next cycle simply sends again — far better than killing the process and
        leaving the motors latched. It is counted so `diagnose()` can report it.
        """
        self._require_open()
        try:
            self._bus.send(
                can.Message(
                    arbitration_id=arb_id,
                    data=data,
                    is_remote_frame=rtr,
                    is_extended_id=False,
                )
            )
            return True
        except can.CanError as e:
            self.state.tx_dropped += 1
            if "No buffer space" in str(e) or getattr(e, "error_code", None) == 105:
                n = self.state.tx_dropped
                if n == 1 or n == 100 or n % 5000 == 0:
                    logger.warning(
                        "Transmit queue full on %s (%d frames dropped). Nothing is "
                        "acknowledging. %s", self.channel, n, describe_link(self.channel),
                    )
                return False
            raise AllegroCANError(f"CAN send failed (id 0x{arb_id:03X}): {e}") from e

    def servo_on(self) -> None:
        """Engage the joint motor drivers."""
        self._send(proto.frame_id(MsgID.SERVO_ON))
        self.servo_is_on = True
        logger.info("Servo ON")

    def servo_off(self) -> None:
        """Disable the joint motor drivers. The hand goes limp."""
        self._send(proto.frame_id(MsgID.SERVO_OFF))
        self.servo_is_on = False
        logger.info("Servo OFF")

    def send_currents(self, currents_ma) -> None:
        """Send Set Torque for all four fingers. `currents_ma` is 16 values in mA."""
        currents_ma = np.asarray(currents_ma, dtype=np.float64)
        if currents_ma.shape != (NUM_JOINTS,):
            raise ValueError(f"expected {NUM_JOINTS} currents, got {currents_ma.shape}")
        for finger in range(NUM_FINGERS):
            arb_id, data = proto.encode_torque(finger, currents_ma[proto.finger_slice(finger)])
            self._send(arb_id, data)

    def set_period(self, position_ms: int = 3, imu_ms: int = 0, temperature_ms: int = 0) -> None:
        """Set the hand's autonomous report periods, in milliseconds."""
        arb_id, data = proto.encode_period(position_ms, imu_ms, temperature_ms)
        self._send(arb_id, data)

    def request_info(self) -> None:
        """RTR for the Information message."""
        self._send(proto.frame_id(MsgID.INFO), rtr=True)

    def request_serial(self) -> None:
        """RTR for the Serial Number message."""
        self._send(proto.frame_id(MsgID.SERIAL), rtr=True)

    def request_positions(self) -> None:
        """RTR for all four Position messages, for polling instead of streaming."""
        for finger in range(NUM_FINGERS):
            self._send(proto.frame_id(MsgID.POSITION + finger), rtr=True)

    # ==================== Receive ====================

    def poll(self, timeout: float = 0.0, max_frames: int = 64) -> int:
        """
        Drain waiting frames into `state`. Returns how many were decoded.

        Args:
            timeout: Per-frame receive timeout; 0.0 is non-blocking.
            max_frames: Stop after this many, so a flood cannot stall the loop.
        """
        if self._bus is None:
            return 0

        count = 0
        for _ in range(max_frames):
            try:
                msg = self._bus.recv(timeout=timeout)
            except can.CanError as e:
                logger.warning("CAN receive error: %s", e)
                break
            if msg is None:
                break
            if msg.is_error_frame:
                # Bus-level trouble (usually unacknowledged transmits), not a
                # report from the hand. Log the first one loudly, then count.
                self.state.can_error_frames += 1
                if self.state.can_error_frames == 1:
                    logger.warning("CAN error frame on %s. %s",
                                   self.channel, describe_link(self.channel))
                continue
            self._decode(proto.message_id(msg.arbitration_id), bytes(msg.data))
            count += 1
        return count

    def _decode(self, msg_id: int, data: bytes) -> None:
        try:
            if MsgID.POSITION <= msg_id < MsgID.POSITION + NUM_FINGERS:
                finger = msg_id - MsgID.POSITION
                self.state.positions[proto.finger_slice(finger)] = proto.decode_position(data)
                self.state.position_flags |= 1 << finger
                self.state.last_position_time = time.monotonic()

            elif (base := proto.pressure_finger_base(msg_id)) is not None:
                first, second = proto.decode_pressure(data)
                self.state.pressures[base] = proto.sanitize_pressure(first)
                if base + 1 < NUM_FINGERS:
                    self.state.pressures[base + 1] = proto.sanitize_pressure(second)
                self.state.last_pressure_time = time.monotonic()

            elif msg_id == MsgID.INFO:
                self.info.hardware_version, self.info.firmware_version = proto.decode_info(data)

            elif msg_id == MsgID.SERIAL:
                self.info.serial_number = proto.decode_serial(data)

            elif msg_id == MsgID.ERROR:
                err = proto.decode_error(data)
                if 0 <= err.motor_id < NUM_JOINTS:
                    self.state.errors[err.motor_id] = int(err.code)
                self.state.error_count += 1
                self.state.last_error = err
                logger.error("Hand error: %s", err)

            elif msg_id in (MsgID.PICK, MsgID.PLACE):
                logger.debug("Hand reported %s", MsgID(msg_id).name)

            else:
                logger.debug("Unhandled message ID 0x%03X", msg_id)

        except ValueError as e:
            logger.warning("Malformed frame 0x%03X: %s", msg_id, e)

    def clear_errors(self) -> None:
        """Forget latched error codes. Does not clear anything on the hand."""
        self.state.errors[:] = 0
        self.state.last_error = None

    def read_positions(self, timeout: float = 0.5, poll_rtr: bool = False) -> np.ndarray:
        """
        Block until every finger has reported once.

        Args:
            timeout: Give up after this long.
            poll_rtr: Send position RTRs instead of relying on the stream.

        Returns:
            16 joint angles in radians.
        """
        self._require_open()
        self.state.reset_position_flags()
        deadline = time.monotonic() + timeout

        while not self.state.all_positions_ready():
            if time.monotonic() > deadline:
                seen = f"{self.state.position_flags:04b}"
                raise AllegroTimeoutError(
                    f"Only fingers {seen} reported a position within {timeout}s"
                    f"{' (nothing at all)' if seen == '0000' else ''}.\n"
                    f"{describe_link(self.channel)}"
                )
            if poll_rtr:
                self.request_positions()
            self.poll(timeout=0.002)

        return self.state.positions.copy()

    def diagnose(self) -> str:
        """Human-readable summary of the link and what the hand has said so far."""
        lines = [describe_link(self.channel)]
        lines.append(f"  bus open: {self.is_open}, servo: {'on' if self.servo_is_on else 'off'}")
        lines.append(f"  hand info: {self.info if self.info.complete else 'no reply yet'}")
        s = self.state
        lines.append(
            f"  fingers reporting: {s.position_flags:04b}, "
            f"hand errors: {s.error_count}"
            + (f" (last: {s.last_error})" if s.last_error else "")
        )
        lines.append(
            f"  bus error frames: {s.can_error_frames}, frames dropped on send: {s.tx_dropped}"
        )
        return "\n".join(lines)

    def __enter__(self) -> "AllegroCANBus":
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> bool:
        self.close()
        return False

    def __repr__(self) -> str:
        return (
            f"AllegroCANBus(channel={self.channel!r}, "
            f"{'open' if self.is_open else 'closed'}, "
            f"servo={'on' if self.servo_is_on else 'off'})"
        )
