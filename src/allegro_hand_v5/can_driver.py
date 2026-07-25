"""
SocketCAN driver for the Allegro Hand V5, using python-can.

The V5 hardware is torque-only: the sole actuation command is ``Set Torque``
(a signed PWM/current value per joint). Joint positions are streamed back by
the hand at a fixed period configured with ``set_period()``.

Every arbitration ID below is the *unshifted* command ID from ``candef.h``;
the wire ID is ``id << 2``.
"""

import logging
import math
import struct
import time
from dataclasses import dataclass, field
from typing import Optional, Tuple

import can
import numpy as np

from allegro_hand_v5.exceptions import (
    AllegroCANError,
    AllegroConnectionError,
    AllegroTimeoutError,
)

logger = logging.getLogger(__name__)

# --- Command IDs ---
ID_CMD_SYSTEM_ON = 0x40
ID_CMD_SYSTEM_OFF = 0x41
ID_CMD_SET_TORQUE = 0x60  # base ID, +0..3 per finger
ID_CMD_SET_PERIOD = 0x81

# --- Request (RTR) / response IDs ---
ID_RTR_HAND_INFO = 0x80
ID_RTR_SERIAL = 0x88
ID_RTR_FINGER_POSE = 0x20  # base ID, +0..3 per finger

NUM_FINGERS = 4
NUM_JOINTS_PER_FINGER = 4
NUM_JOINTS = NUM_FINGERS * NUM_JOINTS_PER_FINGER  # 16

# Encoder LSB -> radians.
POSITION_SCALE = (math.pi / 180.0) * 0.088

CAN_BITRATE = 1_000_000  # 1 Mbps


@dataclass
class HandInfo:
    """Identity of the connected hand, filled in during initialize_hand()."""

    hardware_version: str = ""
    firmware_version: str = ""
    serial_number: str = ""
    hand_type: str = ""  # "right" or "left"
    hardware_type: str = ""  # "A" (non-geared) or "B" (geared)
    servo_on: bool = False


@dataclass
class HandState:
    """Latest decoded state of the hand."""

    positions: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS))
    position_ready_flags: int = 0  # bitmask of fingers seen since the last reset
    timestamp: float = 0.0

    def all_positions_ready(self) -> bool:
        return self.position_ready_flags == 0x0F

    def reset_ready_flags(self) -> None:
        self.position_ready_flags = 0


class AllegroCANDriver:
    """Low-level CAN transport: servo on/off, torque TX, position RX."""

    def __init__(
        self,
        channel: str = "can0",
        bitrate: int = CAN_BITRATE,
        strict: bool = False,
    ):
        """
        Args:
            channel: SocketCAN interface name (e.g. "can0").
            bitrate: Bus bitrate. SocketCAN ignores this if the link is already up.
            strict: Raise on errors instead of logging them.
        """
        self.channel = channel
        self.bitrate = bitrate
        self.strict = strict

        self._bus: Optional[can.BusABC] = None
        self._connected = False
        self._hand_info = HandInfo()
        self._state = HandState()

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def hand_info(self) -> HandInfo:
        return self._hand_info

    @property
    def state(self) -> HandState:
        return self._state

    # ==================== Connection ====================

    def connect(self) -> None:
        """Open the CAN bus. Raises AllegroConnectionError in strict mode."""
        if self._connected:
            logger.warning("Already connected to CAN bus")
            return

        try:
            self._bus = can.Bus(
                channel=self.channel,
                interface="socketcan",
                bitrate=self.bitrate,
            )
            self._connected = True
            logger.info("Connected to CAN bus on %s", self.channel)
            self._flush_rx_buffer()
        except Exception as e:
            msg = f"Failed to connect to CAN bus on {self.channel}: {e}"
            if self.strict:
                raise AllegroConnectionError(msg) from e
            logger.error(msg)

    def disconnect(self) -> None:
        """Zero torques, drop the servos, and close the bus."""
        if not self._connected:
            return

        try:
            self.set_torques(np.zeros(NUM_JOINTS))
            self.servo_off()
            time.sleep(0.01)

            if self._bus is not None:
                self._bus.shutdown()
                self._bus = None

            self._connected = False
            logger.info("Disconnected from CAN bus")
        except Exception as e:
            logger.warning("Error during disconnect: %s", e)
            self._connected = False

    def initialize_hand(self) -> bool:
        """
        Bring the hand up: identify it, set the pose report period, servos on.

        Returns:
            True once the handshake has been sent.
        """
        if not self._connected:
            if self.strict:
                raise AllegroConnectionError("Not connected to CAN bus")
            return False

        logger.info("Initializing hand...")

        self._flush_rx_buffer()

        # Servos off first, so a stale torque command cannot fire on power-up.
        self.servo_off()
        time.sleep(0.01)

        self._send_message(ID_RTR_HAND_INFO, is_rtr=True)
        time.sleep(0.1)
        self.process_messages()

        self._send_message(ID_RTR_SERIAL, is_rtr=True)
        time.sleep(0.1)
        self.process_messages()

        # Stream joint positions every 3 ms; IMU and temperature stay off.
        self.set_period(position_period_ms=3)
        time.sleep(0.01)

        self.servo_on()
        time.sleep(0.1)

        logger.info("Hand initialization complete")
        return True

    # ==================== Raw frame I/O ====================

    def _flush_rx_buffer(self) -> None:
        if self._bus is None:
            return
        for _ in range(100):
            if self._bus.recv(timeout=0.001) is None:
                break

    def _send_message(self, arbitration_id: int, data: bytes = b"", is_rtr: bool = False) -> bool:
        if not self._connected or self._bus is None:
            if self.strict:
                raise AllegroCANError("Not connected to CAN bus")
            return False

        try:
            self._bus.send(
                can.Message(
                    arbitration_id=arbitration_id << 2,
                    data=data,
                    is_remote_frame=is_rtr,
                    is_extended_id=False,
                )
            )
            return True
        except can.CanError as e:
            msg = f"Failed to send CAN message: {e}"
            if self.strict:
                raise AllegroCANError(msg) from e
            logger.warning(msg)
            return False

    def _receive_message(self, timeout: float = 0.001) -> Optional[Tuple[int, bytes]]:
        if not self._connected or self._bus is None:
            return None

        try:
            msg = self._bus.recv(timeout=timeout)
            if msg is None:
                return None
            if msg.is_error_frame:
                logger.warning("Received CAN error frame")
                return None
            return (msg.arbitration_id >> 2, bytes(msg.data))
        except can.CanError as e:
            logger.warning("CAN receive error: %s", e)
            return None

    # ==================== Commands ====================

    def servo_on(self) -> bool:
        logger.info("Turning servos ON")
        ok = self._send_message(ID_CMD_SYSTEM_ON)
        if ok:
            self._hand_info.servo_on = True
        return ok

    def servo_off(self) -> bool:
        logger.info("Turning servos OFF")
        ok = self._send_message(ID_CMD_SYSTEM_OFF)
        if ok:
            self._hand_info.servo_on = False
        return ok

    def set_period(
        self,
        position_period_ms: int = 3,
        imu_period_ms: int = 0,
        temperature_period_ms: int = 0,
    ) -> bool:
        """Set the hand's autonomous report periods (0 disables a stream)."""
        data = struct.pack("<HHH", position_period_ms, imu_period_ms, temperature_period_ms)
        return self._send_message(ID_CMD_SET_PERIOD, data)

    def set_torques(self, pwm: np.ndarray) -> bool:
        """
        Send raw PWM/current commands for all 16 joints, one frame per finger.

        Args:
            pwm: 16 already-scaled, already-clamped command values.
        """
        if len(pwm) != NUM_JOINTS:
            raise ValueError(f"Expected {NUM_JOINTS} torques, got {len(pwm)}")

        ok = True
        for finger_idx in range(NUM_FINGERS):
            base = finger_idx * NUM_JOINTS_PER_FINGER
            values = [int(np.clip(v, -32767, 32767)) for v in pwm[base : base + NUM_JOINTS_PER_FINGER]]
            data = struct.pack("<hhhh", *values)
            if not self._send_message(ID_CMD_SET_TORQUE + finger_idx, data):
                ok = False
        return ok

    # ==================== Reception ====================

    def process_messages(self, timeout: float = 0.001, max_messages: int = 100) -> int:
        """Drain up to max_messages frames into the state. Returns the count."""
        count = 0
        for _ in range(max_messages):
            result = self._receive_message(timeout=timeout)
            if result is None:
                break
            self._parse_message(*result)
            count += 1
        return count

    def _parse_message(self, arb_id: int, data: bytes) -> None:
        if ID_RTR_FINGER_POSE <= arb_id <= ID_RTR_FINGER_POSE + 3:
            self._parse_finger_pose(arb_id - ID_RTR_FINGER_POSE, data)
        elif arb_id == ID_RTR_HAND_INFO:
            self._parse_hand_info(data)
        elif arb_id == ID_RTR_SERIAL:
            self._parse_serial(data)
        else:
            logger.debug("Unhandled CAN message ID: 0x%02X", arb_id)

    def _parse_finger_pose(self, finger_idx: int, data: bytes) -> None:
        if len(data) < 8:
            logger.warning("Invalid finger pose data length: %d", len(data))
            return

        raw = struct.unpack("<hhhh", data[:8])
        base = finger_idx * NUM_JOINTS_PER_FINGER
        for i, value in enumerate(raw):
            self._state.positions[base + i] = value * POSITION_SCALE

        self._state.position_ready_flags |= 1 << finger_idx
        self._state.timestamp = time.time()

    def _parse_hand_info(self, data: bytes) -> None:
        if len(data) < 7:
            return
        self._hand_info.hardware_version = f"0x{data[1]:02X}{data[0]:02X}"
        self._hand_info.firmware_version = f"0x{data[3]:02X}{data[2]:02X}"
        self._hand_info.servo_on = bool(data[6] & 0x01)
        logger.info(
            "Hand info - HW: %s, FW: %s, Servo: %s",
            self._hand_info.hardware_version,
            self._hand_info.firmware_version,
            self._hand_info.servo_on,
        )

    def _parse_serial(self, data: bytes) -> None:
        if len(data) < 8:
            return
        serial = data[:8].decode("ascii", errors="replace")
        self._hand_info.serial_number = serial
        if len(serial) > 3:
            self._hand_info.hand_type = "right" if serial[3] == "R" else "left"
        if len(serial) > 2:
            self._hand_info.hardware_type = serial[2]
        logger.info(
            "Hand serial: %s, Type: %s, Hardware: %s",
            serial,
            self._hand_info.hand_type,
            self._hand_info.hardware_type,
        )

    def read_positions(self, timeout: float = 0.1) -> np.ndarray:
        """Block until every finger has reported once, then return 16 radians."""
        self._state.reset_ready_flags()
        start = time.time()

        while not self._state.all_positions_ready():
            if time.time() - start > timeout:
                if self.strict:
                    raise AllegroTimeoutError("Timeout waiting for position data")
                logger.warning("Timeout waiting for position data")
                break
            self.process_messages(timeout=0.001)

        return self._state.positions.copy()
