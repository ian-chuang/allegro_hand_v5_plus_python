"""
Allegro Hand V5 CAN protocol: message IDs, encoding, decoding.

Pure functions and constants, no I/O and no state. Follows section 11 of the
V5 (4F) user manual (`docs/allegro_hand_v5_manual.md`) and, where the manual is
silent, WONIK's own `allegro_hand_ros2_v5` driver (`candef.h`, `socket_can.cpp`).

Wire format: the message ID sits in the upper bits of the 11-bit standard
arbitration field, so the frame ID is `message_id << 2`. All multi-byte values
are little-endian.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from enum import IntEnum, IntFlag
from typing import Optional, Tuple

import numpy as np

NUM_FINGERS = 4
NUM_JOINTS_PER_FINGER = 4
NUM_JOINTS = NUM_FINGERS * NUM_JOINTS_PER_FINGER  # 16

FINGER_NAMES = ("index", "middle", "ring", "thumb")

#: Joint names in the order every 16-vector uses, using the manual's naming
#: (section 15.1): MCP-1 is the spread/abduction joint, MCP-2 the base flexion.
JOINT_NAMES = (
    "index_mcp1", "index_mcp2", "index_pip", "index_dip",
    "middle_mcp1", "middle_mcp2", "middle_pip", "middle_dip",
    "ring_mcp1", "ring_mcp2", "ring_pip", "ring_dip",
    "thumb_cmc1", "thumb_cmc2", "thumb_mp", "thumb_ip",
)

JOINT_INDEX = {name: i for i, name in enumerate(JOINT_NAMES)}


class MsgID(IntEnum):
    """Message IDs (pre-shift; the wire arbitration ID is `id << 2`)."""

    # --- commands, host -> hand ---
    SERVO_ON = 0x040
    SERVO_OFF = 0x041
    SET_TORQUE = 0x060  # +0..3, one frame per finger. Payload is motor current.
    SET_POSE = 0x0E0  # +0..3. Undocumented for V5; see bus.send_pose().
    SET_PERIOD = 0x081  # report periods for position / IMU / temperature
    PICK = 0x011
    PLACE = 0x012
    CALIBRATE = 0x089  # start the hand's own position calibration
    CALIBRATE_DONE = 0x092  # the hand's reply when it finishes

    # --- feedback, hand -> host (RTR-readable, and periodic where noted) ---
    INFO = 0x080
    SERIAL = 0x088
    POSITION = 0x020  # +0..3, periodic
    IMU = 0x030  # periodic
    TEMPERATURE = 0x038  # +0..3, periodic
    ERROR = 0x0EE

    # Fingertip pressure. The v1.3 manual documents 0x050/0x052 and so does
    # WONIK's driver, but current firmware streams these at 0x0F0/0x0F2
    # (confirmed on a real V5 Plus, serial 5TBR0017). Both are accepted.
    PRESSURE_1 = 0x0F0  # index, middle
    PRESSURE_2 = 0x0F2  # ring, thumb
    PRESSURE_1_LEGACY = 0x050
    PRESSURE_2_LEGACY = 0x052


ID_SHIFT = 2
CAN_BITRATE = 1_000_000  # 1 Mbps

#: Joint encoder resolution: 0.088 deg per LSB, in radians (manual 11.3.6).
POSITION_SCALE = math.radians(0.088)

#: Absolute per-joint current clamp, mA. The only actuation command is a motor
#: current, and WONIK's driver saturates it here before every transmit.
MAX_CURRENT_MA = 240.0

#: MCP-2 joints of index/middle/ring. On a "Plus" (type B) hand these are geared
#: 576.7:1 instead of 288.35:1, so the same current gives twice the joint torque.
#: Nothing here rescales them — the gain presets carry halved numbers for these
#: joints instead, so what you command is what goes on the wire.
PLUS_GEARED_JOINTS = (1, 5, 9)

#: Fingertip readings outside 0..this many Pa are reported as 0, the same sanity
#: check WONIK's driver applies.
PRESSURE_VALID_MAX = 5000

#: Pressure message ID -> index of the first finger it carries.
PRESSURE_ID_TO_FINGER = {
    MsgID.PRESSURE_1: 0,
    MsgID.PRESSURE_1_LEGACY: 0,
    MsgID.PRESSURE_2: 2,
    MsgID.PRESSURE_2_LEGACY: 2,
}


class ErrorFlag(IntFlag):
    """Error-code bits from manual 11.3.10. Bits 1, 3, 6, 7 are always 0."""

    NONE = 0
    INPUT_VOLTAGE = 1 << 0  # supply outside the specified operating range
    OVERHEATING = 1 << 2  # motor temperature outside its set range
    ELECTRICAL_SHOCK = 1 << 4  # electrical shock or insufficient input power
    OVERLOAD = 1 << 5  # load beyond max motor output, applied continuously

    def describe(self) -> str:
        """Human-readable list of the conditions set in this code."""
        names = {
            ErrorFlag.INPUT_VOLTAGE: "input voltage out of range",
            ErrorFlag.OVERHEATING: "overheating",
            ErrorFlag.ELECTRICAL_SHOCK: "electrical shock / insufficient power",
            ErrorFlag.OVERLOAD: "overload",
        }
        known = [text for flag, text in names.items() if self & flag]
        unknown = int(self) & ~sum(int(f) for f in names)
        if unknown:
            known.append(f"reserved bits 0x{unknown:02X}")
        return ", ".join(known) or "none"


@dataclass
class JointError:
    """One error report, as sent on ID 0xEE."""

    motor_id: int
    code: ErrorFlag

    @property
    def joint_name(self) -> str:
        if 0 <= self.motor_id < NUM_JOINTS:
            return JOINT_NAMES[self.motor_id]
        return f"motor_{self.motor_id}"

    def __str__(self) -> str:
        return f"{self.joint_name} (motor {self.motor_id}): {self.code.describe()}"


@dataclass
class HandInfo:
    """Identity of the connected hand, from the Info and Serial messages."""

    serial_number: str = ""
    hardware_version: Optional[int] = None
    firmware_version: Optional[int] = None

    @property
    def handedness(self) -> str:
        """"right", "left" or "unknown" — character 3 of the serial number."""
        if len(self.serial_number) > 3:
            return {"R": "right", "L": "left"}.get(self.serial_number[3], "unknown")
        return "unknown"

    @property
    def hardware_type(self) -> str:
        """"A" (non-geared), "B" (geared/Plus) or "unknown" — character 2."""
        if len(self.serial_number) > 2 and self.serial_number[2] in "AB":
            return self.serial_number[2]
        return "unknown"

    @property
    def complete(self) -> bool:
        """True once the hand has identified itself.

        Only the serial number is required: it carries handedness and hardware
        type, and real firmware often ignores the Information RTR (0x080).
        """
        return bool(self.serial_number)

    def __str__(self) -> str:
        hw = "?" if self.hardware_version is None else f"0x{self.hardware_version:04X}"
        fw = "?" if self.firmware_version is None else f"0x{self.firmware_version:04X}"
        return (
            f"Allegro Hand V5  serial={self.serial_number or '?'}  "
            f"{self.handedness} hand, type {self.hardware_type}  hw={hw} fw={fw}"
        )


def frame_id(msg_id: int) -> int:
    """Wire arbitration ID for a message ID."""
    return msg_id << ID_SHIFT


def message_id(arbitration_id: int) -> int:
    """Message ID for a wire arbitration ID."""
    return arbitration_id >> ID_SHIFT


def finger_slice(finger: int) -> slice:
    """Slice of a 16-joint array belonging to one finger."""
    return slice(finger * NUM_JOINTS_PER_FINGER, (finger + 1) * NUM_JOINTS_PER_FINGER)


# ==================== Encoding (host -> hand) ====================


def encode_currents(finger: int, currents_ma) -> Tuple[int, bytes]:
    """
    Set Torque frame for one finger (manual 11.3.3).

    Args:
        finger: 0-3.
        currents_ma: 4 motor currents in mA, one per joint of that finger.

    Returns:
        (frame_id, 8 data bytes) — four int16 little-endian.
    """
    if not 0 <= finger < NUM_FINGERS:
        raise ValueError(f"finger must be 0-3, got {finger}")
    values = np.asarray(currents_ma, dtype=np.float64)
    if values.shape != (NUM_JOINTS_PER_FINGER,):
        raise ValueError(f"expected {NUM_JOINTS_PER_FINGER} currents, got {values.shape}")
    clipped = np.clip(np.round(values), -MAX_CURRENT_MA, MAX_CURRENT_MA).astype(np.int64)
    return frame_id(MsgID.SET_TORQUE + finger), struct.pack("<4h", *clipped)


def encode_pose(finger: int, positions_rad) -> Tuple[int, bytes]:
    """
    Set Pose frame for one finger (0x0E0+f). Same scaling as Position Finger.

    Inherited from the V4 protocol and absent from the V5 manual; the firmware
    may ignore it. See `AllegroCANBus.send_pose`.
    """
    if not 0 <= finger < NUM_FINGERS:
        raise ValueError(f"finger must be 0-3, got {finger}")
    values = np.asarray(positions_rad, dtype=np.float64)
    if values.shape != (NUM_JOINTS_PER_FINGER,):
        raise ValueError(f"expected {NUM_JOINTS_PER_FINGER} positions, got {values.shape}")
    counts = np.clip(np.round(values / POSITION_SCALE), -32768, 32767).astype(np.int64)
    return frame_id(MsgID.SET_POSE + finger), struct.pack("<4h", *counts)


def encode_period(position_ms: int = 3, imu_ms: int = 0, temperature_ms: int = 0):
    """
    Set the hand's autonomous report periods, in ms. 0 disables a stream.

    Not in the V5 manual's ID table, but WONIK's driver sends it at start-up and
    it is what makes the hand stream positions instead of answering one RTR per
    cycle. Six data bytes, three int16 little-endian.
    """
    return frame_id(MsgID.SET_PERIOD), struct.pack("<3h", position_ms, imu_ms, temperature_ms)


# ==================== Decoding (hand -> host) ====================


def decode_position(data: bytes) -> np.ndarray:
    """Position Finger frame (manual 11.3.6) -> 4 joint angles in radians."""
    if len(data) < 8:
        raise ValueError(f"position frame needs 8 bytes, got {len(data)}")
    return np.array(struct.unpack("<4h", data[:8]), dtype=np.float64) * POSITION_SCALE


def decode_pressure(data: bytes) -> Tuple[int, int]:
    """Fingertip Pressure frame (manual 11.3.7) -> two pressures in Pa."""
    if len(data) < 8:
        raise ValueError(f"pressure frame needs 8 bytes, got {len(data)}")
    first, second = struct.unpack("<2i", data[:8])
    return sanitize_pressure(first), sanitize_pressure(second)


def sanitize_pressure(value: int) -> int:
    """0 if the reading is outside the plausible range, else the reading."""
    return 0 if value < 0 or value > PRESSURE_VALID_MAX else int(value)


def decode_temperature(data: bytes) -> np.ndarray:
    """
    Temperature frame (0x038+f) -> 4 motor temperatures in degrees Celsius.

    Undocumented for V5; WONIK's driver reads one unsigned byte per joint.
    """
    if len(data) < 4:
        raise ValueError(f"temperature frame needs 4 bytes, got {len(data)}")
    return np.array(struct.unpack("<4B", data[:4]), dtype=np.float64)


def decode_imu(data: bytes) -> np.ndarray:
    """
    IMU frame (0x030) -> raw (roll, pitch, yaw).

    Undocumented for V5, and WONIK's driver only prints the bytes, so the unit
    is unverified. Reported raw.
    """
    if len(data) < 6:
        raise ValueError(f"IMU frame needs 6 bytes, got {len(data)}")
    return np.array(struct.unpack("<3h", data[:6]), dtype=np.float64)


def decode_info(data: bytes) -> Tuple[int, int]:
    """Information frame (manual 11.3.4) -> (hardware_version, firmware_version)."""
    if len(data) < 4:
        raise ValueError(f"info frame needs at least 4 bytes, got {len(data)}")
    return struct.unpack("<2H", data[:4])


def decode_serial(data: bytes) -> str:
    """Serial Number frame (manual 11.3.5) -> up to 8 ASCII characters."""
    return data[:8].decode("ascii", errors="replace").strip("\x00 ")


def decode_error(data: bytes) -> JointError:
    """Error frame (manual 11.3.10) -> motor id and error flags."""
    if len(data) < 2:
        raise ValueError(f"error frame needs 2 bytes, got {len(data)}")
    return JointError(motor_id=data[0], code=ErrorFlag(data[1]))
