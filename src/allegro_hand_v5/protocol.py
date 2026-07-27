"""
Allegro Hand V5 CAN protocol: message IDs, encoding, decoding.

Pure functions and constants only — no I/O, no state. Everything here follows
section 11 of the V5 (4F) user manual; see `docs/allegro_hand_v5_manual.md`.

Wire format: the message ID sits in the upper bits of the 11-bit standard
arbitration field, so the frame ID is `message_id << 2`. All multi-byte values
are little-endian.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field
from enum import IntEnum, IntFlag
from typing import List, Optional, Tuple

import numpy as np

NUM_FINGERS = 4
NUM_JOINTS_PER_FINGER = 4
NUM_JOINTS = NUM_FINGERS * NUM_JOINTS_PER_FINGER  # 16

FINGER_NAMES = ("index", "middle", "ring", "thumb")

#: Machine-friendly joint names, in the order every 16-vector uses.
JOINT_NAMES = (
    "index_spread", "index_mcp", "index_pip", "index_dip",
    "middle_spread", "middle_mcp", "middle_pip", "middle_dip",
    "ring_spread", "ring_mcp", "ring_pip", "ring_dip",
    "thumb_rot", "thumb_mcp", "thumb_pip", "thumb_dip",
)

#: The same joints, for printing.
JOINT_LABELS = tuple(
    f"{name.replace('_', ' ').title()} ({i})" for i, name in enumerate(JOINT_NAMES)
)


class MsgID(IntEnum):
    """Message IDs from manual section 11.2 (pre-shift; wire ID is `id << 2`)."""

    SERVO_ON = 0x040
    SERVO_OFF = 0x041

    SET_TORQUE = 0x060  # +0..3, one per finger

    INFO = 0x080  # RTR-readable
    SERIAL = 0x088  # RTR-readable
    POSITION = 0x020  # +0..3, RTR-readable or periodic

    # Fingertip pressure. The v1.3 manual documents 0x050/0x052, but current
    # firmware actually streams these at 0x0F0/0x0F2 (confirmed on a real V5
    # Plus, serial 5TBR0017). Both are accepted; see PRESSURE_ID_TO_FINGER.
    PRESSURE_1 = 0x0F0  # index, middle
    PRESSURE_2 = 0x0F2  # ring, thumb
    PRESSURE_1_LEGACY = 0x050
    PRESSURE_2_LEGACY = 0x052

    PICK = 0x011
    PLACE = 0x012
    ERROR = 0x0EE

    # Not in the V5 manual's ID table, but the firmware accepts it and it is
    # what makes the hand stream positions instead of needing an RTR per cycle.
    # Inherited from the V4 protocol (candef.h).
    SET_PERIOD = 0x081


ID_SHIFT = 2
CAN_BITRATE = 1_000_000  # 1 Mbps

#: Joint encoder resolution: 0.088 deg per LSB, converted to radians.
POSITION_SCALE = (math.pi / 180.0) * 0.088

#: Motor current per unit of joint torque, mA/Nm. The Set Torque field is in mA
#: (manual 11.3.3); this is the conversion the reference stack uses.
TORQUE_TO_CURRENT = 1.43 * 1000.0

#: Per-joint current clamp used by the reference stack, in mA.
DEFAULT_MAX_CURRENT_MA = 240.0

#: Every accepted fingertip-pressure message ID -> index of the first finger it
#: carries. The low ID reports [index, middle]; the high ID [ring, thumb].
PRESSURE_ID_TO_FINGER = {
    MsgID.PRESSURE_1: 0,
    MsgID.PRESSURE_1_LEGACY: 0,
    MsgID.PRESSURE_2: 2,
    MsgID.PRESSURE_2_LEGACY: 2,
}

#: On "Plus" (type B) hands the MCP-2 joint of index/middle/ring has roughly
#: twice the gear ratio, so its current command is halved. The thumb is not
#: affected. Matches the stock driver.
PLUS_HALVED_JOINTS = (1, 5, 9)

#: Fingertip readings outside 0..this many Pa are treated as invalid and
#: reported as 0, the same sanity check the stock driver applies.
PRESSURE_VALID_MAX = 5000


class ErrorFlag(IntFlag):
    """Error-code bits from manual 11.3.10. Bits 1, 3, 6, 7 are always 0."""

    NONE = 0
    INPUT_VOLTAGE = 1 << 0  # supply outside the specified operating range
    OVERHEATING = 1 << 2  # motor temperature outside its set range
    ELECTRICAL_SHOCK = 1 << 4  # electrical shock or insufficient input power
    OVERLOAD = 1 << 5  # load beyond max motor output, applied continuously

    def describe(self) -> str:
        """Human-readable list of the conditions set in this code."""
        if not self:
            return "none"
        names = {
            ErrorFlag.INPUT_VOLTAGE: "input voltage out of range",
            ErrorFlag.OVERHEATING: "overheating",
            ErrorFlag.ELECTRICAL_SHOCK: "electrical shock / insufficient power",
            ErrorFlag.OVERLOAD: "overload",
        }
        known = [text for flag, text in names.items() if self & flag]
        unknown = int(self) & ~int(
            ErrorFlag.INPUT_VOLTAGE | ErrorFlag.OVERHEATING
            | ErrorFlag.ELECTRICAL_SHOCK | ErrorFlag.OVERLOAD
        )
        if unknown:
            known.append(f"reserved bits 0x{unknown:02X}")
        return ", ".join(known)


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

    hardware_version: Optional[int] = None
    firmware_version: Optional[int] = None
    serial_number: str = ""

    @property
    def handedness(self) -> str:
        """"right", "left", or "unknown". Character 3 of the serial number."""
        if len(self.serial_number) > 3:
            return {"R": "right", "L": "left"}.get(self.serial_number[3], "unknown")
        return "unknown"

    @property
    def hardware_type(self) -> str:
        """"A" (non-geared), "B" (geared), or "unknown". Character 2 of the serial."""
        if len(self.serial_number) > 2 and self.serial_number[2] in "AB":
            return self.serial_number[2]
        return "unknown"

    @property
    def complete(self) -> bool:
        """True once the hand has identified itself.

        Only the serial number is required: it carries handedness and hardware
        type, and real firmware frequently ignores the Information RTR (0x080).
        Gating on the version would mean never recognising a working hand.
        """
        return bool(self.serial_number)

    def __str__(self) -> str:
        hw = f"0x{self.hardware_version:04X}" if self.hardware_version is not None else "?"
        fw = f"0x{self.firmware_version:04X}" if self.firmware_version is not None else "?"
        return (
            f"Allegro Hand V5  serial={self.serial_number or '?'}  "
            f"{self.handedness} hand, type {self.hardware_type}  hw={hw} fw={fw}"
        )


# ==================== Encoding (host -> hand) ====================


def frame_id(message_id: int) -> int:
    """Wire arbitration ID for a message ID."""
    return message_id << ID_SHIFT


def message_id(frame_id_: int) -> int:
    """Message ID for a wire arbitration ID."""
    return frame_id_ >> ID_SHIFT


def encode_torque(finger: int, currents_ma) -> Tuple[int, bytes]:
    """
    Set Torque frame for one finger (manual 11.3.3).

    Args:
        finger: 0-3.
        currents_ma: 4 motor currents in mA, one per joint.

    Returns:
        (frame_id, 8 data bytes) — four int16 little-endian.
    """
    if not 0 <= finger < NUM_FINGERS:
        raise ValueError(f"finger must be 0-3, got {finger}")
    values = [int(round(float(v))) for v in currents_ma]
    if len(values) != NUM_JOINTS_PER_FINGER:
        raise ValueError(f"expected {NUM_JOINTS_PER_FINGER} currents, got {len(values)}")
    clipped = [max(-32768, min(32767, v)) for v in values]
    return frame_id(MsgID.SET_TORQUE + finger), struct.pack("<hhhh", *clipped)


def encode_period(position_ms: int = 3, imu_ms: int = 0, temperature_ms: int = 0) -> Tuple[int, bytes]:
    """
    Set the hand's autonomous report periods. 0 disables a stream.

    Not documented in the V5 manual's ID table, but accepted by the firmware and
    required to make the hand stream positions rather than answering RTRs.
    """
    return frame_id(MsgID.SET_PERIOD), struct.pack("<3h", position_ms, imu_ms, temperature_ms)


def torque_to_current(torque_nm, scale: float = TORQUE_TO_CURRENT) -> np.ndarray:
    """Joint torque in Nm -> motor current in mA."""
    return np.asarray(torque_nm, dtype=np.float64) * scale


def current_to_torque(current_ma, scale: float = TORQUE_TO_CURRENT) -> np.ndarray:
    """Motor current in mA -> joint torque in Nm."""
    return np.asarray(current_ma, dtype=np.float64) / scale


# ==================== Decoding (hand -> host) ====================


def decode_position(data: bytes) -> np.ndarray:
    """Position Finger frame (manual 11.3.6) -> 4 joint angles in radians."""
    if len(data) < 8:
        raise ValueError(f"position frame needs 8 bytes, got {len(data)}")
    return np.array(struct.unpack("<hhhh", data[:8]), dtype=np.float64) * POSITION_SCALE


def decode_pressure(data: bytes) -> Tuple[int, int]:
    """Fingertip Pressure frame (manual 11.3.7) -> two pressures in Pa."""
    if len(data) < 8:
        raise ValueError(f"pressure frame needs 8 bytes, got {len(data)}")
    return struct.unpack("<ii", data[:8])


def decode_info(data: bytes) -> Tuple[int, int]:
    """Information frame (manual 11.3.4) -> (hardware_version, firmware_version)."""
    if len(data) < 4:
        raise ValueError(f"info frame needs at least 4 bytes, got {len(data)}")
    hardware, firmware = struct.unpack("<HH", data[:4])
    return hardware, firmware


def decode_serial(data: bytes) -> str:
    """Serial Number frame (manual 11.3.5) -> 8 ASCII characters."""
    return data[:8].decode("ascii", errors="replace").strip("\x00 ")


def decode_error(data: bytes) -> JointError:
    """Error frame (manual 11.3.10) -> (motor id, error flags)."""
    if len(data) < 2:
        raise ValueError(f"error frame needs 2 bytes, got {len(data)}")
    return JointError(motor_id=data[0], code=ErrorFlag(data[1]))


def finger_slice(finger: int) -> slice:
    """Slice of the 16-joint arrays belonging to one finger."""
    return slice(finger * NUM_JOINTS_PER_FINGER, (finger + 1) * NUM_JOINTS_PER_FINGER)


def pressure_finger_base(msg_id: int) -> Optional[int]:
    """First finger index reported by a pressure message, or None."""
    return PRESSURE_ID_TO_FINGER.get(msg_id)


def scale_plus_currents(currents_ma: np.ndarray, is_plus: bool) -> np.ndarray:
    """Halve the MCP-2 currents on a type B (Plus) hand. Returns a new array."""
    if not is_plus:
        return currents_ma
    out = np.asarray(currents_ma, dtype=np.float64).copy()
    out[list(PLUS_HALVED_JOINTS)] *= 0.5
    return out


def sanitize_pressure(value: int) -> int:
    """Clamp a fingertip reading to the plausible range; 0 if out of it."""
    return 0 if value < 0 or value > PRESSURE_VALID_MAX else int(value)
