"""Protocol codec: pure encode/decode of Allegro Hand V5 CAN frames.

No I/O and no hardware dependency, so every function here is unit-testable against a
fake bus.  A "frame" is represented as ``(msg_id, data_bytes, is_rtr)`` where ``msg_id``
is the value *before* the ``<< 2`` arbitration shift.  The driver applies the shift when
talking to python-can.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import NamedTuple

from . import constants as C


class Frame(NamedTuple):
    """A logical CAN frame keyed by message id (pre-shift)."""

    msg_id: int
    data: bytes = b""
    is_rtr: bool = False


# ---------------------------------------------------------------------------
# Arbitration id helpers
# ---------------------------------------------------------------------------
def to_arbitration_id(msg_id: int) -> int:
    return msg_id << C.ARBITRATION_SHIFT


def from_arbitration_id(arb_id: int) -> int:
    return arb_id >> C.ARBITRATION_SHIFT


# ---------------------------------------------------------------------------
# Encoders (host -> hand)
# ---------------------------------------------------------------------------
def encode_servo_on() -> Frame:
    return Frame(C.ID_SYSTEM_ON)


def encode_servo_off() -> Frame:
    return Frame(C.ID_SYSTEM_OFF)


def _clamp_int16(v: int) -> int:
    return max(-32768, min(32767, int(v)))


def encode_set_torque(finger: int, torque_ma) -> Frame:
    """Encode a per-finger torque frame.

    ``torque_ma`` is an iterable of 4 joint currents in mA (already clamped/scaled).
    """
    if not 0 <= finger < C.NUM_FINGERS:
        raise ValueError(f"finger must be in 0..3, got {finger}")
    vals = [int(round(v)) for v in torque_ma]
    if len(vals) != C.JOINTS_PER_FINGER:
        raise ValueError(f"expected 4 joint values, got {len(vals)}")
    data = struct.pack("<4h", *(_clamp_int16(v) for v in vals))
    return Frame(C.ID_SET_TORQUE + finger, data)


def encode_set_pose(finger: int, raw_positions) -> Frame:
    """Experimental on-board position command (0xE0+). Not documented in the manual."""
    if not 0 <= finger < C.NUM_FINGERS:
        raise ValueError(f"finger must be in 0..3, got {finger}")
    vals = [_clamp_int16(v) for v in raw_positions]
    if len(vals) != C.JOINTS_PER_FINGER:
        raise ValueError(f"expected 4 joint values, got {len(vals)}")
    return Frame(C.ID_SET_POSE + finger, struct.pack("<4h", *vals))


def encode_set_period(period_ms=C.DEFAULT_PERIOD_MS) -> Frame:
    """Enable/adjust periodic streaming: ``[pos_ms, imu_ms, temp_ms]``. 0 stops a stream."""
    pos, imu, temp = period_ms
    return Frame(C.ID_SET_PERIOD, struct.pack("<3h", int(pos), int(imu), int(temp)))


def encode_stop_streams() -> Frame:
    return Frame(C.ID_SET_PERIOD, struct.pack("<3h", 0, 0, 0))


# RTR requests (dlc 0, remote flag set) -------------------------------------
def request_hand_info() -> Frame:
    return Frame(C.ID_RTR_HAND_INFO, is_rtr=True)


def request_serial() -> Frame:
    return Frame(C.ID_RTR_SERIAL, is_rtr=True)


def request_finger_pose(finger: int) -> Frame:
    if not 0 <= finger < C.NUM_FINGERS:
        raise ValueError(f"finger must be in 0..3, got {finger}")
    return Frame(C.ID_RTR_FINGER_POSE + finger, is_rtr=True)


# ---------------------------------------------------------------------------
# Torque scaling (Nm -> mA, with clamp and Plus handling)
# ---------------------------------------------------------------------------
def torque_nm_to_ma(torque_nm, is_plus: bool = False):
    """Convert a 16-vector of joint torques [Nm] to clamped motor currents [mA].

    Mirrors the stock driver: ``mA = Nm * 1.43e3``, clamp to ``±240`` mA, and on Plus
    (type B) hands halve joints 1/5/9 (MCP-2 of index/middle/ring).
    """
    if len(torque_nm) != C.DOF:
        raise ValueError(f"expected {C.DOF} torques, got {len(torque_nm)}")
    out = []
    for i, tau in enumerate(torque_nm):
        ma = tau * C.NM_TO_MA
        if is_plus and i in C.PLUS_HALVED_JOINTS:
            ma *= 0.5
        ma = max(-C.TORQUE_LIMIT_MA, min(C.TORQUE_LIMIT_MA, ma))
        out.append(ma)
    return out


# ---------------------------------------------------------------------------
# Decoders (hand -> host)
# ---------------------------------------------------------------------------
def decode_finger_pose(finger: int, data: bytes):
    """Return the 4 joint angles [rad] for ``finger`` from an 8-byte pose frame."""
    raw = struct.unpack("<4h", data[:8])
    return [r * C.POSITION_SCALE for r in raw]


def decode_fingertip(msg_id: int, data: bytes):
    """Return ``(finger0, value0_pa, finger1, value1_pa)`` for a fingertip frame.

    The low id (0xF0 / legacy 0x50) carries [index(0), middle(1)]; the high id
    (0xF2 / 0x52) carries [ring(2), thumb(3)].
    """
    v0, v1 = struct.unpack("<2i", data[:8])
    base = C.FINGERTIP_ID_TO_BASE[msg_id]
    return base, v0, base + 1, v1


@dataclass(frozen=True)
class HandInfo:
    hardware_version: int
    firmware_version: int
    servo_on: bool


def decode_hand_info(data: bytes) -> HandInfo:
    hw = struct.unpack_from("<H", data, 0)[0]
    fw = struct.unpack_from("<H", data, 2)[0]
    servo = bool(data[6] & 0x01) if len(data) > 6 else False
    return HandInfo(hw, fw, servo)


@dataclass(frozen=True)
class HandSerial:
    serial: str
    is_right: bool
    is_type_a: bool

    @property
    def handedness(self) -> str:
        return "right" if self.is_right else "left"

    @property
    def hand_type(self) -> str:
        return "A" if self.is_type_a else "B"


def decode_serial(data: bytes) -> HandSerial:
    """Decode the 8-byte serial reply into serial string + handedness/type flags."""
    text = bytes(data[:8]).decode("ascii", errors="replace")
    is_right = len(data) > 3 and data[3] == ord("R")
    is_type_a = len(data) > 2 and data[2] == ord("A")
    return HandSerial(text, is_right, is_type_a)


# Error bit meanings (see manual §11.3.10) ----------------------------------
ERROR_BITS = {
    5: "overload",
    4: "electrical_shock",
    2: "overheating",
    0: "input_voltage",
}


@dataclass(frozen=True)
class MotorError:
    motor_id: int
    code: int

    @property
    def flags(self):
        return [name for bit, name in ERROR_BITS.items() if self.code & (1 << bit)]


def decode_error(data: bytes) -> MotorError:
    return MotorError(data[0], data[1])
