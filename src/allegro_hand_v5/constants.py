"""Verified hardware constants for the Allegro Hand V5 (F4) / (F4) Plus.

Cross-checked against Wonik's ``allegro_hand_ros2_v5`` driver and the V5 user manual.
See ``docs/driver_constants.md`` for provenance of every number here.
"""

from __future__ import annotations

import math

# ---------------------------------------------------------------------------
# Bus
# ---------------------------------------------------------------------------
CAN_BITRATE = 1_000_000  # 1 Mbps, CAN 2.0, 11-bit standard IDs

#: Arbitration id = ``msg_id << ARBITRATION_SHIFT``; on RX, ``msg_id = can_id >> 2``.
ARBITRATION_SHIFT = 2

# ---------------------------------------------------------------------------
# Message ids (the value *before* the ``<< 2`` arbitration shift)
# ---------------------------------------------------------------------------
ID_SYSTEM_ON = 0x40
ID_SYSTEM_OFF = 0x41
ID_SET_TORQUE = 0x60  # + finger index (0..3)
ID_SET_POSE = 0xE0  # + finger index (0..3); undocumented on-board position, experimental
ID_SET_PERIOD = 0x81
ID_CONFIG = 0x68
ID_CALIBRATION = 0x89
ID_CALIBRATION_DONE = 0x92

ID_RTR_HAND_INFO = 0x80
ID_RTR_SERIAL = 0x88
ID_RTR_FINGER_POSE = 0x20  # + finger index (0..3)
ID_RTR_IMU = 0x30
ID_RTR_TEMPERATURE = 0x38  # + sensor index (0..3)

# Fingertip pressure frames. The v1.3 manual documents 0x50/0x52, but current firmware
# actually streams them at 0xF0/0xF2 (confirmed on a real V5 Plus). We accept both.
ID_FINGERTIP_0 = 0xF0  # payload: [index, middle] pressures  (manual: 0x50)
ID_FINGERTIP_2 = 0xF2  # payload: [ring, thumb] pressures    (manual: 0x52)
ID_FINGERTIP_0_LEGACY = 0x50
ID_FINGERTIP_2_LEGACY = 0x52

#: Map every accepted fingertip frame id -> base finger index (0 or 2).
FINGERTIP_ID_TO_BASE = {
    ID_FINGERTIP_0: 0, ID_FINGERTIP_0_LEGACY: 0,
    ID_FINGERTIP_2: 2, ID_FINGERTIP_2_LEGACY: 2,
}

ID_PICK = 0x11
ID_PLACE = 0x12
ID_ERROR = 0xEE

# ---------------------------------------------------------------------------
# Kinematic / actuation constants
# ---------------------------------------------------------------------------
NUM_FINGERS = 4
JOINTS_PER_FINGER = 4
DOF = NUM_FINGERS * JOINTS_PER_FINGER  # 16

#: Joint resolution in degrees per LSB of the raw int16 position value.
DEG_PER_LSB = 0.088

#: raw int16 -> radians.  angle_rad = raw * POSITION_SCALE
POSITION_SCALE = math.radians(DEG_PER_LSB)  # (pi/180) * 0.088

#: Desired joint torque [Nm] -> motor current command [mA].
NM_TO_MA = 1.43e3

#: Hard per-joint clamp applied by the firmware/stock driver, in mA.
TORQUE_LIMIT_MA = 240

#: On "Plus" (type B) hands the 2nd joint (MCP-2) of index/middle/ring has ~2x gear
#: ratio, so its torque command is halved.  Thumb is unaffected.
PLUS_HALVED_JOINTS = (1, 5, 9)

#: Finger names in canonical order (finger index 0..3).
FINGER_NAMES = ("index", "middle", "ring", "thumb")

#: Human-readable joint names, canonical 16-vector order.
JOINT_NAMES = (
    "index_mcp1", "index_mcp2", "index_pip", "index_dip",
    "middle_mcp1", "middle_mcp2", "middle_pip", "middle_dip",
    "ring_mcp1", "ring_mcp2", "ring_pip", "ring_dip",
    "thumb_cmc1", "thumb_cmc2", "thumb_mp", "thumb_ip",
)

# ---------------------------------------------------------------------------
# Streaming / timing
# ---------------------------------------------------------------------------
#: Default SET_PERIOD payload [pos_ms, imu_ms, temp_ms]; 3 ms position stream (~333 Hz).
DEFAULT_PERIOD_MS = (3, 0, 0)

#: Default host-side control loop rate (matches the hand's internal 2 ms / 500 Hz loop).
DEFAULT_CONTROL_HZ = 500.0

# ---------------------------------------------------------------------------
# Joint limits per hand configuration (canonical 16-vector order).
#
# ``right_B`` is MEASURED on a real V5 Plus (right, type B, serial 5TBR0017): each joint
# was driven to its mechanical stop and the min/max recorded. It is the validated set and
# this project's default. The measured travel ranges match the manual ROM (§15.1); note
# the thumb has encoder-zero OFFSETS vs the manual's nominal frame (CMC-2 ≈ +84°, CMC-1
# ≈ −20°) — the ranges are correct, the zero is shifted. Limits and reported angles are in
# the same (raw hardware) frame, so clamping stays consistent.
#
# The other three configs are NOT hardware-verified:
#   * ``right_A``  = measured Plus limits, but MCP-2 lower widened to the manual's standard
#                    −10° (the only documented A/B kinematic difference).
#   * ``left_A/B`` = ``right_A/B`` mirrored on the abduction joints (finger MCP-1 + thumb
#                    CMC-1). The thumb mirror in particular is a best-effort guess.
# Verify on the actual hand before trusting anything but ``right_B``.
# ---------------------------------------------------------------------------

# Measured extremes, degrees.
_RIGHT_B_LOWER_DEG = (
    -18.744, -2.904, -0.528, -4.664,
    -19.096, -5.808, -0.968, -1.496,
    -18.128, -2.992, -0.880, -0.968,
    3.5, 78.320, -6.512, -3.872,
)
_RIGHT_B_UPPER_DEG = (
    20.680, 101.200, 111.584, 105.512,
    18.568, 101.288, 110.880, 109.384,
    18.040, 102.256, 107.888, 109.824,
    88.176, 190.520, 108.416, 106.832,
)

#: Abduction joints that mirror (sign-flip) between a right and left hand.
ABDUCTION_JOINTS = (0, 4, 8, 12)  # index/middle/ring MCP-1 + thumb CMC-1

#: The MCP-2 (2nd) joints, whose lower limit differs between type A and B.
MCP2_JOINTS = (1, 5, 9)


def _deg2rad(t):
    return tuple(math.radians(x) for x in t)


def _mirror(lower, upper):
    lo, up = list(lower), list(upper)
    for j in ABDUCTION_JOINTS:
        lo[j], up[j] = -upper[j], -lower[j]
    return tuple(lo), tuple(up)


# right_A: widen MCP-2 lower limit to the manual's standard (non-Plus) -10°.
_RIGHT_A_LOWER_DEG = tuple(-10.0 if i in MCP2_JOINTS else v
                          for i, v in enumerate(_RIGHT_B_LOWER_DEG))
_RIGHT_A_UPPER_DEG = _RIGHT_B_UPPER_DEG

#: {"<handedness>_<type>": (lower_rad(16,), upper_rad(16,))}
JOINT_LIMITS = {
    "right_B": (_deg2rad(_RIGHT_B_LOWER_DEG), _deg2rad(_RIGHT_B_UPPER_DEG)),
    "right_A": (_deg2rad(_RIGHT_A_LOWER_DEG), _deg2rad(_RIGHT_A_UPPER_DEG)),
    "left_B": tuple(map(_deg2rad, _mirror(_RIGHT_B_LOWER_DEG, _RIGHT_B_UPPER_DEG))),
    "left_A": tuple(map(_deg2rad, _mirror(_RIGHT_A_LOWER_DEG, _RIGHT_A_UPPER_DEG))),
}

#: This project's hand.
DEFAULT_HAND_CONFIG = "right_B"

# Backwards-compatible flat aliases (the default configuration).
JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER = JOINT_LIMITS[DEFAULT_HAND_CONFIG]

#: Fingertip pressure sanity clamp used by the stock driver (Pa).
PRESSURE_VALID_MAX = 5000
