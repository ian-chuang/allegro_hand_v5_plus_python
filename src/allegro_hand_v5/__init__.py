"""
Python driver for the Allegro Hand V5 (F4) / (F4) Plus.

Torque-only CAN hardware, with a host-side PD loop running in its own process.
No ROS, no libBHand.

    from allegro_hand_v5 import AllegroHand, COMPLIANT

    with AllegroHand("can0", gains=COMPLIANT) as hand:
        print(hand.info)
        hand.set_position_deg([0, 40, 40, 40] * 3 + [40, 140, 40, 40])
        print(hand.positions_deg, hand.pressures, hand.errors)
"""

from allegro_hand_v5.bus import AllegroCANBus, BusState
from allegro_hand_v5.calibration import (
    BUNDLED_DIR,
    DEFAULT_RANGES,
    HandCalibration,
    available_calibrations,
    calibration_search_paths,
    default_calibration_path,
    load_calibration,
)
from allegro_hand_v5.driver import AllegroHand, DriverConfig, HandState
from allegro_hand_v5.exceptions import (
    AllegroCANError,
    AllegroConnectionError,
    AllegroError,
    AllegroStateError,
    AllegroTimeoutError,
)
from allegro_hand_v5.gains import (
    BHAND_HOME,
    BHAND_JOINT_PD,
    COMPLIANT,
    DEFAULT,
    PROFILES,
    SAFE,
    SOFT,
    ZERO,
    GainProfile,
    get_profile,
)
from allegro_hand_v5.protocol import (
    FINGER_NAMES,
    JOINT_LABELS,
    JOINT_NAMES,
    NUM_FINGERS,
    NUM_JOINTS,
    POSITION_SCALE,
    TORQUE_TO_CURRENT,
    ErrorFlag,
    HandInfo,
    JointError,
    MsgID,
)

__version__ = "0.2.0"

__all__ = [
    # driver
    "AllegroHand",
    "DriverConfig",
    "HandState",
    # transport / protocol
    "AllegroCANBus",
    "BusState",
    "MsgID",
    "HandInfo",
    "JointError",
    "ErrorFlag",
    "POSITION_SCALE",
    "TORQUE_TO_CURRENT",
    "NUM_JOINTS",
    "NUM_FINGERS",
    "JOINT_NAMES",
    "JOINT_LABELS",
    "FINGER_NAMES",
    # gains
    "GainProfile",
    "BHAND_HOME",
    "BHAND_JOINT_PD",
    "COMPLIANT",
    "SOFT",
    "SAFE",
    "ZERO",
    "DEFAULT",
    "PROFILES",
    "get_profile",
    # calibration
    "HandCalibration",
    "load_calibration",
    "default_calibration_path",
    "calibration_search_paths",
    "available_calibrations",
    "BUNDLED_DIR",
    "DEFAULT_RANGES",
    # errors
    "AllegroError",
    "AllegroConnectionError",
    "AllegroCANError",
    "AllegroTimeoutError",
    "AllegroStateError",
]
