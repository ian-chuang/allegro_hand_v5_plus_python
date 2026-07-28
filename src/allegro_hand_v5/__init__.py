"""
Pure-Python CAN driver for the Allegro Hand V5 (F4) / (F4) Plus.

The hand is current-controlled hardware. This package talks to it over CAN,
closes a PD loop on the host in its own process, and stays out of the way:

    from allegro_hand_v5 import AllegroHand, COMPLIANT

    with AllegroHand("can0", gains=COMPLIANT) as hand:
        print(hand.info)
        hand.set_position(hand.calibration.center)
        print(hand.positions, hand.pressures, hand.errors)
"""

from allegro_hand_v5.bus import AllegroCANBus, BusState, describe_link, link_status
from allegro_hand_v5.calibration import (
    CALIBRATION_DIR,
    NOMINAL_MAX,
    NOMINAL_MIN,
    Calibration,
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
    COMPLIANT,
    DEFAULT,
    PRESETS,
    SAFE,
    SOFT,
    ZERO,
    Gains,
    preset,
)
from allegro_hand_v5.protocol import (
    FINGER_NAMES,
    JOINT_INDEX,
    JOINT_NAMES,
    MAX_CURRENT_MA,
    NUM_FINGERS,
    NUM_JOINTS,
    POSITION_SCALE,
    ErrorFlag,
    HandInfo,
    JointError,
    MsgID,
)

__version__ = "0.3.0"

__all__ = [
    # driver
    "AllegroHand",
    "DriverConfig",
    "HandState",
    # transport and protocol
    "AllegroCANBus",
    "BusState",
    "describe_link",
    "link_status",
    "MsgID",
    "HandInfo",
    "JointError",
    "ErrorFlag",
    "POSITION_SCALE",
    "MAX_CURRENT_MA",
    "NUM_JOINTS",
    "NUM_FINGERS",
    "JOINT_NAMES",
    "JOINT_INDEX",
    "FINGER_NAMES",
    # gains
    "Gains",
    "DEFAULT",
    "COMPLIANT",
    "SOFT",
    "SAFE",
    "ZERO",
    "PRESETS",
    "preset",
    # calibration
    "Calibration",
    "load_calibration",
    "CALIBRATION_DIR",
    "NOMINAL_MIN",
    "NOMINAL_MAX",
    # errors
    "AllegroError",
    "AllegroConnectionError",
    "AllegroCANError",
    "AllegroTimeoutError",
    "AllegroStateError",
]
