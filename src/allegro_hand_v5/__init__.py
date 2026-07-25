"""
Python driver for the Allegro Hand V5 (F4) / (F4) Plus.

Torque-only CAN hardware, driven by WONIK's bundled libBHand grasp library
plus a host-side control loop. No ROS.
"""

from allegro_hand_v5.bhand import BHand, HandType, HardwareType, MotionType
from allegro_hand_v5.calibration import (
    DEFAULT_RANGES,
    JOINT_NAMES,
    HandCalibration,
    default_calibration_path,
    load_calibration,
)
from allegro_hand_v5.can_driver import (
    NUM_FINGERS,
    NUM_JOINTS,
    AllegroCANDriver,
    HandInfo,
    HandState,
)
from allegro_hand_v5.control_loop import ControlLoop, ControlLoopStats, EmergencyStop
from allegro_hand_v5.exceptions import (
    AllegroBHandError,
    AllegroCANError,
    AllegroConnectionError,
    AllegroError,
    AllegroStateError,
    AllegroTimeoutError,
)
from allegro_hand_v5.hand import AllegroHand

__version__ = "0.1.0"

__all__ = [
    "AllegroHand",
    "AllegroCANDriver",
    "BHand",
    "MotionType",
    "HandType",
    "HardwareType",
    "ControlLoop",
    "ControlLoopStats",
    "EmergencyStop",
    "HandInfo",
    "HandState",
    "HandCalibration",
    "load_calibration",
    "default_calibration_path",
    "DEFAULT_RANGES",
    "JOINT_NAMES",
    "NUM_JOINTS",
    "NUM_FINGERS",
    "AllegroError",
    "AllegroConnectionError",
    "AllegroCANError",
    "AllegroBHandError",
    "AllegroTimeoutError",
    "AllegroStateError",
]
