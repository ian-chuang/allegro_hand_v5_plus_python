"""
ctypes wrapper for the BHand library (libBHand.so).

BHand is WONIK's precompiled grasp/gravity-compensation algorithm. It exports
C++ class methods rather than a C API, so every call below goes through the
Itanium-mangled symbol name with the instance pointer passed as the implicit
first argument.
"""

import ctypes
import logging
from ctypes import POINTER, c_double, c_int, c_void_p
from enum import IntEnum
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

from allegro_hand_v5.exceptions import AllegroBHandError

logger = logging.getLogger(__name__)

NUM_FINGERS = 4
NUM_JOINTS_PER_FINGER = 4
NUM_JOINTS = NUM_FINGERS * NUM_JOINTS_PER_FINGER  # 16


class MotionType(IntEnum):
    """Motion types supported by BHand."""

    NONE = 0  # power off
    HOME = 1  # go to the home position
    GRASP_3 = 2  # grasp with 3 fingers
    GRASP_4 = 3  # grasp with 4 fingers
    PINCH_IT = 4  # pinch, index + thumb
    PINCH_MT = 5  # pinch, middle + thumb
    ENVELOP = 6  # enveloping grasp
    JOINT_PD = 7  # joint-space PD
    POSE_PD = 8  # custom pose PD
    GRAVITY_COMP = 9  # gravity compensation
    SAVE = 10  # save the current pose
    TYPE_A = 11  # hand type A (non-geared)
    TYPE_B = 12  # hand type B (geared)


class HandType(IntEnum):
    LEFT = 0
    RIGHT = 1


class HardwareType(IntEnum):
    B_GEARED = 0
    A_NON_GEARED = 1


def _find_library() -> Path:
    """Locate libBHand.so: bundled first, then the usual system paths."""
    pkg_lib = Path(__file__).parent / "lib" / "libBHand.so"
    if pkg_lib.exists():
        return pkg_lib

    for path in (Path("/usr/local/lib/libBHand.so"), Path("/usr/lib/libBHand.so")):
        if path.exists():
            return path

    raise AllegroBHandError(
        "libBHand.so not found. Ensure it is installed or bundled with the package."
    )


class BHand:
    """Python binding to the BHand grasp algorithm."""

    _lib: Optional[ctypes.CDLL] = None
    _lib_path: Optional[Path] = None

    @classmethod
    def _load_library(cls) -> ctypes.CDLL:
        """Load the shared library once, process-wide."""
        if cls._lib is None:
            cls._lib_path = _find_library()
            cls._lib = ctypes.CDLL(str(cls._lib_path))

            cls._lib.bhCreateLeftHand.argtypes = []
            cls._lib.bhCreateLeftHand.restype = c_void_p
            cls._lib.bhCreateRightHand.argtypes = []
            cls._lib.bhCreateRightHand.restype = c_void_p

            logger.info("Loaded BHand library from %s", cls._lib_path)
        return cls._lib

    def __init__(self, hand_type: HandType = HandType.RIGHT):
        self._lib = self._load_library()
        self._hand_type = hand_type

        if hand_type == HandType.LEFT:
            self._handle = self._lib.bhCreateLeftHand()
        else:
            self._handle = self._lib.bhCreateRightHand()

        if not self._handle:
            raise AllegroBHandError(f"Failed to create BHand instance for {hand_type.name} hand")

        # Buffers shared with C++; reused every cycle to avoid per-call allocation.
        self._q = (c_double * NUM_JOINTS)()
        self._q_des = (c_double * NUM_JOINTS)()
        self._tau = (c_double * NUM_JOINTS)()
        self._kp = (c_double * NUM_JOINTS)()
        self._kd = (c_double * NUM_JOINTS)()
        self._x = (c_double * NUM_FINGERS)()
        self._y = (c_double * NUM_FINGERS)()
        self._z = (c_double * NUM_FINGERS)()
        self._f = (c_double * NUM_FINGERS)()

        logger.debug("Created BHand instance for %s hand", hand_type.name)

    def _method(self, symbol: str, argtypes, restype=None):
        """Bind a mangled C++ method, with the instance pointer prepended."""
        try:
            func = getattr(self._lib, symbol)
        except AttributeError as e:
            raise AllegroBHandError(f"BHand symbol not found: {symbol}") from e
        func.argtypes = [c_void_p, *argtypes]
        func.restype = restype
        return func

    @property
    def hand_type(self) -> HandType:
        return self._hand_type

    # ==================== Configuration ====================

    def set_hardware_type(self, hw_type: HardwareType) -> None:
        """Select geared (B) or non-geared (A). The C++ setter is named GetType."""
        try:
            self._method("_ZN5BHand7GetTypeE13eHardwareType", [c_int])(
                self._handle, int(hw_type)
            )
        except AllegroBHandError:
            logger.warning("GetType not found; falling back to the TYPE_A/TYPE_B motion type")
            self.set_motion_type(
                MotionType.TYPE_A if hw_type == HardwareType.A_NON_GEARED else MotionType.TYPE_B
            )

    def set_time_interval(self, dt: float) -> None:
        """Control period in seconds (e.g. 0.002 for 500 Hz)."""
        self._method("_ZN5BHand15SetTimeIntervalEd", [c_double])(self._handle, dt)

    def get_time_interval(self) -> float:
        return self._method("_ZN5BHand15GetTimeIntervalEv", [], c_double)(self._handle)

    def set_motion_time(self, seconds: float) -> None:
        """Duration used by the HOME and JOINT_PD motions."""
        self._method("_ZN5BHand13SetMotiontimeEd", [c_double])(self._handle, seconds)

    def set_motion_type(self, motion_type: MotionType) -> None:
        self._method("_ZN5BHand13SetMotionTypeEi", [c_int])(self._handle, int(motion_type))

    # ==================== Per-cycle control ====================

    def set_joint_position(self, positions: np.ndarray) -> None:
        """Feed the measured joint angles (16, radians)."""
        if len(positions) != NUM_JOINTS:
            raise ValueError(f"Expected {NUM_JOINTS} positions, got {len(positions)}")
        for i in range(NUM_JOINTS):
            self._q[i] = float(positions[i])
        self._method("_ZN5BHand16SetJointPositionEPd", [POINTER(c_double)])(self._handle, self._q)

    def set_desired_position(self, positions: np.ndarray) -> None:
        """Set the PD target (16, radians)."""
        if len(positions) != NUM_JOINTS:
            raise ValueError(f"Expected {NUM_JOINTS} positions, got {len(positions)}")
        for i in range(NUM_JOINTS):
            self._q_des[i] = float(positions[i])
        self._method("_ZN5BHand23SetJointDesiredPositionEPd", [POINTER(c_double)])(
            self._handle, self._q_des
        )

    def update_control(self, time: float) -> None:
        """Run one control iteration at the given elapsed time."""
        self._method("_ZN5BHand13UpdateControlEd", [c_double])(self._handle, time)

    def get_joint_torque(self) -> np.ndarray:
        """Torques computed by the last update_control()."""
        self._method("_ZN5BHand14GetJointTorqueEPd", [POINTER(c_double)])(self._handle, self._tau)
        return np.array([self._tau[i] for i in range(NUM_JOINTS)])

    # ==================== Optional tuning ====================

    def set_gains(self, kp: np.ndarray, kd: np.ndarray) -> None:
        if len(kp) != NUM_JOINTS or len(kd) != NUM_JOINTS:
            raise ValueError(f"Expected {NUM_JOINTS} gains each")
        for i in range(NUM_JOINTS):
            self._kp[i] = float(kp[i])
            self._kd[i] = float(kd[i])
        self._method("_ZN5BHand10SetGainsExEPdS0_", [POINTER(c_double), POINTER(c_double)])(
            self._handle, self._kp, self._kd
        )

    def set_grasping_force(self, forces: np.ndarray) -> None:
        if len(forces) != NUM_FINGERS:
            raise ValueError(f"Expected {NUM_FINGERS} forces")
        for i in range(NUM_FINGERS):
            self._f[i] = float(forces[i])
        self._method("_ZN5BHand16SetGraspingForceEPd", [POINTER(c_double)])(self._handle, self._f)

    def set_envelop_torque_scalar(self, scalar: float = 1.0) -> None:
        self._method("_ZN5BHand22SetEnvelopTorqueScalarEd", [c_double])(self._handle, scalar)

    def set_orientation(self, roll: float, pitch: float, yaw: float) -> None:
        """Palm orientation, used by gravity compensation."""
        self._method("_ZN5BHand14SetOrientationEddd", [c_double, c_double, c_double])(
            self._handle, roll, pitch, yaw
        )

    def get_fingertip_positions(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Forward kinematics: (x, y, z), 4 elements each."""
        self._method(
            "_ZN5BHand11GetFKResultEPdS0_S0_",
            [POINTER(c_double), POINTER(c_double), POINTER(c_double)],
        )(self._handle, self._x, self._y, self._z)
        return (
            np.array([self._x[i] for i in range(NUM_FINGERS)]),
            np.array([self._y[i] for i in range(NUM_FINGERS)]),
            np.array([self._z[i] for i in range(NUM_FINGERS)]),
        )
