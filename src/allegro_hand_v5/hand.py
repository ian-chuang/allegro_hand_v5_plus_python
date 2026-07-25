"""
High-level interface to the Allegro Hand V5.

``AllegroHand`` owns the CAN driver, a BHand instance, and the background
:class:`ControlLoop`. BHand drives the hand for all of the built-in motions
(home, grasps, gravity compensation, joint PD). Direct torque control bypasses
BHand's *output* but not its loop — see :meth:`AllegroHand.set_torques`.
"""

import logging
import time
from contextlib import contextmanager
from typing import Callable, List, Optional, Tuple, Union

import numpy as np

from allegro_hand_v5.bhand import (
    NUM_FINGERS,
    NUM_JOINTS,
    BHand,
    HandType,
    HardwareType,
    MotionType,
)
from allegro_hand_v5.calibration import HandCalibration, load_calibration
from allegro_hand_v5.can_driver import AllegroCANDriver, HandInfo
from allegro_hand_v5.control_loop import (
    PWM_LIMIT,
    TORQUE_TO_PWM,
    ControlLoop,
    ControlLoopStats,
    EmergencyStop,
)
from allegro_hand_v5.exceptions import AllegroConnectionError, AllegroStateError

logger = logging.getLogger(__name__)


class AllegroHand:
    """
    Example:
        >>> with AllegroHand(hand_type="right", hardware_type="B") as hand:
        ...     hand.home()
        ...     print(hand.get_positions())
    """

    def __init__(
        self,
        hand_type: Union[str, HandType] = "right",
        hardware_type: Union[str, HardwareType] = "B",
        can_channel: str = "can0",
        control_frequency: float = 500.0,
        strict: bool = False,
        calibration: Union[None, bool, str, HandCalibration] = None,
        limit_margin: float = 0.0,
    ):
        """
        Args:
            hand_type: "left" or "right".
            hardware_type: "A" (non-geared) or "B" (geared).
            can_channel: SocketCAN interface name.
            control_frequency: Rate of the background BHand/CAN thread, in Hz.
            strict: Raise on CAN errors instead of logging them.
            calibration: True to load ``calibration/<hand_type>.json``, a path,
                a HandCalibration, or None for no calibration.
            limit_margin: Radians trimmed from each side of the calibrated
                range, as a safety buffer away from the hard stops.
        """
        if isinstance(hand_type, str):
            hand_type = HandType.RIGHT if hand_type.lower() == "right" else HandType.LEFT
        self._hand_type = hand_type

        if isinstance(hardware_type, str):
            hardware_type = (
                HardwareType.A_NON_GEARED
                if hardware_type.upper() == "A"
                else HardwareType.B_GEARED
            )
        self._hardware_type = hardware_type

        self._can_channel = can_channel
        self._control_frequency = control_frequency
        self._strict = strict

        self._limit_margin = limit_margin
        self._calibration = load_calibration(calibration, hand_type=self._hand_type.name.lower())

        self._can_driver: Optional[AllegroCANDriver] = None
        self._bhand: Optional[BHand] = None
        self._control_loop: Optional[ControlLoop] = None
        self._emergency_stop: Optional[EmergencyStop] = None

        self._connected = False
        self._motion_time = 1.0
        self._grasp_force = 2.0

    # ==================== Properties ====================

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def hand_type(self) -> HandType:
        return self._hand_type

    @property
    def hardware_type(self) -> HardwareType:
        return self._hardware_type

    @property
    def hand_info(self) -> Optional[HandInfo]:
        return self._can_driver.hand_info if self._can_driver else None

    @property
    def stats(self) -> Optional[ControlLoopStats]:
        return self._control_loop.stats if self._control_loop else None

    @property
    def calibration(self) -> Optional[HandCalibration]:
        return self._calibration

    # ==================== Connection ====================

    def connect(self) -> None:
        """Open CAN, hand-shake, create BHand, and start the control thread."""
        if self._connected:
            logger.warning("Already connected")
            return

        try:
            logger.info(
                "Connecting to Allegro Hand (%s, %s) on %s",
                self._hand_type.name,
                self._hardware_type.name,
                self._can_channel,
            )

            self._can_driver = AllegroCANDriver(channel=self._can_channel, strict=self._strict)
            self._can_driver.connect()

            if not self._can_driver.initialize_hand():
                raise AllegroConnectionError("Failed to initialize hand")

            self._bhand = BHand(self._hand_type)
            self._bhand.set_hardware_type(self._hardware_type)
            self._bhand.set_time_interval(1.0 / self._control_frequency)
            self._bhand.set_motion_time(self._motion_time)

            self._control_loop = ControlLoop(
                can_driver=self._can_driver,
                bhand=self._bhand,
                frequency=self._control_frequency,
            )
            self._emergency_stop = EmergencyStop(self._control_loop, self._can_driver)
            self._control_loop.start()

            self._connected = True
            logger.info("Connected to Allegro Hand")

            # Let a few position frames land before anyone reads state.
            time.sleep(0.1)

        except Exception as e:
            self._cleanup()
            if self._strict:
                raise AllegroConnectionError(f"Failed to connect: {e}") from e
            logger.error("Failed to connect: %s", e)

    def disconnect(self) -> None:
        """Power the motors down, stop the control thread, close the bus."""
        if not self._connected:
            return

        logger.info("Disconnecting from Allegro Hand")
        try:
            self.off()
            time.sleep(0.1)
        except Exception as e:
            logger.warning("Error during safe shutdown: %s", e)

        self._cleanup()
        logger.info("Disconnected from Allegro Hand")

    def _cleanup(self) -> None:
        if self._control_loop is not None:
            self._control_loop.stop()
            self._control_loop = None
        if self._can_driver is not None:
            self._can_driver.disconnect()
            self._can_driver = None
        self._bhand = None
        self._emergency_stop = None
        self._connected = False

    def _check_connected(self) -> None:
        if not self._connected:
            raise AllegroStateError("Not connected to hand. Call connect() first.")

    # ==================== BHand motions ====================

    def home(self) -> None:
        """Move to the BHand home position."""
        self._check_connected()
        logger.info("Moving to home position")
        self._control_loop.set_motion_type(MotionType.HOME)

    def grasp_3(self) -> None:
        """Three-finger grasp (index, middle, thumb)."""
        self._check_connected()
        self._control_loop.set_motion_type(MotionType.GRASP_3)

    def grasp_4(self) -> None:
        """Four-finger grasp."""
        self._check_connected()
        self._control_loop.set_motion_type(MotionType.GRASP_4)

    def pinch_it(self) -> None:
        """Index-thumb pinch."""
        self._check_connected()
        self._control_loop.set_motion_type(MotionType.PINCH_IT)

    def pinch_mt(self) -> None:
        """Middle-thumb pinch."""
        self._check_connected()
        self._control_loop.set_motion_type(MotionType.PINCH_MT)

    def envelop(self) -> None:
        """Enveloping grasp."""
        self._check_connected()
        self._control_loop.set_motion_type(MotionType.ENVELOP)

    def gravity_comp(self) -> None:
        """Gravity compensation — the hand holds itself but stays backdrivable."""
        self._check_connected()
        logger.info("Enabling gravity compensation")
        self._control_loop.set_motion_type(MotionType.GRAVITY_COMP)

    def off(self) -> None:
        """Motors off (BHand outputs zero torque)."""
        self._check_connected()
        logger.info("Turning motors off")
        self._control_loop.set_motion_type(MotionType.NONE)

    # ==================== Position control ====================

    def move_to(
        self,
        positions: Union[List[float], np.ndarray],
        wait: bool = True,
        timeout: float = 5.0,
        threshold: float = 0.05,
        clip: bool = True,
    ) -> bool:
        """
        Drive to a joint target using BHand's joint-space PD.

        Args:
            positions: 16 target angles in radians.
            wait: Block until reached or timed out.
            timeout: Maximum wait in seconds.
            threshold: Per-joint error counting as "reached", in radians.
            clip: Clip the target into the calibrated range, if one is set.

        Returns:
            True if the target was reached (or wait=False).
        """
        self._check_connected()

        positions = np.asarray(positions, dtype=np.float64)
        if len(positions) != NUM_JOINTS:
            raise ValueError(f"Expected {NUM_JOINTS} positions, got {len(positions)}")

        if clip and self._calibration is not None:
            clipped = self._calibration.clip(positions)
            if not np.allclose(clipped, positions):
                logger.warning(
                    "Target outside calibrated range, clipping joints %s",
                    np.flatnonzero(~np.isclose(clipped, positions)).tolist(),
                )
            positions = clipped

        self._control_loop.set_desired_positions(positions)
        self._control_loop.set_motion_type(MotionType.JOINT_PD)

        if not wait:
            return True

        start = time.time()
        error = float("inf")
        while time.time() - start < timeout:
            error = float(np.abs(self.get_positions() - positions).max())
            if error < threshold:
                return True
            time.sleep(0.01)

        logger.warning("Timeout waiting for position (error: %.4f)", error)
        return False

    # ==================== State ====================

    def get_positions(self) -> np.ndarray:
        """Current joint positions, 16 values in radians."""
        self._check_connected()
        return self._control_loop.current_positions

    def get_velocities(self) -> np.ndarray:
        """Filtered joint velocities, 16 values in rad/s."""
        self._check_connected()
        return self._control_loop.current_velocities

    def get_torques(self) -> np.ndarray:
        """Torques most recently computed by BHand."""
        self._check_connected()
        return self._control_loop.current_torques

    def get_fingertip_positions(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Fingertip forward kinematics: (x, y, z), 4 elements each."""
        self._check_connected()
        return self._bhand.get_fingertip_positions()

    # ==================== BHand parameters ====================

    def set_motion_time(self, seconds: float) -> None:
        """Duration BHand uses for the home and joint-PD motions."""
        self._check_connected()
        self._motion_time = seconds
        self._bhand.set_motion_time(seconds)

    def set_grasp_force(self, force: float) -> None:
        """Grasping force applied by the grasp/pinch motions."""
        self._check_connected()
        self._grasp_force = force
        self._bhand.set_grasping_force(np.full(NUM_FINGERS, force))

    def set_orientation(self, roll: float, pitch: float, yaw: float) -> None:
        """Palm orientation in radians, used by gravity compensation."""
        self._check_connected()
        self._bhand.set_orientation(roll, pitch, yaw)

    def set_pd_gains(self, kp: np.ndarray, kd: np.ndarray) -> None:
        """Override BHand's PD gains (16 values each)."""
        self._check_connected()
        self._bhand.set_gains(np.asarray(kp), np.asarray(kd))

    # ==================== Direct torque control ====================

    @contextmanager
    def torque_mode(self):
        """
        Scope for direct torque control; returns to gravity compensation on exit.

        Example:
            >>> with hand.torque_mode():
            ...     hand.set_torques(tau)
        """
        self._check_connected()
        try:
            yield self
        finally:
            self.gravity_comp()

    def set_torques(self, torques: Union[List[float], np.ndarray]) -> None:
        """
        Put a torque command on the bus immediately, from the calling thread.

        This bypasses BHand's output but *not* its loop: the control thread is
        still running the current motion type and still transmitting its own
        torques at ``control_frequency``, so the two interleave on the bus.
        Set the motion type to ``off()`` first if you want this to be the only
        command reaching the hand.

        Args:
            torques: 16 torque values. Scaled by 1430 into raw current units
                and clamped to +/-240 before transmission.
        """
        self._check_connected()

        torques = np.asarray(torques, dtype=np.float64)
        if len(torques) != NUM_JOINTS:
            raise ValueError(f"Expected {NUM_JOINTS} torques, got {len(torques)}")

        pwm = np.clip(torques * TORQUE_TO_PWM, -PWM_LIMIT, PWM_LIMIT)
        self._can_driver.set_torques(pwm)

    # ==================== Callbacks ====================

    def set_position_callback(
        self, callback: Optional[Callable[[float, np.ndarray], None]]
    ) -> None:
        """Run (time, positions) each cycle, before BHand."""
        self._check_connected()
        self._control_loop.set_pre_control_callback(callback)

    def set_control_callback(
        self, callback: Optional[Callable[[float, np.ndarray, np.ndarray], None]]
    ) -> None:
        """Run (time, positions, torques) each cycle, after BHand."""
        self._check_connected()
        self._control_loop.set_post_control_callback(callback)

    # ==================== Emergency stop ====================

    def emergency_stop(self) -> None:
        """Stop the control loop, zero the torques, drop the servos."""
        if self._emergency_stop:
            self._emergency_stop.trigger()
        elif self._can_driver:
            self._can_driver.servo_off()

    def reset_emergency_stop(self) -> None:
        if self._emergency_stop:
            self._emergency_stop.reset()

    # ==================== Context manager ====================

    def __enter__(self) -> "AllegroHand":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> bool:
        self.disconnect()
        return False

    def __repr__(self) -> str:
        status = "connected" if self._connected else "disconnected"
        return (
            f"AllegroHand(type={self._hand_type.name}, "
            f"hardware={self._hardware_type.name}, "
            f"channel={self._can_channel}, status={status})"
        )
