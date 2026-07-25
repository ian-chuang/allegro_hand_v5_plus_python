"""
Fixed-rate control loop for the Allegro Hand V5.

A background thread runs, at ``frequency`` Hz:

    CAN RX  ->  state estimate  ->  BHand.UpdateControl  ->  torque TX

BHand owns whatever the current motion type is (HOME, GRAVITY_COMP, JOINT_PD,
a grasp, ...). Direct torque commands from the application bypass this loop and
go straight to the CAN driver, exactly as in the reference C++/Python stack —
see :meth:`allegro_hand_v5.hand.AllegroHand.set_torques`.

Timing is soft real-time (plain ``time.sleep``), not hard real-time.
"""

import logging
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np

from allegro_hand_v5.bhand import NUM_JOINTS, BHand, MotionType
from allegro_hand_v5.can_driver import AllegroCANDriver

logger = logging.getLogger(__name__)

DEFAULT_CONTROL_FREQUENCY = 500.0  # Hz

# BHand torque -> raw PWM/current units accepted by the firmware.
TORQUE_TO_PWM = 1.43 * 1000.0
PWM_LIMIT = 240.0

# Low-pass coefficient for the finite-difference velocity estimate.
VELOCITY_FILTER_ALPHA = 0.3


@dataclass
class ControlLoopStats:
    """Timing statistics for the control thread."""

    iterations: int = 0
    total_time: float = 0.0
    min_period: float = float("inf")
    max_period: float = 0.0
    avg_period: float = 0.0
    missed_deadlines: int = 0

    def update(self, period: float, target_period: float) -> None:
        self.iterations += 1
        self.total_time += period
        self.min_period = min(self.min_period, period)
        self.max_period = max(self.max_period, period)
        self.avg_period = self.total_time / self.iterations
        if period > target_period * 1.5:
            self.missed_deadlines += 1

    def reset(self) -> None:
        self.__init__()


class ControlLoop:
    """Background thread: read positions, run BHand, send its torques."""

    def __init__(
        self,
        can_driver: AllegroCANDriver,
        bhand: BHand,
        frequency: float = DEFAULT_CONTROL_FREQUENCY,
    ):
        self.can_driver = can_driver
        self.bhand = bhand
        self.frequency = frequency
        self.period = 1.0 / frequency

        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()

        self._current_positions = np.zeros(NUM_JOINTS)
        self._current_velocities = np.zeros(NUM_JOINTS)
        self._previous_positions = np.zeros(NUM_JOINTS)
        self._desired_torques = np.zeros(NUM_JOINTS)

        self._start_time = 0.0
        self._current_time = 0.0
        self._stats = ControlLoopStats()

        self._pre_control_callback: Optional[Callable[[float, np.ndarray], None]] = None
        self._post_control_callback: Optional[
            Callable[[float, np.ndarray, np.ndarray], None]
        ] = None

        self.bhand.set_time_interval(self.period)

    @property
    def running(self) -> bool:
        return self._running

    @property
    def stats(self) -> ControlLoopStats:
        return self._stats

    @property
    def current_positions(self) -> np.ndarray:
        with self._lock:
            return self._current_positions.copy()

    @property
    def current_velocities(self) -> np.ndarray:
        with self._lock:
            return self._current_velocities.copy()

    @property
    def current_torques(self) -> np.ndarray:
        with self._lock:
            return self._desired_torques.copy()

    @property
    def elapsed_time(self) -> float:
        if self._start_time == 0.0:
            return 0.0
        return time.time() - self._start_time

    def set_pre_control_callback(
        self, callback: Optional[Callable[[float, np.ndarray], None]]
    ) -> None:
        """Called with (time, positions) before BHand runs."""
        self._pre_control_callback = callback

    def set_post_control_callback(
        self, callback: Optional[Callable[[float, np.ndarray, np.ndarray], None]]
    ) -> None:
        """Called with (time, positions, torques) after BHand runs."""
        self._post_control_callback = callback

    def start(self) -> None:
        if self._running:
            logger.warning("Control loop already running")
            return

        self._running = True
        self._start_time = time.time()
        self._stats.reset()

        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()
        logger.info("Control loop started at %.1f Hz", self.frequency)

    def stop(self, timeout: float = 1.0) -> None:
        if not self._running:
            return

        self._running = False
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=timeout)
            if self._thread.is_alive():
                logger.warning("Control loop thread did not stop cleanly")
        self._thread = None

        logger.info(
            "Control loop stopped. %d iterations, avg period %.2f ms, %d missed",
            self._stats.iterations,
            self._stats.avg_period * 1000.0,
            self._stats.missed_deadlines,
        )

    def _control_loop(self) -> None:
        last_time = time.time()

        while self._running:
            loop_start = time.time()

            try:
                self.can_driver.process_messages(timeout=0.0001, max_messages=50)

                self._current_time = loop_start - self._start_time

                with self._lock:
                    new_positions = self.can_driver.state.positions.copy()

                    dt = loop_start - last_time
                    if dt > 0:
                        raw_velocity = (new_positions - self._previous_positions) / dt
                        self._current_velocities = (
                            VELOCITY_FILTER_ALPHA * raw_velocity
                            + (1.0 - VELOCITY_FILTER_ALPHA) * self._current_velocities
                        )

                    self._previous_positions = self._current_positions.copy()
                    self._current_positions = new_positions

                if self._pre_control_callback:
                    self._pre_control_callback(self._current_time, self._current_positions)

                self.bhand.set_joint_position(self._current_positions)
                self.bhand.update_control(self._current_time)

                with self._lock:
                    self._desired_torques = self.bhand.get_joint_torque()

                if self._post_control_callback:
                    self._post_control_callback(
                        self._current_time,
                        self._current_positions,
                        self._desired_torques,
                    )

                self._send_torques()
                last_time = loop_start

            except Exception as e:
                logger.error("Error in control loop: %s", e)

            sleep_time = self.period - (time.time() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

            self._stats.update(time.time() - loop_start, self.period)

    def _send_torques(self) -> None:
        """Scale BHand's torques into PWM units and put them on the bus."""
        pwm = np.clip(self._desired_torques * TORQUE_TO_PWM, -PWM_LIMIT, PWM_LIMIT)
        self.can_driver.set_torques(pwm)

    def set_motion_type(self, motion_type: MotionType) -> None:
        """Change the BHand motion type while the loop is running."""
        with self._lock:
            self.bhand.set_motion_type(motion_type)
        logger.debug("Motion type set to %s", motion_type.name)

    def set_desired_positions(self, positions: np.ndarray) -> None:
        """Update the BHand PD target while the loop is running."""
        if len(positions) != NUM_JOINTS:
            raise ValueError(f"Expected {NUM_JOINTS} positions, got {len(positions)}")
        with self._lock:
            self.bhand.set_desired_position(positions)


class EmergencyStop:
    """Stops the control loop, zeroes the torques, and drops the servos."""

    def __init__(self, control_loop: ControlLoop, can_driver: AllegroCANDriver):
        self.control_loop = control_loop
        self.can_driver = can_driver
        self._triggered = False

    @property
    def triggered(self) -> bool:
        return self._triggered

    def trigger(self) -> None:
        if self._triggered:
            return

        logger.warning("EMERGENCY STOP TRIGGERED")
        self._triggered = True

        self.control_loop.stop(timeout=0.5)
        self.can_driver.set_torques(np.zeros(NUM_JOINTS))
        self.can_driver.servo_off()

    def reset(self) -> None:
        self._triggered = False
        logger.info("Emergency stop reset")
