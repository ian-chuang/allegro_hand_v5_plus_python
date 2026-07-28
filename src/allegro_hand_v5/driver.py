"""
`AllegroHand` — the driver.

The hand is current-controlled: the only actuation command is a motor current
per joint. This driver runs the CAN traffic and the host-side PD in a child
process, so nothing your script does can stall them, and exposes state and
commands through shared memory.

The control law is exactly this, and nothing else::

    dq      = a * (q - q_prev) / dt + (1 - a) * dq      # EMA on the difference
    current = kp * (q_desired - q) - kd * dq            # position mode
    current = i_desired                                 # current mode
    current = 0                                         # idle
    send(clip(current, -max_current, +max_current))

The loop is driven by the hand's position stream: WONIK's own driver computes
and transmits once per complete set of four finger reports, and so does this
one. If those reports stop for `stale_timeout`, the current goes to zero.

Shutdown matters more than usual here: the hand latches the last current
command, so if the host stops talking while the servos are engaged the motors
keep pushing. Servo OFF is enforced four ways:

1. the child's `finally` — a normal exit or any exception in the loop;
2. a SIGTERM handler in the child, so `terminate()` unwinds through that finally;
3. a `getppid()` watchdog in the child, for a parent that died without asking;
4. `atexit` in the parent, plus a direct Servo OFF from the parent if the child
   died without confirming (SIGKILL, segfault).

    from allegro_hand_v5 import AllegroHand, COMPLIANT

    with AllegroHand("can0", gains=COMPLIANT) as hand:
        print(hand.info)
        hand.set_position(q_des)
        print(hand.positions, hand.pressures, hand.errors)
"""

from __future__ import annotations

import atexit
import ctypes
import logging
import multiprocessing as mp
import os
import signal
import time
import weakref
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Union

import numpy as np

from allegro_hand_v5 import gains as gains_module
from allegro_hand_v5.bus import AllegroCANBus, describe_link
from allegro_hand_v5.calibration import Calibration, load_calibration
from allegro_hand_v5.exceptions import AllegroStateError
from allegro_hand_v5.gains import Gains
from allegro_hand_v5.protocol import (
    NUM_FINGERS,
    NUM_JOINTS,
    ErrorFlag,
    HandInfo,
    JointError,
)

logger = logging.getLogger(__name__)

MODE_IDLE, MODE_POSITION, MODE_CURRENT = 0.0, 1.0, 2.0
_MODE_NAMES = {MODE_IDLE: "idle", MODE_POSITION: "position", MODE_CURRENT: "current"}

_SERIAL_LEN = 8


@dataclass
class DriverConfig:
    """Everything the control process needs that is not a gain."""

    position_period_ms: int = 3
    """How often the hand reports joint positions. Drives the control rate."""

    imu_period_ms: int = 0
    """IMU report period. 0 disables the stream."""

    temperature_period_ms: int = 0
    """Motor temperature report period. 0 disables the stream."""

    velocity_filter: float = 0.3
    """EMA coefficient on the finite-difference velocity. 1.0 = unfiltered."""

    stale_timeout: float = 0.1
    """Command zero current if no position update arrives for this long."""

    poll_timeout: float = 0.002
    """How long each CAN receive may block. Sets the idle loop rate."""


@dataclass
class HandState:
    """One consistent snapshot of everything the control process publishes."""

    t: float  # seconds since the control loop started
    positions: np.ndarray  # (16,) rad, offset frame
    velocities: np.ndarray  # (16,) rad/s, filtered
    currents: np.ndarray  # (16,) mA, as sent
    pressures: np.ndarray  # (4,) Pa: index, middle, ring, thumb
    temperatures: np.ndarray  # (16,) deg C, zero unless temperature_period_ms is set
    imu: np.ndarray  # (3,) raw roll/pitch/yaw, zero unless imu_period_ms is set
    errors: np.ndarray  # (16,) ErrorFlag bitmasks
    servo_on: bool
    mode: str  # "idle", "position" or "current" — what the loop is running
    position_age: float  # seconds since the last complete position update
    updates: int  # control updates since start
    rate: float  # measured control rate, Hz

    @property
    def joint_errors(self) -> List[JointError]:
        """One JointError per joint currently reporting a non-zero code."""
        return [JointError(int(i), ErrorFlag(int(c))) for i, c in enumerate(self.errors) if c]

    @property
    def healthy(self) -> bool:
        return not self.errors.any()


# Shared-memory layout: one float64 per slot, three seqlock-guarded blocks plus
# four lifecycle flags with a single writer each.
_LAYOUT = (
    # child -> parent, guarded by state_seq
    ("state_seq", 1), ("t", 1), ("q", NUM_JOINTS), ("dq", NUM_JOINTS),
    ("current", NUM_JOINTS), ("pressures", NUM_FINGERS), ("temperatures", NUM_JOINTS),
    ("imu", 3), ("errors", NUM_JOINTS), ("servo_on", 1), ("active_mode", 1),
    ("position_age", 1), ("updates", 1), ("rate", 1),
    # child -> parent, written once after the handshake, guarded by info_seq
    ("info_seq", 1), ("hw_version", 1), ("fw_version", 1),
    ("serial", _SERIAL_LEN), ("serial_len", 1),
    # parent -> child, guarded by cmd_seq
    ("cmd_seq", 1), ("q_des", NUM_JOINTS), ("i_des", NUM_JOINTS), ("kp", NUM_JOINTS),
    ("kd", NUM_JOINTS), ("i_max", NUM_JOINTS), ("offset", NUM_JOINTS),
    ("mode", 1), ("want_servo", 1),
    # lifecycle
    ("running", 1), ("ready", 1), ("failed", 1), ("stopped", 1),
)


class _Shared:
    """Named numpy views onto one shared array, with seqlock read/write."""

    def __init__(self):
        offsets, total = {}, 0
        for name, n in _LAYOUT:
            offsets[name] = (total, n)
            total += n
        self.buf = np.frombuffer(mp.RawArray(ctypes.c_double, total), dtype=np.float64)
        for name, (start, n) in offsets.items():
            setattr(self, name, self.buf[start:start + n])

    def _write(self, seq, fields: dict) -> None:
        # Bump the counter to odd, write, bump to even. A reader that sees the
        # same even value before and after its copy read a consistent snapshot.
        seq[0] += 1
        for name, value in fields.items():
            target = getattr(self, name)
            target[:] = value if target.size > 1 else float(value)
        seq[0] += 1

    @staticmethod
    def _read(seq, build, attempts: int = 1000):
        for _ in range(attempts):
            before = seq[0]
            if before % 2:
                continue
            snapshot = build()
            if seq[0] == before:
                return snapshot
        raise AllegroStateError("could not read a consistent snapshot from shared memory")

    def publish_state(self, **fields) -> None:
        self._write(self.state_seq, fields)

    def read_state(self) -> dict:
        return self._read(self.state_seq, lambda: {
            "t": float(self.t[0]),
            "positions": self.q.copy(),
            "velocities": self.dq.copy(),
            "currents": self.current.copy(),
            "pressures": self.pressures.copy(),
            "temperatures": self.temperatures.copy(),
            "imu": self.imu.copy(),
            "errors": self.errors.astype(np.int64),
            "servo_on": bool(self.servo_on[0]),
            "mode": _MODE_NAMES.get(float(self.active_mode[0]), "unknown"),
            "position_age": float(self.position_age[0]),
            "updates": int(self.updates[0]),
            "rate": float(self.rate[0]),
        })

    def publish_info(self, info: HandInfo) -> None:
        serial = (info.serial_number or "")[:_SERIAL_LEN]
        codes = np.zeros(_SERIAL_LEN)
        codes[:len(serial)] = [ord(c) for c in serial]
        self._write(self.info_seq, {
            "hw_version": -1 if info.hardware_version is None else info.hardware_version,
            "fw_version": -1 if info.firmware_version is None else info.firmware_version,
            "serial": codes,
            "serial_len": len(serial),
        })

    def read_info(self) -> HandInfo:
        def build():
            hw, fw = int(self.hw_version[0]), int(self.fw_version[0])
            return HandInfo(
                serial_number="".join(chr(int(c)) for c in self.serial[:int(self.serial_len[0])]),
                hardware_version=None if hw < 0 else hw,
                firmware_version=None if fw < 0 else fw,
            )
        return self._read(self.info_seq, build)

    def write_command(self, **fields) -> None:
        self._write(self.cmd_seq, fields)

    def read_command(self) -> dict:
        return self._read(self.cmd_seq, lambda: {
            "q_des": self.q_des.copy(),
            "i_des": self.i_des.copy(),
            "kp": self.kp.copy(),
            "kd": self.kd.copy(),
            "i_max": self.i_max.copy(),
            "offset": self.offset.copy(),
            "mode": float(self.mode[0]),
            "want_servo": float(self.want_servo[0]),
        })


def _control_loop(shared: _Shared, config: DriverConfig, bus_kwargs: dict, parent_pid: int) -> None:
    """The control process. Guarantees Servo OFF on every exit path."""
    # Ctrl+C belongs to the parent, which runs an orderly shutdown. SIGTERM must
    # unwind through the finally below rather than killing us where we stand.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, lambda *_: shared.running.__setitem__(0, 0.0))

    bus = None
    try:
        try:
            os.nice(-10)  # best effort; needs privileges
        except OSError:
            pass

        bus = AllegroCANBus(**bus_kwargs)
        bus.open()
        shared.publish_info(bus.handshake(
            position_period_ms=config.position_period_ms,
            imu_period_ms=config.imu_period_ms,
            temperature_period_ms=config.temperature_period_ms,
        ))
        q = bus.wait_for_positions(timeout=1.5)

        cmd = shared.read_command()
        # Start the target at the measured pose, so switching to position mode
        # before setting one holds still instead of lurching.
        shared.write_command(q_des=q + cmd["offset"])
        cmd = shared.read_command()
        if cmd["want_servo"] < 0.5:
            bus.servo_off()  # the handshake had to engage them to get feedback

        q_prev = q + cmd["offset"]
        dq = np.zeros(NUM_JOINTS)
        current = np.zeros(NUM_JOINTS)
        updates, rate = 0, 0.0

        shared.ready[0] = 1.0
        t0 = last_update = last_zero = time.monotonic()

        while shared.running[0] > 0.5:
            if os.getppid() != parent_pid:
                logger.warning("Parent process is gone; shutting the hand down")
                break

            bus.poll(timeout=config.poll_timeout)
            now = time.monotonic()
            cmd = shared.read_command()

            want_servo = cmd["want_servo"] > 0.5
            if want_servo != bus.servo_is_on:
                bus.servo_on() if want_servo else bus.servo_off()

            mode = cmd["mode"]
            if bus.state.positions_fresh:
                # A complete set of four finger reports: one control update.
                bus.state.clear_position_flags()
                dt = now - last_update
                q = bus.state.positions + cmd["offset"]
                if dt > 0:
                    a = config.velocity_filter
                    dq = a * ((q - q_prev) / dt) + (1.0 - a) * dq
                    rate = rate + 0.02 * (1.0 / dt - rate) if rate else 1.0 / dt
                q_prev, last_update = q, now
                updates += 1

                if mode == MODE_POSITION:
                    current = cmd["kp"] * (cmd["q_des"] - q) - cmd["kd"] * dq
                elif mode == MODE_CURRENT:
                    current = cmd["i_des"].copy()
                else:
                    current = np.zeros(NUM_JOINTS)
                np.clip(current, -cmd["i_max"], cmd["i_max"], out=current)
                bus.send_currents(current)

            elif now - last_update > config.stale_timeout:
                # Feedback stopped. Never run PD on stale positions, and never
                # leave the hand sitting on the last command it was given: keep
                # zero going out, slowly, so a dead bus is not flooded.
                mode = MODE_IDLE
                current = np.zeros(NUM_JOINTS)
                if now - last_zero > 0.02:
                    bus.send_currents(current)
                    last_zero = now

            shared.publish_state(
                t=now - t0, q=q, dq=dq, current=current,
                pressures=bus.state.pressures, temperatures=bus.state.temperatures,
                imu=bus.state.imu, errors=bus.state.errors,
                servo_on=1.0 if bus.servo_is_on else 0.0, active_mode=mode,
                position_age=now - last_update, updates=updates, rate=rate,
            )

    except Exception as e:
        logger.exception("Control process failed: %s", e)
        shared.failed[0] = 1.0
    finally:
        # This block is the whole safety story. It must not raise.
        if bus is not None:
            for attempt in (1, 2):
                try:
                    bus.send_currents(np.zeros(NUM_JOINTS))
                    bus.servo_off()
                    break
                except Exception as e:
                    logger.warning("Servo-off attempt %d failed: %s", attempt, e)
            try:
                bus.close()
            except Exception as e:
                logger.warning("Error closing the bus: %s", e)
        shared.stopped[0] = 1.0
        shared.ready[0] = 0.0


class AllegroHand:
    """
    Allegro Hand V5 over CAN, with the control loop in a child process.

    Example:
        >>> with AllegroHand("can0") as hand:
        ...     print(hand.info)
        ...     hand.set_position(hand.calibration.center)
        ...     print(hand.positions)
    """

    _instances: "weakref.WeakSet[AllegroHand]" = weakref.WeakSet()
    _atexit_registered = False

    def __init__(
        self,
        channel: str = "can0",
        gains: Union[Gains, str, None] = None,
        config: Optional[DriverConfig] = None,
        calibration: Union[None, bool, str, Calibration] = True,
        interface: str = "socketcan",
        bitrate: int = 1_000_000,
        bus_factory: Optional[Callable] = None,
    ):
        """
        Args:
            channel: CAN interface name, e.g. "can0".
            gains: A `Gains`, a preset name ("default", "compliant", "soft",
                "safe", "zero"), or None for `gains.DEFAULT`.
            config: Report periods and safety settings. Defaults to `DriverConfig()`.
            calibration: True (the default) to load the file matching whatever
                serial number the hand reports; a path or `Calibration` to force
                one; None to disable limits and homing offsets entirely.
            interface: python-can interface name.
            bitrate: Bus bitrate, used by interfaces that configure it.
            bus_factory: Callable returning an open bus, for tests.
        """
        self.gains = gains_module.preset(gains) if isinstance(gains, str) else (
            gains or gains_module.DEFAULT)
        self.config = config or DriverConfig()
        self.channel = channel

        self._calibration_spec = calibration
        self.calibration = load_calibration(calibration, serial="")

        self._bus_kwargs = {"channel": channel, "interface": interface,
                            "bitrate": bitrate, "bus_factory": bus_factory}
        self._shared = _Shared()
        self._proc: Optional[mp.Process] = None
        self._info = HandInfo()

        self._shared.write_command(
            q_des=np.zeros(NUM_JOINTS), i_des=np.zeros(NUM_JOINTS),
            kp=self.gains.kp, kd=self.gains.kd, i_max=self.gains.max_current,
            offset=self._offset(), mode=MODE_IDLE, want_servo=0.0,
        )

        AllegroHand._instances.add(self)
        if not AllegroHand._atexit_registered:
            atexit.register(AllegroHand._close_all)
            AllegroHand._atexit_registered = True

    def _offset(self) -> np.ndarray:
        return np.zeros(NUM_JOINTS) if self.calibration is None else self.calibration.offset

    # ==================== Lifecycle ====================

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.is_alive()

    def start(self, timeout: float = 5.0, servo: bool = True) -> None:
        """
        Spawn the control process and wait until it is streaming state.

        Args:
            timeout: Give up if the loop is not running within this long.
            servo: Leave the motor drivers engaged. The mode is idle either way,
                so zero current is commanded until you ask for something.
        """
        if self.running:
            logger.warning("Control process is already running")
            return

        self._shared.running[0] = 1.0
        for flag in ("ready", "failed", "stopped"):
            getattr(self._shared, flag)[0] = 0.0
        self._shared.write_command(want_servo=1.0 if servo else 0.0)

        # fork, so the child inherits the shared array and any bus_factory closure
        self._proc = mp.get_context("fork").Process(
            target=_control_loop,
            args=(self._shared, self.config, self._bus_kwargs, os.getpid()),
            daemon=True, name="allegro-control",
        )
        self._proc.start()

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._shared.failed[0] > 0.5 or not self._proc.is_alive():
                self.close()
                raise AllegroStateError(
                    f"Control process failed to start on {self.channel!r}; the reason "
                    f"was logged by that process.\n{describe_link(self.channel)}"
                )
            if self._shared.ready[0] > 0.5:
                self._info = self._shared.read_info()
                self._load_calibration_for_hand()
                logger.info("Control process up. %s", self._info)
                return
            time.sleep(0.005)

        self.close()
        raise AllegroStateError(f"Control process was not ready within {timeout}s")

    def _load_calibration_for_hand(self) -> None:
        """Pick up the calibration for the hand that actually answered."""
        if self._calibration_spec is not True:
            return
        previous = self._offset()
        self.calibration = Calibration.for_serial(self._info.serial_number)
        # The control process started its target at the measured pose under the
        # old offset; shift it so the pose it holds does not move underneath it.
        self._shared.write_command(
            offset=self.calibration.offset,
            q_des=self._shared.q_des + (self.calibration.offset - previous),
        )
        logger.info("Calibration: %s", self.calibration.path or "nominal limits")

    def close(self, timeout: float = 2.0) -> None:
        """
        Go idle, servo off, stop the control process, release the bus.

        Safe to call repeatedly. If the child died without confirming Servo OFF,
        this sends it from here instead.
        """
        if self._proc is None:
            return
        try:
            self._shared.write_command(mode=MODE_IDLE, want_servo=0.0)
            time.sleep(0.05)  # let the child transmit one zero-current cycle
        except Exception:
            pass

        self._shared.running[0] = 0.0
        self._proc.join(timeout=timeout)
        if self._proc.is_alive():
            logger.warning("Control process did not exit; sending SIGTERM")
            self._proc.terminate()  # its SIGTERM handler unwinds cleanly
            self._proc.join(timeout=1.0)
        if self._proc.is_alive():
            logger.error("Control process ignored SIGTERM; killing it")
            self._proc.kill()
            self._proc.join(timeout=1.0)

        confirmed = self._shared.stopped[0] > 0.5
        self._proc = None
        if not confirmed:
            logger.error("Control process died without confirming Servo OFF; forcing it")
            self._force_servo_off()
        logger.info("Control process stopped")

    def _force_servo_off(self) -> None:
        """Last resort: open the bus from here and command Servo OFF."""
        try:
            with AllegroCANBus(**self._bus_kwargs) as bus:
                for _ in range(3):
                    bus.send_currents(np.zeros(NUM_JOINTS))
                    bus.servo_off()
                    time.sleep(0.005)
            logger.info("Emergency Servo OFF sent")
        except Exception as e:
            logger.critical("EMERGENCY SERVO OFF FAILED: %s. Power the hand down.", e)

    @classmethod
    def _close_all(cls) -> None:
        """atexit hook: close every driver still running."""
        for hand in list(cls._instances):
            try:
                if hand.running:
                    hand.close()
            except Exception as e:
                logger.error("Error closing a driver at exit: %s", e)

    def _require_running(self) -> None:
        if not self.running:
            raise AllegroStateError("Control process is not running; call start() first")

    # ==================== Commands ====================

    def set_position(self, q_des: Sequence[float], clip: bool = True) -> np.ndarray:
        """
        Set the PD target and switch to position mode.

        Args:
            q_des: 16 joint angles in radians.
            clip: Clip into the calibrated range, if a calibration is loaded.

        Returns:
            The target actually written, after clipping.
        """
        self._require_running()
        q = np.asarray(q_des, dtype=np.float64)
        if q.shape != (NUM_JOINTS,):
            raise ValueError(f"expected {NUM_JOINTS} positions, got {q.shape}")
        if clip and self.calibration is not None:
            q = self.calibration.clip(q)
        self._shared.write_command(q_des=q, mode=MODE_POSITION)
        return q

    def set_current(self, current_ma: Sequence[float]) -> None:
        """
        Command motor currents directly and switch to current mode.

        Values still pass the gains' `max_current` clamp and the hardware limit
        of +/-240 mA. Raise `max_current` first if you need more.

        Args:
            current_ma: 16 motor currents in mA.
        """
        self._require_running()
        i = np.asarray(current_ma, dtype=np.float64)
        if i.shape != (NUM_JOINTS,):
            raise ValueError(f"expected {NUM_JOINTS} currents, got {i.shape}")
        self._shared.write_command(i_des=i, mode=MODE_CURRENT)

    def hold(self) -> np.ndarray:
        """Hold the current measured pose."""
        return self.set_position(self.positions)

    def relax(self) -> None:
        """Command zero current on every joint. The servos stay engaged."""
        if self._proc is not None:
            self._shared.write_command(mode=MODE_IDLE)

    def servo_on(self) -> None:
        """Engage the motor drivers. The control mode is unchanged."""
        self._require_running()
        self._shared.write_command(want_servo=1.0)

    def servo_off(self) -> None:
        """Disable the motor drivers and go idle. The hand goes limp."""
        if self._proc is not None:
            self._shared.write_command(mode=MODE_IDLE, want_servo=0.0)

    def set_gains(self, kp=None, kd=None, max_current=None) -> Gains:
        """
        Retune the PD live. Scalars broadcast to all 16 joints.

        Returns:
            The gains now in force.
        """
        self.gains = self.gains.replace(kp=kp, kd=kd, max_current=max_current, name="custom")
        self._shared.write_command(kp=self.gains.kp, kd=self.gains.kd,
                                   i_max=self.gains.max_current)
        return self.gains

    def set_preset(self, name: Union[str, Gains]) -> Gains:
        """Install a gain preset by name, or a `Gains` instance."""
        self.gains = gains_module.preset(name) if isinstance(name, str) else name
        self._shared.write_command(kp=self.gains.kp, kd=self.gains.kd,
                                   i_max=self.gains.max_current)
        return self.gains

    # ==================== State ====================

    def read(self) -> HandState:
        """One consistent snapshot of everything the control process publishes."""
        return HandState(**self._shared.read_state())

    @property
    def info(self) -> HandInfo:
        """Serial number, handedness, hardware type, hardware/firmware version."""
        if not self._info.complete and self.running:
            self._info = self._shared.read_info()
        return self._info

    @property
    def serial_number(self) -> str:
        return self.info.serial_number

    @property
    def positions(self) -> np.ndarray:
        """Measured joint angles, (16,) rad, with the homing offset applied."""
        return self._shared.read_state()["positions"]

    @property
    def raw_positions(self) -> np.ndarray:
        """Measured joint angles before the homing offset, (16,) rad."""
        return self.positions - self._offset()

    @property
    def velocities(self) -> np.ndarray:
        """Filtered joint velocities, (16,) rad/s."""
        return self._shared.read_state()["velocities"]

    @property
    def currents(self) -> np.ndarray:
        """Motor currents as sent, (16,) mA."""
        return self._shared.read_state()["currents"]

    @property
    def pressures(self) -> np.ndarray:
        """Fingertip pressures, (4,) Pa: index, middle, ring, thumb."""
        return self._shared.read_state()["pressures"]

    @property
    def temperatures(self) -> np.ndarray:
        """Motor temperatures, (16,) deg C. Needs `temperature_period_ms` set."""
        return self._shared.read_state()["temperatures"]

    @property
    def errors(self) -> List[JointError]:
        """Joints currently reporting an error. Empty when the hand is healthy."""
        return self.read().joint_errors

    @property
    def target(self) -> np.ndarray:
        """The position target currently in shared memory."""
        return self._shared.q_des.copy()

    @property
    def mode(self) -> str:
        """"idle", "position" or "current" — what the control loop is running."""
        return self._shared.read_state()["mode"]

    @property
    def servo_is_on(self) -> bool:
        """Whether the control process believes the motor drivers are engaged."""
        return bool(self._shared.servo_on[0])

    # ==================== Context manager ====================

    def __enter__(self) -> "AllegroHand":
        if not self.running:
            self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> bool:
        self.close()
        return False

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    def __repr__(self) -> str:
        return (f"AllegroHand(channel={self.channel!r}, gains={self.gains.name!r}, "
                f"{'running' if self.running else 'stopped'})")
