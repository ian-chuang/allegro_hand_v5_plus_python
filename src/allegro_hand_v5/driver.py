"""
`AllegroHand` — the driver.

The real-time cycle (CAN RX → velocity estimate → PD → CAN TX) runs in a child
process with its own GIL, so nothing the parent does can stall it. The parent
writes commands into shared memory and reads state back.

Shutdown is the part worth reading twice. The hand is torque-only: if the host
stops talking while the servos are engaged, the last current command stays
latched and the motors keep pushing. So Servo OFF is enforced at four levels:

1. the child's `finally` — covers a normal exit and any exception in the loop;
2. a SIGTERM handler in the child, so `terminate()` still unwinds through it;
3. `atexit` plus SIGINT/SIGTERM handlers in the parent, so Ctrl+C and a plain
   `sys.exit()` both close the driver even without a `with` block;
4. if the child died without confirming (SIGKILL, segfault), the parent opens
   the bus itself and sends Servo OFF directly.

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
from dataclasses import dataclass, replace
from typing import Callable, List, Optional, Sequence, Union

import numpy as np

from allegro_hand_v5 import gains as gain_profiles
from allegro_hand_v5.bus import AllegroCANBus, describe_link, link_status
from allegro_hand_v5.calibration import HandCalibration, load_calibration
from allegro_hand_v5.exceptions import (
    AllegroConnectionError,
    AllegroStateError,
    AllegroTimeoutError,
)
from allegro_hand_v5.gains import GainProfile
from allegro_hand_v5 import protocol as proto
from allegro_hand_v5.protocol import (
    DEFAULT_MAX_CURRENT_MA,
    NUM_FINGERS,
    NUM_JOINTS,
    TORQUE_TO_CURRENT,
    ErrorFlag,
    HandInfo,
    JointError,
)

logger = logging.getLogger(__name__)

MODE_IDLE = 0.0
MODE_POSITION = 1.0
MODE_TORQUE = 2.0

_MODE_NAMES = {MODE_IDLE: "idle", MODE_POSITION: "position", MODE_TORQUE: "torque"}

_SERIAL_LEN = 8


@dataclass
class DriverConfig:
    """Everything the control process needs that is not a gain."""

    rate: float = 500.0
    """Control frequency, Hz. The hand's own loop runs at 500 Hz."""

    velocity_filter_alpha: float = 0.3
    """Low-pass coefficient on the finite-difference velocity. 1.0 = unfiltered."""

    torque_to_current: float = TORQUE_TO_CURRENT
    """mA of motor current per Nm of joint torque."""

    max_current_ma: float = DEFAULT_MAX_CURRENT_MA
    """Hard per-joint current clamp, applied after the gain profile's torque clamp."""

    position_period_ms: int = 3
    """How often the hand streams positions. 0 disables the stream."""

    poll_rtr: bool = False
    """Request positions with an RTR every cycle instead of using the stream."""

    stale_timeout: float = 0.1
    """Zero the torque if no position frame arrives for this long. 0 disables."""

    command_timeout: Optional[float] = None
    """Zero the torque if the parent has not sent a command for this long."""

    stop_on_error: bool = False
    """Servo off as soon as the hand reports any error code."""


@dataclass
class HandState:
    """One consistent snapshot of everything the control process publishes."""

    t: float
    positions: np.ndarray  # (16,) rad
    velocities: np.ndarray  # (16,) rad/s
    torques: np.ndarray  # (16,) Nm, commanded
    currents: np.ndarray  # (16,) mA, as sent
    pressures: np.ndarray  # (4,) Pa
    errors: np.ndarray  # (16,) ErrorFlag bitmasks
    servo_on: bool
    mode: str
    iterations: int
    missed_deadlines: int
    period_avg: float
    period_max: float
    busy_avg: float
    rx_frames: int
    error_count: int
    position_age: float  # seconds since the last position frame

    @property
    def joint_errors(self) -> List[JointError]:
        """One JointError per joint currently reporting a non-zero code."""
        return [
            JointError(int(i), ErrorFlag(int(c))) for i, c in enumerate(self.errors) if c
        ]

    @property
    def healthy(self) -> bool:
        return not self.errors.any()


_LAYOUT = (
    # child -> parent, guarded by state_seq
    ("state_seq", 1), ("t", 1),
    ("q", NUM_JOINTS), ("dq", NUM_JOINTS), ("tau", NUM_JOINTS), ("current", NUM_JOINTS),
    ("pressures", NUM_FINGERS), ("errors", NUM_JOINTS),
    ("servo_on", 1), ("active_mode", 1), ("position_age", 1),
    ("iterations", 1), ("missed", 1), ("period_avg", 1), ("period_max", 1),
    ("busy_avg", 1), ("rx_frames", 1), ("error_count", 1),
    # child -> parent, written once at startup, guarded by info_seq
    ("info_seq", 1), ("hw_version", 1), ("fw_version", 1),
    ("serial", _SERIAL_LEN), ("serial_len", 1),
    # parent -> child, guarded by cmd_seq
    ("cmd_seq", 1), ("q_des", NUM_JOINTS), ("kp", NUM_JOINTS), ("kd", NUM_JOINTS),
    ("tau_max", NUM_JOINTS), ("tau_direct", NUM_JOINTS),
    ("mode", 1), ("want_servo", 1), ("cmd_stamp", 1),
    # lifecycle, single writer each
    ("running", 1), ("child_ready", 1), ("child_error", 1), ("servo_off_done", 1),
)


class _Shared:
    """Named numpy views onto one shared RawArray, with seqlock helpers."""

    def __init__(self):
        offsets, off = {}, 0
        for name, n in _LAYOUT:
            offsets[name] = (off, n)
            off += n
        self.size = off
        self.raw = mp.RawArray(ctypes.c_double, self.size)
        self.buf = np.frombuffer(self.raw, dtype=np.float64, count=self.size)
        for name, (start, n) in offsets.items():
            setattr(self, name, self.buf[start : start + n])

    # The writer bumps a counter to odd, writes, then bumps to even. A reader
    # that sees the same even value before and after its copy read a consistent
    # snapshot. x86 does not reorder stores, so no explicit fence is needed.
    @staticmethod
    def _read_guarded(seq, build, attempts: int = 2000):
        for _ in range(attempts):
            s0 = seq[0]
            if s0 % 2:
                continue
            snapshot = build()
            if seq[0] == s0:
                return snapshot
        return None

    def publish_state(self, **fields) -> None:
        self.state_seq[0] += 1
        for name, value in fields.items():
            target = getattr(self, name)
            target[:] = value if target.size > 1 else float(value)
        self.state_seq[0] += 1

    def read_state(self) -> dict:
        snap = self._read_guarded(
            self.state_seq,
            lambda: {
                "t": float(self.t[0]),
                "positions": self.q.copy(),
                "velocities": self.dq.copy(),
                "torques": self.tau.copy(),
                "currents": self.current.copy(),
                "pressures": self.pressures.copy(),
                "errors": self.errors.astype(np.int64),
                "servo_on": bool(self.servo_on[0]),
                "mode": _MODE_NAMES.get(float(self.active_mode[0]), "unknown"),
                "position_age": float(self.position_age[0]),
                "iterations": int(self.iterations[0]),
                "missed_deadlines": int(self.missed[0]),
                "period_avg": float(self.period_avg[0]),
                "period_max": float(self.period_max[0]),
                "busy_avg": float(self.busy_avg[0]),
                "rx_frames": int(self.rx_frames[0]),
                "error_count": int(self.error_count[0]),
            },
        )
        if snap is None:
            raise AllegroStateError("could not read a consistent state snapshot")
        return snap

    def publish_info(self, info: HandInfo) -> None:
        self.info_seq[0] += 1
        self.hw_version[0] = -1 if info.hardware_version is None else info.hardware_version
        self.fw_version[0] = -1 if info.firmware_version is None else info.firmware_version
        text = (info.serial_number or "")[:_SERIAL_LEN]
        self.serial[:] = 0.0
        for i, ch in enumerate(text):
            self.serial[i] = ord(ch)
        self.serial_len[0] = len(text)
        self.info_seq[0] += 1

    def read_info(self) -> HandInfo:
        def build():
            n = int(self.serial_len[0])
            hw, fw = int(self.hw_version[0]), int(self.fw_version[0])
            return HandInfo(
                hardware_version=None if hw < 0 else hw,
                firmware_version=None if fw < 0 else fw,
                serial_number="".join(chr(int(c)) for c in self.serial[:n]),
            )

        return self._read_guarded(self.info_seq, build) or HandInfo()

    def write_command(self, **fields) -> None:
        self.cmd_seq[0] += 1
        for name, value in fields.items():
            target = getattr(self, name)
            target[:] = value if target.size > 1 else float(value)
        self.cmd_stamp[0] = time.monotonic()
        self.cmd_seq[0] += 1

    def read_command(self) -> Optional[dict]:
        return self._read_guarded(
            self.cmd_seq,
            lambda: {
                "q_des": self.q_des.copy(),
                "kp": self.kp.copy(),
                "kd": self.kd.copy(),
                "tau_max": self.tau_max.copy(),
                "tau_direct": self.tau_direct.copy(),
                "mode": float(self.mode[0]),
                "want_servo": float(self.want_servo[0]),
                "cmd_stamp": float(self.cmd_stamp[0]),
            },
        )


def _child_main(shared: _Shared, config: DriverConfig, bus_kwargs: dict, parent_pid: int) -> None:
    """The control process. Guarantees Servo OFF on every exit path."""
    # Ctrl+C belongs to the parent: it runs an orderly shutdown. SIGTERM must
    # unwind through the finally rather than killing us where we stand.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, lambda *_: shared.running.__setitem__(0, 0.0))

    bus = None
    try:
        try:
            os.nice(-10)  # best effort, needs privileges
        except OSError:
            pass

        bus = AllegroCANBus(**bus_kwargs)
        bus.open()
        shared.publish_info(bus.handshake(position_period_ms=config.position_period_ms))

        # Getting the first position matters, but not enough to refuse to start:
        # try the stream, then RTR polling, then come up anyway. The stale_timeout
        # interlock holds torque at zero until real feedback arrives, and the loop
        # picks it up by itself the moment the hand starts talking.
        q = None
        if not config.poll_rtr:
            try:
                q = bus.read_positions(timeout=1.5, poll_rtr=False)
            except AllegroTimeoutError as e:
                logger.warning("No position stream. %s", e)
                logger.warning("Falling back to RTR polling for positions")
        if q is None:
            try:
                q = bus.read_positions(timeout=1.5, poll_rtr=True)
                config.poll_rtr = True
            except AllegroTimeoutError:
                if not bus.info.complete:
                    # No identity and no positions: the hand is not on this bus.
                    # Refusing to start is the honest answer.
                    raise AllegroConnectionError(
                        "No response from the hand — it answered neither the identity "
                        "request nor any position report.\n" + bus.diagnose()
                    ) from None
                # It identified itself, so it is there; the stream may just be
                # slow to start. Come up in idle and let the loop pick it up.
                logger.error(
                    "The hand identified itself but is not reporting positions. Starting "
                    "in idle; torque stays at zero until feedback arrives.\n%s",
                    bus.diagnose(),
                )
                q = bus.state.positions.copy()

        # Start the target at the measured pose: if the parent switches to
        # position mode before setting one, the hand holds still instead of
        # lurching towards whatever was left in shared memory.
        shared.write_command(q_des=q)

        q_prev = q.copy()
        dq = np.zeros(NUM_JOINTS)
        tau = np.zeros(NUM_JOINTS)
        current = np.zeros(NUM_JOINTS)

        period = 1.0 / config.rate
        alpha = config.velocity_filter_alpha
        # Type B ("Plus") hands gear the MCP-2 joints roughly 2x, so their
        # current command is halved. The serial number tells us which we have.
        is_plus = bus.info.hardware_type != "A"
        if is_plus:
            logger.info("Type B hand: halving current on joints %s",
                        list(proto.PLUS_HALVED_JOINTS))
        cmd = shared.read_command()

        iterations = missed = rx_total = 0
        cycle_total = cycle_max = busy_total = 0.0

        # handshake() had to engage the servos to get the position stream; if
        # the caller asked to start with them off, honour that now.
        if shared.read_command()["want_servo"] < 0.5:
            bus.servo_off()

        shared.child_ready[0] = 1.0
        t0 = last = prev_start = time.perf_counter()
        next_deadline = t0 + period

        while shared.running[0] > 0.5:
            if os.getppid() != parent_pid:
                logger.warning("Parent process is gone; shutting the hand down")
                break

            loop_start = time.perf_counter()

            if config.poll_rtr:
                bus.request_positions()
            rx_total += bus.poll()
            now = time.monotonic()  # after poll(), so position_age is never negative

            q = bus.state.positions.copy()
            dt = loop_start - last
            if dt > 0:
                dq = alpha * ((q - q_prev) / dt) + (1.0 - alpha) * dq
            q_prev, last = q, loop_start

            fresh = shared.read_command()
            if fresh is not None:
                cmd = fresh

            # --- servo state follows the parent's request ---
            want_servo = cmd["want_servo"] > 0.5
            if want_servo != bus.servo_is_on:
                bus.servo_on() if want_servo else bus.servo_off()

            # --- safety interlocks, each of which forces zero torque ---
            mode = cmd["mode"]
            position_age = now - bus.state.last_position_time if bus.state.last_position_time else 0.0

            if config.stale_timeout and position_age > config.stale_timeout:
                mode = MODE_IDLE  # feedback is stale; do not run PD on old data
            if config.command_timeout and now - cmd["cmd_stamp"] > config.command_timeout:
                mode = MODE_IDLE
            if config.stop_on_error and bus.state.errors.any():
                mode = MODE_IDLE
                if bus.servo_is_on:
                    logger.error("Hand reported an error; servo off. %s",
                                 "; ".join(str(e) for e in bus.state.active_errors()))
                    bus.servo_off()

            # --- control law ---
            if mode == MODE_POSITION:
                tau = cmd["kp"] * (cmd["q_des"] - q) - cmd["kd"] * dq
            elif mode == MODE_TORQUE:
                tau = cmd["tau_direct"].copy()
            else:
                tau = np.zeros(NUM_JOINTS)

            np.clip(tau, -cmd["tau_max"], cmd["tau_max"], out=tau)
            current = proto.scale_plus_currents(tau * config.torque_to_current, is_plus)
            np.clip(current, -config.max_current_ma, config.max_current_ma, out=current)
            bus.send_currents(current)

            busy = time.perf_counter() - loop_start
            cycle = loop_start - prev_start
            prev_start = loop_start
            iterations += 1
            busy_total += busy
            if iterations > 1:
                cycle_total += cycle
                cycle_max = max(cycle_max, cycle)
                if cycle > period * 1.5:
                    missed += 1

            shared.publish_state(
                t=loop_start - t0, q=q, dq=dq, tau=tau, current=current,
                pressures=bus.state.pressures, errors=bus.state.errors,
                servo_on=1.0 if bus.servo_is_on else 0.0, active_mode=mode,
                position_age=position_age,
                iterations=iterations, missed=missed,
                period_avg=cycle_total / max(iterations - 1, 1), period_max=cycle_max,
                busy_avg=busy_total / iterations, rx_frames=rx_total,
                error_count=bus.state.error_count,
            )

            sleep = next_deadline - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
                next_deadline += period
            else:
                next_deadline = time.perf_counter() + period  # resync after an overrun

    except Exception as e:
        logger.exception("Control process failed: %s", e)
        shared.child_error[0] = 1.0
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
        shared.servo_off_done[0] = 1.0
        shared.child_ready[0] = 0.0


class AllegroHand:
    """
    Allegro Hand V5 over CAN, with the control loop in a child process.

    Example:
        >>> with AllegroHand("can0") as hand:
        ...     print(hand.info)
        ...     hand.set_position(np.zeros(16))
        ...     print(hand.positions)
    """

    _instances: "weakref.WeakSet[AllegroHand]" = weakref.WeakSet()
    _atexit_registered = False

    def __init__(
        self,
        channel: str = "can0",
        gains: Union[GainProfile, str, None] = None,
        config: Optional[DriverConfig] = None,
        calibration: Union[None, bool, str, HandCalibration] = True,
        handedness: str = "right",
        hardware_type: Optional[str] = None,
        interface: str = "socketcan",
        bitrate: int = 1_000_000,
        bus_factory: Optional[Callable] = None,
        autostart: bool = False,
    ):
        """
        Args:
            channel: CAN interface name, e.g. "can0".
            gains: A GainProfile, a profile name ("compliant", "soft", ...), or
                None for `gains.DEFAULT`.
            config: Rate and safety settings. Defaults to `DriverConfig()`.
            calibration: True (the default) to pick the calibration for whatever
                hand answers on the bus — the serial number gives handedness and
                hardware type, and `start()` reloads accordingly. Also accepts a
                path or a HandCalibration; None disables target clipping.
            handedness: "left" or "right". Only used to choose a calibration
                before the hand has identified itself.
            hardware_type: "A" or "B", same. Leave as None to let the hand say.
            interface: python-can interface name.
            bitrate: Bus bitrate, used by interfaces that configure it.
            bus_factory: Callable returning an open bus, for tests.
            autostart: Start the control process immediately.
        """
        if isinstance(gains, str):
            gains = gain_profiles.get_profile(gains)
        self.gains: GainProfile = gains or gain_profiles.DEFAULT
        self.config = config or DriverConfig()

        self.handedness = str(handedness).lower()
        self.hardware_type = hardware_type
        # True means "resolve from the hand's own serial number once we have it".
        self._calibration_is_auto = calibration is True
        self.calibration = load_calibration(
            calibration, handedness=self.handedness, hardware_type=hardware_type
        )

        self._bus_kwargs = {
            "channel": channel,
            "interface": interface,
            "bitrate": bitrate,
            "bus_factory": bus_factory,
        }
        self.channel = channel

        self._shared = _Shared()
        self._proc: Optional[mp.Process] = None
        self._info = HandInfo()
        self._prev_handlers: dict = {}

        self._shared.write_command(
            q_des=np.zeros(NUM_JOINTS),
            kp=self.gains.kp, kd=self.gains.kd, tau_max=self.gains.max_torque,
            tau_direct=np.zeros(NUM_JOINTS),
            mode=MODE_IDLE, want_servo=0.0,
        )

        AllegroHand._instances.add(self)
        if not AllegroHand._atexit_registered:
            atexit.register(AllegroHand._shutdown_all)
            AllegroHand._atexit_registered = True

        if autostart:
            self.start()

    # ==================== Lifecycle ====================

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.is_alive()

    def start(self, timeout: float = 5.0, servo: bool = True) -> None:
        """
        Spawn the control process and wait until it is streaming state.

        Args:
            timeout: Give up if the child is not ready in this long.
            servo: Engage the motors once the child is up. The hand stays in
                idle mode (zero torque) either way until you command something.
        """
        if self.running:
            logger.warning("Control process is already running")
            return

        self._shared.running[0] = 1.0
        self._shared.child_ready[0] = 0.0
        self._shared.child_error[0] = 0.0
        self._shared.servo_off_done[0] = 0.0

        # fork so the child inherits the shared array and any bus_factory closure
        ctx = mp.get_context("fork")
        self._proc = ctx.Process(
            target=_child_main,
            args=(self._shared, self.config, self._bus_kwargs, os.getpid()),
            daemon=True,
            name="allegro-control",
        )
        self._proc.start()
        self._install_signal_handlers()

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._shared.child_error[0] > 0.5 or not self._proc.is_alive():
                self._proc.join(timeout=1.0)
                self._proc = None
                raise AllegroStateError(
                    f"Control process failed to start on {self.channel!r}.\n"
                    f"{describe_link(self.channel)}"
                )
            if self._shared.child_ready[0] > 0.5:
                self._info = self._shared.read_info()
                logger.info("Control process up at %.0f Hz. %s", self.config.rate, self._info)
                self._resolve_calibration()
                if servo:
                    self.servo_on()
                    self._await_servo(True)
                return
            time.sleep(0.005)

        self.close()
        raise AllegroStateError(f"Control process was not ready within {timeout}s")

    def _resolve_calibration(self) -> None:
        """
        Reload the calibration for the hand that actually answered.

        The serial number carries handedness and hardware type, so a
        `calibration=True` driver picks `right_B.json` over `right.json`
        without being told which hand is plugged in.
        """
        if not self._calibration_is_auto:
            return

        handedness = self._info.handedness
        hardware_type = self._info.hardware_type
        if handedness == "unknown":
            handedness = self.handedness
        if hardware_type == "unknown":
            hardware_type = self.hardware_type

        self.handedness = handedness
        self.hardware_type = hardware_type
        self.calibration = load_calibration(
            True, handedness=handedness, hardware_type=hardware_type
        )
        if self.calibration is not None:
            logger.info(
                "Calibration for the %s/%s hand: %s",
                handedness, hardware_type or "?",
                self.calibration.source or "URDF defaults",
            )

    def close(self, timeout: float = 2.0) -> None:
        """
        Go idle, servo off, stop the control process, release the bus.

        Safe to call repeatedly. If the child died without confirming Servo OFF,
        this sends it directly.
        """
        self._restore_signal_handlers()

        if self._proc is None:
            return

        try:
            self._shared.write_command(mode=MODE_IDLE, want_servo=0.0)
            time.sleep(0.05)  # let the child transmit one zero-torque cycle
        except Exception:
            pass

        self._shared.running[0] = 0.0
        self._proc.join(timeout=timeout)

        if self._proc.is_alive():
            logger.warning("Control process did not exit; sending SIGTERM")
            self._proc.terminate()  # the child's SIGTERM handler unwinds cleanly
            self._proc.join(timeout=1.0)
        if self._proc.is_alive():
            logger.error("Control process ignored SIGTERM; killing it")
            self._proc.kill()
            self._proc.join(timeout=1.0)

        confirmed = self._shared.servo_off_done[0] > 0.5
        self._proc = None

        if not confirmed:
            logger.error("Control process died without confirming Servo OFF; forcing it")
            self._emergency_servo_off()

        logger.info("Control process stopped")

    def _emergency_servo_off(self) -> None:
        """Last resort: open the bus from the parent and command Servo OFF."""
        try:
            bus = AllegroCANBus(**self._bus_kwargs)
            bus.open()
            try:
                for _ in range(3):
                    bus.send_currents(np.zeros(NUM_JOINTS))
                    bus.servo_off()
                    time.sleep(0.005)
            finally:
                bus.close()
            logger.info("Emergency Servo OFF sent")
        except Exception as e:
            logger.critical(
                "EMERGENCY SERVO OFF FAILED: %s. Power the hand down at the supply.", e
            )

    # --- process-wide safety nets ---

    def _install_signal_handlers(self) -> None:
        """Close cleanly on Ctrl+C or SIGTERM, then hand back to whatever ran before."""
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                previous = signal.getsignal(sig)
            except (ValueError, OSError):
                continue

            def handler(signum, frame, _prev=previous):
                logger.info("Signal %s received; shutting the hand down", signum)
                try:
                    self.close()
                finally:
                    if callable(_prev) and _prev not in (signal.SIG_IGN, signal.SIG_DFL):
                        _prev(signum, frame)
                    elif _prev == signal.SIG_DFL:
                        signal.signal(signum, signal.SIG_DFL)
                        os.kill(os.getpid(), signum)

            try:
                signal.signal(sig, handler)
                self._prev_handlers[sig] = previous
            except ValueError:
                pass  # not the main thread; atexit still covers us

    def _restore_signal_handlers(self) -> None:
        for sig, previous in list(self._prev_handlers.items()):
            try:
                signal.signal(sig, previous)
            except (ValueError, OSError, TypeError):
                pass
        self._prev_handlers.clear()

    @classmethod
    def _shutdown_all(cls) -> None:
        """atexit hook: close every driver that is still running."""
        for hand in list(cls._instances):
            try:
                if hand.running:
                    hand.close()
            except Exception as e:
                logger.error("Error closing a driver at exit: %s", e)

    def _check_running(self) -> None:
        if not self.running:
            raise AllegroStateError("Control process is not running; call start() first")

    # ==================== Servo ====================

    def _await_servo(self, state: bool, timeout: float = 0.5) -> bool:
        """Block until the control process reports the servos in `state`."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if bool(self._shared.servo_on[0]) == state:
                return True
            time.sleep(0.002)
        logger.warning("Servos did not go %s within %.2fs", "on" if state else "off", timeout)
        return False

    def servo_on(self, wait: bool = True) -> None:
        """Engage the motor drivers. The control mode is unchanged."""
        self._check_running()
        self._shared.write_command(want_servo=1.0)
        if wait:
            self._await_servo(True)

    def servo_off(self, wait: bool = True) -> None:
        """Disable the motor drivers and go idle. The hand goes limp."""
        if self._proc is None:
            return
        self._shared.write_command(mode=MODE_IDLE, want_servo=0.0)
        if wait and self.running:
            self._await_servo(False)

    @property
    def servo_is_on(self) -> bool:
        """Whether the child believes the motors are engaged."""
        return bool(self._shared.servo_on[0])

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
        self._check_running()
        q = np.asarray(q_des, dtype=np.float64)
        if q.shape != (NUM_JOINTS,):
            raise ValueError(f"expected {NUM_JOINTS} positions, got {q.shape}")
        if clip and self.calibration is not None:
            q = self.calibration.clip(q)
        self._shared.write_command(q_des=q, mode=MODE_POSITION)
        return q

    def set_position_deg(self, q_des_deg: Sequence[float], clip: bool = True) -> np.ndarray:
        """`set_position` in degrees. Returns the target in radians."""
        return self.set_position(np.radians(np.asarray(q_des_deg, dtype=np.float64)), clip=clip)

    def set_torque(self, torque_nm: Sequence[float]) -> None:
        """
        Command joint torques directly and switch to torque mode.

        Values still pass through the gain profile's `max_torque` clamp and the
        config's `max_current_ma` clamp — raise those first if you need more.

        Args:
            torque_nm: 16 joint torques in Nm.
        """
        self._check_running()
        tau = np.asarray(torque_nm, dtype=np.float64)
        if tau.shape != (NUM_JOINTS,):
            raise ValueError(f"expected {NUM_JOINTS} torques, got {tau.shape}")
        self._shared.write_command(tau_direct=tau, mode=MODE_TORQUE)

    def set_current(self, current_ma: Sequence[float]) -> None:
        """`set_torque` in raw motor current (mA), the unit the wire uses."""
        self.set_torque(np.asarray(current_ma, dtype=np.float64) / self.config.torque_to_current)

    def hold(self) -> np.ndarray:
        """Freeze at the current measured pose."""
        return self.set_position(self.positions)

    def relax(self) -> None:
        """Zero torque on every joint, servos left engaged."""
        if self._proc is not None:
            self._shared.write_command(mode=MODE_IDLE)

    # ==================== Gains ====================

    def set_gains(
        self,
        kp: Union[None, float, Sequence[float]] = None,
        kd: Union[None, float, Sequence[float]] = None,
        max_torque: Union[None, float, Sequence[float]] = None,
    ) -> GainProfile:
        """
        Retune the PD live. Scalars broadcast to all 16 joints.

        Returns:
            The resulting GainProfile.
        """
        def to16(value):
            return np.broadcast_to(np.asarray(value, dtype=np.float64), (NUM_JOINTS,)).copy()

        self.gains = self.gains.replace(
            kp=None if kp is None else to16(kp),
            kd=None if kd is None else to16(kd),
            max_torque=None if max_torque is None else to16(max_torque),
            name="custom",
        )
        self._shared.write_command(
            kp=self.gains.kp, kd=self.gains.kd, tau_max=self.gains.max_torque
        )
        return self.gains

    def set_gain_profile(self, profile: Union[GainProfile, str]) -> GainProfile:
        """Install a named profile or a GainProfile instance."""
        if isinstance(profile, str):
            profile = gain_profiles.get_profile(profile)
        self.gains = profile
        self._shared.write_command(
            kp=profile.kp, kd=profile.kd, tau_max=profile.max_torque
        )
        logger.info("Gain profile set to %r", profile.name)
        return profile

    def get_gains(self) -> GainProfile:
        """The gains the control process is actually using, read back from shared memory."""
        return GainProfile(
            name=self.gains.name,
            kp=self._shared.kp.copy(),
            kd=self._shared.kd.copy(),
            max_torque=self._shared.tau_max.copy(),
        )

    # ==================== State ====================

    def read(self) -> HandState:
        """One consistent snapshot of everything the control process publishes."""
        return HandState(**self._shared.read_state())

    @property
    def info(self) -> HandInfo:
        """Hardware/firmware version, serial number, handedness, hand type."""
        if not self._info.complete and self.running:
            self._info = self._shared.read_info()
        return self._info

    @property
    def serial_number(self) -> str:
        return self.info.serial_number

    @property
    def positions(self) -> np.ndarray:
        """Measured joint angles, (16,) rad."""
        return self._shared.read_state()["positions"]

    @property
    def positions_deg(self) -> np.ndarray:
        return np.degrees(self.positions)

    @property
    def velocities(self) -> np.ndarray:
        """Filtered joint velocities, (16,) rad/s."""
        return self._shared.read_state()["velocities"]

    @property
    def torques(self) -> np.ndarray:
        """Commanded joint torques, (16,) Nm."""
        return self._shared.read_state()["torques"]

    @property
    def currents(self) -> np.ndarray:
        """Motor currents as sent, (16,) mA."""
        return self._shared.read_state()["currents"]

    @property
    def pressures(self) -> np.ndarray:
        """Fingertip pressures, (4,) Pa: index, middle, ring, thumb."""
        return self._shared.read_state()["pressures"]

    @property
    def errors(self) -> List[JointError]:
        """Joints currently reporting an error. Empty when the hand is healthy."""
        return self.read().joint_errors

    @property
    def error_codes(self) -> np.ndarray:
        """Raw (16,) array of ErrorFlag bitmasks, one per joint."""
        return self._shared.read_state()["errors"]

    @property
    def target(self) -> np.ndarray:
        """The position target currently in shared memory."""
        return self._shared.q_des.copy()

    @property
    def mode(self) -> str:
        """"idle", "position", or "torque" — what the child is actually running."""
        return self._shared.read_state()["mode"]

    @property
    def stats(self) -> dict:
        """Control-loop timing and link health."""
        s = self._shared.read_state()
        avg = s["period_avg"]
        return {
            "iterations": s["iterations"],
            "missed_deadlines": s["missed_deadlines"],
            "rate_hz": 1.0 / avg if avg > 0 else 0.0,
            "period_avg_ms": avg * 1000.0,
            "period_max_ms": s["period_max"] * 1000.0,
            "busy_avg_ms": s["busy_avg"] * 1000.0,
            "utilisation": s["busy_avg"] / avg if avg > 0 else 0.0,
            "rx_frames": s["rx_frames"],
            "position_age_ms": s["position_age"] * 1000.0,
            "error_count": s["error_count"],
        }

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
            if self.running:
                self.close()
        except Exception:
            pass

    def __repr__(self) -> str:
        return (
            f"AllegroHand(channel={self.channel!r}, rate={self.config.rate:.0f}Hz, "
            f"gains={self.gains.name!r}, "
            f"{'running' if self.running else 'stopped'})"
        )
