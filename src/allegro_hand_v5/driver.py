"""Main Allegro Hand V5 driver — real-time control in a separate process.

``AllegroHand`` runs the entire real-time loop — CAN receive, PD position control, and
CAN transmit — in a **child process** with its own interpreter and GIL, so nothing the
parent (your RL policy, logging, plotting, …) does can stall control. The parent only
reads/writes shared-memory arrays. The child runs a single deterministic loop
(drain RX → control → TX → sleep to rate), the same structure as the vendor C driver.

    from allegro_hand_v5 import AllegroHand, DriverConfig

    with AllegroHand(channel="can0", config=DriverConfig.safe(0.03)) as hand:
        hand.set_target(q_des)          # position mode (default): PD chases this target
        hand.set_torque(tau_nm)         # torque mode: send these joint torques directly
        q, qd = hand.positions, hand.velocities
        print(hand.loop_rate_hz)        # measured child loop rate — a health check

Linux/SocketCAN in practice; uses the ``fork`` start method so the shared ctypes buffers
are inherited by the child.
"""

from __future__ import annotations

import atexit
import multiprocessing as mp
import time
from ctypes import c_double, c_int
from typing import Optional

import numpy as np

from . import constants as C
from . import protocol as P
from .config import DriverConfig
from .model import HandModel

MODE_POSITION = 0
MODE_TORQUE = 1


def pd_torque(kp, kd, ki, target, positions, velocities, integral, rate_hz, i_clamp):
    """Pure PD(+I) control law. Returns ``(tau_nm, err, new_integral)``. No I/O, no state."""
    err = target - positions
    integral = np.clip(integral + err / rate_hz, -i_clamp, i_clamp)
    tau = kp * err - kd * velocities + ki * integral
    return tau, err, integral


def _np(raw) -> np.ndarray:
    """numpy view over a multiprocessing RawArray (shared, zero-copy)."""
    return np.frombuffer(raw, dtype=np.float64)


def emergency_off_frames():
    """One round of the torque-off sequence: zero current to every joint, then SERVO_OFF.

    Zero current first means that even if SERVO_OFF is dropped on the bus, the last thing
    each motor received is 0 mA (no force). SERVO_OFF then disables the motor drivers.
    """
    frames = [P.encode_set_torque(f, (0, 0, 0, 0)) for f in range(C.NUM_FINGERS)]
    frames.append(P.encode_servo_off())
    return frames


class AllegroHand:
    def __init__(
        self,
        channel: str = "can0",
        interface: str = "socketcan",
        config: Optional[DriverConfig] = None,
        period_ms=C.DEFAULT_PERIOD_MS,
        rate_hz: Optional[float] = None,
    ):
        self.channel = channel
        self.interface = interface
        self.config = config or DriverConfig()
        self.period_ms = period_ms
        self.rate_hz = float(rate_hz if rate_hz is not None else self.config.rate_hz)

        ctx = mp.get_context("fork")
        # Shared numeric buffers: child writes state/torque, parent writes commands.
        self._pos = ctx.RawArray(c_double, C.DOF)
        self._vel = ctx.RawArray(c_double, C.DOF)
        self._press = ctx.RawArray(c_double, C.NUM_FINGERS)
        self._tgt = ctx.RawArray(c_double, C.DOF)
        self._torque_cmd = ctx.RawArray(c_double, C.DOF)   # for MODE_TORQUE
        self._tau_out = ctx.RawArray(c_double, C.DOF)       # applied torque [Nm]
        # Scalars / flags.
        self._mode = ctx.Value(c_int, MODE_POSITION, lock=False)
        self._running = ctx.Value(c_int, 0, lock=False)
        self._pose_ready = ctx.Value(c_int, 0, lock=False)
        self._err_motor = ctx.Value(c_int, -1, lock=False)
        self._err_code = ctx.Value(c_int, 0, lock=False)
        self._loop_hz = ctx.Value(c_double, 0.0, lock=False)
        self._stamp = ctx.Value(c_double, 0.0, lock=False)
        self._lock = ctx.Lock()
        self._ready_q = ctx.Queue()
        self._ctx = ctx
        self._proc: Optional[mp.process.BaseProcess] = None
        self._model: Optional[HandModel] = None

    # -- lifecycle ----------------------------------------------------------
    def connect(self, initial_target=None, timeout: float = 5.0) -> HandModel:
        """Spawn the control process and run the handshake. Returns the HandModel."""
        self._running.value = 1
        self._proc = self._ctx.Process(target=_child_main, args=(self,), daemon=True)
        self._proc.start()
        # Guarantee torque-off on *any* parent exit (normal, exception, sys.exit),
        # even if the caller forgets the context manager. Idempotent with __exit__.
        atexit.register(self.disconnect)
        try:
            tag, payload = self._ready_q.get(timeout=timeout)
        except Exception:
            self.disconnect()
            raise
        if tag == "error":
            self.disconnect()
            raise RuntimeError(f"control process failed to start: {payload}")
        self._model = payload
        if initial_target is not None:
            self.set_target(initial_target)
        return self._model

    def disconnect(self) -> None:
        """Stop the control process, ensuring the motors are torqued off. Idempotent.

        Signals the child to stop (it torques off in its cleanup), waits, then escalates
        SIGTERM -> SIGKILL only as a last resort. The child also self-protects: it ignores
        the terminal's Ctrl-C and torques off if the parent dies unexpectedly.
        """
        self._running.value = 0
        proc = self._proc
        if proc is None:
            return
        self._proc = None
        proc.join(timeout=2.0)                 # child exits its loop and torques off
        if proc.is_alive():
            proc.terminate()                   # SIGTERM -> child cleanup handler
            proc.join(timeout=1.5)
        if proc.is_alive():
            proc.kill()                        # last resort
            proc.join(timeout=1.0)

    def __enter__(self) -> "AllegroHand":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.disconnect()

    # -- commands -----------------------------------------------------------
    def set_target(self, q_des) -> None:
        """Position mode: set desired joint angles [rad] (16-vector, clamped to limits)."""
        q = np.asarray(q_des, dtype=np.float64).reshape(C.DOF)
        if self._model is not None:
            q = self._model.clamp_positions(q)
        with self._lock:
            _np(self._tgt)[:] = q
            self._mode.value = MODE_POSITION

    def set_torque(self, tau_nm) -> None:
        """Torque mode: send these joint torques [Nm] directly (bypasses the PD)."""
        t = np.asarray(tau_nm, dtype=np.float64).reshape(C.DOF)
        with self._lock:
            _np(self._torque_cmd)[:] = t
            self._mode.value = MODE_TORQUE

    def relax(self) -> None:
        """Command zero torque (limp, backdrivable) while still streaming state."""
        self.set_torque(np.zeros(C.DOF))

    def move(self, target, duration: float, rate_hz: float = 100.0) -> None:
        """Smoothly move to ``target`` joint angles [rad] over ``duration`` s (blocking).

        Reads the current measured positions and feeds a smoothstep-interpolated target to
        the control process; the child's PD closes the loop toward the moving setpoint.
        ``target`` is a 16-vector in radians (clamped to the model's joint limits).
        """
        target = np.asarray(target, dtype=np.float64).reshape(C.DOF)
        if self._model is not None:
            target = self._model.clamp_positions(target)
        q0 = self.positions
        if duration <= 0:
            self.set_target(target)
            return
        dt = 1.0 / rate_hz
        t0 = time.monotonic()
        while (t := time.monotonic() - t0) < duration:
            a = t / duration
            a = a * a * (3 - 2 * a)  # smoothstep ease-in/out
            self.set_target(q0 + a * (target - q0))
            time.sleep(dt)
        self.set_target(target)

    # -- state (read-only copies) -------------------------------------------
    @property
    def positions(self) -> np.ndarray:
        return _np(self._pos).copy()

    @property
    def velocities(self) -> np.ndarray:
        return _np(self._vel).copy()

    @property
    def fingertip_pressures(self) -> np.ndarray:
        return _np(self._press).copy()

    @property
    def target(self) -> np.ndarray:
        with self._lock:
            return _np(self._tgt).copy()

    @property
    def last_torque(self) -> np.ndarray:
        return _np(self._tau_out).copy()

    @property
    def loop_rate_hz(self) -> float:
        return float(self._loop_hz.value)

    @property
    def stamp(self) -> float:
        return float(self._stamp.value)

    @property
    def model(self) -> HandModel:
        if self._model is None:
            raise RuntimeError("not connected; call connect() first")
        return self._model

    @property
    def last_error(self) -> Optional[P.MotorError]:
        m = self._err_motor.value
        return None if m < 0 else P.MotorError(m, self._err_code.value)


# ===========================================================================
# Child process — the entire real-time loop lives here.
# ===========================================================================
def _child_main(hand: "AllegroHand") -> None:
    import os
    import signal
    import time

    # SAFETY 1: ignore the terminal's Ctrl-C. SIGINT is delivered to the whole process
    # group, so without this the child would die mid-actuation and leave the motors
    # energized. Instead the child keeps running until told to stop, then torques off.
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    # SAFETY 2: on SIGTERM (parent terminate() / kill <pid>) request a clean stop.
    stop = {"flag": False}

    def _on_term(signum, frame):
        stop["flag"] = True

    signal.signal(signal.SIGTERM, _on_term)

    # SAFETY 3: if the parent dies for ANY reason (even SIGKILL), we get reparented and
    # os.getppid() changes — detect that and torque off.
    orig_ppid = os.getppid()

    try:
        import can
    except Exception as exc:  # pragma: no cover
        hand._ready_q.put(("error", f"python-can import failed: {exc}"))
        return

    cfg = hand.config
    pos, vel, press = _np(hand._pos), _np(hand._vel), _np(hand._press)
    tgt, torque_cmd, tau_out = _np(hand._tgt), _np(hand._torque_cmd), _np(hand._tau_out)

    # ---- open bus ----
    kwargs = {} if hand.interface == "socketcan" else {"bitrate": C.CAN_BITRATE}
    try:
        bus = can.Bus(channel=hand.channel, interface=hand.interface, **kwargs)
    except Exception as exc:
        hand._ready_q.put(("error", f"could not open CAN bus: {exc}"))
        return

    def send(frame: P.Frame):
        bus.send(can.Message(arbitration_id=P.to_arbitration_id(frame.msg_id),
                             data=frame.data, is_remote_frame=frame.is_rtr,
                             is_extended_id=False,
                             dlc=0 if frame.is_rtr else len(frame.data)))

    def emergency_off():
        """Belt-and-suspenders torque-off, tolerant of a flaky bus. Never raises.

        Sends zero current to every joint (so even if SERVO_OFF is dropped the last
        command is 0), then SERVO_OFF (disables the motor drivers), repeated a few times,
        then stops the streams. Runs on every exit path.
        """
        for _ in range(5):
            try:
                for frame in emergency_off_frames():
                    send(frame)
            except Exception:
                pass
            time.sleep(0.002)
        try:
            send(P.encode_stop_streams())
        except Exception:
            pass
        time.sleep(0.02)  # let the kernel flush TX before the socket closes

    alpha = 0.2
    last_stamp = np.zeros(C.DOF)
    serial = None
    ready_mask = 0

    def drain_rx(now):
        nonlocal serial, ready_mask
        while True:
            msg = bus.recv(timeout=0.0)
            if msg is None:
                return
            if msg.is_error_frame or msg.is_remote_frame:
                continue
            mid = P.from_arbitration_id(msg.arbitration_id)
            data = bytes(msg.data)
            if C.ID_RTR_FINGER_POSE <= mid < C.ID_RTR_FINGER_POSE + C.NUM_FINGERS:
                f = mid - C.ID_RTR_FINGER_POSE
                base = f * C.JOINTS_PER_FINGER
                for j, a in enumerate(P.decode_finger_pose(f, data)):
                    idx = base + j
                    dt = now - last_stamp[idx]
                    if 0.0 < dt < 0.1:
                        v = (a - pos[idx]) / dt
                        vel[idx] = alpha * v + (1 - alpha) * vel[idx]
                    pos[idx] = a
                    last_stamp[idx] = now
                ready_mask |= (1 << f)
                hand._stamp.value = now
            elif mid in C.FINGERTIP_ID_TO_BASE:
                i0, v0, i1, v1 = P.decode_fingertip(mid, data)
                for idx, v in ((i0, v0), (i1, v1)):
                    press[idx] = 0 if (v < 0 or v > C.PRESSURE_VALID_MAX) else v
            elif mid == C.ID_RTR_SERIAL:
                serial = P.decode_serial(data)
            elif mid == C.ID_ERROR:
                e = P.decode_error(data)
                hand._err_motor.value, hand._err_code.value = e.motor_id, e.code

    def should_run() -> bool:
        return (bool(hand._running.value) and not stop["flag"]
                and os.getppid() == orig_ppid)

    # Everything from here on torques off in the finally, no matter how we exit.
    try:
        # ---- handshake (servo ON is required for streaming) ----
        for _ in range(100):
            if bus.recv(timeout=0.0) is None:
                break
        send(P.encode_servo_off())
        send(P.request_hand_info())
        send(P.request_serial())
        send(P.encode_set_period(hand.period_ms))
        send(P.encode_servo_on())

        # ---- wait for serial + all four poses (or time out) ----
        t_end = time.monotonic() + 2.0
        while time.monotonic() < t_end and (serial is None or ready_mask != 0x0F):
            drain_rx(time.monotonic())
            time.sleep(0.001)

        model = HandModel.from_serial(serial) if serial is not None else HandModel.default()
        hand._pose_ready.value = ready_mask
        tgt[:] = pos  # hold current pose until the parent commands otherwise
        hand._ready_q.put(("ready", model))

        # ---- real-time control loop ----
        kp, kd, ki = cfg.kp, cfg.kd, cfg.ki
        integral = np.zeros(C.DOF)
        dt_nom = 1.0 / hand.rate_hz
        max_cmd = cfg.max_current_ma
        scale = cfg.nm_to_ma
        is_plus = model.is_plus
        plus_j = list(cfg.plus_halved_joints)

        hz_ema = hand.rate_hz
        next_t = prev_t = time.monotonic()
        while should_run():
            now = time.monotonic()
            drain_rx(now)

            with hand._lock:
                mode = hand._mode.value
                target = tgt.copy() if mode == MODE_POSITION else None
                tau = None if mode == MODE_POSITION else torque_cmd.copy()

            if mode == MODE_POSITION:
                tau, _, integral = pd_torque(kp, kd, ki, target, pos, vel,
                                             integral, hand.rate_hz, cfg.i_clamp)

            cmd = tau * scale                      # Nm -> mA
            if is_plus and plus_j:
                cmd[plus_j] *= 0.5                 # Plus MCP-2 halving
            np.clip(cmd, -max_cmd, max_cmd, out=cmd)
            for f in range(C.NUM_FINGERS):
                b = f * C.JOINTS_PER_FINGER
                send(P.encode_set_torque(f, cmd[b:b + C.JOINTS_PER_FINGER]))
            tau_out[:] = tau

            loop_dt = now - prev_t
            prev_t = now
            if loop_dt > 0:
                hz_ema = 0.99 * hz_ema + 0.01 * (1.0 / loop_dt)
                hand._loop_hz.value = hz_ema

            next_t += dt_nom
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()  # fell behind; resync
    except BaseException:
        # includes KeyboardInterrupt (if SIGINT ever slips through) and any bus error
        pass
    finally:
        emergency_off()
        try:
            bus.shutdown()
        except Exception:
            pass
