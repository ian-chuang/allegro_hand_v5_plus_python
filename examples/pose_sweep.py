#!/usr/bin/env python3
"""
Full-hand pose sweep between two poses.

There is no control loop in this script. It only decides where the hand should
be and calls `set_position()`; the PD, the CAN traffic, and the 500 Hz timing
all live in the driver's control process. So this script can run at whatever
rate is convenient and print as much as it likes without perturbing control.

    uv run examples/pose_sweep.py
    uv run examples/pose_sweep.py --gains compliant --cycles 5
    uv run examples/pose_sweep.py --gains soft --period 1.5 --csv sweep.csv
"""

import argparse
import csv
import time

import numpy as np

from allegro_hand_v5 import PROFILES, AllegroHand, DriverConfig, JOINT_LABELS, get_profile

# Absolute joint angles in degrees, in the order every 16-vector uses.
START_DEG = (
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    3.5, 140, 0, 0,
)
END_DEG = (
    0, 40, 40, 40,
    0, 40, 40, 40,
    0, 40, 40, 40,
    40, 140, 40, 40,
)


def min_jerk(u: float) -> float:
    """Minimum-jerk blend: s(0)=0, s(1)=1, zero velocity and accel at both ends."""
    u = min(max(u, 0.0), 1.0)
    return 10 * u**3 - 15 * u**4 + 6 * u**5


def goto(hand, q_from, q_to, duration, rate, log=None, phase=""):
    """Interpolate the target from q_from to q_to over `duration` seconds."""
    dt = 1.0 / rate
    t0 = time.perf_counter()
    while (t := time.perf_counter() - t0) < duration:
        q_des = q_from + min_jerk(t / duration) * (q_to - q_from)
        hand.set_position(q_des)
        _sample(hand, q_des, log, phase)
        time.sleep(dt)
    return hand.set_position(q_to)


def dwell(hand, q_target, duration, rate, log=None, phase=""):
    """Hold a target, sampling state the whole time."""
    dt = 1.0 / rate
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < duration:
        _sample(hand, q_target, log, phase)
        time.sleep(dt)


def _sample(hand, q_des, log, phase):
    if log is None:
        return
    s = hand.read()
    log.append((phase, s.t, np.asarray(q_des).copy(), s.positions, s.torques))


def main():
    p = argparse.ArgumentParser(description="Full-hand pose sweep")
    p.add_argument("--can", default="can0")
    p.add_argument("--hand", default="right", choices=["left", "right"],
                   help="Only a hint; the hand's serial number overrides it")
    p.add_argument("--gains", default="bhand_home", choices=sorted(PROFILES),
                   help="Gain profile (default: bhand_home)")
    p.add_argument("--kp", type=float, help="Override the profile's kp on every joint")
    p.add_argument("--kd", type=float, help="Override the profile's kd on every joint")
    p.add_argument("--max-torque", type=float, help="Override the per-joint torque clamp (Nm)")
    p.add_argument("--calibration", help="Calibration JSON; default is auto-selected from the serial")
    p.add_argument("--control-rate", type=float, default=500.0, help="Control process rate (Hz)")
    p.add_argument("--command-rate", type=float, default=100.0, help="Rate this script sets targets")
    p.add_argument("--approach", type=float, default=2.0, help="Ramp from the current pose to START (s)")
    p.add_argument("--hold", type=float, default=1.5, help="Dwell at each end (s)")
    p.add_argument("--period", type=float, default=2.0, help="Duration of each START<->END move (s)")
    p.add_argument("--cycles", type=int, default=3, help="Round trips")
    p.add_argument("--csv", help="Write the log to this CSV")
    args = p.parse_args()

    profile = get_profile(args.gains)
    if args.max_torque is not None:
        profile = profile.with_max_torque(args.max_torque)

    q_start = np.radians(np.array(START_DEG, dtype=np.float64))
    q_end = np.radians(np.array(END_DEG, dtype=np.float64))

    print("Full-hand pose sweep")
    print("=" * 72)

    log = []
    with AllegroHand(
        args.can,
        gains=profile,
        config=DriverConfig(rate=args.control_rate),
        calibration=args.calibration or True,
        handedness=args.hand,
    ) as hand:
        if args.kp is not None or args.kd is not None:
            hand.set_gains(kp=args.kp, kd=args.kd)

        print(f"{hand.info}")
        print(f"Control: {args.control_rate:.0f} Hz in a child process; "
              f"this script sets targets at {args.command_rate:.0f} Hz")
        print(hand.get_gains().table())

        cal = hand.calibration
        print(f"Calibration: {cal.source if cal and cal.source else 'URDF defaults'}")
        for name, q in (("START", q_start), ("END", q_end)):
            clipped = cal.clip(q) if cal else q
            for j in np.flatnonzero(~np.isclose(clipped, q)):
                lo, hi = cal.limits(j)
                print(f"  ! {name} {JOINT_LABELS[j]} {np.degrees(q[j]):+.1f} deg outside "
                      f"[{np.degrees(lo):+.1f}, {np.degrees(hi):+.1f}] "
                      f"-> {np.degrees(clipped[j]):+.1f}")
        print("\nCtrl+C to stop.\n")

        try:
            print(f"Approaching START over {args.approach:.1f}s...")
            goto(hand, hand.positions, q_start, args.approach, args.command_rate, log, "approach")
            dwell(hand, q_start, args.hold, args.command_rate, log, "hold_start")

            for i in range(args.cycles):
                goto(hand, q_start, q_end, args.period, args.command_rate, log, "out")
                dwell(hand, q_end, args.hold, args.command_rate, log, "hold_end")
                goto(hand, q_end, q_start, args.period, args.command_rate, log, "back")
                dwell(hand, q_start, args.hold, args.command_rate, log, "hold_start")

                err = np.degrees(np.abs(q_start - hand.positions))
                st = hand.stats
                errors = hand.errors
                print(f"  cycle {i + 1}/{args.cycles}  |err| at START: mean {err.mean():5.2f} deg, "
                      f"max {err.max():5.2f} deg   loop {st['rate_hz']:.0f} Hz, "
                      f"missed {st['missed_deadlines']}"
                      + (f"   ERRORS: {'; '.join(str(e) for e in errors)}" if errors else ""))

        except KeyboardInterrupt:
            print("\nInterrupted!")

        st = hand.stats
        print(f"\nControl process: {st['iterations']} cycles at {st['rate_hz']:.1f} Hz "
              f"(period avg {st['period_avg_ms']:.2f} ms, worst {st['period_max_ms']:.2f} ms), "
              f"{st['missed_deadlines']} missed, {st['rx_frames']} CAN frames in")
        print(f"                 {st['busy_avg_ms']:.2f} ms of work per cycle "
              f"= {st['utilisation'] * 100:.0f}% utilisation")

    if log:
        phases = [r[0] for r in log]
        des = np.array([r[2] for r in log])
        pos = np.array([r[3] for r in log])
        err = np.degrees(np.abs(des - pos))
        held = np.array([p.startswith("hold") for p in phases])
        moving = np.array([p in ("out", "back") for p in phases])

        print("\nTracking error (degrees)")
        if held.any():
            print(f"  steady state, mean : {err[held].mean():.3f}")
        if moving.any():
            print(f"  moving RMS         : {np.sqrt((err[moving] ** 2).mean()):.3f}")
            print(f"  moving peak        : {err[moving].max():.3f}")

        if held.any():
            per_joint = err[held].mean(axis=0)
            print(f"\n{'joint':<24}{'steady-state |err| deg':>24}")
            print("-" * 48)
            for j in np.argsort(-per_joint)[:8]:
                print(f"{JOINT_LABELS[j]:<24}{per_joint[j]:>24.3f}")

    if args.csv and log:
        with open(args.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["phase", "t"] + [f"des{j}" for j in range(16)]
                       + [f"pos{j}" for j in range(16)] + [f"tau{j}" for j in range(16)])
            for phase, t, d, q, tau in log:
                w.writerow([phase, f"{t:.5f}"] + [f"{v:.6f}" for v in d]
                           + [f"{v:.6f}" for v in q] + [f"{v:.6f}" for v in tau])
        print(f"\nWrote {args.csv}")


if __name__ == "__main__":
    main()
