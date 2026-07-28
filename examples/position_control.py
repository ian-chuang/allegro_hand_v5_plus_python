#!/usr/bin/env python3
"""
Sweep the hand between two poses with the driver's PD controller.

There is no control loop here. This script only decides where the hand should
be and calls `set_position()`; the PD, the CAN traffic and the timing all live
in the driver's control process, so printing as much as you like costs nothing.

    uv run examples/position_control.py
    uv run examples/position_control.py --gains compliant --cycles 5 --period 1.5
"""

import argparse
import time

import numpy as np

from allegro_hand_v5 import PRESETS, AllegroHand, JOINT_NAMES, preset

# Absolute joint angles in degrees, in the 16-joint order. These are measured
# poses that clear the hand's own geometry — the thumb in particular has to stay
# swung out (joint 13 near 140 deg) or the fingers close onto it. They are in
# the encoder frame of the reference hand, so if you set homing offsets in
# examples/calibrate.py, re-measure them. set_position() clips to the calibrated
# range regardless, and anything it moves is reported at start-up.
START_DEG = np.array([
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    3.5, 140, 0, 0,
], dtype=float)
END_DEG = np.array([
    0, 40, 40, 40,
    0, 40, 40, 40,
    0, 40, 40, 40,
    40, 140, 40, 40,
], dtype=float)


def min_jerk(u: float) -> float:
    """s(0)=0, s(1)=1, with zero velocity and acceleration at both ends."""
    u = min(max(u, 0.0), 1.0)
    return 10 * u**3 - 15 * u**4 + 6 * u**5


def move(hand, q_from, q_to, duration, rate=100.0):
    """Interpolate the PD target from one pose to another."""
    t0 = time.perf_counter()
    while (t := time.perf_counter() - t0) < duration:
        hand.set_position(q_from + min_jerk(t / duration) * (q_to - q_from))
        time.sleep(1.0 / rate)
    return hand.set_position(q_to)


def main():
    p = argparse.ArgumentParser(description="Open/close the hand under PD control")
    p.add_argument("--can", default="can0")
    p.add_argument("--gains", default="default", choices=sorted(PRESETS))
    p.add_argument("--period", type=float, default=2.0, help="Seconds per move")
    p.add_argument("--hold", type=float, default=1.0, help="Seconds to dwell at each end")
    p.add_argument("--cycles", type=int, default=3)
    args = p.parse_args()

    with AllegroHand(args.can, gains=preset(args.gains)) as hand:
        print(hand.info)
        print(f"calibration: {hand.calibration.path or 'nominal limits'}")
        print(hand.gains)

        q_start = np.radians(START_DEG)
        q_end = np.radians(END_DEG)
        for name, q in (("START", q_start), ("END", q_end)):
            clipped = hand.calibration.clip(q)
            for j in np.flatnonzero(~np.isclose(clipped, q)):
                print(f"  ! {name} {JOINT_NAMES[j]} {np.degrees(q[j]):+.1f} deg is outside "
                      f"[{np.degrees(hand.calibration.lower[j]):+.1f}, "
                      f"{np.degrees(hand.calibration.upper[j]):+.1f}] "
                      f"-> {np.degrees(clipped[j]):+.1f}")

        print(f"\nApproaching START over {args.period:.1f}s. Ctrl+C to stop.")
        move(hand, hand.positions, q_start, args.period)
        time.sleep(args.hold)

        try:
            for i in range(args.cycles):
                move(hand, q_start, q_end, args.period)
                time.sleep(args.hold)
                end_err = np.degrees(np.abs(q_end - hand.positions))

                move(hand, q_end, q_start, args.period)
                time.sleep(args.hold)
                start_err = np.degrees(np.abs(q_start - hand.positions))

                state = hand.read()
                print(f"  cycle {i + 1}/{args.cycles}   steady-state |error| deg: "
                      f"END mean {end_err.mean():4.1f} max {end_err.max():4.1f}, "
                      f"START mean {start_err.mean():4.1f} max {start_err.max():4.1f}   "
                      f"loop {state.rate:.0f} Hz")
                for err in state.joint_errors:
                    print(f"    ! {err}")
        except KeyboardInterrupt:
            print("\nInterrupted.")

        err = np.degrees(np.abs(hand.target - hand.positions))
        print("\nWorst-tracking joints at the last target:")
        for j in np.argsort(-err)[:5]:
            print(f"  {JOINT_NAMES[j]:<14}{err[j]:6.2f} deg   "
                  f"{hand.currents[j]:+7.1f} mA")


if __name__ == "__main__":
    main()
