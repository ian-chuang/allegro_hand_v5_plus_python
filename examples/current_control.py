#!/usr/bin/env python3
"""
Direct current control on a single joint, with everything else limp.

Bypasses the driver's PD and writes motor current straight through — the only
thing the hand's own board understands. Values still pass the gains'
`max_current` clamp and the 240 mA hardware limit, so a mistake costs you a weak
push rather than a stalled motor.

    # Push the index PIP gently in the positive direction
    uv run examples/current_control.py --joint 2 --current 40

    # Square wave, flipping sign every 2 s
    uv run examples/current_control.py --joint 2 --current 40 --alternate 2.0

    # Close the loop yourself, in this script, at whatever rate you like
    uv run examples/current_control.py --joint 2 --pd 20 --kp 700 --kd 30
"""

import argparse
import time

import numpy as np

from allegro_hand_v5 import AllegroHand, JOINT_NAMES, NUM_JOINTS, SAFE


def main():
    p = argparse.ArgumentParser(description="Direct current control on one joint")
    p.add_argument("--can", default="can0")
    p.add_argument("--joint", type=int, default=2, help="Joint index 0-15")
    p.add_argument("--current", type=float, default=40.0, help="Constant current, mA")
    p.add_argument("--alternate", type=float, help="Flip the sign every N seconds")
    p.add_argument("--pd", type=float,
                   help="Run a PD in this script onto this target, in degrees")
    p.add_argument("--kp", type=float, default=700.0, help="mA per rad, for --pd")
    p.add_argument("--kd", type=float, default=30.0, help="mA per rad/s, for --pd")
    p.add_argument("--max-current", type=float, default=80.0, help="Safety clamp, mA")
    p.add_argument("--rate", type=float, default=200.0, help="Command rate of this script")
    p.add_argument("--duration", type=float, help="Seconds; default until Ctrl+C")
    args = p.parse_args()

    if not 0 <= args.joint < NUM_JOINTS:
        p.error(f"--joint must be in [0, {NUM_JOINTS - 1}]")
    j = args.joint

    print(f"Joint {j} ({JOINT_NAMES[j]}), clamped to +/-{args.max_current:.0f} mA.")
    print("Every other joint is commanded zero current and hangs limp.\n")

    with AllegroHand(args.can, gains=SAFE) as hand:
        hand.set_gains(max_current=args.max_current)
        print(f"{hand.info}\n")

        currents = np.zeros(NUM_JOINTS)
        target = np.radians(args.pd) if args.pd is not None else None
        t0 = time.perf_counter()

        try:
            while True:
                t = time.perf_counter() - t0
                if args.duration is not None and t >= args.duration:
                    break
                s = hand.read()

                if target is not None:
                    current = args.kp * (target - s.positions[j]) - args.kd * s.velocities[j]
                elif args.alternate:
                    current = args.current * (1 if int(t / args.alternate) % 2 == 0 else -1)
                else:
                    current = args.current

                currents[:] = 0.0
                currents[j] = current
                hand.set_current(currents)

                extra = f"  target {np.degrees(target):+6.1f} deg" if target is not None else ""
                print(f"\rt={t:6.2f}s{extra}  pos {np.degrees(s.positions[j]):+7.2f} deg  "
                      f"vel {np.degrees(s.velocities[j]):+7.1f} deg/s  "
                      f"sent {s.currents[j]:+6.1f} mA", end="", flush=True)
                time.sleep(1.0 / args.rate)
        except KeyboardInterrupt:
            print("\n\nInterrupted.")

        hand.relax()
        for err in hand.errors:
            print(f"  ! {err}")

    print("Servos off.")


if __name__ == "__main__":
    main()
