#!/usr/bin/env python3
"""
Direct torque control on a single joint, with everything else limp.

Bypasses the driver's PD and writes joint torques straight through. Values still
pass the gain profile's `max_torque` clamp and the config's `max_current_ma`
clamp, so a mistake costs you a weak push rather than a stalled motor.

    # Push the index PIP gently in the positive direction
    uv run examples/torque_control.py --joint 2 --torque 0.03

    # Square wave, alternating sign every 2 s
    uv run examples/torque_control.py --joint 2 --torque 0.03 --alternate 2.0

    # Close the loop yourself: PD onto a target, computed here
    uv run examples/torque_control.py --joint 2 --pd 20 --kp 0.5 --kd 0.02
"""

import argparse
import time

import numpy as np

from allegro_hand_v5 import AllegroHand, DriverConfig, JOINT_LABELS, TORQUE_TO_CURRENT


def main():
    p = argparse.ArgumentParser(description="Direct torque control on one joint")
    p.add_argument("--can", default="can0")
    p.add_argument("--joint", type=int, default=2, help="Joint index 0-15")
    p.add_argument("--torque", type=float, default=0.03, help="Constant torque, Nm")
    p.add_argument("--alternate", type=float, default=None,
                   help="Flip the sign every N seconds instead of holding it")
    p.add_argument("--pd", type=float, default=None,
                   help="Instead of a constant torque, run a PD in this script onto "
                        "this target in degrees")
    p.add_argument("--kp", type=float, default=0.5, help="Proportional gain for --pd")
    p.add_argument("--kd", type=float, default=0.02, help="Derivative gain for --pd")
    p.add_argument("--max-torque", type=float, default=0.05, help="Safety clamp, Nm")
    p.add_argument("--rate", type=float, default=200.0, help="Rate this script commands at")
    p.add_argument("--duration", type=float, default=None, help="Seconds; default until Ctrl+C")
    args = p.parse_args()

    if not 0 <= args.joint < 16:
        p.error("--joint must be in [0, 15]")
    j = args.joint

    print("Direct torque control")
    print("=" * 66)
    print(f"Joint:  {JOINT_LABELS[j]}")
    print(f"Clamp:  {args.max_torque:.3f} Nm "
          f"= {args.max_torque * TORQUE_TO_CURRENT:.0f} mA")
    if args.pd is not None:
        print(f"Mode:   PD in this script onto {args.pd:+.1f} deg, "
              f"Kp={args.kp} Kd={args.kd}")
    elif args.alternate:
        print(f"Mode:   +/-{args.torque:.3f} Nm, flipping every {args.alternate:.1f}s")
    else:
        print(f"Mode:   constant {args.torque:+.3f} Nm")
    print("\nAll other joints are commanded zero torque and go limp.")
    print("Ctrl+C to stop.\n")

    with AllegroHand(args.can, config=DriverConfig()) as hand:
        # The clamp lives in the gain profile even in torque mode.
        hand.set_gains(max_torque=args.max_torque)
        print(f"{hand.info}\n")

        torques = np.zeros(16)
        target_rad = np.radians(args.pd) if args.pd is not None else None
        dt = 1.0 / args.rate
        t0 = time.perf_counter()

        try:
            while True:
                t = time.perf_counter() - t0
                if args.duration is not None and t >= args.duration:
                    break

                s = hand.read()

                if target_rad is not None:
                    tau = args.kp * (target_rad - s.positions[j]) - args.kd * s.velocities[j]
                elif args.alternate:
                    tau = args.torque * (1 if int(t / args.alternate) % 2 == 0 else -1)
                else:
                    tau = args.torque

                torques[:] = 0.0
                torques[j] = tau
                hand.set_torque(torques)

                extra = f"  target={np.degrees(target_rad):+6.1f}" if target_rad is not None else ""
                print(f"\rt={t:6.2f}s{extra}  pos={np.degrees(s.positions[j]):+7.2f} deg  "
                      f"vel={np.degrees(s.velocities[j]):+7.1f} d/s  "
                      f"tau={s.torques[j]:+.4f} Nm  ({s.currents[j]:+6.1f} mA)",
                      end="", flush=True)
                time.sleep(dt)

        except KeyboardInterrupt:
            print("\n\nInterrupted!")

        hand.relax()
        if hand.errors:
            print("\nHand reported errors:")
            for err in hand.errors:
                print(f"  ! {err}")

    print("\nServos off.")


if __name__ == "__main__":
    main()
