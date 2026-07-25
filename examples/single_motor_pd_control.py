#!/usr/bin/env python3
"""
Single-motor PD control example for the Allegro Hand V5.

Runs a PD torque law on exactly ONE joint (by default the index finger DIP,
joint 3). Every other joint is commanded zero torque, so the rest of the hand
goes limp while the selected motor holds or tracks a target.

The target is given as a fraction of the joint's *calibrated* range
(0 = measured minimum, 1 = measured maximum), so the same command works on any
hand once calibration/<hand>.json exists for it.

WARNING: Direct torque control bypasses the BHand safety features.
Start with small gains and a small torque limit, and be ready to Ctrl+C.

Examples:
    # Hold the index MCP at 40% of its range
    uv run examples/single_motor_pd_control.py --joint 1 --target 0.4

    # Oscillate the index PIP between 20% and 60% every 3 seconds
    uv run examples/single_motor_pd_control.py --joint 2 --target 0.2 --sweep-to 0.6 --period 3.0
"""

import argparse
import time

import numpy as np

from allegro_hand_v5 import AllegroHand, HandCalibration
from allegro_hand_v5.calibration import JOINT_NAMES


def main():
    parser = argparse.ArgumentParser(description="Single-motor PD control on the Allegro Hand V5")
    parser.add_argument("--hand", type=str, default="right", choices=["left", "right"])
    parser.add_argument("--type", type=str, default="B", choices=["A", "B"])
    parser.add_argument("--can", type=str, default="can0")
    parser.add_argument(
        "--calibration",
        type=str,
        help="Calibration JSON (default: calibration/<hand>.json, else URDF ranges)",
    )
    parser.add_argument(
        "--joint", type=int, default=3, help="Joint index 0-15 to control (default: 3, Index DIP)"
    )
    parser.add_argument(
        "--target", type=float, default=0.4, help="Target as fraction of calibrated range (0-1)"
    )
    parser.add_argument(
        "--sweep-to",
        type=float,
        default=None,
        help="If set, sinusoidally sweep between --target and this fraction",
    )
    parser.add_argument("--period", type=float, default=3.0, help="Sweep period (s)")
    parser.add_argument("--kp", type=float, default=1.0, help="Proportional gain")
    parser.add_argument("--kd", type=float, default=0.02, help="Derivative gain")
    parser.add_argument("--max-torque", type=float, default=0.1, help="Torque clamp (abs)")
    parser.add_argument("--rate", type=float, default=500.0, help="Control rate (Hz)")
    parser.add_argument(
        "--duration", type=float, default=None, help="Run time (s), default: until Ctrl+C"
    )
    parser.add_argument("--no-home", action="store_true", help="Skip homing before torque control")
    args = parser.parse_args()

    if not 0 <= args.joint < 16:
        parser.error("--joint must be in [0, 15]")

    j = args.joint

    print("Allegro Hand V5 Single-Motor PD Control")
    print("=" * 40)

    # Measured joint ranges for this particular hand.
    cal = HandCalibration.load(path=args.calibration, hand_type=args.hand)
    print(f"Calibration: {cal.source or 'URDF defaults'}")

    lo, hi = cal.limits(j)
    target_rad = cal.joint(j, args.target)
    sweep_rad = cal.joint(j, args.sweep_to) if args.sweep_to is not None else None

    print(f"Joint:  {JOINT_NAMES[j]}")
    print(f"Range:  [{lo:+.3f}, {hi:+.3f}] rad")
    if sweep_rad is None:
        print(f"Target: {args.target:.2f} of range = {target_rad:+.3f} rad")
    else:
        print(
            f"Sweep:  {args.target:.2f} -> {args.sweep_to:.2f} of range "
            f"({target_rad:+.3f} -> {sweep_rad:+.3f} rad), period {args.period:.1f}s"
        )
    print(f"Gains:  Kp={args.kp}  Kd={args.kd}  |tau| <= {args.max_torque}")
    print("\nAll other joints are commanded ZERO torque and will go limp.")
    print("Press Ctrl+C to stop.\n")

    with AllegroHand(
        hand_type=args.hand,
        hardware_type=args.type,
        can_channel=args.can,
        control_frequency=args.rate,
        calibration=cal,
    ) as hand:
        if not args.no_home:
            print("Moving to home position...")
            hand.home()
            time.sleep(2.0)

        print("Starting PD control...\n")
        dt = 1.0 / args.rate

        with hand.torque_mode():
            torques = np.zeros(16)
            t0 = time.perf_counter()

            try:
                while True:
                    t = time.perf_counter() - t0
                    if args.duration is not None and t >= args.duration:
                        break

                    # Setpoint for this instant (constant, or sinusoidal sweep).
                    if sweep_rad is None:
                        setpoint = target_rad
                    else:
                        phase = 0.5 * (1.0 - np.cos(2 * np.pi * t / args.period))
                        setpoint = target_rad + phase * (sweep_rad - target_rad)

                    pos = hand.get_positions()
                    vel = hand.get_velocities()

                    # PD law on the single selected joint.
                    error = setpoint - pos[j]
                    tau = args.kp * error - args.kd * vel[j]
                    tau = float(np.clip(tau, -args.max_torque, args.max_torque))

                    # Everything else stays at zero torque.
                    torques[:] = 0.0
                    torques[j] = tau
                    hand.set_torques(torques)

                    print(
                        f"\rt={t:6.2f}s  target={setpoint:+.3f}  pos={pos[j]:+.3f}  "
                        f"err={error:+.3f}  vel={vel[j]:+.3f}  tau={tau:+.4f}",
                        end="",
                        flush=True,
                    )

                    time.sleep(dt)

            except KeyboardInterrupt:
                print("\n\nInterrupted!")

        # torque_mode() returns the hand to gravity compensation on exit.
        print("\n\nReturned to safe mode")


if __name__ == "__main__":
    main()
