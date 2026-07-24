"""Trajectory demo — set uniform PD gains and sweep a pose back and forth.

Sets every joint's kp/kd to the values you pass, then slowly moves between two poses
(START <-> END, given in degrees) three times, taking ``--duration`` seconds per leg.
Uses the driver's blocking ``hand.move(target_rad, duration)`` helper, which reads the
current joint positions and feeds a smooth interpolated target to the control process.

Run:
    uv run python examples/move.py --channel can0 --kp 0.6 --kd 0.02
"""

import argparse
import time

import numpy as np

from allegro_hand_v5 import AllegroHand, DriverConfig
from allegro_hand_v5 import constants as C

# Poses in DEGREES, canonical joint order (index, middle, ring, thumb).
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--interface", default="socketcan")
    # ap.add_argument("--kp", type=float, default=0.6, help="proportional gain, all joints")
    # ap.add_argument("--kd", type=float, default=0.02, help="derivative gain, all joints")
    ap.add_argument("--max-torque", type=float, default=0.1,
                    help="per-joint torque ceiling [Nm] (safety clamp)")
    ap.add_argument("--duration", type=float, default=2.0, help="seconds per move leg")
    ap.add_argument("--cycles", type=int, default=3, help="back-and-forth repetitions")
    args = ap.parse_args()

    start = np.radians(START_DEG)
    end = np.radians(END_DEG)

    # All joints use the same kp/kd; keep a safety torque ceiling.
    cfg = DriverConfig()
    # cfg.kp = np.full(C.DOF, args.kp)
    # cfg.kd = np.full(C.DOF, args.kd)
    cfg.set_max_torque(args.max_torque)

    with AllegroHand(channel=args.channel, interface=args.interface, config=cfg) as hand:
        m = hand.model
        print(f"Connected: {m.handedness} type {m.hand_type} (plus={m.is_plus})")
        print(f"ceiling={args.max_torque} Nm  "
              f"duration={args.duration}s/leg  cycles={args.cycles}")

        try:
            print("moving to START...")
            hand.move(start, args.duration)
            for i in range(args.cycles):
                print(f"cycle {i + 1}/{args.cycles}: START -> END")
                hand.move(end, args.duration)
                print(f"cycle {i + 1}/{args.cycles}: END -> START")
                hand.move(start, args.duration)
            time.sleep(0.3)
        except KeyboardInterrupt:
            print("\ninterrupted — releasing torque")
            hand.relax()
        print("done.")


if __name__ == "__main__":
    main()
