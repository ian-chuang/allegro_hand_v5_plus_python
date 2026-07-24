"""Read-only joint monitor — commands ZERO torque, so the hand stays limp/backdrivable.

The hand only streams joint positions when the servo is on, so the driver servos on but we
call ``hand.relax()`` (0 mA to every joint). The mainboard's current loop then holds zero
torque — the hand is free to move by hand while the encoders stream.

Move each joint and watch its number change; the running min/max columns reveal each
joint's range and sign.

Run:
    uv run python examples/read_joints.py --channel can0            # radians
    uv run python examples/read_joints.py --deg                     # degrees
    uv run python examples/read_joints.py --csv joints.csv          # also log to CSV
"""

import argparse
import csv
import time

import numpy as np

from allegro_hand_v5 import AllegroHand
from allegro_hand_v5 import constants as C


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--rate", type=float, default=10.0, help="table refresh rate [Hz]")
    ap.add_argument("--deg", action="store_true", help="show degrees instead of radians")
    ap.add_argument("--csv", default=None, help="optional CSV log path")
    args = ap.parse_args()

    unit = "deg" if args.deg else "rad"
    conv = np.degrees if args.deg else (lambda x: x)

    with AllegroHand(channel=args.channel, interface=args.interface) as hand:
        hand.relax()  # 0 mA everywhere -> backdrivable, read only
        m = hand.model
        print(f"Connected: {m.handedness} hand, type {m.hand_type}, serial={m.serial!r} "
              f"(0 mA commanded — read only)\n")

        lo = np.full(C.DOF, np.inf)
        hi = np.full(C.DOF, -np.inf)

        csv_writer = csv_file = None
        if args.csv:
            csv_file = open(args.csv, "w", newline="")
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(["t"] + list(C.JOINT_NAMES))

        period = 1.0 / args.rate
        printed = 0
        t0 = time.monotonic()
        try:
            while True:
                hand.relax()
                q = conv(hand.positions)
                lo, hi = np.minimum(lo, q), np.maximum(hi, q)

                lines = [f"{'idx':>3}  {'joint':<12} {'pos(' + unit + ')':>10} "
                         f"{'min':>9} {'max':>9} {'range':>9}"]
                for i, name in enumerate(C.JOINT_NAMES):
                    rng = (hi[i] - lo[i]) if np.isfinite(lo[i]) else 0.0
                    lines.append(f"{i:>3}  {name:<12} {q[i]:>10.3f} "
                                 f"{lo[i]:>9.3f} {hi[i]:>9.3f} {rng:>9.3f}")
                    if i % 4 == 3 and i != C.DOF - 1:
                        lines.append("-" * 55)
                age = time.monotonic() - t0
                lines.append(f"\npressures (Pa): {np.round(hand.fingertip_pressures, 1)}   "
                             f"stamp={hand.stamp:.3f}   (Ctrl-C to stop)")

                out = "\n".join(lines)
                if printed:
                    print(f"\033[{printed}A", end="")
                print(out + "\033[K")
                printed = out.count("\n") + 1

                if csv_writer is not None:
                    csv_writer.writerow([f"{age:.4f}"] + [f"{v:.5f}" for v in q])
                time.sleep(period)
        except KeyboardInterrupt:
            print("\nstopping.")
        finally:
            if csv_file is not None:
                csv_file.close()
                print(f"CSV written to {args.csv}")


if __name__ == "__main__":
    main()
