#!/usr/bin/env python3
"""
Read everything the hand reports. Commands no torque at all.

Prints the identity (hardware/firmware version, serial, handedness, hand type),
then streams joint positions, fingertip pressures, error codes, and control-loop
health. Safe to run at any time — the driver stays in idle mode, so the motors
are never driven, and the servos are left off unless you pass --servo.

    uv run examples/read_state.py
    uv run examples/read_state.py --duration 30 --rate 5
"""

import argparse
import time

import numpy as np

from allegro_hand_v5 import AllegroHand, DriverConfig, JOINT_NAMES


def main():
    p = argparse.ArgumentParser(description="Read Allegro Hand V5 state")
    p.add_argument("--can", default="can0")
    p.add_argument("--rate", type=float, default=2.0, help="Print rate (Hz)")
    p.add_argument("--duration", type=float, default=None, help="Seconds; default until Ctrl+C")
    p.add_argument("--servo", action="store_true",
                   help="Engage the motors (still zero torque, but the hand stiffens slightly)")
    args = p.parse_args()

    with AllegroHand(args.can, config=DriverConfig()) as hand:
        if not args.servo:
            hand.servo_off()

        info = hand.info
        print("=" * 74)
        print(f"  {info}")
        print("=" * 74)
        print(f"  serial number    : {info.serial_number or '(no reply)'}")
        print(f"  handedness       : {info.handedness}")
        print(f"  hardware type    : {info.hardware_type}"
              f"{'  (geared)' if info.hardware_type == 'B' else ''}")
        print(f"  hardware version : "
              f"{f'0x{info.hardware_version:04X}' if info.hardware_version is not None else '?'}")
        print(f"  firmware version : "
              f"{f'0x{info.firmware_version:04X}' if info.firmware_version is not None else '?'}")
        print(f"  servos           : {'ON' if hand.servo_is_on else 'off'}")
        print(f"  gain profile     : {hand.gains.name}")
        print()

        dt = 1.0 / args.rate
        t0 = time.perf_counter()
        try:
            while args.duration is None or time.perf_counter() - t0 < args.duration:
                s = hand.read()
                st = hand.stats

                print(f"t = {s.t:7.2f} s   mode={s.mode}   servo={'on' if s.servo_on else 'off'}")
                print("  position (deg) " + " ".join(f"{v:7.1f}" for v in np.degrees(s.positions)))
                print("  velocity (d/s) " + " ".join(f"{v:7.1f}" for v in np.degrees(s.velocities)))
                print(f"  pressure (Pa)  " + " ".join(f"{v:9.0f}" for v in s.pressures)
                      + "     (index, middle, ring, thumb)")

                if s.joint_errors:
                    print(f"  ERRORS ({s.error_count} frames seen):")
                    for err in s.joint_errors:
                        print(f"    ! {err}")
                else:
                    print(f"  errors         none"
                          f"{f'  ({s.error_count} seen since start)' if s.error_count else ''}")

                print(f"  loop           {st['rate_hz']:6.1f} Hz, {st['missed_deadlines']} missed, "
                      f"{st['utilisation'] * 100:3.0f}% busy, "
                      f"feedback {st['position_age_ms']:.1f} ms old, "
                      f"{st['rx_frames']} frames in")
                print()
                time.sleep(dt)

        except KeyboardInterrupt:
            print("\nStopped.")

        print("Joint map:")
        for i in range(0, 16, 4):
            print("  " + "  ".join(f"{j:2d}={JOINT_NAMES[j]}" for j in range(i, i + 4)))


if __name__ == "__main__":
    main()
