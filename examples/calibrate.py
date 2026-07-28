#!/usr/bin/env python3
"""
Interactive joint calibration.

The hand is left limp — servos engaged so the encoders report, zero current
commanded — so you can move every joint by hand while this watches. It records
the travel of each joint, lets you set a homing offset, and writes the result to
`calibration_data/<serial>.json`, which the driver then loads automatically.

    uv run examples/calibrate.py
    uv run examples/calibrate.py --fresh          # ignore the existing file
    uv run examples/calibrate.py --output /tmp/my_hand.json

Move each joint slowly through its whole range, both ways, then press `s`.
"""

import argparse
import select
import sys
import termios
import time
import tty
from pathlib import Path

import numpy as np

from allegro_hand_v5 import AllegroHand, DriverConfig, JOINT_NAMES, NUM_JOINTS
from allegro_hand_v5.calibration import Calibration

NUDGE = 0.01  # radians per +/- keypress

HELP = """\
  arrows / j k  select joint      z  zero selected joint here      s  save
  + -           offset +/-0.01    Z  clear selected offset         q  quit
  r             re-record ranges  A  clear every offset            ?  help
"""


class RawKeys:
    """Read single keypresses without waiting for Enter."""

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
        return False

    def get(self):
        """The next key, or None if nothing is waiting. Arrows come back as ^ and v."""
        if not select.select([sys.stdin], [], [], 0)[0]:
            return None
        key = sys.stdin.read(1)
        if key == "\x1b" and select.select([sys.stdin], [], [], 0.01)[0]:
            return {"A": "^", "B": "v"}.get(sys.stdin.read(2)[-1], "")
        return key


def render(cal, raw, selected, state, message):
    """The whole screen, as one string."""
    out = [
        "\033[H\033[J",  # home, clear
        f"Allegro Hand V5 calibration — {cal.serial}   "
        f"{state.rate:.0f} Hz   servo {'on' if state.servo_on else 'OFF'}\n",
        "All values in radians. Position is what the encoder reads plus the offset.\n",
        f"\n{'':2}{'joint':<13}{'raw':>9}{'offset':>9}{'position':>10}"
        f"{'min':>9}{'max':>9}{'span':>8}{'deg':>8}\n",
        "  " + "-" * 75 + "\n",
    ]
    for i, name in enumerate(JOINT_NAMES):
        span = cal.upper[i] - cal.lower[i]
        out.append(
            f"{'>' if i == selected else ' '} {name:<13}"
            f"{raw[i]:>9.3f}{cal.offset[i]:>9.3f}{raw[i] + cal.offset[i]:>10.3f}"
            f"{cal.lower[i]:>9.3f}{cal.upper[i]:>9.3f}{span:>8.3f}"
            f"{np.degrees(raw[i] + cal.offset[i]):>8.1f}\n"
        )
    errors = state.joint_errors
    out.append(
        f"\n  pressure (Pa)  {'  '.join(f'{v:5.0f}' for v in state.pressures)}"
        f"     errors: {'; '.join(str(e) for e in errors) if errors else 'none'}\n"
    )
    out.append("\n" + HELP)
    out.append(f"\n  {message}\n")
    return "".join(out)


def main():
    p = argparse.ArgumentParser(description="Interactive Allegro Hand V5 calibration")
    p.add_argument("--can", default="can0")
    p.add_argument("--fresh", action="store_true",
                   help="Start the ranges from scratch instead of extending the saved ones")
    p.add_argument("--output", type=Path,
                   help="Where to save; default is calibration_data/<serial>.json")
    p.add_argument("--rate", type=float, default=20.0, help="Screen refresh rate (Hz)")
    args = p.parse_args()

    # calibration=None: this tool owns the offsets, so the driver reports raw
    # encoder angles and clips nothing.
    with AllegroHand(args.can, calibration=None, config=DriverConfig()) as hand:
        print(hand.info)
        if not hand.serial_number and not args.output:
            sys.exit("The hand did not report a serial number; pass --output.")

        cal = Calibration.for_serial(hand.serial_number)
        raw = hand.positions
        if args.fresh or cal.path is None:
            cal.min[:], cal.max[:] = raw, raw
            message = "New calibration. Move every joint through its full range."
        else:
            message = f"Loaded {cal.path}. Ranges will only widen; press r to re-record."

        selected = 0
        dt = 1.0 / args.rate
        with RawKeys() as keys:
            while True:
                state = hand.read()
                raw = state.positions
                cal.min = np.minimum(cal.min, raw)
                cal.max = np.maximum(cal.max, raw)

                sys.stdout.write(render(cal, raw, selected, state, message))
                sys.stdout.flush()

                key = keys.get()
                if key in ("q", "\x03"):
                    print("\nQuit without saving.")
                    return
                elif key == "s":
                    cal.date = time.strftime("%Y-%m-%d")
                    path = cal.save(args.output)
                    print(f"\nSaved {path}\n")
                    print(cal.summary())
                    return
                elif key in ("^", "k"):
                    selected = (selected - 1) % NUM_JOINTS
                elif key in ("v", "j"):
                    selected = (selected + 1) % NUM_JOINTS
                elif key == "z":
                    # Homing: make this joint read zero where it is standing now.
                    cal.offset[selected] = -raw[selected]
                    message = f"{JOINT_NAMES[selected]} zeroed at {raw[selected]:+.3f} rad"
                elif key == "Z":
                    cal.offset[selected] = 0.0
                    message = f"{JOINT_NAMES[selected]} offset cleared"
                elif key in ("+", "="):
                    cal.offset[selected] += NUDGE
                    message = f"{JOINT_NAMES[selected]} offset {cal.offset[selected]:+.3f} rad"
                elif key in ("-", "_"):
                    cal.offset[selected] -= NUDGE
                    message = f"{JOINT_NAMES[selected]} offset {cal.offset[selected]:+.3f} rad"
                elif key == "A":
                    cal.offset[:] = 0.0
                    message = "All offsets cleared"
                elif key == "r":
                    cal.min[:], cal.max[:] = raw, raw
                    message = "Ranges reset. Move every joint through its full range."
                elif key == "?":
                    message = "Move joints by hand; the range is recorded as you go."

                time.sleep(dt)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # Ctrl+C survives cbreak mode, so it lands here rather than as a keypress.
        print("\nQuit without saving.")
