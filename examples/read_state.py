#!/usr/bin/env python3
"""
Live view of everything the hand reports. Commands no current at all.

Redraws a table of all sixteen joints — position, velocity, commanded current,
and where each joint sits inside its calibrated travel — plus fingertip
pressure, motor errors and control-loop health. Safe to run at any time: the
driver stays in idle mode, so zero current is commanded throughout.

    uv run examples/read_state.py
    uv run examples/read_state.py --rate 20 --temperature
    uv run examples/read_state.py --duration 30 > log.txt

Piping to a file falls back to plain scrolling frames, with no escape codes.
"""

import argparse
import sys
import time

import numpy as np

from allegro_hand_v5 import AllegroHand, DriverConfig, FINGER_NAMES, JOINT_NAMES

BAR_WIDTH = 13


class Style:
    """ANSI attributes, or nothing at all when the output is not a terminal."""

    CODES = {"bold": "1", "dim": "2", "red": "31", "green": "32",
             "yellow": "33", "blue": "34", "cyan": "36"}

    def __init__(self, enabled: bool):
        self.enabled = enabled

    def __call__(self, text, *attrs) -> str:
        attrs = [a for a in attrs if a]  # so `"red" if bad else None` reads well
        if not self.enabled or not attrs:
            return text
        codes = ";".join(self.CODES[a] for a in attrs)
        return f"\033[{codes}m{text}\033[0m"


def travel_bar(fraction: float, style: Style) -> str:
    """Where a joint sits between its calibrated limits, as `[...|.....]`."""
    if not np.isfinite(fraction):
        return "[" + " " * BAR_WIDTH + "]"
    clamped = min(max(fraction, 0.0), 1.0)
    at_limit = not 0.02 < fraction < 0.98
    marker = int(round(clamped * (BAR_WIDTH - 1)))
    track = "".join("|" if i == marker else "." for i in range(BAR_WIDTH))
    return "[" + style(track, "yellow" if at_limit else "dim") + "]"


def render(hand, state, style: Style, temperature: bool) -> str:
    info = hand.info
    cal = hand.calibration
    lower, upper = np.degrees(cal.lower), np.degrees(cal.upper)
    fractions = cal.normalize(state.positions)
    positions, velocities = np.degrees(state.positions), np.degrees(state.velocities)
    max_current = hand.gains.max_current

    stale = state.position_age > 0.05
    lines = [
        style(f"Allegro Hand V5  {info.serial_number or '?'}", "bold")
        + style(f"   {info.handedness} hand, type {info.hardware_type}"
                f"{' (Plus)' if info.hardware_type == 'B' else ''}"
                f"   {hand.channel}", "dim"),
        style("  mode ", "dim") + style(f"{state.mode:<9}", "cyan")
        + style("servo ", "dim")
        + style("on " if state.servo_on else "off", "green" if state.servo_on else "yellow")
        + style("   loop ", "dim") + f"{state.rate:5.0f} Hz"
        + style("   feedback ", "dim")
        + style(f"{state.position_age * 1000:5.1f} ms", "red" if stale else "green")
        + style(f"   gains {hand.gains.name}   t {state.t:7.1f} s", "dim"),
        "",
        style(f"  {'#':>2}  {'joint':<12}{'pos':>8}{'vel':>9}{'current':>9}"
              + f"{'temp':>7}" * temperature
              + f"    {'min':>7} {'travel':^{BAR_WIDTH + 2}} {'max':>7}", "dim"),
        style(f"  {'':2}  {'':12}{'[deg]':>8}{'[deg/s]':>9}{'[mA]':>9}"
              + f"{'[degC]':>7}" * temperature
              + f"    {'[deg]':>7} {'':^{BAR_WIDTH + 2}} {'[deg]':>7}", "dim"),
    ]

    for i, name in enumerate(JOINT_NAMES):
        if i % 4 == 0:
            lines.append("")
        saturated = abs(state.currents[i]) >= 0.99 * max_current[i] > 0
        errored = bool(state.errors[i])
        hot = state.temperatures[i] >= 60
        lines.append(
            f"  {style(f'{i:>2}', 'red' if errored else 'dim')}  "
            f"{style(f'{name:<12}', 'bold' if errored else None)}"
            f"{positions[i]:>8.1f}{velocities[i]:>9.1f}"
            f"{style(f'{state.currents[i]:>9.1f}', 'yellow' if saturated else None)}"
            + style(f"{state.temperatures[i]:>7.0f}", "red" if hot else None) * temperature
            + f"    {lower[i]:>7.1f} {travel_bar(fractions[i], style)} {upper[i]:>7.1f}"
        )

    lines.append("")
    lines.append(style("  fingertip [Pa]  ", "dim") + "   ".join(
        f"{style(finger, 'dim')} {state.pressures[f]:>5.0f}"
        for f, finger in enumerate(FINGER_NAMES)))

    errors = state.joint_errors
    lines.append(style("  errors          ", "dim") + (
        style("none", "green") if not errors
        else style("; ".join(str(e) for e in errors), "red")))
    lines.append("")
    lines.append(style("  Ctrl+C to stop.", "dim"))
    return "\n".join(lines)


def main():
    p = argparse.ArgumentParser(description="Live view of Allegro Hand V5 state")
    p.add_argument("--can", default="can0")
    p.add_argument("--rate", type=float, default=10.0, help="Refresh rate (Hz)")
    p.add_argument("--duration", type=float, default=None, help="Seconds; default until Ctrl+C")
    p.add_argument("--temperature", action="store_true", help="Also stream motor temperatures")
    p.add_argument("--plain", action="store_true", help="No colour and no redrawing")
    args = p.parse_args()

    live = sys.stdout.isatty() and not args.plain
    style = Style(live)

    config = DriverConfig(temperature_period_ms=100 if args.temperature else 0)
    with AllegroHand(args.can, config=config) as hand:
        print(f"{hand.info}\ncalibration: {hand.calibration.path or 'nominal limits'}")
        time.sleep(0.2)

        t0 = time.perf_counter()
        try:
            while args.duration is None or time.perf_counter() - t0 < args.duration:
                frame = render(hand, hand.read(), style, args.temperature)
                sys.stdout.write(("\033[H\033[J" if live else "\n") + frame + "\n")
                sys.stdout.flush()
                time.sleep(1.0 / args.rate)
        except KeyboardInterrupt:
            pass
    print("\nStopped.")


if __name__ == "__main__":
    main()
