#!/usr/bin/env python3
"""
Work out why the hand is not talking. Commands no current at any point.

Walks down the stack — interface, socket, transmit, identity, position stream —
and stops at the first broken thing, with the command that fixes it.

    uv run examples/diagnose.py
    uv run examples/diagnose.py --can can1
"""

import argparse
import sys
import time

import numpy as np

from allegro_hand_v5 import AllegroCANBus, describe_link, link_status


def section(title):
    print(f"\n{title}\n{'-' * len(title)}")


def main():
    p = argparse.ArgumentParser(description="Diagnose the Allegro Hand CAN link")
    p.add_argument("--can", default="can0")
    p.add_argument("--timeout", type=float, default=2.0, help="Per-step wait (s)")
    args = p.parse_args()
    ch = args.can
    bounce = (f"sudo ip link set {ch} down && "
              f"sudo ip link set {ch} up type can bitrate 1000000 restart-ms 100")

    print("=" * 74)
    print(f"  Allegro Hand V5 link diagnosis on {ch}")
    print("=" * 74)

    section("1. Interface")
    info = link_status(ch)
    if not info:
        print(f"  Could not read {ch}. Does it exist?   ip link show")
        return 1
    print(f"  {describe_link(ch)}")
    if not info.get("up") or info.get("state") == "BUS-OFF":
        print(f"\n  STOP. Nothing can work until the link is healthy:\n    {bounce}")
        return 1

    section("2. Open the socket")
    bus = AllegroCANBus(ch)
    try:
        bus.open()
        print("  OK, socket open")
    except Exception as e:
        print(f"  FAILED: {e}")
        return 1

    try:
        section("3. Transmit (Servo OFF — harmless, and leaves the motors disabled)")
        bus.servo_off()
        time.sleep(0.2)
        after = link_status(ch)
        before_tx, after_tx = info.get("tx_errors", 0), after.get("tx_errors", 0)
        if after_tx > before_tx:
            print(f"  tx error counter rose {before_tx} -> {after_tx}: nothing on the bus\n"
                  "  acknowledged that frame. The hand is off, unplugged, or at another\n"
                  "  bitrate. Everything below will fail for the same reason.")
        else:
            print(f"  OK, frame accepted (tx errors steady at {after_tx})")

        section("4. Identity (RTR for Information and Serial Number)")
        bus.flush()
        bus.request_info()
        bus.request_serial()
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline and not bus.info.complete:
            bus.poll(timeout=0.005)
        if bus.info.complete:
            print(f"  OK  {bus.info}")
        else:
            print(f"  NO REPLY within {args.timeout}s")

        section("5. Position stream")
        print("  Engaging the servos: the hand only reports positions with the motor")
        print("  drivers on. No current is commanded, and they go off again below.")
        bus.set_period(position_ms=3)
        bus.servo_on()
        bus.state.clear_position_flags()
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline and not bus.state.positions_fresh:
            bus.poll(timeout=0.005)
        streaming = bus.state.positions_fresh

        if streaming:
            print("  OK, streaming. All four fingers reported.")
        else:
            print(f"  Stream gave fingers {bus.state.position_flags:04b}; trying RTR polling")
            bus.state.clear_position_flags()
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline and not bus.state.positions_fresh:
                bus.request_positions()
                bus.poll(timeout=0.005)
            if bus.state.positions_fresh:
                print("  Positions answer an RTR but do not stream: the hand is ignoring\n"
                      "  Set Period (0x081), which the V5 manual does not document. The\n"
                      "  driver needs the stream — check your firmware version.")
            else:
                print("  FAILED: no positions at all")
        bus.servo_off()

        if bus.state.position_flags:
            print("  positions (deg): "
                  + " ".join(f"{v:7.1f}" for v in np.degrees(bus.state.positions)))

        section("6. Other traffic")
        bus.poll(timeout=0.05, max_frames=200)
        s = bus.state
        print(f"  fingertip pressure : {' '.join(f'{v:.0f}' for v in s.pressures)}")
        print(f"  hand error reports : {s.error_count}"
              + (f"  last: {s.last_error}" if s.last_error else ""))
        print(f"  bus error frames   : {s.can_error_frames}")

        section("Verdict")
        if bus.info.complete and streaming:
            print("  The hand is healthy and talking. Try examples/read_state.py.")
            return 0
        print(f"  {describe_link(ch)}\n\n  Most common causes, in order:")
        print("    1. hand not powered, or the power switch is off")
        print(f"    2. controller stuck in BUS-OFF from an earlier run: {bounce}")
        print("    3. CAN H/L swapped, or missing termination")
        print("    4. wrong bitrate (the hand is 1 Mbps)")
        return 1
    finally:
        bus.close()
        print()


if __name__ == "__main__":
    sys.exit(main())
