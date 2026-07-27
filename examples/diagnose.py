#!/usr/bin/env python3
"""
Work out why the hand is not talking. Commands no torque and never servos on.

Goes down the stack one layer at a time — interface, socket, transmit, replies,
position stream — and stops at the first thing that is wrong, with the command
that fixes it.

    uv run examples/diagnose.py
    uv run examples/diagnose.py --can can1
"""

import argparse
import sys
import time

import numpy as np

from allegro_hand_v5 import protocol as proto
from allegro_hand_v5.bus import AllegroCANBus, describe_link, link_status


def section(title):
    print()
    print(title)
    print("-" * len(title))


def main():
    p = argparse.ArgumentParser(description="Diagnose the Allegro Hand CAN link")
    p.add_argument("--can", default="can0")
    p.add_argument("--timeout", type=float, default=2.0, help="Per-step wait (s)")
    args = p.parse_args()
    ch = args.can

    print("=" * 74)
    print(f"  Allegro Hand V5 link diagnosis on {ch}")
    print("=" * 74)

    # ---- 1. the interface itself ----
    section("1. Interface")
    info = link_status(ch)
    if not info:
        print(f"  Could not read {ch}. Does it exist?   ip link show")
        return 1
    print(f"  {describe_link(ch)}")

    state = info.get("state", "")
    fix = (f"sudo ip link set {ch} down && "
           f"sudo ip link set {ch} up type can bitrate 1000000 restart-ms 100")

    if not info.get("up"):
        print(f"\n  STOP: {ch} is down.\n    {fix}")
        return 1
    if state == "BUS-OFF":
        print(f"\n  STOP: the controller is BUS-OFF. It will not transmit anything until\n"
              f"  the link is bounced, and restart-ms={info.get('restart_ms')} means it will\n"
              f"  not recover on its own.\n    {fix}")
        return 1
    if state in ("ERROR-PASSIVE", "ERROR-WARNING"):
        print(f"\n  WARNING: state is {state} with tx_err={info.get('tx_errors')}. Frames are\n"
              "  going unacknowledged, so either the hand is off, it is not wired to this\n"
              "  bus, or it is at a different bitrate. The counters do not reset by\n"
              "  themselves — after fixing the cause, bounce the link:\n"
              f"    {fix}")
    if info.get("bitrate") != 1_000_000:
        print(f"\n  WARNING: bitrate is {info.get('bitrate')}, the hand needs 1000000.")

    # ---- 2. open the socket ----
    section("2. Open the socket")
    bus = AllegroCANBus(ch)
    try:
        bus.open()
        print("  OK, socket open")
    except Exception as e:
        print(f"  FAILED: {e}")
        return 1

    try:
        # ---- 3. can we transmit? ----
        section("3. Transmit (Servo OFF — harmless, and leaves the motors disabled)")
        try:
            bus.servo_off()
            print("  OK, frame accepted by the driver")
        except Exception as e:
            print(f"  FAILED: {e}")
            return 1

        time.sleep(0.2)
        after = link_status(ch)
        tx_before, tx_after = info.get("tx_errors", 0), after.get("tx_errors", 0)
        rx_before, rx_after = info.get("rx_packets", 0), after.get("rx_packets", 0)
        if tx_after > tx_before:
            print(f"  tx error counter rose {tx_before} -> {tx_after}: nothing on the bus\n"
                  "  acknowledged that frame. The hand is off, unplugged, or at another\n"
                  "  bitrate. Everything below will fail for the same reason.")
        elif rx_after > rx_before or tx_after < tx_before:
            print(f"  tx errors {tx_before} -> {tx_after}, rx packets {rx_before} -> {rx_after}:"
                  " the bus is live")
        else:
            print(f"  tx errors steady at {tx_after}, no new rx. Inconclusive — the counter\n"
                  "  saturates near bus-off, so a flat reading is not proof of health.")

        # ---- 4. does the hand answer an RTR? ----
        section("4. Identity (RTR for Information and Serial Number)")
        bus.flush()
        bus.request_info()
        bus.request_serial()
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline and not bus.info.complete:
            bus.poll(timeout=0.005)

        if bus.info.complete:
            print(f"  OK  {bus.info}")
            print(f"      handedness    : {bus.info.handedness}")
            print(f"      hardware type : {bus.info.hardware_type}")
        else:
            print(f"  NO REPLY within {args.timeout}s")
            print(f"      hardware version: {bus.info.hardware_version}")
            print(f"      serial number   : {bus.info.serial_number or '(none)'}")

        # ---- 5. position stream ----
        section("5. Position reporting")
        print("  Engaging the servos — the hand only reports positions with the motor")
        print("  drivers on. No current is commanded, and they go off again below.")
        bus.set_period(position_ms=3)
        bus.servo_on()
        time.sleep(0.05)
        bus.state.reset_position_flags()
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline and not bus.state.all_positions_ready():
            bus.poll(timeout=0.005)

        if bus.state.all_positions_ready():
            print(f"  OK, streaming. All four fingers reported.")
        else:
            print(f"  Stream gave fingers {bus.state.position_flags:04b} in {args.timeout}s; "
                  "trying RTR polling")
            bus.state.reset_position_flags()
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline and not bus.state.all_positions_ready():
                bus.request_positions()
                bus.poll(timeout=0.005)
            if bus.state.all_positions_ready():
                print("  OK via RTR polling. Use DriverConfig(poll_rtr=True) — the hand is\n"
                      "  ignoring the Set Period command (0x081), which is undocumented in\n"
                      "  the V5 manual and may not exist in your firmware.")
            else:
                print(f"  FAILED: fingers {bus.state.position_flags:04b}, no positions at all")
        bus.servo_off()

        if bus.state.position_flags:
            print("  positions (deg): "
                  + " ".join(f"{v:7.1f}" for v in np.degrees(bus.state.positions)))

        # ---- 6. anything else on the bus ----
        section("6. Other traffic")
        bus.poll(timeout=0.05, max_frames=200)
        s = bus.state
        print(f"  fingertip pressure : "
              + (" ".join(f"{v:.0f}" for v in s.pressures) if s.last_pressure_time
                 else "no frames seen (sensor may not be fitted)"))
        print(f"  hand error reports : {s.error_count}"
              + (f"  last: {s.last_error}" if s.last_error else ""))
        print(f"  bus error frames   : {s.can_error_frames}")

        # ---- verdict ----
        section("Verdict")
        if bus.info.complete and bus.state.all_positions_ready():
            print("  The hand is healthy and talking. Run examples/read_state.py.")
            return 0
        print(f"  {describe_link(ch)}")
        print("\n  Most common causes, in order:")
        print("    1. hand not powered, or the power switch is off")
        print("    2. controller stuck in BUS-OFF or ERROR-PASSIVE from an earlier run —")
        print(f"       bounce it: {fix}")
        print("    3. CAN H/L swapped, or missing termination")
        print("    4. wrong bitrate (the hand is 1 Mbps)")
        return 1

    finally:
        bus.close()
        print()


if __name__ == "__main__":
    sys.exit(main())
