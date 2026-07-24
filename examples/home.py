"""SAFE move-to-pose demo with live debug logging.

Ramps every joint to ``--pos`` deg (thumb roll joints overridden: ``thumb_cmc1`` and
``thumb_cmc2``), holds, then returns home — all under a tiny per-joint torque ceiling so
nothing can push hard. Control runs in the driver's separate process; this script only
sets targets and logs.

Live table per joint:
    pos    - measured joint angle [deg]           (same source as read_joints.py)
    cmd    - the PD position target [deg]
    err    - cmd - pos [deg]
    tau    - PD output [Nm]
    mA     - motor-current command sent [mA]; '*' = saturated at the ceiling

The header shows the child's measured control-loop rate. Use the table to tell whether a
joint that misses its target is torque-saturated (raise --max-torque / needs gravity comp)
vs. driven the wrong way (err grows -> sign flip).

Run:
    uv run python examples/safe_move.py --channel can0
"""

import argparse
import time

import numpy as np

from allegro_hand_v5 import AllegroHand, DriverConfig
from allegro_hand_v5 import constants as C

THUMB_CMC1 = C.JOINT_NAMES.index("thumb_cmc1")  # 12
THUMB_CMC2 = C.JOINT_NAMES.index("thumb_cmc2")  # 13


def smoothstep(x: float) -> float:
    x = min(1.0, max(0.0, x))
    return x * x * (3 - 2 * x)


def ramped_target(t, home, goal, ramp, hold):
    """Target pose at elapsed time t (ramp-up / hold / ramp-down)."""
    if t < ramp:
        return home + smoothstep(t / ramp) * (goal - home), "ramp-up"
    if t < ramp + hold:
        return goal.copy(), "hold"
    if t < 2 * ramp + hold:
        return goal + smoothstep((t - ramp - hold) / ramp) * (home - goal), "ramp-down"
    return home.copy(), "done"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--max-torque", type=float, default=0.05,
                    help="per-joint torque ceiling [Nm]; keep tiny for first runs")
    ap.add_argument("--pos", type=float, default=0.0,
                    help="target angle for all joints [deg] (thumb roll joints overridden)")
    ap.add_argument("--thumb-cmc1", type=float, default=3.5, help="thumb CMC-1 target [deg]")
    ap.add_argument("--thumb-cmc2", type=float, default=140.0, help="thumb CMC-2 target [deg]")
    ap.add_argument("--ramp", type=float, default=3.0, help="ramp time each way [s]")
    ap.add_argument("--hold", type=float, default=300.0, help="hold time at target [s]")
    ap.add_argument("--log-hz", type=float, default=10.0, help="table refresh rate")
    args = ap.parse_args()

    target_deg = np.full(C.DOF, args.pos)
    target_deg[THUMB_CMC1] = args.thumb_cmc1
    target_deg[THUMB_CMC2] = args.thumb_cmc2
    goal = np.radians(target_deg)

    cfg = DriverConfig.safe(max_torque_nm=args.max_torque)
    print(f"Per-joint torque ceiling: {args.max_torque} Nm (≈ {float(cfg.max_current_ma[1]):.0f} mA)")

    with AllegroHand(channel=args.channel, interface=args.interface, config=cfg) as hand:
        m = hand.model
        print(f"Connected: {m.handedness} type {m.hand_type} (plus={m.is_plus})")

        goal = m.clamp_positions(goal)
        home = hand.positions.copy()
        print("home  (deg):", np.array2string(np.degrees(home), precision=1,
                                               separator=",", max_line_width=220))
        print("target(deg):", np.array2string(np.degrees(goal), precision=1,
                                               separator=",", max_line_width=220))

        total = 2 * args.ramp + args.hold
        dt = 1.0 / args.log_hz
        printed = 0
        t0 = time.monotonic()
        try:
            while (t := time.monotonic() - t0) < total:
                q_target, phase = ramped_target(t, home, goal, args.ramp, args.hold)
                hand.set_target(q_target)

                pos = np.degrees(hand.positions)
                cmd = np.degrees(hand.target)
                err = cmd - pos
                tau = hand.last_torque
                ma = hand.config.torque_to_current(tau, is_plus=m.is_plus)

                lines = [f"t={t:6.1f}s  phase={phase:9s}  loop={hand.loop_rate_hz:6.1f}Hz  "
                         f"max|err|={np.max(np.abs(err)):6.1f}deg  ceiling=±{float(cfg.max_current_ma[1]):.0f}mA"]
                lines.append(f"{'idx':>3} {'joint':<12} {'pos°':>8} {'cmd°':>8} "
                             f"{'err°':>8} {'tau(Nm)':>9} {'mA':>7} sat")
                for i, name in enumerate(C.JOINT_NAMES):
                    sat = "*" if abs(ma[i]) >= float(cfg.max_current_ma[i]) - 1e-6 else " "
                    lines.append(f"{i:>3} {name:<12} {pos[i]:>8.1f} {cmd[i]:>8.1f} "
                                 f"{err[i]:>8.1f} {tau[i]:>9.4f} {ma[i]:>7.1f}  {sat}")
                    if i % 4 == 3 and i != C.DOF - 1:
                        lines.append("-" * 60)

                out = "\n".join(lines)
                if printed:
                    print(f"\033[{printed}A", end="")
                print(out + "\033[K")
                printed = out.count("\n") + 1
                time.sleep(dt)
        except KeyboardInterrupt:
            print("\ninterrupted — releasing torque")
            hand.relax()  # zero current now; __exit__ then torques off + servos off
        print("done.")


if __name__ == "__main__":
    main()
