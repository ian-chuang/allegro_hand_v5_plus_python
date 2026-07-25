# allegro-hand-v5

Python driver for the **Allegro Hand V5 (F4) / (F4) Plus** (WONIK ROBOTICS),
aimed at sim-to-real / RL deployment. No ROS.

The V5 hardware is **torque-only** over CAN — the only actuation command is
`Set Torque` (a signed current/PWM value per joint). WONIK's precompiled
`libBHand.so` supplies the built-in motions (home, grasps, gravity
compensation, joint PD); it is bundled in `src/allegro_hand_v5/lib/`.

## Layout

| Module | Responsibility |
|--------|----------------|
| `can_driver` | `AllegroCANDriver`: SocketCAN transport, handshake, frame encode/decode. |
| `bhand` | ctypes binding to `libBHand.so` (mangled C++ symbols). |
| `control_loop` | `ControlLoop`: the 500 Hz RX → BHand → TX cycle, plus `EmergencyStop`. |
| `calibration` | `HandCalibration`: measured per-joint travel, as editable JSON. |
| `hand` | `AllegroHand`: owns the driver, BHand, and loop. |

### Two writers on the bus

The control thread transmits BHand's torques every cycle. `hand.set_torques()`
transmits from the **calling** thread, immediately — it bypasses BHand's output
but not its loop, so while a motion type other than `off()` is active the two
streams interleave on the bus. This is deliberate: it is exactly what the
reference stack does, and the built-in motions' behaviour depends on it. Call
`hand.off()` first if you want your torque commands to be the only ones
reaching the hand.

## Install

```bash
uv sync
```

Requires a CAN interface at 1 Mbps. On Linux with SocketCAN:

```bash
sudo ip link set can0 up type can bitrate 1000000
```

## Quick start

```python
import numpy as np
from allegro_hand_v5 import AllegroHand

with AllegroHand(hand_type="right", hardware_type="B",
                 can_channel="can0", calibration=True) as hand:
    hand.home()                  # BHand motions: home / grasp_3 / pinch_it /
    hand.gravity_comp()          #   envelop / gravity_comp / off
    hand.move_to(q_des)          # BHand joint-space PD

    q  = hand.get_positions()    # (16,) rad
    dq = hand.get_velocities()   # (16,) rad/s

    with hand.torque_mode():     # returns to gravity comp on exit
        hand.set_torques(np.zeros(16))
```

`AllegroHand` connects on `__enter__` and, on `__exit__`, sets the motion type
to `off()`, stops the loop, drops the servos, and closes the bus.

## Example

```bash
# Hold the index MCP at 40% of its calibrated range
uv run examples/single_motor_pd_control.py --joint 1 --target 0.4

# Sweep the index PIP between 20% and 60% every 3 s
uv run examples/single_motor_pd_control.py --joint 2 --target 0.2 --sweep-to 0.6 --period 3.0
```

The hand homes first (`--no-home` to skip), then the PD law runs on the chosen
joint while the rest are commanded zero torque.

## Calibration

`calibration/right.json` holds the measured `min`/`max` of each joint in
radians. Joints absent from the file fall back to the nominal URDF ranges in
`calibration.DEFAULT_RANGES`. Targets are written as fractions of that measured
range, so a pose is portable across hands:

```python
cal = HandCalibration.load(hand_type="right")
cal.limits(3)          # (-0.100, 1.837) rad
cal.joint(3, 0.4)      # 0.675 rad — 40% of joint 3's travel
cal.from_fractions(f)  # (16,) fractions -> (16,) rad, clipped to the range
```

## Units

`set_torques()` takes the same units as BHand's own output: each value is
scaled by `1.43 * 1000` into raw current units and clamped to ±240 before
transmission. Positions decode at `(pi/180) * 0.088` rad per encoder LSB.
Velocities are a filtered finite difference (α = 0.3) computed in the loop.
