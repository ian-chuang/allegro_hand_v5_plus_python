# allegro-hand-v5

Python driver and control library for the **Allegro Hand V5 (F4) / (F4) Plus**
(WONIK ROBOTICS), aimed at sim-to-real / RL deployment.

The V5 hardware is **torque-only** over CAN — the only actuation command is `Set Torque`
(motor current in mA). Position control is closed on the host. `AllegroHand` runs that
control loop (CAN RX + PD + CAN TX) in a **separate process** with its own GIL, so nothing
the parent (RL inference, plotting, logging) does can stall real-time control. The parent
talks to it through shared memory.

## Modules

| Module | Responsibility |
|--------|----------------|
| `protocol` | Pure CAN frame encode/decode. No I/O — fully unit-tested. |
| `driver` | `AllegroHand`: the process-based real-time driver (position + torque control, state). |
| `config` | `DriverConfig`: torque→current scaling, per-joint current limits, PID gains. |
| `model` | Handedness/type + per-config joint limits (right/left × A/B constants). |
| `constants` | Verified hardware constants. |

## Install

Install straight from GitHub:

```bash
pip install git+https://github.com/ian-chuang/allegro_hand_v5_plus_python.git
```

or with uv:

```bash
uv pip install git+https://github.com/ian-chuang/allegro_hand_v5_plus_python.git
# or add it to a project:
uv add git+https://github.com/ian-chuang/allegro_hand_v5_plus_python.git
```

To develop locally, clone the repo and run `uv sync`.

Requires a CAN interface. On Linux with a PEAK PCAN-USB via SocketCAN:

```bash
sudo ip link set can0 up type can bitrate 1000000
```

(Or use `interface="pcan"` to let python-can set the bitrate itself.)

## Quick start

```python
from allegro_hand_v5 import AllegroHand, DriverConfig

with AllegroHand(channel="can0", config=DriverConfig.safe(0.03)) as hand:
    print(hand.model.handedness, hand.model.hand_type, hand.model.is_plus)

    hand.set_target(q_des)        # position mode: PD (in the child process) chases this
    q  = hand.positions           # (16,) rad
    qd = hand.velocities          # (16,) rad/s, filtered finite-diff
    p  = hand.fingertip_pressures # (4,) Pa  [index, middle, ring, thumb]
    print(hand.loop_rate_hz)      # measured child control rate — a health check
```

Torque policies bypass the PD:

```python
hand.set_torque(tau_nm)   # 16-vector Nm -> mA, clamped, Plus-adjusted
hand.relax()              # 0 mA everywhere: limp/backdrivable, still streaming
```

`connect()` / `disconnect()` are the non-context-manager form; `connect()` spawns the
control process and returns the `HandModel`. The child servos on (required for streaming)
and holds the current pose until you set a target.

## Configuration

Everything tunable lives in `DriverConfig` — command semantics, torque→command scaling,
per-joint limits, and PID gains:

```python
from allegro_hand_v5 import AllegroHand, DriverConfig

cfg = DriverConfig()
cfg.set_max_torque(0.05)          # per-joint ceiling in Nm (or a length-16 array)
cfg.set_joint_max_current(1, 60)  # or clamp a single joint directly in mA
cfg.kp[:] *= 0.8                  # soften the P gains
hand = AllegroHand(channel="can0", config=cfg)

# First-run safety preset: tiny ceiling + softened gains — can't push hard.
hand = AllegroHand(channel="can0", config=DriverConfig.safe(max_torque_nm=0.02))
```

The V5 is **current-controlled**: the wire value is a motor-current setpoint in mA, so
desired torque is converted via `nm_to_ma` (≈1.43e3) and clamped per joint by
`max_current_ma`. See [`docs/driver_constants.md`](docs/driver_constants.md#current-vs-pwm-resolved).

## Examples

- `read_joints.py` — read-only (0 mA, backdrivable); inspect encoder readings by hand.
- `safe_move.py` — ramp to a pose under tiny torque limits, with a live pos/cmd/err/tau/mA table and the child loop rate.
- `rl_deploy_loop.py` — sim-to-real skeleton: policy sets targets while the child closes the loop.

## Joint order (canonical 16-vector)

```
0..3   index : MCP-1(spread), MCP-2, PIP, DIP
4..7   middle
8..11  ring
12..15 thumb : CMC-1, CMC-2, MP, IP
```

## Hardware facts baked in

- CAN 2.0 @ 1 Mbps, 11-bit IDs, `arbitration_id = msg_id << 2`.
- Position: `angle_rad = raw_int16 * (π/180) * 0.088` (no offset).
- Torque: `mA = Nm * 1.43e3`, clamped to the config's per-joint ceiling (firmware hard
  limit ±240 mA); "Plus" hands halve MCP-2 of index/middle/ring (joints 1/5/9).
- Handedness/type read from the serial reply and used to pick the joint-limit set
  (`constants.JOINT_LIMITS["right_B"]` etc.; `right_B` is measured on a real Plus).
- Streaming requires servo-on; enabled via `SET_PERIOD` (default 3 ms position stream).
  Fingertip pressures arrive at `0xF0`/`0xF2` on current firmware (manual says 0x50/0x52).

Full protocol reference: [`docs/driver_constants.md`](docs/driver_constants.md) and the
transcribed manual [`docs/allegro_hand_v5_manual.md`](docs/allegro_hand_v5_manual.md).

## Not yet verified on hardware

- `SET_POSE` (0xE0+) — an on-board position command present in firmware but undocumented;
  if it works it could replace the host PID. Currently exposed only in `protocol` as experimental.
- Per-joint torque **sign** for left hands (the manual defines mirrored `+` directions).
- Exact Nm→mA constant across hand variants.

## Tests

```bash
uv run pytest -q
```

Protocol, config, model, and driver-plumbing paths are covered without hardware.
