# allegro-hand-v5

Pure-Python driver for the **Allegro Hand V5 (F4) / (F4) Plus** (WONIK ROBOTICS),
aimed at sim-to-real / RL deployment. CAN only — no ROS, no `libBHand.so`.

The hand is current-controlled hardware: the only actuation command it accepts is
a signed motor current per joint. So this driver commands current, and position
control is a plain PD **in current units** closed on the host, running in its own
process so nothing your script does can stall it.

```python
from allegro_hand_v5 import AllegroHand, COMPLIANT

with AllegroHand("can0", gains=COMPLIANT) as hand:
    print(hand.info)                          # serial, handedness, hw/fw version
    hand.set_position(hand.calibration.center)
    print(hand.positions, hand.pressures, hand.errors)
```

## Install

```bash
uv sync
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100
```

Requires Linux (SocketCAN) and a 1 Mbps CAN interface. A PEAK PCAN-USB works;
pass `interface="pcan"` to let python-can set the bitrate itself.

**Use `restart-ms 100`.** Without it the controller latches BUS-OFF and never
recovers. Every frame the host sends while the hand is powered off goes
unacknowledged and bumps the transmit error counter; at 256 the interface stops
transmitting entirely, and stays that way across runs until the link is bounced
— which looks exactly like "it worked yesterday and now nothing responds".

When something goes wrong, start here:

```bash
uv run examples/diagnose.py
```

It walks the stack one layer at a time — interface, socket, transmit, identity,
position stream — and stops at the first broken thing with the command that
fixes it. It never commands any current.

## Layout

| Module | Responsibility |
|--------|----------------|
| `protocol` | Message IDs, encode/decode, error flags. Pure functions, no I/O. |
| `bus` | `AllegroCANBus`: the socket, every CAN command, and frame decoding. |
| `driver` | `AllegroHand`: the control process and the shared-memory API. **Start here.** |
| `gains` | `Gains` and five presets. |
| `calibration` | `Calibration`: measured travel plus a homing offset, as editable JSON. |

## The control law

There is nothing between your gains and the wire — no torque model, no hidden
rescaling:

```
dq      = a * (q - q_prev) / dt + (1 - a) * dq      # EMA, a = 0.3 by default
current = kp * (q_desired - q) - kd * dq            # position mode
current = i_desired                                 # current mode
current = 0                                         # idle
send(clip(current, -max_current, +max_current))
```

`kp` is **mA per radian**, `kd` is **mA per rad/s**, `max_current` is **mA**. The
velocity EMA is the only filter anywhere in the loop.

One control update happens per complete set of four finger position reports,
which is how WONIK's own driver does it: the loop is driven by the hand's
stream rather than by a timer that would recompute PD on positions that have not
changed. With the default 3 ms report period that is ~333 Hz.

## API

```python
hand = AllegroHand("can0", gains="compliant", calibration=True)

# lifecycle — start()/close(), or just use it as a context manager
hand.start(); hand.close()
hand.servo_on(); hand.servo_off()

# commands
hand.set_position(q_rad)          # PD mode; returns the target after clipping
hand.set_current(ma)              # bypass the PD, command current directly
hand.hold()                       # hold the measured pose
hand.relax()                      # zero current, servos still engaged

# gains, live
hand.set_preset("soft")
hand.set_gains(kp=700, kd=30)     # scalars broadcast; (16,) arrays also fine

# state
hand.info                         # serial, handedness, hardware type, versions
hand.positions                    # (16,) rad, homing offset applied
hand.raw_positions                # (16,) rad, straight off the encoders
hand.velocities                   # (16,) rad/s, filtered
hand.currents                     # (16,) mA, as sent
hand.pressures                    # (4,) Pa — index, middle, ring, thumb
hand.temperatures                 # (16,) deg C, with temperature_period_ms set
hand.errors                       # [JointError], empty when healthy
hand.mode                         # "idle" | "position" | "current"
hand.read()                       # all of it, plus loop rate and feedback age
```

`read()` and every property go through a seqlock, so a 16-vector is never a mix
of two control cycles.

Anything the CAN protocol offers that the driver does not surface is one level
down, on `AllegroCANBus`: `pick()`, `place()`, `start_motor_calibration()`,
`send_pose()`, `request_imu()`, `request_temperatures()`, `set_period()`.

```python
from allegro_hand_v5 import AllegroCANBus

with AllegroCANBus("can0") as bus:
    bus.handshake()
    bus.request_temperatures()
    bus.poll(timeout=0.01)
    print(bus.state.temperatures)
```

## Gains

Five presets, written out as plain per-joint numbers rather than computed:

| Preset | kp (finger / MCP-2 / thumb) | kd | max current | Use |
|--------|------------------------------|----|-------------|-----|
| `default` | 1400 / 700 / 1150 | 45, 70, 45, 45 | 150 mA (75 on MCP-2) | libBHand's home gains, converted. |
| `compliant` | 700 / 350 / 575 | 32, 50, 32, 32 | same | Backdrivable by hand, still holds a pose. |
| `soft` | 350 / 175 / 290 | 22, 35, 22, 22 | same | Very backdrivable; expect gravity sag. |
| `safe` | same as `soft` | same as `soft` | 50 mA (25 on MCP-2) | First power-on, or a hand you don't trust. |
| `zero` | 0 | 0 | — | Limp in position mode. |

Two per-joint asymmetries, both deliberate:

- **MCP-2 (joints 1, 5, 9) carries half the numbers of its neighbours.** A "Plus"
  (type B) hand gears those joints 2:1, so half the current is the same joint
  torque. WONIK's driver halves the current silently; here it lives in the gains,
  where you can see it. **On a non-geared type A hand, double those three
  entries.**
- **MCP-2 also carries ~3× the damping** of everything else. That is libBHand's
  own choice, reproduced.

```python
from allegro_hand_v5 import Gains, DEFAULT, preset

preset("compliant")
DEFAULT.replace(max_current=100)
Gains(kp=800, kd=30, max_current=120, name="mine")   # scalars broadcast to 16
```

Every value is capped at the hardware limit of **240 mA** on the way out, the
same saturation WONIK's driver applies.

There is **no gravity compensation**, so a soft preset will sag. If you need it,
take the model from your simulator's URDF and add a feed-forward current —
libBHand's own gravity term peaks at only 52 of the 240 available mA and ignores
palm orientation entirely. See [docs/bhand_gains.md](docs/bhand_gains.md).

## Calibration

Encoder zeros and mechanical stops differ from hand to hand — on the reference
V5 Plus the thumb sits more than a radian from its nominal range — so a
calibration file records, per joint and per physical hand:

- `min` / `max`: the raw encoder angles at that joint's travel limits;
- `offset`: a **homing offset** added to every reading, `0.0` until you set it.

Everything the driver reports and accepts is in the offset frame, so re-editing
an offset shifts a joint's zero and its limits together:

```
position = raw + offset
limits   = (min + offset, max + offset)
```

Files live in `allegro_hand_v5/calibration_data/<serial>.json`, so they travel
with a `pip install`, and **the driver picks the right one by itself** from the
serial number the hand reports. One is bundled: `5TBR0017.json`.

To measure your own hand:

```bash
uv run examples/calibrate.py
```

The hand goes limp — servos engaged so the encoders report, zero current
commanded — and you move every joint through its range by hand while a live
table shows each joint's position and the min/max recorded so far. Keys let you
select a joint, zero it where it stands (`z`), nudge its offset by 0.01 rad
(`+` / `-`), re-record the ranges (`r`), and save (`s`). Re-run it any time to
adjust an offset without re-recording anything.

```python
from allegro_hand_v5 import Calibration

cal = Calibration.for_serial("5TBR0017")
cal.lower, cal.upper, cal.center     # (16,) rad, offset applied
cal.denormalize(fractions)           # fractions of travel -> radians, portable across hands
cal.normalize(q)
print(cal.summary())

AllegroHand("can0", calibration="my_hand.json")   # explicit file
AllegroHand("can0", calibration=None)             # no limits, no offsets
```

## Safety

The hand latches the last current command, so **if the host stops talking while
the servos are engaged, the motors keep pushing.** Servo OFF is enforced four
ways:

1. the control process's `finally` — a normal exit and any exception in the loop;
2. a SIGTERM handler in that process, so `terminate()` unwinds through it;
3. a `getppid()` watchdog in that process, for a parent that died without asking;
4. `atexit` in the parent, plus a direct Servo OFF from the parent if the child
   died without confirming (SIGKILL, segfault).

Two further interlocks force zero current:

| Setting | Default | Effect |
|---------|---------|--------|
| `stale_timeout` | 0.1 s | No complete position update for this long ⇒ zero current, rather than running PD on stale feedback. |
| `max_current` | per preset | Per-joint clamp on everything the loop sends, in both position and current mode. |

**The handshake engages the servos**, because the hand only reports joint
positions once the motor drivers are on — leave them off and you get no feedback
at all. Nothing is driven: the mode is `idle`, so the commanded current is zero,
and `start(servo=False)` turns them back off.

## Firmware quirks

Two places where the shipped hand does not match the v1.3 manual, both confirmed
on a real V5 Plus (serial `5TBR0017`):

- **Fingertip pressure arrives on `0x0F0` / `0x0F2`**, not the documented
  `0x050` / `0x052`. Both are decoded.
- **Positions only stream while the servos are on.** The manual ties reporting to
  the period, not the servo state.

The driver also needs `Set Period` (`0x081`), which is absent from the manual's
ID table but present in WONIK's own driver and required to make the hand stream
positions instead of answering one RTR per cycle.

## Examples

```bash
uv run examples/diagnose.py                             # why is the hand not talking?
uv run examples/read_state.py                           # live table of all 16 joints
uv run examples/calibrate.py                            # record travel and homing offsets
uv run examples/position_control.py --gains compliant   # sweep between two poses under PD
uv run examples/current_control.py --joint 2 --current 40
```

None of them contains a control loop: they set targets at whatever rate is
convenient while the control process runs the PD off the hand's own stream.

## Tests

```bash
uv run pytest
```

Everything runs offline against a simulated hand (`tests/fake_hand.py`) that
speaks the real CAN protocol, including the driver's control process and every
example script. No hardware needed.

## Units

| Quantity | Unit | Notes |
|----------|------|-------|
| Position | rad | `(pi/180) * 0.088` per encoder LSB |
| Velocity | rad/s | One-step finite difference, EMA with α = 0.3 |
| Current | mA | What the wire carries; ±240 mA hardware limit |
| kp / kd | mA/rad, mA per rad/s | No torque model anywhere |
| Pressure | Pa | ~101325 at atmosphere; sensor range 30–125 kPa |

Protocol reference: [docs/allegro_hand_v5_manual.md](docs/allegro_hand_v5_manual.md)
— note the warning at the top: it is an **unverified LLM transcription** of the
PDF, and the PDF ships alongside it.
