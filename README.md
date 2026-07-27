# allegro-hand-v5

Pure-Python driver for the **Allegro Hand V5 (F4) / (F4) Plus** (WONIK ROBOTICS),
aimed at sim-to-real / RL deployment. CAN only — no ROS, no `libBHand.so`.

The V5 is torque-only hardware: the sole actuation command is `Set Torque`, a
signed motor current per joint. Position control is closed on the host, by a PD
loop that runs in **its own process** so nothing your script does can stall it.

```python
from allegro_hand_v5 import AllegroHand, COMPLIANT

with AllegroHand("can0", gains=COMPLIANT) as hand:
    print(hand.info)                    # serial, handedness, hw/fw version
    hand.set_position_deg([0, 40, 40, 40] * 3 + [40, 140, 40, 40])
    print(hand.positions_deg, hand.pressures, hand.errors)
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

It walks the stack one layer at a time — interface state, socket, transmit,
identity reply, position stream — and stops at the first broken thing with the
command that fixes it. It never enables the servos. The usual fix is:

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100
```

`ip -details -statistics link show can0` shows the same state by hand: look for
`can state` (want `ERROR-ACTIVE`) and `berr-counter tx`.

## Layout

| Module | Responsibility |
|--------|----------------|
| `protocol` | Message IDs, encode/decode, error flags. Pure functions, no I/O. |
| `bus` | `AllegroCANBus`: the python-can socket and frame ↔ state translation. |
| `driver` | `AllegroHand`: the control process, shared-memory API. **Start here.** |
| `gains` | `GainProfile` and the presets. |
| `calibration` | `HandCalibration`: measured per-joint travel, as editable JSON. |

## Safety

The hand is torque-only, so **if the host stops talking while the servos are
engaged, the last current command stays latched and the motors keep pushing.**
Servo OFF is therefore enforced at four levels:

1. the control process's `finally` — normal exit and any exception in the loop;
2. a SIGTERM handler in that process, so `terminate()` still unwinds through it;
3. `atexit` plus SIGINT/SIGTERM handlers in the parent, so Ctrl+C and a bare
   `sys.exit()` shut down even without a `with` block;
4. a `getppid()` watchdog in the control process, and — if it died without
   confirming — the parent opens the bus itself and sends Servo OFF directly.

Verified offline against a simulated hand for: uncaught exception, `sys.exit()`,
falling off the end of `main`, `os._exit()` (skips atexit *and* every `finally`),
Ctrl+C with no `try`/`finally` anywhere, and `SIGKILL` of the control process.

Three further interlocks force zero torque, each configurable in `DriverConfig`:

| Setting | Default | Effect |
|---------|---------|--------|
| `stale_timeout` | 0.1 s | No position frame for this long ⇒ zero torque, rather than running PD on stale feedback. |
| `command_timeout` | off | No command from the parent for this long ⇒ zero torque. |
| `stop_on_error` | off | Any error code from the hand ⇒ zero torque and Servo OFF. |

Every command also passes the gain profile's per-joint `max_torque` clamp and
then `config.max_current_ma` (240 mA), including in torque mode.

**The handshake engages the servos**, because the hand only reports joint
positions once the motor drivers are on — leave them off and you get no feedback
at all. Nothing is driven: the mode is `idle`, so the commanded current is zero,
and `start(servo=False)` turns them back off as soon as the first positions
arrive.

## Firmware quirks

Two places where the shipped hand does not match the v1.3 manual, both confirmed
on a real V5 Plus (serial `5TBR0017`):

- **Fingertip pressure arrives on `0x0F0` / `0x0F2`**, not the documented
  `0x050` / `0x052`. Both are decoded.
- **Positions only stream while the servos are on.** The manual ties reporting
  to the period, not the servo state.

On **type B ("Plus") hands the MCP-2 joints — indices 1, 5, 9 — are geared about
2×**, so the driver halves their current command. Gains and `set_torque()` are
therefore in consistent joint-space units across all sixteen joints. Handedness
and type come from the serial number, so this needs no configuration.

## API

```python
hand = AllegroHand("can0", gains="compliant", calibration=True)

# lifecycle — start()/close(), or just use it as a context manager
hand.start(); hand.close()
hand.servo_on(); hand.servo_off()      # both block until the hand confirms

# commands
hand.set_position(q_rad)               # PD mode; returns the clipped target
hand.set_position_deg(q_deg)
hand.set_torque(tau_nm)                # bypass the PD
hand.set_current(ma)                   # same, in the wire's own unit
hand.hold()                            # freeze at the measured pose
hand.relax()                           # zero torque, servos still engaged

# gains, live
hand.set_gain_profile("soft")
hand.set_gains(kp=1.2, kd=0.03)        # scalars broadcast; (16,) arrays also fine
hand.get_gains()                       # read back what the control process is using

# state
hand.info                              # serial, handedness, hardware type, versions
hand.positions, hand.positions_deg     # (16,) rad
hand.velocities                        # (16,) rad/s, filtered
hand.torques, hand.currents            # (16,) Nm and mA, as commanded
hand.pressures                         # (4,) Pa — index, middle, ring, thumb
hand.errors                            # [JointError], empty when healthy
hand.mode                              # "idle" | "position" | "torque"
hand.stats                             # rate, missed deadlines, utilisation, feedback age
hand.read()                            # all of it, one torn-read-free snapshot
```

`read()` and every property go through a seqlock, so a 16-vector is never a mix
of two control cycles.

## Gain profiles

`BHAND_HOME` and `BHAND_JOINT_PD` are the gains WONIK's `libBHand.so` actually
uses, read back out of a live instance. The compliant profiles are derived from
`BHAND_HOME` by `scaled()`, which multiplies **kp by `s` and kd by `sqrt(s)`** —
that keeps `kd/sqrt(kp)` constant, so a softer hand stays just as well damped
instead of turning springy.

| Profile | kp (finger / thumb) | kd (spread, MCP, PIP, DIP) | Use |
|---------|--------------------|----------------------------|-----|
| `bhand_home` | 1.0 / 0.8 | 0.03, 0.10, 0.03, 0.03 | libBHand's home gains. The default. |
| `bhand_joint_pd` | 1.0 / 0.8 | 0.04, 0.15, 0.04, 0.04 | libBHand's joint-PD gains; same stiffness, more damping. |
| `compliant` | 0.5 / 0.4 | 0.021, 0.071, 0.021, 0.021 | Backdrivable by hand, still holds a pose. |
| `soft` | 0.25 / 0.2 | 0.015, 0.050, 0.015, 0.015 | Very backdrivable; expect gravity sag. |
| `safe` | 0.25 / 0.2 | same as `soft`, clamp 0.05 Nm | First power-on, or a hand you don't trust yet. |
| `zero` | 0 | 0 | Limp in position mode. |

Note the MCP joints (index 1 of each finger) carry ~3× the damping of the rest —
that is libBHand's own choice, and it is reproduced here.

```python
from allegro_hand_v5 import BHAND_HOME, COMPLIANT

BHAND_HOME.scaled(0.3)                    # your own point on the same curve
COMPLIANT.with_max_torque(0.08)
print(COMPLIANT.table())
```

There is **no gravity compensation**, so a soft profile will sag. If you need
it, take the model from your simulator's URDF — libBHand's own gravity term
peaks at only 52 of the 240 available mA, and ignores palm orientation entirely
(`SetOrientation` is a no-op in the shipped build). See
[docs/bhand_gains.md](docs/bhand_gains.md).

## Examples

```bash
uv run examples/diagnose.py                         # why is the hand not talking?
uv run examples/read_state.py                       # identity, joints, pressure, errors
uv run examples/pose_sweep.py --gains compliant     # full-hand sweep between two poses
uv run examples/torque_control.py --joint 2 --torque 0.03
```

`pose_sweep.py` contains no control loop — it only sets targets at 100 Hz while
the control process runs the PD at 500 Hz.

## Calibration

Each joint's measured `min`/`max` in radians, holding what the hand can actually
reach — joints absent from a file fall back to the nominal URDF ranges. Position
targets are clipped to this range before they reach shared memory.

Calibrations ship inside the package as
`allegro_hand_v5/calibration_data/<handedness>_<hardware type>.json`, so they
travel with a `pip install` and need no working directory. One is bundled:
**`right_B.json`** — right hand, type B (geared).

**The driver picks the right file by itself.** The serial number encodes
handedness (character 3) and hardware type (character 2), so `start()` reloads
the calibration for whatever hand actually answered:

```python
with AllegroHand("can0") as hand:          # calibration=True is the default
    print(hand.info.handedness, hand.info.hardware_type)   # right B
    print(hand.calibration.source)                          # .../right_B.json
```

Search order, most specific first:

1. `$ALLEGRO_CALIBRATION`, if set
2. `./calibration/right_B.json`, then `./calibration/right.json`
3. the bundled `right_B.json`, then a bundled `right.json`

So dropping a file in a local `./calibration/` overrides the bundled one without
touching the installed package. To measure a second hand, add
`calibration/left_A.json` next to it — no code change.

```python
cal = HandCalibration.load(handedness="right", hardware_type="B")
cal.limits(3)          # (-0.100, 1.837) rad
cal.from_fractions(f)  # (16,) fractions of travel -> radians
print(cal.summary())

AllegroHand("can0", calibration="my_hand.json")   # explicit file
AllegroHand("can0", calibration=None)             # no clipping
```

## Units

| Quantity | Unit | Notes |
|----------|------|-------|
| Position | rad | `(pi/180) * 0.088` per encoder LSB |
| Velocity | rad/s | Filtered one-step finite difference, α = 0.3 |
| Torque | Nm | Gains are Nm/rad and Nm/(rad/s) |
| Current | mA | What the wire carries; 1430 mA per Nm |
| Pressure | Pa | ~101325 at atmosphere; sensor range 30–125 kPa |

Protocol reference: [docs/allegro_hand_v5_manual.md](docs/allegro_hand_v5_manual.md).
