# Where the gain presets come from

This package does not use `libBHand.so`. The `default` preset in
`allegro_hand_v5.gains` is nonetheless WONIK's own number, converted into this
package's units: it was read out of a live libBHand instance before that
dependency was dropped, so out of the box the hand behaves like the vendor stack
without shipping the blob.

**Units.** libBHand works in joint torque; this package commands motor current,
which is the only thing the hand's board actually accepts. Everything below is
in the original Nm units. To get the numbers in `gains.py`, multiply by the
1.43 A/Nm the vendor driver uses (`AllegroHandDrv::setTorque`) and halve
joints 1, 5 and 9, which a Plus hand gears 2:1 — the same halving WONIK applies
to the current, moved into the gains so nothing is rescaled behind your back:

| | libBHand | here |
|---|---|---|
| kp, fingers | 1.0 Nm/rad | 1400 mA/rad (700 on MCP-2) |
| kp, thumb | 0.8 Nm/rad | 1150 mA/rad |
| kd, MCP-2 | 0.10 Nm/(rad/s) | 70 mA/(rad/s) |
| kd, everything else | 0.03 Nm/(rad/s) | 45 mA/(rad/s) |
| clamp | 0.1 Nm | 150 mA (75 on MCP-2) |

WONIK publishes no source for libBHand — only the binary and a
[header][hdr] — so everything below was measured by black-box probing: drive
`SetJointPosition` / `UpdateControl` with known inputs, read `GetJointTorque`,
and fit. Right hand, `B_GEARED`, `SetTimeInterval(0.002)`.

[hdr]: https://github.com/felixduvallet/allegro-hand-ros/blob/master/bhand/include/bhand/BHand.h

## The control law

Every computing mode is the same diagonal PD plus a gravity term:

```
tau = Kp · (q_des − q) − Kd · q̇ + g(q)
```

**Kp**, exact after subtracting the gravity gradient `dg/dq`:

| joints | Kp |
|---|---|
| 0–11 (index, middle, ring) | **1.0** |
| 12–15 (thumb) | **0.8** |

A naive `d(tau)/dq` reads 1.066 / 1.033 / 1.018; the excess is `−dg/dq`, and
subtracting the separately-measured gravity gradient lands on exactly 1.0 / 0.8.

**Kd** depends on which motion is active — `SetMotionType()` reinstalls gains,
which is why libBHand has two sets:

| | MCP-1 | MCP-2 | PIP | DIP | thumb (all 4) |
|---|---|---|---|---|---|
| `Motion_HomePosition` (the source of `default`) | 0.03 | **0.10** | 0.03 | 0.03 | 0.03 |
| joint PD (`SetMotionType`) | 0.04 | **0.15** | 0.04 | 0.04 | 0.03 |

The MCP-2 joints carry roughly 3× the damping of everything else. An independent
measurement — two constant-velocity ramps ending at the same position, so every
position-dependent term including gravity cancels — gave 0.0280 and 0.0979 for
the home gains, i.e. the 0.03 / 0.10 above.

## Deriving the softer presets

`compliant` and `soft` scale **kp by `s` and kd by `sqrt(s)`** (s = 0.5 and 0.25).
The damping ratio of a second-order joint goes as `kd / sqrt(kp · I)`, so scaling
both gains by the same factor leaves the hand underdamped — the usual mistake.
Scaling kd by `sqrt(s)` holds `kd/sqrt(kp)` constant:

| preset | kp (mA/rad) | kd, MCP-2 | kd/√kp |
|---|---|---|---|
| `default` | 1400 | 70 | 2.6 |
| `compliant` (s = 0.5) | 700 | 50 | 2.6 |
| `soft` (s = 0.25) | 350 | 35 | 2.6 |

The presets are written out as plain numbers rather than computed, so what you
read in `gains.py` is what the loop uses.

## Gravity compensation

`g(q)` peaks at 0.036 Nm — 52 of the 240 available mA — and is **not**
velocity-dependent. Two things worth knowing before you try to replicate it:

- **`SetOrientation()` is a no-op** in the shipped build. `(0, π/2, 0)` and
  `(π, 0, 0)` produce byte-identical `g(q)`. Tilting the palm is not compensated.
- Because it is small and orientation-blind, a gravity term computed from your
  simulator's URDF would be strictly better. This package ships none, so a soft
  preset will visibly sag. If you add one, it goes in your own code: compute a
  feed-forward current and add it to what `set_current` sends.

## Other findings, for anyone comparing against the vendor stack

- **`NONE` emits exactly zero torque**, and it is the motion type a fresh
  instance starts in. Any run that did not explicitly ask libBHand for a motion
  got nothing from it numerically.
- **`SetMotiontime` scales the steady state by `exp(1 − motion_time)`**, not just
  the ramp — fits to four decimals across six values. It only reaches the true
  home pose at exactly 1.0.
- **`UpdateControl(time)` ignores its argument.** It counts cycles internally and
  multiplies by `SetTimeInterval`, so a slow loop stretches the motion in real
  time rather than skipping ahead.
- **`ENVELOP` outputs 600 torque units** — 858,000 mA before clamping, saturating
  11 of 16 joints at full current.
- The home pose recovered as `(tau − g) / Kp` lands on round degrees:
  index `[−5°, 0°, 45°, 45°]`, middle `[0°, 0°, 45°, 45°]`,
  ring `[+5°, 0°, 45°, 45°]`, thumb `[90°, 20°, 15°, 45°]`.

```python
Q_HOME = np.radians([-5, 0, 45, 45,  0, 0, 45, 45,  5, 0, 45, 45,  90, 20, 15, 45])
```

## One architectural note

The vendor stack has its control thread and the application's `set_torques()`
both transmitting, unsynchronised, at ~500 Hz. Torque goes out as one CAN frame
per finger, so each finger ends up at a different duty cycle between 50% and
100%, drifting over seconds as the two loops beat against each other. Measured
against a simulated hand, steady-state error under two writers spread over a 3×
range between fingers; with a single writer it was uniform to three decimals.

`AllegroHand` here is the only writer on the bus, which is why its behaviour is
reproducible enough to model in simulation.
