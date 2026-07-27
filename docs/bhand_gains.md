# Where the gain profiles come from

This package does not use `libBHand.so`. The two `bhand_*` profiles in
`allegro_hand_v5.gains` are nonetheless WONIK's own numbers: they were read out
of a live libBHand instance before that dependency was dropped, so the default
behaviour matches the vendor stack without shipping the blob.

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
which is why the two profiles differ:

| | spread / rot | MCP | PIP | DIP | thumb (all 4) |
|---|---|---|---|---|---|
| `Motion_HomePosition` → `BHAND_HOME` | 0.03 | **0.10** | 0.03 | 0.03 | 0.03 |
| joint PD → `BHAND_JOINT_PD` | 0.04 | **0.15** | 0.04 | 0.04 | 0.03 |

The MCP joints carry roughly 3× the damping of everything else. An independent
measurement — two constant-velocity ramps ending at the same position, so every
position-dependent term including gravity cancels — gave 0.0280 and 0.0979 for
the home profile, i.e. the 0.03 / 0.10 above.

## Deriving the compliant profiles

`GainProfile.scaled(s)` multiplies **kp by `s` and kd by `sqrt(s)`**. The damping
ratio of a second-order joint goes as `kd / sqrt(kp · I)`, so scaling both gains
by the same factor leaves the hand underdamped — the usual mistake. Scaling kd
by `sqrt(s)` holds `kd/sqrt(kp)` constant:

| profile | kp | kd (MCP) | kd/√kp |
|---|---|---|---|
| `bhand_home` | 1.00 | 0.100 | 0.100 |
| `compliant` = `scaled(0.5)` | 0.50 | 0.071 | 0.100 |
| `soft` = `scaled(0.25)` | 0.25 | 0.050 | 0.100 |

## Gravity compensation

`g(q)` peaks at 0.036 Nm — 52 of the 240 available mA — and is **not**
velocity-dependent. Two things worth knowing before you try to replicate it:

- **`SetOrientation()` is a no-op** in the shipped build. `(0, π/2, 0)` and
  `(π, 0, 0)` produce byte-identical `g(q)`. Tilting the palm is not compensated.
- Because it is small and orientation-blind, a gravity term computed from your
  simulator's URDF would be strictly better. This package ships none, so a soft
  profile will visibly sag.

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
