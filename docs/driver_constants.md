# Allegro Hand V5 — Verified Driver Constants

Cross-checked against Wonik's official `allegro_hand_ros2_v5` driver
(`allegro_hand_driver/src/candrv/candef.h`, `socket_can.cpp`, `AllegroHandDrv.cpp`)
in addition to the User Manual. Where the manual and code disagree, the code wins
(it is what runs on hardware). See also [allegro_hand_v5_manual.md](allegro_hand_v5_manual.md).

## Bus
- CAN 2.0, **1 Mbps**, 11-bit standard IDs.
- **Arbitration ID = `msg_id << 2`** (TX and RX). On receive, `msg_id = can_id >> 2`.
- RTR remote frames used to poll info/serial/pose on demand (dlc = 0, RTR flag set).

## Message IDs (msg_id, before `<< 2`)
| Name | ID | Dir | Payload |
|------|----|----|---------|
| SYSTEM_ON (servo on) | 0x40 | TX | len 0 |
| SYSTEM_OFF (servo off) | 0x41 | TX | len 0 |
| SET_TORQUE finger f | 0x60 + f | TX | 4× int16 LE, **mA** |
| SET_POSE finger f | 0xE0 + f | TX | 4× int16 LE — **on-board position; undocumented, stock driver unused. Experimental.** |
| SET_PERIOD | 0x81 | TX | 3× int16 LE = `[pos_ms, imu_ms, temp_ms]`; 0 stops that stream |
| CONFIG (device id / rs485) | 0x68 | TX | see driver |
| CALIBRATION (start) | 0x89 | TX | len 0 |
| CALIBRATION (done reply) | 0x92 | RX | — |
| HAND_INFO | 0x80 | RTR→RX | hw/fw ver + servo status |
| SERIAL | 0x88 | RTR→RX | 8 ASCII bytes |
| FINGER_POSE finger f | 0x20 + f | RTR/stream→RX | 4× int16 LE |
| FINGERTIP 0 (index,middle) | **0xF0** (manual: 0x50) | RX (stream) | 2× int32 LE, Pa |
| FINGERTIP 2 (ring,thumb) | **0xF2** (manual: 0x52) | RX (stream) | 2× int32 LE, Pa |
| IMU | 0x30 | RTR→RX | roll/pitch/yaw |
| TEMPERATURE sensor s | 0x38 + s | RTR→RX | int32 °C-ish |
| PICK | 0x11 | TX | len 0 |
| PLACE | 0x12 | TX | len 0 |
| ERROR | 0xEE | RX | [motor_id, error_code] |

`f` (finger index) ∈ {0,1,2,3} = **index, middle, ring, thumb**.

## Joint indexing (canonical, 16-vector)
```
0..3   index : MCP-1(spread), MCP-2(base), PIP, DIP
4..7   middle: MCP-1,         MCP-2,       PIP, DIP
8..11  ring  : MCP-1,         MCP-2,       PIP, DIP
12..15 thumb : CMC-1,         CMC-2,       MP,  IP
```
Per-finger CAN payload order (bytes 0-1,2-3,4-5,6-7) = joint 0,1,2,3 of that finger.

## Position decode
```
angle_rad = int16_le(raw) * (pi / 180.0) * 0.088
```
- 0.088°/LSB (joint resolution). **No offset** — raw 0 ⇒ 0 rad.
- No velocity from hardware → estimate by filtered finite difference on the host.

## Current vs PWM (resolved)
**V5 is current-controlled — the wire value is a motor current setpoint in mA**, and the
mainboard closes a current loop per joint. Evidence:
- Manual FAQ §15.2.2: "The unit of Torque data is **[mA]** … converting joint torque into
  current for motor operation."
- Wonik V5 product page: joints are "independently **current-controlled**"; "precise
  current control to prevent overload."
- Windows V5 driver: `// convert desired torque to desired current and PWM count` →
  `cur_des[i] = tau_des[i] * 1.43 * 1000;` (Nm→mA) stored into `pwm_demand`.

The `pwm` / `pwm_demand` variable names and the `PWM_LIMIT_GLOBAL_8V/12V/24V` voltage
defines are **legacy carryover from V4**, which genuinely was PWM/voltage-duty based.
They are unused on the V5 send path. Caveat: this is *motor current* (∝ motor torque via
Kt, × gear ratio × efficiency), open-loop w.r.t. real joint torque — no torque sensing.

The library treats the command as current throughout: `DriverConfig.nm_to_ma` (torque→mA)
and per-joint `DriverConfig.max_current_ma` (clamp).

## Torque encode (command)
- Wire value: **int16 little-endian, milliamps (mA)**, 4 joints per finger frame.
- Stock node converts desired joint torque τ[Nm] → mA as `mA = τ * 1.43 * 1000`
  (motor+gear current constant ≈ 1.43 A/Nm). Treat `1.43e3` as the Nm→mA factor.
- **Hard clamp at send: ±240 mA per joint.** (The `PWM_LIMIT_*` #defines in the driver
  are legacy/unused; the effective limit is `±240`.)
- **"Plus" (type B) hands:** halve the command on joints **1, 5, 9** (MCP-2 of index/
  middle/ring) because those joints have ~2× gear ratio. Do NOT halve the thumb.

## Handedness / hand type (from SERIAL reply, 8 bytes)
- `data[3] == 'R'` → **right hand**, else **left hand**.
- `data[2] == 'A'` → **type A** (standard); else **type B** (e.g. Plus, apply joint-1/5/9 halving).
- Printed serial form: `SAH040 <8 chars>`.
- V5 software is expected to auto-configure handedness/type from this — the driver
  should read serial on connect and select URDF + sign/limit set accordingly.

## Fingertip pressure decode
- 2× int32 LE per frame, pascals. 0x50 = [index, middle], 0x52 = [ring, thumb].
- Stock driver post-processing: clamp invalid (`<0` or `>5000`) → 0, then EMA filter
  `y = 0.25*x + 0.75*y_prev`. (Filtering is optional in our lib; expose raw + filtered.)

## Lifecycle (from stock driver)
**Connect:** open bus → flush RX → `SYSTEM_OFF` → RTR `HAND_INFO` → RTR `SERIAL`
→ `SET_PERIOD([3,0,0])` (3 ms ≈ 333 Hz position stream) → `SYSTEM_ON`.

> **Verified on hardware (V5 Plus, serial `5TBR0017`, right/type-B):** the hand is
> **silent until `SYSTEM_ON`** — `SET_PERIOD` alone does not start the position stream
> (only the RTR serial reply comes back). So a "read-only, servo-off" mode is not
> possible; to read encoders you must servo on and (for zero force) command 0 mA — the
> current loop then holds zero torque and the hand stays backdrivable. Fingertip frames
> were observed at **0xF0 / 0xF2**, not the manual's 0x50 / 0x52. Poses decode to sane
> angles except **joint 13 (thumb CMC-2) reads ~188°** (out of its −6…107° range) —
> likely a per-joint encoder offset; investigate calibration (`0x89`).
**Gate:** don't send torque until all 4 `FINGER_POSE` frames have arrived once
(bitmask `0x0F`) — ensures fresh state before actuating.
**Shutdown:** `SET_PERIOD(0)` (stop streams) → close bus. (Optionally `SYSTEM_OFF` first.)

## Joint limits (deg) — MEASURED on the real hand (right, type B / Plus)
Each joint driven to its mechanical stop; min/max recorded. Canonical 16-vector order.
Lives in `constants.JOINT_LIMITS["right_B"]` (radians). The URDFs are no longer used.

| idx | joint | min° | max° | range° | manual ROM° | note |
|----|-------|------|------|--------|-------------|------|
| 0 | index MCP-1 | -18.7 | 20.7 | 39.4 | ±16 | wider (real + overshoot) |
| 1 | index MCP-2 | -2.9 | 101.2 | 104.1 | -5…110 | |
| 2 | index PIP | -0.5 | 111.6 | 112.1 | -3…102 | |
| 3 | index DIP | -4.7 | 105.5 | 110.2 | -5…105 | |
| 4 | middle MCP-1 | -19.1 | 18.6 | 37.7 | ±16 | |
| 5 | middle MCP-2 | -5.8 | 101.3 | 107.1 | -5…110 | |
| 6 | middle PIP | -1.0 | 110.9 | 111.8 | -3…102 | |
| 7 | middle DIP | -1.5 | 109.4 | 110.9 | -5…105 | |
| 8 | ring MCP-1 | -18.1 | 18.0 | 36.2 | ±16 | |
| 9 | ring MCP-2 | -3.0 | 102.3 | 105.2 | -5…110 | |
| 10 | ring PIP | -0.9 | 107.9 | 108.8 | -3…102 | |
| 11 | ring DIP | -1.0 | 109.8 | 110.8 | -5…105 | |
| 12 | thumb CMC-1 | -20.8 | 88.2 | 108.9 | 0…105 | zero offset ≈ -20° (range ok) |
| 13 | thumb CMC-2 | 78.3 | 190.5 | 112.2 | -6…107 | **zero offset ≈ +84°** (range ok) |
| 14 | thumb MP | -6.5 | 108.4 | 114.9 | -5…106 | |
| 15 | thumb IP | -3.9 | 106.8 | 110.7 | -4…104 | |

All travel ranges match the manual ROM; the thumb has encoder-zero offsets vs the manual's
nominal frame (limits and reported angles share the same raw frame, so clamping is
consistent). `right_A` widens MCP-2 lower to the manual's -10°; `left_A/B` mirror the
abduction joints (0/4/8/12) — those three are NOT hardware-verified.

## Still to verify on real hardware
1. `SET_POSE` (0xE0+) behavior — does on-board position control actually work? Could
   remove the need for host PID entirely. Manual says torque-only; firmware suggests otherwise.
2. Per-joint current direction **signs for left** hands (§6 mirror). The `1.43e3` Nm→mA
   constant is Wonik's; treat it as approximate and tune per joint if needed.
3. Whether `HAND_TYPE_A` vs Plus is the only distinction, or "A/B" also encodes size.

**Resolved:** current-vs-PWM (see "Current vs PWM" above) — V5 is current-controlled (mA).
