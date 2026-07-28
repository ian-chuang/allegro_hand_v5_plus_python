# Allegro Hand V5 (F4) / (F4) Plus — User's Manual

> ## ⚠️ Unverified transcription
>
> **This file was written by an LLM (Claude) reading
> [`AllegroHandV5(4F)UserManual_v1.3.pdf`](AllegroHandV5%284F%29UserManual_v1.3.pdf),
> not by WONIK ROBOTICS, and nobody has checked it line by line against the
> original.** Numbers may have been transcribed wrongly, tables may have been
> reflowed incorrectly, and the "Driver-relevant summary" at the bottom is
> interpretation, not text from the manual.
>
> Treat it as a searchable index into the PDF, not as a source of truth. Before
> relying on any value here — a message ID, a scale factor, a range of motion —
> **open the PDF and check it.** The PDF ships in this repo next to this file.
>
> **Spot-checked so far** (re-extracted from the PDF and compared word for word,
> 2026-07-28): §3 technical specifications, §10 CAN protocol, §11.1–11.3 in full
> (message ID table, every data structure, the angle equation, the error bit
> table), and the §15.1 range-of-motion table. Those sections are the ones this
> package depends on, and they match. **Everything else is unchecked**, and
> figures and dimensioned drawings were never text to begin with — they are
> described from images and are the least trustworthy part of this file.
>
> Where this package's behaviour disagrees with this document, the disagreement
> is noted in code and was resolved against the real hand or against WONIK's own
> driver, [allegro_hand_ros2_v5](https://github.com/Wonikrobotics-git/allegro_hand_ros2_v5).

> Source: `AllegroHandV5(4F)UserManual_v1.3.pdf` (WONIK ROBOTICS, rev 1.3, 2025-07-10).
> Figures and diagrams are described in text; refer to the PDF for the originals.

Copyright © WONIK ROBOTICS. All rights reserved. Allegro and Allegro Hand are
trademarks of WONIK ROBOTICS.

---

## 1. Quick Start Guide

### 1.1 In the Box
| # | Item | Qty |
|---|------|-----|
| 1 | Allegro Hand | 1 EA |
| 2 | Allegro Hand Case | 1 EA |
| 3 | Power and Communication Cable (8 pin) | 1 EA |
| 4 | Desktop Stand | 1 EA |
| 5 | AC/DC Power Adapter | 1 EA |

### 1.2 Power Supply
When providing a separate power source, the following specs must be followed:
- **Voltage:** 24.0 V
- **Amperage:** 5.0 A
- **Power:** 120 W

### 1.3 CAN Device
Recommended: **PEAK PCAN-USB**, model **IPEH-002021** or **IPEH-002022**.
**Wonik Robotics does not sell CAN equipment.**

---

## 2. Allegro Hand Overview

A low-cost, highly adaptive anthropomorphic robotic hand. Four fingers, sixteen
independent current-controlled joints — a platform for grasp and manipulation research.

**Features**
- Lightweight, portable anthropomorphic design.
- Low-cost dexterous manipulation for research and industry.
- Multiple ready-to-use grasping algorithms for varied object geometries.
- **16 independent current-controlled joints (4 fingers × 4 DOF each).**
- Real-time control and online simulation.
- Optional omnidirectional tactile sensors on the fingertips returning pressure values.

---

## 3. Technical Specifications

| Spec | Value |
|------|-------|
| Number of Fingers | 3 fingers + 1 thumb = 4 |
| Degrees of Freedom | 4 fingers × 4 = **16 (active)** |
| Actuation — Type | DC motor |
| Actuation — Gear Ratio | 288.35:1 (**Plus: 576.7:1** on 2nd joint of the finger, excluding thumb) |
| Actuation — Stall Torque | 0.92 Nm (**Plus: 1.84 Nm** on 2nd joint of the finger, excluding thumb) |
| Actuation — Nominal Torque | 0.23 Nm (**Plus: 0.46 Nm** on 2nd joint of the finger, excluding thumb) |
| Payload | Max. 10 kg (**Plus: 15 kg**) |
| Weight | 1000 g (**Plus: 1024 g**) |
| Joint Resolution | **0.088 deg** |
| Communication — Type | CAN |
| Communication — Frequency | **500 Hz (CAN)** |
| Power Requirement | 24.0 V / 5.0 A / 120 W |

**Tactile Sensor (optional, pressure sensor)**
- Pressure operating range: 30–125 kPa
- Color indicator (returns `0` at atmospheric pressure, 101.3 kPa):
  - Blue: 0–124 Pa
  - Cyan: 125–249 Pa
  - Green: 250–375 Pa
  - Yellow: 376–500 Pa
  - Red: 500–24,000 Pa
- Temperature operating range: −40–85 °C
- Pressure accuracy: 6 Pa

---

## 4. System Requirements
| | |
|---|---|
| CPU | Intel® Core™ i3-8109U or higher |
| RAM | ≥ 2 GB |
| Storage | ≥ 2 GB |
| Graphics | OpenGL 3.0 H/W acceleration, ≥ 64 MB video RAM |
| OS | Windows 10 & 11; Ubuntu 20.04 LTS & 22.04 LTS |
| Additional S/W | Visual Studio 2022, ROS |

---

## 5. Joint Dimensions
All dimensions in mm and degrees. Overall span ≈ 265.45 mm; hand height ≈ 252.6 mm;
finger spacing ≈ 136.87 mm; base thickness ≈ 53.5 mm. Fingers are angled ~5° apart.
(See PDF p.6 for the dimensioned drawing and the per-joint X/Y/Z origin table for
JT1x, JT2x, JT3x, JT4x.)

---

## 6. Joint Directions
Positive rotation direction is defined per joint and differs between the **Right Hand**
and **Left Hand** (mirror image). See PDF p.7 diagrams for the `+` rotation arrows on
each of the 16 joints. This sign convention matters for the driver's angle/torque signs.

---

## 7. Mounting the Allegro Hand
- **7.1 Mounting block removal:** connected with six M3 flat-head screws (3 per side).
  Also remove the back cover (4 × M2.5 hex socket head cap screws). Secure the hand
  while unscrewing to avoid dropping it. Block removes from the bottom.
- **7.2 Mounting:** block mounts to a surface with an alternator bracket, four M3 screws
  and four M6 flat-head screws. Mount on a raised area to avoid thumb-mount interference.
- **7.3 Reassembly:** place hand onto block, replace the six M3 flat-head screws.
- **7.4 Mount block dimensions:** 48 mm square, 6 × M3 tap through on P.C.D 28,
  M18 P1.5 tap center, 2 × 3 M3 tap DP7. (See PDF p.11.)

---

## 8. Allegro Hand Wiring

### 8.1 Wiring
Connector on the back supplies power and provides the external (CAN) interface.
- Panel Mount Connector: **MB08MSAFF08ST** — M8 Stecker 8P, A-coding
- Cable Connector: **M8-F-Angle-8P-3m**

**Connector Pinout (8-pin)**
| Pin | Description | Wire Color |
|-----|-------------|-----------|
| 1 | N/A | White |
| 2 | N/A | Brown |
| 3 | **CAN L** | Green |
| 4 | **CAN H** | Yellow |
| 5 | 24VDC_in | Gray |
| 6 | GND | Pink |
| 7 | 24VDC_in | Blue |
| 8 | GND | Red |

> DB9/D-SUB adapter for PCAN-USB (FAQ §15.1.7): CAN L (green) → pin 2, CAN H (yellow) → pin 7.

### 8.2 Power
Must be powered by DC 24 V, 5 A. Insufficient supply capacity ⇒ improper operation.
After the rated power is supplied, the power switch on the back turns the hand on.

---

## 9. CAN Driver Installation

### 9.1 PEAK PCAN-USB (drivers for Windows and Linux)
Install the PCAN-USB driver before use.
- **Windows check:** Start Menu → Control Panel → Device Manager.
- **Linux check:** run `pcaninfo` in a terminal. **If you use the Allegro Hand with ROS2, you do not need to install the CAN driver.**

Links:
- Product: http://www.peak-system.com/PCAN-USB.199.0.html?&L=1
- Windows driver: https://www.peak-system.com/Drivers.523.0.html?&L=1
- Linux driver: https://www.peak-system.com/fileadmin/media/linux/index.php

> May require a reboot after installation.

---

## 10. CAN Protocol
Designed to **CAN specification 2.0**.

### 10.1 Baud-Rate
**1 Mbps.**

### 10.2 Periodic Communication
The control software communicates at a regular control interval. **Every 2 ms**, joint
torques are calculated and joint angles are updated. (⇒ 500 Hz control loop.)

---

## 11. CAN Frames

### 11.1 Arbitration Identifier
11-bit standard arbitration identifier. The 11-bit ID = **(Message ID << 2)**; i.e. the
Message ID occupies bits [10:2] and the low 2 bits are `0`.

| MAB | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | LSB |
|-----|---|---|---|---|---|---|---|---|---|-----|
| — | ← Message ID → | | | | | | | | 0 | 0 |

### 11.2 Message ID
Messages marked **RTR** can be sent by the host as a remote frame; the hand responds.

| Message | Message ID | RTR | Description |
|---------|-----------|-----|-------------|
| Servo ON | 0x040 | | Engage joint motor driver |
| Servo OFF | 0x041 | | Disable joint motor driver |
| Set Torque Finger 1 | 0x060 | | Set torque of finger #1 |
| Set Torque Finger 2 | 0x061 | | Set torque of finger #2 |
| Set Torque Finger 3 | 0x062 | | Set torque of finger #3 |
| Set Torque Finger 4 | 0x063 | | Set torque of finger #4 |
| Information | 0x080 | R | Product information and status |
| Serial Number Read | 0x088 | R | Product serial number read |
| Position Finger 1 | 0x020 | R | Current angle position of finger #1 |
| Position Finger 2 | 0x021 | R | Current angle position of finger #2 |
| Position Finger 3 | 0x022 | R | Current angle position of finger #3 |
| Position Finger 4 | 0x023 | R | Current angle position of finger #4 |
| Fingertip pressure 1 | 0x50 | | Pressure of fingertip sensors #1 (index, middle) |
| Fingertip pressure 2 | 0x52 | | Pressure of fingertip sensors #2 (ring, thumb) |
| Pick | 0x11 | | Pick motion command |
| Place | 0x12 | | Place motion command |
| Error | 0xEE | | Error |

### 11.3 Data Structure
Each message has a different data-field format. **All multi-byte data uses little-endian.**

- **11.3.1 Servo ON** — data length = 0. No data field.
- **11.3.2 Servo OFF** — data length = 0. No data field.

- **11.3.3 Set Torque Finger** — data length = 8. Each finger has 4 joints. Sets joint
  torque set-points; each set-point is a **2-byte signed** value. Torque unit is **mA**
  (value obtained by converting joint torque into motor current — see Software FAQ).

  | Byte | 0–1 | 2–3 | 4–5 | 6–7 |
  |------|-----|-----|-----|-----|
  | Field | Joint1 | Joint2 | Joint3 | Joint4 |

- **11.3.4 Information** — data length = 7. Hardware + firmware version. Hand sends it
  when the host requests via remote frame.

  | Byte | 0–1 | 2–3 | 4 | 5 | 6 |
  |------|-----|-----|---|---|---|
  | Field | Hardware Ver. | Firmware Ver. | 0 | 0 | 1 |

  Hardware Ver. and Firmware Ver. are each a 2-byte number.

- **11.3.5 Serial Number Read** — data length = 8. Serial number stored in ASCII
  characters. Sent on remote-frame request.

- **11.3.6 Position Finger** — data length = 8. Joint position; sent on remote-frame
  request or when report period comes. Values are **signed 2-byte** integers.

  | Byte | 0–1 | 2–3 | 4–5 | 6–7 |
  |------|-----|-----|-----|-----|
  | Field | Joint1 | Joint2 | Joint3 | Joint4 |

  **Angle conversion:**
  ```
  angle[rad] = (position_data * π / 180) * 0.088
  ```
  (0.088 deg per LSB — the joint resolution — then deg→rad.)

- **11.3.7 Fingertip Pressure** — data length = 8. Sent when report period comes. Values
  are **signed 4-byte** integers, expressed in **pascals (Pa)**.

  | Message | Byte 0–3 | Byte 4–7 |
  |---------|----------|----------|
  | Fingertip Pressure 1 (0x50) | Index Finger Pressure | Middle Finger Pressure |
  | Fingertip Pressure 2 (0x52) | Ring Finger Pressure | Thumb Finger Pressure |

- **11.3.8 Pick** — data length = 0. Indicates the hand was commanded to pick.
- **11.3.9 Place** — data length = 0. Indicates the hand was commanded to place.

- **11.3.10 Error** — data length = 2. Byte 0 = Motor ID, Byte 1 = Error code.
  Error code is a bitwise OR of the conditions below:

  | Bit | Error | Description |
  |-----|-------|-------------|
  | 7 | — | Always 0 |
  | 6 | — | Always 0 |
  | 5 | Overload | Load beyond motor max output continuously applied |
  | 4 | Electrical Shock | Electrical shock / insufficient input power; motor may fail |
  | 3 | — | Always 0 |
  | 2 | Overheating | Motor temperature out of set range |
  | 1 | — | Always 0 |
  | 0 | Input Voltage | Applied voltage out of specified operating range |

  On error, the LED on the back of the hand changes **green → red**.

---

## 12. Using the Allegro Hand Sample Program

### 12.1 Download Sample Program
- Windows: https://github.com/Wonikrobotics-git/allegro_hand_windows_v5.git
- Linux (ROS1): https://github.com/Wonikrobotics-git/allegro_hand_ros_v5.git
- Linux (ROS2): https://github.com/Wonikrobotics-git/allegro_hand_ros2_v5.git

### 12.2 BHand Library (grasping algorithms; keyboard keys)
| Key | Action | Notes |
|-----|--------|-------|
| H | **Home** | Starting posture; all joints oriented for a grasp |
| G | **Grasp 3** | Torque-controlled 3-finger grip (thumb, index, middle) |
| K | **Grasp 4** | Torque-controlled 4-finger grip (thumb + 3 fingers) |
| P | **Pinching (index)** | Torque-controlled 2-finger pinch (thumb + index) |
| M | **Pinching (middle)** | Torque-controlled 2-finger pinch (thumb + middle) |
| E | **Envelop** | Fully envelop an object within four fingers |
| A | **Gravity Compensation** | Fixes all joints regardless of movement |
| f | **Torque Off** | All motors torqued off; encoder values still readable |

### 12.3 ROS Program (ROS environments only)
- **12.3.1 Rviz** — real-time visualization; launch main program with `VISUALIZE:=true`.
- **12.3.2 MoveIt** — visualize + make/save positions easily.
- **12.3.3 Control with GUI** — change position, movement time, grasp force; repeat actions.

---

## 13. Firmware Update

### 13.1 Download Firmware Image
https://github.com/Wonikrobotics-git/allegro_hand_v5_firmware

### 13.2 Update procedure
1. **Connect:** remove back cover, connect the USB-C port to a PC (USB + Reset buttons on the board).
2. Open a serial console (recommended: Extra PuTTY): **115200 baud, 8 data bits, 1 stop bit, no parity, no flow control.**
3. **Enter boot loader:** push reset; within 3 s (as "Booting 3, 2, 1" displays) press **spacebar** and enter password **`wonik1234`**. (If the count reaches 0, the app runs automatically.) STM32F429 In-Application Programming (IAP) menu appears.
4. **File transfer:** press `1` + Enter to enter download mode → Files Transfer → **Y modem → Send** → select firmware binary.
5. **Run:** press `2` + Enter to run the application.

---

## 14. Technical Support
- Website: https://www.allegrohand.com/  (has a user forum)
- Address: Wonik Bldg. 4F, 20, Pangyo-ro 255beon-gil, Bundang-gu, Seongnam-si, Gyeonggi-do, Republic of Korea
- Phone: +82-31-8038-9180 · Fax: +82-31-8038-9190 · Email: robotics.biz@wonik.com

---

## 15. FAQ

### 15.1 Hardware FAQ
1. **CAD STL files** — provided on purchase; give your hand's serial number.
2. **Communication interface** — only CAN is supported. RS-485 is planned for the future.
3. **CAN device purchase** — recommend PEAK PCAN-USB (IPEH-002021 or IPEH-002022). Wonik does not sell CAN equipment.
4. **Fingertip sensor** — silicone contact surface, measures internal air pressure. Puncture (awl/knife) leaks air ⇒ malfunction.
5. **Robustness to collisions** — for research only; difficult in harsh environments. Avoid hammer blows / excessive force.
6. **Range of motion** — see table below.
7. **CAN wiring** — for PCAN-USB, connect to the D-SUB: green (CAN L) → pin 2, yellow (CAN H) → pin 7.
8./9. **Changing a fingertip sensor** — remove the M2 socket bolts on both sides of the fingertip; reassemble in reverse. Ensure pogo pins are oriented correctly at the finger/sensor interface.

**Range of Motion (ROM)** — joint naming: MCP-1, MCP-2, PIP, DIP (fingers); CMC-1, CMC-2, MP, IP (thumb).

| Group | Joint | ROM [deg] |
|-------|-------|-----------|
| Finger | MCP-1 | −16 ~ 16 |
| Finger | MCP-2 | −10 ~ 110 (**Plus: −5 ~ 110**) |
| Finger | PIP | −3 ~ 102 |
| Finger | DIP | −5 ~ 105 |
| Thumb | CMC-1 | 0 ~ 105 |
| Thumb | CMC-2 | −6 ~ 107 |
| Thumb | MP | −5 ~ 106 |
| Thumb | IP | −4 ~ 104 |

> **Inference, not manual text:** the manual never says which CAN field is which joint.
> Mapping Joint1→Joint4 in the Set Torque / Position messages onto MCP-1, MCP-2, PIP, DIP
> (thumb: CMC-1, CMC-2, MP, IP) comes from WONIK's driver and URDF, where joint index
> 4·finger+0 is the spread/abduction joint. This package uses that ordering throughout.

### 15.2 Software FAQ
1. **Use V3/V4 software?** No — many differences between V5 and previous versions.
2. **How are joints controlled?** CAN from the external controller; UART between the 16
   joints and the main board inside the hand. **The only motor command is `Set Torque
   Finger`.** Torque data unit is **[mA]** — joint torque converted to motor current.
3. **PID position control?** The main board only does torque control; position control
   must be implemented on the PC side (this is what "Home" motion does). It needs the
   joint position encoder value. You must build your own controller; because the board
   only provides encoder values, precise control can be challenging — mind the control
   frequency.
4. **Set left/right and A/B hand type in SW?** From V5 onward, all software (Windows,
   ROS1, ROS2) auto-applies handedness and hand type from the serial number — **use as is.**
5. **Where to get PC S/W support?** Forum https://www.allegrohand.com/forum or GitHub
   https://github.com/Wonikrobotics-git

---

## Release Note
| Date | Ver. | Description | Editor |
|------|------|-------------|--------|
| 2024-11-12 | 1.0 | Initial draft | Lucas |
| 2024-11-25 | 1.1 | CAN wiring correction | Lucas |
| 2025-01-09 | 1.2 | Power supply correction | Lucas |
| 2025-03-26 | 1.2.1 | Update [Changing a Fingertip sensor] | Lucas |
| 2025-07-10 | 1.3 | Changing CAN protocol (fingertip) | Lucas |

---

## Driver-relevant summary (engineering notes — interpretation, not manual text)

Everything below this line is a reading of the manual plus WONIK's
[ROS 2 driver](https://github.com/Wonikrobotics-git/allegro_hand_ros2_v5), which
is the more reliable of the two where they differ: it is what actually runs.

**Bus:** CAN 2.0, **1 Mbps**, 11-bit standard IDs. Arbitration ID = `msg_id << 2`.

**Everything is current control.** The only actuation command is `Set Torque Finger N`
(IDs 0x060–0x063), 8 data bytes = four int16 little-endian values in **mA**, one per
joint (MCP-1, MCP-2, PIP, DIP / thumb CMC-1, CMC-2, MP, IP). Despite the name, the
payload is a motor current, not a torque. Position control (including "home") must be
closed on the host from encoder feedback. WONIK's driver saturates every value at
**±240 mA** before transmitting.

**Feedback (read via RTR remote frame or periodic report):**
- Position Finger N (0x020–0x023): 4× int16 LE. `angle_rad = pos * (π/180) * 0.088`.
- Fingertip Pressure 1/2: 2× int32 LE, Pa. (First message = index, middle; second =
  ring, thumb.) The manual and WONIK's driver both say **0x50 / 0x52**; the firmware
  on the reference hand here streams them at **0xF0 / 0xF2**. This package accepts both.
- Information (0x080), Serial Number (0x088): RTR-readable; the serial decides
  handedness and hardware type. Real firmware often ignores the Information RTR.
- Error (0xEE): [motor_id, error_code] bitmask (overload/eshock/overheat/input-voltage).

**Message IDs in WONIK's `candef.h` but not in the manual's table** — undocumented, and
the firmware may or may not act on them:
- `0x081` Set Period — three int16 (position, IMU, temperature) in ms. **Required**: it
  is what makes the hand stream positions instead of answering one RTR per cycle.
- `0x030` IMU, `0x038`–`0x03B` Temperature — RTR-readable and streamable via Set Period.
- `0x0E0`–`0x0E3` Set Pose — inherited from the V4 protocol. The manual is explicit that
  the board does torque control only, so this is presumed inert on V5.
- `0x089` / `0x092` — start the hand's own position calibration, and its reply when done.
- `0x068` Config — sets the CAN device ID and RS-485 baud rate. **Not implemented here**:
  it writes persistent device configuration, is undocumented for V5, and getting it wrong
  takes the hand off the bus.

**Lifecycle:** power on → Servo ON (0x040) to engage motors → stream Set Torque, one
update per complete set of four position reports → Servo OFF (0x041) to release. The
hand's internal loop is 2 ms; the default 3 ms position period gives ~333 Hz.

**Handedness:** V5 auto-detects from the serial number; character 2 is the hardware type
(A non-geared / B geared "Plus") and character 3 is handedness (R / L).
