# Embedded Systems Manual — Torque Controller

## 1. Introduction

### 1.1 Purpose of This Document

This manual documents the firmware, hardware integration, and communication
architecture of the **torque-controller** test project. Its purpose is to let a
developer (a) reproduce a working Pi → STM32 → Motor torque command path,
(b) understand the design decisions that fix prior CAN-bus issues, and
(c) use this project as the blueprint for the production joint controller
firmware.

### 1.2 Scope and Intended Audience

**Scope:** the firmware in
`testing/stm32/torque-controller/` and the companion Pi scripts in
`testing/stm32/torque-controller/scripts/`. This is a single-joint, one-way
(Pi → motor) proof-of-concept. It does not cover IMU data paths, multi-motor
coordination, or machine-learning control layers.

**Audience:** embedded firmware developers on the exoskeleton team, integrators
bringing up new joint boards, and anyone porting this pattern into the
production joint-controller codebase.

### 1.3 System Overview Summary

A Raspberry Pi sends torque commands (in Nm) over a 1 Mbit/s CAN bus to an
STM32F446RE Nucleo board. The STM32 converts the commanded torque into a
motor-current setpoint using the AK70-9 KV60 torque constant, then forwards a
VESC `SET_CURRENT` command to the motor over the same bus via extended CAN
frames. A 50 ms refresh loop keeps the motor driven between new commands. An
ESTOP broadcast from the Pi immediately zeroes the commanded current and
latches the STM32 until power-cycle.

### 1.4 Definitions and Acronyms

| Term | Meaning |
|------|---------|
| CAN  | Controller Area Network (1 Mbit/s bus used for all inter-node comms) |
| VESC | Open-source motor-controller firmware running on the AK70-9 |
| Kt   | Motor torque constant (Nm per ampere) |
| Iq   | Quadrature-axis current — the torque-producing current component |
| FIFO | First-In-First-Out receive queue inside the STM32 CAN peripheral |
| ISR  | Interrupt Service Routine |
| ESTOP| Emergency-stop message — latches the STM32 into a zero-output state |
| HAL  | STM32 Hardware Abstraction Layer |
| ERPM | Electrical RPM (motor-pole × mechanical RPM, used by VESC) |
| DLC  | Data Length Code — number of bytes in a CAN frame (0–8) |
| MCU  | Microcontroller Unit |
| SCE  | Status Change Error — STM32 CAN interrupt for bus errors |

---

## 2. System Architecture

### 2.1 High-Level Architecture

```
┌─────────────┐   Standard CAN (11-bit)    ┌─────────────┐   Extended CAN (29-bit)   ┌─────────────┐
│  Raspberry  │  TORQUE_CMD (0x081)         │    STM32    │  SET_CURRENT (0x168)      │   AK70-9    │
│     Pi      │ ──────────────────────────► │   Gateway   │ ────────────────────────► │    Motor    │
│  (Node 0)   │  float torque_nm            │  (Node 1-4) │  float current_A          │  (ID 101-4) │
│             │ ◄────────────────────────── │             │ ◄──────────────────────── │  (VESC FW)  │
└─────────────┘  MOTOR_STATUS (future)      └─────────────┘  Feedback (ext frame)     └─────────────┘
```

All three devices share a single 1 Mbit/s CAN bus with 120 Ω termination at
both physical ends.

#### 2.1.1 Embedded Layers and Roles

From bottom to top:

1. **HAL / Hardware layer** — STM32 HAL drivers and the Linux socketCAN
   subsystem on the Pi.
2. **Transport layer** — `can_common.{c,h}` on STM32, `can_common.py` on
   Pi. Handles filter setup, ISR callbacks, ring-buffered RX on STM32, and
   bus creation on the Pi.
3. **Message layer** — `can_motor.{c,h}` (TORQUE_CMD, MOTOR_STATUS),
   `can_system.py` (ESTOP), and `ak70_9.{c,h}` (VESC servo commands).
   Each message type has a paired encode/decode function.
4. **Application layer** — `main.c` (control loop, torque-to-current math,
   refresh timer, ESTOP latch) and `torque_cmd.py` (interactive CLI).

#### 2.1.2 Board Responsibilities (Leader vs Followers)

| Role     | Device   | Node ID | Responsibility                                |
|----------|----------|---------|-----------------------------------------------|
| Leader   | Pi       | 0       | Originates torque commands and ESTOPs         |
| Follower | STM32 #1 | 1       | Drives Left Hip motor                         |
| Follower | STM32 #2 | 2       | Drives Right Hip motor                        |
| Follower | STM32 #3 | 3       | Drives Left Knee motor                        |
| Follower | STM32 #4 | 4       | Drives Right Knee motor                       |

Each STM32 only processes TORQUE_CMD frames addressed to its own node ID
(hardware-filtered at the CAN peripheral level). ESTOP is a broadcast and is
accepted by every follower.

### 2.2 Hardware Components

#### 2.2.1 Microcontrollers

**STM32F446RE Nucleo-64**

- Core: ARM Cortex-M4 @ 84 MHz
- CAN1 on PA11 (RX) / PA12 (TX) — AF9
- USART2 on PA2 (TX) / PA3 (RX) — routed to ST-Link VCP at 115200 8N1 for
  debug output
- LD2 (green LED, PA5) used as visual status indicator
- Clock tree: HSI 16 MHz → PLL (M=16, N=336, P=4) → SYSCLK 84 MHz,
  HCLK 84 MHz, APB1 42 MHz (CAN clock)
- CAN bit timing: Prescaler 3, BS1 10 TQ, BS2 3 TQ → 1 Mbit/s, 75 % sample
  point

#### 2.2.3 Actuators (Motors, Drivers)

**CubeMars AK70-9 KV60**

| Parameter                  | Value          |
|----------------------------|----------------|
| Torque constant (Kt)       | 0.159 Nm/A     |
| Gear ratio                 | 9 : 1          |
| Effective output Kt        | 1.431 Nm/A     |
| Rated torque               | 8.5 Nm @ 6.25 A|
| Peak torque                | 29.2 Nm @ 23.8 A|
| Rated voltage              | 48 V           |
| Rated speed (output shaft) | 260 rpm        |
| CAN interface              | 29-bit extended, 1 Mbit/s, big-endian payload |
| Firmware                   | VESC-based     |
| Default CAN ID             | 104            |

The motor speaks an extended CAN protocol defined by the VESC firmware. The
STM32 emits `SET_CURRENT` frames (mode ID 1) with the motor's CAN ID in the
low 8 bits of the extended identifier.

#### 2.2.4 Communication Interfaces

| Interface  | Purpose                                        | Parameters                  |
|------------|------------------------------------------------|-----------------------------|
| CAN1       | All inter-device messaging                     | 1 Mbit/s, 120 Ω termination |
| USART2     | STM32 debug output via ST-Link VCP             | 115200 8N1                  |

No I2C, SPI, or other interfaces are used by this project.

### 2.3 Software Architecture

#### 2.3.1 Firmware Structure

```
testing/stm32/torque-controller/
├── Core/
│   ├── Inc/
│   │   ├── can/ak70_9.h          # VESC protocol API
│   │   ├── can_common.h          # Transport layer
│   │   ├── can_motor.h           # TORQUE_CMD / MOTOR_STATUS encoding
│   │   ├── main.h
│   │   ├── stm32f4xx_hal_conf.h
│   │   └── stm32f4xx_it.h
│   └── Src/
│       ├── can/ak70_9.c
│       ├── can_common.c
│       ├── can_motor.c
│       ├── main.c                # Control loop, state machine
│       ├── stm32f4xx_hal_msp.c
│       ├── stm32f4xx_it.c        # IRQ handlers (RX0, RX1, SCE)
│       └── system_stm32f4xx.c
├── Drivers/                       # STM32 HAL (generated)
├── docs/                          # Project documentation
└── scripts/
    ├── can_common.py              # Mirror of can_common.h
    ├── can_motor.py               # Mirror of can_motor.h
    ├── can_system.py              # ESTOP messages
    ├── can_test.py                # Minimal CAN self-test
    ├── motor_test.py              # Legacy UART test
    └── torque_cmd.py              # Interactive Pi CLI
```

#### 2.3.3 Module Breakdown

| Module         | Role                                                  |
|----------------|-------------------------------------------------------|
| `can_common`   | HAL filter setup, ISR callbacks, ring buffer, TX/RX   |
| `can_motor`    | TORQUE_CMD / MOTOR_STATUS encode + decode             |
| `ak70_9`       | VESC extended-frame commands (SET_CURRENT, etc.)      |
| `main`         | Main loop, state machine, torque→current conversion  |
| `stm32f4xx_it` | CAN IRQ handlers (RX0, RX1, SCE)                      |

Python side mirrors this structure 1:1 so the C and Python constants stay in
sync.

---

## 3. Hardware Integration

### 3.1 Wiring and Connections

#### 3.1.1 Board-to-Board Connections

All three devices share one CAN bus. Wire as a linear trunk (not a star):

```
  Pi  ──┐
        │              ┌── STM32 ──┐
        ├── CANH ───────┤           │
        ├── CANL ───────┤           │
        ├── GND  ───────┤           │
        │              └──  Motor──┘
        │
        │  120 Ω at each physical end
```

- **CANH ↔ CANH**, **CANL ↔ CANL**, **GND shared**
- Termination: exactly **two** 120 Ω resistors, one at each physical end of
  the trunk. The motor's VESC board has a built-in termination resistor on
  most AK-series units; verify with a multimeter (~60 Ω across CANH/CANL when
  the bus is de-powered and only both terminators are in place).

On the Pi side, the CAN HAT appears as `can1` (some HATs enumerate as `can0`;
confirm with `ip link show`). On the STM32 the interface is `CAN1` using PA11
(RX) and PA12 (TX) at alternate function 9.

#### 3.1.3 Motor and Driver Connections

- **Motor power:** 48 V DC to the VESC power input.
- **Motor CAN:** the VESC CANH/CANL pins connect to the same trunk as the Pi
  and STM32. The motor itself is driven by the VESC; there are no separate
  phase-wire connections to manage on the user side.
- **Motor CAN ID:** assign each motor a unique CAN ID using the VESC Tool
  (suggested: 101 Left Hip, 102 Right Hip, 103 Left Knee, 104 Right Knee).
  The default from the factory is 104, which means two unconfigured motors
  on the same bus will both respond to the same command — configure before
  bringing multiple motors up.

### 3.2 Power Distribution

#### 3.2.1 Voltage Requirements

| Component      | Supply    | Source                                |
|----------------|-----------|---------------------------------------|
| AK70-9 motor   | 48 V DC   | Bench PSU or battery pack             |
| STM32 Nucleo   | 5 V USB   | Laptop (ST-Link) or 5 V bench supply  |
| Raspberry Pi   | 5 V 3 A   | Official Pi PSU                       |

#### 3.2.2 Power Regulation and Protection

- The STM32 Nucleo has an onboard LDO and ST-Link; no external regulation
  required for the MCU.
- The motor's 48 V rail should be protected by a fuse or e-stop contactor
  sized for the expected peak current.
- Do **not** backfeed 48 V into the CAN bus wiring — CANH/CANL are logic
  level (≤ 5 V differential).

### 3.3 Assembly Guidelines

#### 3.3.1 Physical Layout Considerations

- Keep the CAN trunk as straight as possible. Avoid long star-topology stubs
  from each node — they create signal reflections that degrade margin at
  1 Mbit/s.
- Motor power wires carry high-current switching noise. Route them away from
  CAN signal wires; twist CANH/CANL together where possible.
- Share ground between all three nodes. Missing ground reference will cause
  intermittent bit errors that look random.

#### 3.3.2 Common Mistakes

| Mistake | Symptom |
|---------|---------|
| Single 120 Ω resistor (or none) | Intermittent TX errors, bus-off |
| Three or more terminators       | ~40 Ω bus impedance, weak differential signal |
| CANH/CANL swapped               | No ACK from any node, all TX fails |
| Missing ground between devices  | Intermittent errors that come and go with load |
| `can0` vs `can1` misconfig on Pi| Script runs silently, no frames on wire |

---

## 4. Communication Protocols

### 4.1 Overview of Communication System

Two distinct protocols share the same physical 1 Mbit/s bus:

1. **Inter-node protocol** (Pi ↔ STM32) — 11-bit standard CAN IDs,
   little-endian payloads. Defined by this project.
2. **VESC motor protocol** (STM32 ↔ Motor) — 29-bit extended CAN IDs,
   big-endian payloads. Defined by VESC firmware.

Standard and extended frames coexist natively per CAN 2.0B. The STM32 uses
hardware filters to route each type to a different RX FIFO so motor feedback
cannot starve Pi commands.

### 4.2 CAN Bus Implementation

#### 4.2.1 Network Topology

Linear multi-drop trunk with 120 Ω termination at both ends. No isolators
required (all three devices are on the same ground). Typical distances are
well under 1 m for a bench setup — trunk length limits are not a concern.

#### 4.2.2 Node Roles and Addressing

**Node IDs (3 bits, inter-node only):**

| Node ID | Name              |
|---------|-------------------|
| 0       | `CAN_NODE_PI`     |
| 1       | `CAN_NODE_LEFT_HIP`   |
| 2       | `CAN_NODE_RIGHT_HIP`  |
| 3       | `CAN_NODE_LEFT_KNEE`  |
| 4       | `CAN_NODE_RIGHT_KNEE` |

**Motor IDs (8 bits, VESC protocol):**

Assigned independently via VESC Tool. Each motor must have a unique 8-bit
CAN ID on the bus.

#### 4.2.3 Message Flow (Leader ↔ Followers)

```
Pi (leader)                STM32 (follower)            Motor
───────────                ────────────────            ─────
TORQUE_CMD (std) ────────►
                           parse τ
                           Iq = τ / Kt_eff
                           SET_CURRENT (ext) ────────► drives motor
                                                       feedback ──┐
                                               ◄──────────────────┘
                           (every 50 ms)
                           SET_CURRENT refresh ──────►

ESTOP (std, broadcast) ─► zeroes current,
                          latches off
                          SET_CURRENT(0) ────────────► stops
```

### 4.3 Message Definitions

#### 4.3.1 Message ID Structure

**Standard frames (11-bit):**

```
Bits [10:7] = Message Type  (4 bits)
Bits [6:4]  = Source Node   (3 bits)
Bits [3:0]  = Destination   (4 bits)
```

Built with `CAN_BUILD_ID(type, src, dest)` in C or `build_can_id(type, src, dest)`
in Python.

**Extended frames (29-bit, VESC):**

```
Bits [28:8] = Control Mode ID (VESC CAN_PACKET_ID enum)
Bits [7:0]  = Motor Driver ID
```

#### 4.3.2 Payload Format

| Message        | DLC | Encoding                                          |
|----------------|-----|---------------------------------------------------|
| ESTOP          | 1   | reason byte (uint8)                               |
| TORQUE_CMD     | 2   | int16 little-endian, `raw = torque_nm * 1000`     |
| MOTOR_STATUS   | 8   | int16×3 LE + int8 + uint8 (pos, spd, cur, T, err) |
| SET_CURRENT    | 4   | int32 big-endian, `raw = current_A * 1000`        |
| Motor feedback | 8   | int16×3 BE + int8 + uint8 (VESC convention)       |

#### 4.3.3 Command vs Telemetry Messages

| Type          | Direction     | Role      |
|---------------|---------------|-----------|
| ESTOP         | Pi → all      | Command   |
| TORQUE_CMD    | Pi → STM32    | Command   |
| SET_CURRENT   | STM32 → Motor | Command   |
| MOTOR_STATUS  | STM32 → Pi    | Telemetry (future) |
| VESC feedback | Motor → bus   | Telemetry |

#### 4.3.4 Error and Acknowledgment Handling

CAN itself provides hardware-level ACK at the bit-stream layer — any healthy
node on the bus acknowledges every frame. The application protocol does not
layer explicit ACKs on top.

Error handling relies on CAN error frames and the STM32 error counters:

- `AutoBusOff = ENABLE` on STM32 and `restart-ms 100` on the Pi make both
  sides self-recover from bus-off.
- `AutoRetransmission = ENABLE` retries failed TX attempts automatically.
- The `CAN1_SCE_IRQHandler` on STM32 must be present to drain error-state
  interrupts; without it a CAN error freezes the board in the default IRQ
  trap.

Application-layer error feedback (VESC reports the motor's error code in
byte 7 of its feedback frame) is parsed by `motor_receive()` into
`MotorStatus.error` and printed over UART debug output.

---

## 6. Motor Control System

### 6.1 Motor Control Overview

The STM32 acts as a thin torque-to-current gateway. Closed-loop current
regulation happens inside the VESC firmware on the motor — the STM32 just
tells the VESC what current to produce.

### 6.2 Control Algorithms

**Open-loop torque-to-current conversion.** No STM32-side PID loop.

```
Iq = τ_desired / Kt_effective
where Kt_effective = Kt × gear_ratio = 0.159 × 9 = 1.431 Nm/A
```

**Verification from AK70-9 KV60 datasheet:**

- Rated: 6.25 A × 1.431 = 8.94 Nm (datasheet rated 8.5 Nm — ~5 % gearbox loss)
- Peak:  23.8 A × 1.431 = 34.0 Nm (datasheet peak 29.2 Nm)

The small residual error is gearbox friction and can be calibrated out later
if needed.

**Keepalive refresh:** the VESC firmware times out a commanded current after
roughly 1–2 seconds of silence and lets the motor coast. The STM32 re-sends
the last commanded current every 50 ms to keep the motor actively driven
between new torque commands.

### 6.3 Command Interface

#### 6.3.1 Input Commands

From the Pi CLI (`torque_cmd.py`):

| Command              | Effect                                   |
|----------------------|------------------------------------------|
| `<node_id> <τ_Nm>`   | Send torque to one joint                 |
| `all <τ_Nm>`         | Broadcast same torque to all four joints |
| `stop`               | Zero torque on all joints                |
| `estop`              | Broadcast ESTOP (latches STM32)          |
| `help`, `quit`       | UI commands                              |

From the STM32 side, only two CAN message types trigger state changes:
`ESTOP` and `TORQUE_CMD`. All other message types are filtered out at the
hardware level.

#### 6.3.2 Safety Constraints

| Constraint            | Where enforced         | Value                |
|-----------------------|-----------------------|----------------------|
| ESTOP latching        | STM32 `main.c`         | Until power cycle    |
| Test-safe current     | STM32 `clampf`         | ±5.0 A               |
| VESC hardware limit   | Motor firmware         | ±60 A (not approached)|
| Refresh timeout       | STM32 timer            | 50 ms                |
| Filter rejection      | STM32 CAN peripheral   | Non-matching frames dropped in hardware |
| Bus-off auto-recovery | STM32 + Pi             | Enabled both sides   |

**ESTOP is one-way on purpose.** Once received, the STM32 ignores all
subsequent TORQUE_CMD frames until power cycle. If a soft-reset flow is
needed later, add a dedicated `RESUME` message type rather than allowing
ESTOP to time out.

---

## 7. Firmware Implementation

### 7.1 Code Structure

See section 2.3.1 for the directory layout. The layering rule: application
code in `main.c` only calls message-layer functions
(`can_send_torque_cmd`, `comm_can_set_current`, `can_recv`) and never touches
the HAL CAN API directly.

### 7.2 Key Modules

#### 7.2.1 Communication Module

**`can_common.{c,h}`** owns the STM32 CAN peripheral. `can_common_init()`:

1. Configures three filter banks:
   - Bank 0 → FIFO0: all ESTOP messages
   - Bank 1 → FIFO0: TORQUE_CMD addressed to `my_node_id`
   - Bank 2 → FIFO1: all extended frames (motor feedback)
2. Starts the peripheral, enables RX0/RX1/SCE interrupts.
3. Activates FIFO-pending, error, bus-off, and last-error-code notifications.

The ISR callbacks (`HAL_CAN_RxFifoNMsgPendingCallback`) pop a frame from
hardware and push it to a 32-slot ring buffer. The main loop drains the
buffer via `can_recv()`.

**`can_motor.{c,h}`** provides the encode/decode pair for TORQUE_CMD and
MOTOR_STATUS. **`ak70_9.{c,h}`** provides VESC command helpers
(`comm_can_set_current` etc.). Both are thin wrappers over `can_send_std` /
`can_send_ext`.

#### 7.2.3 Control Module

Lives inline in `main.c`. Four state variables:

| Variable           | Purpose                                   |
|--------------------|-------------------------------------------|
| `motor_status`     | Latest parsed motor feedback              |
| `active_current`   | Current the STM32 is commanding           |
| `motor_active`     | 1 while refresh timer is running          |
| `estop_active`     | 1 after ESTOP received (latched)          |

State transitions (see `docs/torque-controller-flow.md` for the full
diagram): BOOT → READY → (COMMANDING ↔ ESTOP).

### 7.3 Build and Deployment

#### 7.3.1 Toolchain Setup

- **STM32CubeIDE** (tested 1.14+) — opens `torque-controller.ioc` and uses
  the bundled GCC ARM toolchain.
- **python-can** ≥ 4.0 on the Pi:
  ```bash
  sudo apt install python3-can can-utils
  ```
- No external dependencies beyond the STM32 HAL (vendored under `Drivers/`).

#### 7.3.2 Flashing Firmware

1. Open the `torque-controller` folder as an existing project in
   STM32CubeIDE.
2. **Before building**, edit `Core/Src/main.c` to set the per-board
   constants:
   ```c
   #define MY_NODE_ID          CAN_NODE_LEFT_HIP   /* change per board */
   #define MY_MOTOR_CAN_ID     101                 /* change per motor */
   ```
3. Connect the Nucleo via USB. `Project → Build Project`, then
   `Run → Run As → STM32 Cortex-M C/C++ Application` to flash via ST-Link.
4. Boot confirmation: the green LED blinks 3 times and the UART debug output
   prints `CAN OK node=<N>`. A solid LED means CAN init failed.

On the Pi:

```bash
sudo ip link set can1 up type can bitrate 1000000 restart-ms 100
cd ~/scripts
python3 torque_cmd.py
```

---

## 8. Testing and Validation

### 8.1 Unit Testing

No automated unit tests. The Pi scripts (`can_test.py`) act as minimal
smoke tests — they open the bus and send a single known frame.

### 8.2 Integration Testing

Bench validation is performed with `candump` + `cansend` + the Pi script,
using the STM32's UART debug output as ground truth for what the STM32
receives.

Layered bring-up sequence (detailed in `docs/torque-controller-flow.md`):

1. **Layer 1** — STM32 boot: green LED blinks 3×, UART prints `CAN OK`.
2. **Layer 2** — Pi interface up: `ip link show can1` reports `state UP`.
3. **Layer 3** — Pi can send: `cansend can1 081#E803` in one terminal,
   `candump can1` in another echoes the frame.
4. **Layer 4** — STM32 receives: UART prints `RX id=0x081 ext=0 dlc=2`
   followed by `torque=1.000 Nm -> current=0.699 A`.
5. **Layer 5** — Motor moves: extended frames `00000168 [4] XX XX XX XX`
   appear on candump every 50 ms and the motor responds.

### 8.3 System-Level Testing

End-to-end via `torque_cmd.py`:

```
Torque> 1 0.0      # zero torque, motor holds
Torque> 1 1.0      # ~0.7 A, may not overcome friction
Torque> 1 5.0      # ~3.5 A, motor spins
Torque> 1 -5.0     # reverses
Torque> stop       # back to 0 Nm
Torque> estop      # latches off, subsequent commands ignored
```

After ESTOP, power-cycle the STM32 to clear the latch.

---

## 9. Troubleshooting and Debugging

### 9.1 Common Issues

| Symptom | Root cause | Fix |
|---------|-----------|-----|
| Pi script prints `Send returned OK` but nothing on `candump` | Pi CAN interface in `BUS-OFF` | `sudo ip link set can1 down && sudo ip link set can1 up type can bitrate 1000000 restart-ms 100` |
| `candump` shows nothing even from `cansend` | Wrong interface name (`can0` vs `can1`) | Check `ip link show`, use the correct name |
| STM32 LED solid on at boot | `can_common_init` failed | Check that filters are valid and HAL CAN handle is initialized before the call |
| STM32 receives one torque command, then nothing | Old `can_bus_init` (accept-all filter) lets motor feedback flood FIFO0 | Switch to `can_common_init` (filtered FIFOs) |
| STM32 never sees motor feedback | Missing `CAN1_RX1_IRQHandler` in `stm32f4xx_it.c` | Add the handler; FIFO1 pending IRQs go there |
| Motor doesn't move even with large torque | Motor not on bus (zero extended frames received) | Check 48 V supply, CANH/CANL, termination at motor end |
| 1 Nm command produces no visible motion | Current (~0.7 A) below static-friction threshold | Try 5 Nm (~3.5 A) |

### 9.2 Debugging Tools and Methods

| Tool | Purpose |
|------|---------|
| `candump can1`              | Watch all CAN traffic on the bus |
| `cansend can1 ID#DATA`      | Inject a raw frame, bypassing the Pi script |
| `ip -details link show can1`| Inspect CAN interface state, error counters, bus-off |
| STM32 UART debug print      | Ground truth for what the STM32 actually received / parsed |
| LD2 green LED               | Quick visual: blinks 3× on boot, toggles per torque command |
| `can_test.py`               | Minimal Python send — isolates library/interface issues from script issues |

### 9.3 Logging and Diagnostics

The STM32 prints a one-line status dump every 1 second over USART2:

```
[STATUS] active=1 estop=0 cmd=3.497A motor_rx=250 | pos=12.3 spd=500 cur=3.50A temp=35C err=0(NONE)
```

Fields:

| Field        | Meaning                                            |
|--------------|----------------------------------------------------|
| `active`     | `motor_active` — refresh loop running              |
| `estop`      | `estop_active` — latched                           |
| `cmd`        | Currently commanded current                        |
| `motor_rx`   | Count of extended frames received from motor       |
| `pos/spd/cur`| Latest parsed motor feedback                       |
| `temp`       | Motor driver-board temperature (°C)                |
| `err`        | VESC error code with decoded name                  |

`motor_rx` is the fastest confidence check that the motor is on the bus and
talking — if it stays at 0, the motor isn't physically connected.

---

## 10. Best Practices and Standards

### 10.1 Coding Standards

- **C:** match the STM32CubeIDE generated style (tabs? spaces? — follow the
  file you're editing). Keep user code inside the `USER CODE BEGIN/END`
  comment pairs so CubeMX regeneration doesn't wipe it.
- **Encoding:** one encode/decode pair per message type, both languages.
  Never pack bytes inline in application code.
- **No blocking calls in ISRs.** Only push to ring buffers and return.
- **Static limits** (`CURRENT_LIMIT`, `TEST_DUTY_MAX`, etc.) live at the top
  of `main.c` / `ak70_9.h` so they're reviewable in one place.

### 10.2 Communication Protocol Standards

- **11-bit standard frames** for inter-node messaging — never reuse this
  range for anything else on the bus.
- **29-bit extended frames** reserved for VESC / motor traffic.
- **Little-endian** for all inter-node payloads. **Big-endian** for VESC
  payloads (fixed by VESC, don't change). The conversion happens once in
  `can_motor.c` / `ak70_9.c` so application code never worries about it.
- **Message types are priority-ordered.** ESTOP = 0x0 (highest CAN priority);
  TORQUE_CMD = 0x1; status/telemetry = higher numbers.
- **Pi node ID = 0.** Keep this so Pi-originated messages always have the
  lowest source bits.

### 10.3 Documentation Guidelines

- Update `docs/torque-controller-flow.md` and this manual together when
  changing the control loop or message layout.
- Keep the comparison doc (`docs/old-vs-new-can-comparison.md`) aligned
  with the production `src/` code until the production code adopts this
  pattern.
- Every new CAN message type needs a row in the Appendix 11.1 table and a
  matching encode/decode pair in both `can_motor.{c,h}` and `can_motor.py`.

---

## 11. Appendices

### 11.1 CAN Message Tables

**Inter-node (11-bit standard, little-endian):**

| Message      | Type | Direction  | DLC | CAN ID (example) | Payload                                     |
|--------------|------|------------|-----|------------------|---------------------------------------------|
| ESTOP        | 0x0  | Pi → all   | 1   | 0x000            | `uint8` reason                              |
| TORQUE_CMD   | 0x1  | Pi → STM32 | 2   | 0x081            | `int16 τ×1000` (Nm)                         |
| MOTOR_STATUS | 0x2  | STM32 → Pi | 8   | 0x110            | `int16 pos×10`, `int16 spd/10`, `int16 I×100`, `int8 T`, `uint8 err` |

Example ID decoding for `0x081`: type `0x1` (TORQUE_CMD), source `0x0` (Pi),
destination `0x1` (Left Hip).

**VESC / motor (29-bit extended, big-endian):**

| Command      | Mode | CAN ID (motor=104) | DLC | Payload                            |
|--------------|------|--------------------|-----|------------------------------------|
| SET_DUTY     | 0    | 0x00000068         | 4   | `int32 duty×100000`                |
| SET_CURRENT  | 1    | 0x00000168         | 4   | `int32 I×1000` (A)                 |
| SET_RPM      | 3    | 0x00000368         | 4   | `int32 erpm`                       |
| Feedback     | 9    | 0x00000968 (rx)    | 8   | `int16 pos×10`, `int16 spd/10`, `int16 I×100`, `int8 T`, `uint8 err` |

### 11.2 Pinout Diagrams

**STM32F446RE Nucleo — pins used:**

| Pin  | Function      | Alt Func | Notes                                |
|------|---------------|----------|--------------------------------------|
| PA2  | USART2_TX     | AF7      | Debug output via ST-Link VCP         |
| PA3  | USART2_RX     | AF7      | Debug input (unused currently)       |
| PA5  | LD2 (green)   | GPIO     | Status LED                           |
| PA11 | CAN1_RX       | AF9      | CAN transceiver RX                   |
| PA12 | CAN1_TX       | AF9      | CAN transceiver TX                   |

### 11.3 Example Code Snippets

**Pi — send a torque command (Python):**

```python
import can_common, can_motor
bus = can_common.create_bus("can1")
can_motor.send_torque_cmd(bus, can_common.NODE_LEFT_HIP, 2.5)  # 2.5 Nm
```

**STM32 — send a torque command (C):**

```c
#include "can_motor.h"
can_send_torque_cmd(CAN_NODE_LEFT_HIP, 2.5f);  // 2.5 Nm
```

**STM32 — main receive loop skeleton:**

```c
CanFrame rx;
while (can_recv(&rx)) {
    if (rx.is_extended) {
        motor_receive(&motor_status, rx.data);
    } else if (can_get_msg_type(rx.id) == CAN_MSG_TORQUE_CMD) {
        float tau;
        if (can_parse_torque_cmd(&rx, &tau)) {
            float iq = clampf(tau / KT_EFFECTIVE, -CURRENT_LIMIT, CURRENT_LIMIT);
            comm_can_set_current(MY_MOTOR_CAN_ID, iq);
        }
    }
}
```

**Pi — bringing up CAN and recovering from bus-off:**

```bash
sudo ip link set can1 down
sudo ip link set can1 up type can bitrate 1000000 restart-ms 100
ip -details link show can1 | grep "can state"   # expect ERROR-ACTIVE
```

### 11.4 Reference Materials

- **CubeMars AK70-9 KV60 product page**
  <https://www.cubemars.com/product/ak70-9-kv60-robotic-actuator.html>
- **VESC firmware CAN command reference** — see `apis/motor/motor-api.md`
  in this repository for the distilled subset this project uses.
- **STM32F446RE reference manual (RM0390)** — ST docs, CAN chapter for
  filter register layouts and bit timing.
- **SocketCAN documentation** — Linux kernel docs at
  `Documentation/networking/can.rst` for `ip link` options, filters, and
  error frames.
- **Related in-repo docs:**
  - `apis/can/CAN_API.md` — the full CAN API specification
  - `docs/torque-controller-flow.md` — state-machine and message-flow detail
  - `docs/old-vs-new-can-comparison.md` — migration guide from the legacy
    `can_bus` pattern
