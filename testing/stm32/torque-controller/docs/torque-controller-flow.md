# Torque Controller — Flow Documentation

This document describes the control flow of the torque-controller test project,
which forwards torque commands from the Raspberry Pi to an AK70-9 KV60 motor
through an STM32 gateway.

## System Architecture

```
┌─────────────┐   Standard CAN (11-bit)    ┌─────────────┐   Extended CAN (29-bit)   ┌─────────────┐
│             │  TORQUE_CMD (0x081)         │             │  SET_CURRENT (0x168)      │             │
│  Raspberry  │ ──────────────────────────► │    STM32    │ ────────────────────────► │  AK70-9     │
│     Pi      │  float torque_nm            │   Gateway   │  float current_A          │   Motor     │
│  (Node 0)   │                             │  (Node 1-4) │                           │  (CAN 104)  │
│             │ ◄────────────────────────── │             │ ◄──────────────────────── │  (VESC FW)  │
│             │  (future: MOTOR_STATUS)     │             │  Feedback (ext frame)     │             │
└─────────────┘                             └─────────────┘                           └─────────────┘
```

**Bus layout:** all three devices share a single 1 Mbit/s CAN bus with 120-ohm
termination at both physical ends. Standard frames (11-bit IDs) are used for
inter-node messages; extended frames (29-bit IDs) are reserved for the VESC
motor protocol.

---

## CAN ID Scheme

### Standard Frames (Pi ↔ STM32)

11-bit ID layout:

```
Bits [10:7] = Message Type  (4 bits)
Bits [6:4]  = Source Node   (3 bits)
Bits [3:0]  = Dest/Context  (4 bits)
```

| Message Type         | Value | Direction       |
|----------------------|-------|-----------------|
| ESTOP                | 0x0   | Any → broadcast |
| TORQUE_CMD           | 0x1   | Pi → STM32      |
| MOTOR_STATUS (future)| 0x2   | STM32 → Pi      |

Example — Pi (node 0) sends TORQUE_CMD to Left Hip (node 1):
`ID = (0x1 << 7) | (0 << 4) | 1 = 0x081`

### Extended Frames (STM32 ↔ Motor)

29-bit ID layout (VESC convention):

```
Bits [28:8] = Control Mode ID (CAN_PACKET_ID enum)
Bits [7:0]  = Motor Driver ID (104 default)
```

Example — SET_CURRENT to motor 104: `ID = 104 | (1 << 8) = 0x168`

---

## STM32 Filter Configuration

The STM32's hardware CAN filter routes messages to two separate FIFOs so that
motor feedback cannot block Pi command reception.

| Filter Bank | Accepts                               | FIFO  |
|-------------|---------------------------------------|-------|
| 0           | All ESTOP messages                    | FIFO0 |
| 1           | TORQUE_CMD addressed to **this node** | FIFO0 |
| 2           | All extended frames (motor feedback)  | FIFO1 |

Each STM32 only receives torque commands for its own node ID — commands to
other joints are filtered out at the hardware level.

---

## STM32 State Machine

The STM32 firmware maintains four state variables:

| Variable           | Purpose                                    |
|--------------------|--------------------------------------------|
| `motor_status`     | Latest parsed feedback from motor          |
| `active_current`   | Current the STM32 is commanding (A)        |
| `motor_active`     | `1` when periodic refresh is running       |
| `estop_active`     | `1` when ESTOP has been received (latched) |

### State Transitions

```
                                 ┌─────────────────────────────┐
                                 │         BOOT / IDLE         │
                                 │   motor_active = 0          │
                                 │   estop_active = 0          │
                                 │   active_current = 0.0      │
                                 └──────────┬──────────────────┘
                                            │ can_common_init()
                                            │ success → LED blinks 3×
                                            ▼
                                 ┌─────────────────────────────┐
                                 │           READY             │
                                 │    (waiting for commands)   │
                                 └────┬───────────────────┬────┘
                                      │                   │
                          TORQUE_CMD  │                   │  ESTOP
                                      │                   │
                                      ▼                   ▼
              ┌───────────────────────────────┐  ┌────────────────────────┐
              │         COMMANDING            │  │      EMERGENCY STOP    │
              │ motor_active = 1              │  │ active_current = 0     │
              │ active_current = τ / Kt_eff   │  │ motor_active = 0       │
              │ (clamped to ±5A)              │  │ estop_active = 1       │
              │                               │  │ SET_CURRENT(0)         │
              │ Every 50ms:                   │  │                        │
              │  SET_CURRENT(active_current)  │  │ (latched: ignores all  │
              │                               │  │  TORQUE_CMD until      │
              │ On new TORQUE_CMD:            │  │  power cycle)          │
              │  update active_current        │  │                        │
              └──────┬────────────────────────┘  └────────────────────────┘
                     │
                     │ ESTOP received
                     │
                     ▼ (enters EMERGENCY STOP above)
```

---

## Main Loop Sequence

```
┌─────────────────────────────────────────────────────────────────────┐
│                          MAIN LOOP (tight)                          │
└─────────────────────────────────────────────────────────────────────┘

      ┌──────────────────────────────────────────┐
      │  1. Drain CAN RX ring buffer             │
      │                                          │
      │     while (can_recv(&rx_frame)) {        │
      │       if (rx_frame.is_extended)          │
      │           → parse motor feedback         │
      │       else if (type == ESTOP)            │
      │           → zero current, latch estop    │
      │       else if (type == TORQUE_CMD)       │
      │           → convert to current           │
      │           → send SET_CURRENT now         │
      │           → mark motor_active            │
      │     }                                    │
      └──────────────────┬───────────────────────┘
                         │
                         ▼
      ┌──────────────────────────────────────────┐
      │  2. Refresh tick (VESC keepalive)        │
      │                                          │
      │     if (motor_active &&                  │
      │         now - last_refresh >= 50ms) {    │
      │       SET_CURRENT(active_current)        │
      │       last_refresh = now                 │
      │     }                                    │
      └──────────────────┬───────────────────────┘
                         │
                         │ (loop)
                         ▼
```

**Why the 50ms refresh matters:** The VESC firmware has an internal ~1-2 second
command timeout. If no command arrives within that window, the motor coasts.
Re-sending the last commanded current every 50ms keeps the motor actively
driven.

---

## Torque-to-Current Conversion

The AK70-9 KV60 has a motor-shaft torque constant of **Kt = 0.159 Nm/A** and a
**9:1 planetary gearbox**. The effective output torque constant is:

```
Kt_effective = Kt × gear_ratio = 0.159 × 9 = 1.431 Nm/A
```

The STM32 converts commanded torque to q-axis current:

```
I_q = τ_desired / Kt_effective = τ_desired / 1.431
```

**Verification from datasheet:**
- Rated: 6.25 A × 1.431 = 8.94 Nm (datasheet rated 8.5 Nm — ~5% gearbox loss)
- Peak:  23.8 A × 1.431 = 34.0 Nm (datasheet peak 29.2 Nm)

Gearbox efficiency can be added as a multiplier if bench testing reveals a
systematic offset.

---

## Message Flow: One Torque Command

```
Time  Pi                    STM32                   Motor
────  ────                  ─────                   ─────
t=0   send_torque_cmd(1.0)  
                            RX id=0x081 dlc=2
                            parse → τ = 1.0 Nm
                            convert → I = 0.699 A
                            SET_CURRENT(104, 0.699)
                            last_refresh = now                     
                                                    receives cmd
                                                    drives motor
                                                    sends feedback
                            RX id=0x168 (feedback)
                            parse → motor_status

t=50ms                      (no new Pi cmd)
                            refresh tick:
                            SET_CURRENT(104, 0.699)
                                                    receives cmd
                                                    keeps driving

t=100ms                     refresh tick:
                            SET_CURRENT(104, 0.699)
                            ...
```

This continues indefinitely until:
- Pi sends a new TORQUE_CMD → `active_current` updates
- Pi sends `0 Nm` → motor coasts (current = 0, refresh continues)
- Pi sends ESTOP → motor_active = 0, refresh stops, latched off

---

## Integration Notes for Main Joint Controller

When porting this to the production joint controller firmware (e.g.,
`src/stm32/joint-controller/`), keep these patterns:

### 1. Use the new `can_common` API, not the old `can_bus` API

The old `can_bus_init()` uses an accept-all filter that lets motor feedback
flood FIFO0 and block Pi commands. This is the root cause of the "one command
then nothing" bug. Always use `can_common_init(&hcan1, MY_NODE_ID)`.

### 2. Set `MY_NODE_ID` per board

Each STM32 needs a unique node ID. Define at compile time:

```c
#define MY_NODE_ID  CAN_NODE_LEFT_HIP   // 1
// or CAN_NODE_RIGHT_HIP  (2)
// or CAN_NODE_LEFT_KNEE  (3)
// or CAN_NODE_RIGHT_KNEE (4)
```

### 3. Assign unique motor CAN IDs

All four motors default to ID 104. If two are on the same bus, they'll both
respond to `0x168` commands. Assign each motor a unique ID via the VESC Tool:

| Joint       | Suggested Motor CAN ID |
|-------------|------------------------|
| Left Hip    | 101                    |
| Right Hip   | 102                    |
| Left Knee   | 103                    |
| Right Knee  | 104                    |

Then set `MY_MOTOR_CAN_ID` to match in each firmware build.

### 4. Required IRQ handlers in `stm32f4xx_it.c`

```c
void CAN1_RX0_IRQHandler(void) { HAL_CAN_IRQHandler(&hcan1); }
void CAN1_RX1_IRQHandler(void) { HAL_CAN_IRQHandler(&hcan1); }  // motor feedback
void CAN1_SCE_IRQHandler(void) { HAL_CAN_IRQHandler(&hcan1); }  // error recovery
```

Missing `CAN1_RX1_IRQHandler` will cause motor feedback to silently stall.

### 5. Bus-off recovery

Keep `hcan1.Init.AutoBusOff = ENABLE` and `hcan1.Init.AutoRetransmission =
ENABLE`. On the Pi side, always bring up the interface with `restart-ms 100`:

```bash
sudo ip link set can1 up type can bitrate 1000000 restart-ms 100
```

Without this, a transient error can put either node in bus-off with no
recovery.

### 6. ESTOP is latching — by design

Once an ESTOP is received, the STM32 ignores all torque commands until power
cycle. This is a safety feature. If a "soft reset" after ESTOP is ever needed,
add a dedicated `RESUME` message type — don't just allow an ESTOP to time out.

### 7. Future: MOTOR_STATUS relay

The STM32 already parses motor feedback into `motor_status`. To relay back to
the Pi, add a periodic send (e.g., every 20ms):

```c
if (now - last_status_tx >= 20) {
    can_send_motor_status(MY_NODE_ID,
        motor_status.position, motor_status.speed,
        motor_status.current, motor_status.temperature,
        motor_status.error);
    last_status_tx = now;
}
```

---

## File Reference

| File                         | Role                                            |
|------------------------------|-------------------------------------------------|
| `Core/Inc/can_common.h/.c`   | CAN transport layer, filters, ISR, ring buffer  |
| `Core/Inc/can_motor.h/.c`    | TORQUE_CMD / MOTOR_STATUS encoding              |
| `Core/Inc/can/ak70_9.h/.c`   | VESC protocol (SET_CURRENT, feedback parsing)   |
| `Core/Src/main.c`            | Main loop, state machine, torque-to-current    |
| `Core/Src/stm32f4xx_it.c`    | CAN IRQ handlers (RX0, RX1, SCE)                |
| `scripts/torque_cmd.py`      | Interactive Pi-side CLI                         |
| `scripts/can_*.py`           | Pi CAN API (mirrors C API)                      |
