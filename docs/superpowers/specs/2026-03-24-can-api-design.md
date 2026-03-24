# CAN API Design Spec

## Overview

A dual-platform CAN communication API (C for STM32, Python for Raspberry Pi) that provides the network communication layer for the McMaster Exoskeleton. The API enables structured message passing between 1 Raspberry Pi, 4 STM32 microcontrollers, and 4 AK70-9 motors on a single CAN bus.

## System Context

### Network Topology

```
                    ┌──────────────┐
                    │ Raspberry Pi │  (ML model: 28 inputs → 4 torques)
                    │   Node 0     │
                    └──────┬───────┘
                           │
    ═══════════════════════════════════════════  CAN Bus (1 Mbit/s)
        │            │            │            │
   ┌────┴────┐  ┌────┴────┐  ┌────┴────┐  ┌────┴────┐
   │ STM32 #1│  │ STM32 #2│  │ STM32 #3│  │ STM32 #4│
   │ Node 1  │  │ Node 2  │  │ Node 3  │  │ Node 4  │
   │ L. Hip  │  │ R. Hip  │  │ L. Knee │  │ R. Knee │
   └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘
        │            │            │            │
   ┌────┴────┐  ┌────┴────┐  ┌────┴────┐  ┌────┴────┐
   │ AK70-9  │  │ AK70-9  │  │ AK70-9  │  │ AK70-9  │
   │ Motor   │  │ Motor   │  │ Motor   │  │ Motor   │
   └─────────┘  └─────────┘  └─────────┘  └─────────┘
```

### Data Flow

1. Each STM32 reads its local IMU (I2C) and motor feedback (VESC extended CAN)
2. Each STM32 broadcasts IMU data and motor status to the Pi via the CAN API (standard 11-bit IDs)
3. The Pi collects 28 inputs (7 per joint: 3 accel + 3 gyro + 1 motor position) from all 4 joints
4. The Pi's ML model produces 4 torque values
5. The Pi sends individual torque commands to each STM32 via the CAN API
6. Each STM32 receives its torque command and drives its motor via the existing ak70_9 API

### Relationship to Existing APIs

```
┌─────────────────────────────────────────────┐
│           Application (main.c)              │
├──────────┬──────────┬───────────────────────┤
│ ak70_9   │ lsm6ds3  │  (future devices)     │  ← Device APIs (unchanged)
│ motor API│ IMU API  │                       │
├──────────┴──────────┴───────────────────────┤
│              CAN API (new)                  │  ← Network transport + message protocol
│  can_common / can_imu / can_motor / can_sys │
├─────────────────────────────────────────────┤
│           STM32 HAL CAN / SocketCAN         │  ← Hardware layer
└─────────────────────────────────────────────┘
```

The CAN API does NOT replace the device APIs. It is the network transport layer:
- `lsm6ds3tr.c` reads IMU data from hardware → CAN API sends it to the Pi
- Pi receives torque command via CAN API → STM32 application calls `ak70_9.c` to drive the motor

## CAN ID Scheme

### 11-bit Standard ID Layout

```
Bit:  10  9  8  7  │  6  5  4  │  3  2  1  0
      ─────────────┼───────────┼─────────────
      Message Type │ Src Node  │ Dest/Context
      (4 bits)     │ (3 bits)  │ (4 bits)
```

### Node IDs

| Node | ID | Description |
|------|----|-------------|
| Pi | 0 | Central ML controller |
| STM32 #1 | 1 | Left Hip joint controller |
| STM32 #2 | 2 | Right Hip joint controller |
| STM32 #3 | 3 | Left Knee joint controller |
| STM32 #4 | 4 | Right Knee joint controller |
| *Reserved* | 5-7 | Future expansion |

### Message Types

Ordered by CAN priority (lower value = higher bus priority):

| Type | Value | Direction | DLC | Description |
|------|-------|-----------|-----|-------------|
| ESTOP | 0x0 | Any → All | 1 | Emergency stop (highest priority) |
| TORQUE_CMD | 0x1 | Pi → STM32 | 2 | Torque command from ML model |
| MOTOR_STATUS | 0x2 | STM32 → Pi | 8 | Motor position, speed, current, temp, error |
| IMU_ACCEL | 0x3 | STM32 → Pi | 6 | Accelerometer XYZ |
| IMU_GYRO | 0x4 | STM32 → Pi | 6 | Gyroscope XYZ |
| HEARTBEAT | 0x5 | Any → All | 0 | Node alive signal (future use) |
| *Reserved* | 0x6-0xF | — | — | Future expansion |

### Example CAN IDs

| Scenario | Type | Src | Dest | CAN ID (hex) |
|----------|------|-----|------|--------------|
| ESTOP from Pi | 0x0 | 0 | 0 | 0x000 |
| ESTOP from Left Hip | 0x0 | 1 | 0 | 0x010 |
| Pi → Left Hip torque | 0x1 | 0 | 1 | 0x081 |
| Pi → Right Knee torque | 0x1 | 0 | 4 | 0x084 |
| Left Hip motor status | 0x2 | 1 | 0 | 0x110 |
| Left Hip IMU accel | 0x3 | 1 | 0 | 0x190 |
| Left Hip IMU gyro | 0x4 | 1 | 0 | 0x210 |
| Right Knee IMU accel | 0x3 | 4 | 0 | 0x1C0 |

### Why Standard 11-bit (not Extended 29-bit)?

- STM32 ↔ Motor communication retains extended 29-bit IDs (VESC protocol, unchanged)
- All inter-node communication (STM32 ↔ Pi, STM32 ↔ STM32) uses standard 11-bit IDs
- This allows hardware filtering: standard frames = network messages, extended frames = motor feedback
- 11 bits provides 16 message types × 8 nodes × 16 destinations = more than enough

### Migration from Legacy IDs

The existing production firmware uses hardcoded standard CAN IDs:
- `0x123` — IMU accelerometer data
- `0x124` — IMU gyroscope data

Under the new ID scheme, these values fall within the MOTOR_STATUS range and would be misinterpreted. **All nodes must be updated to the new API simultaneously.** The old hardcoded IDs are fully replaced by the structured IDs from this spec (e.g., Left Hip IMU accel becomes `0x190` instead of `0x123`). Do not run old and new firmware on the same bus at the same time.

## Message Data Formats

All CAN API messages use **little-endian** byte order (ARM native).

**Note on VESC motor protocol:** The VESC motor driver protocol (used by ak70_9 over extended CAN frames) uses big-endian byte order. The MOTOR_STATUS message in this API is NOT a raw passthrough of VESC data — the STM32 application reads motor feedback via `ak70_9.c` (which decodes big-endian VESC frames), then re-encodes the values into little-endian format when calling `can_send_motor_status()`. This keeps all inter-node messages consistently little-endian.

### ESTOP (DLC: 1)

```
Byte 0: reason (uint8)
  0 = manual
  1 = comm_loss
  2 = motor_error
  3 = software
```

Broadcast to all nodes. Any node can send. No response expected.

### TORQUE_CMD (DLC: 2)

```
Bytes 0-1: torque (int16, scaled × 1000)
  e.g., 5500 = 5.5 Nm
  Range: ±32,000 (±32 Nm, matching AK70-9 limits)
  int16 range of ±32,767 covers the full motor torque range at this scale
```

Direction: Pi → specific STM32.

### MOTOR_STATUS (DLC: 8)

```
Bytes 0-1: position    (int16, scaled × 10)     — degrees
Bytes 2-3: speed       (int16, raw × 10 = ERPM)  — ERPM
Bytes 4-5: current     (int16, scaled × 100)    — amps
Byte  6:   temperature (int8)                   — degrees C
Byte  7:   error_code  (uint8)                  — MotorErrorCode enum
```

Re-encoded from VESC big-endian feedback into little-endian. Direction: STM32 → Pi.

### IMU_ACCEL (DLC: 6)

```
Bytes 0-1: ax (int16, scaled × 100)  — m/s^2
Bytes 2-3: ay (int16, scaled × 100)  — m/s^2
Bytes 4-5: az (int16, scaled × 100)  — m/s^2
```

Matches existing format. Direction: STM32 → Pi.

### IMU_GYRO (DLC: 6)

```
Bytes 0-1: gx (int16, scaled × 100)  — deg/s
Bytes 2-3: gy (int16, scaled × 100)  — deg/s
Bytes 4-5: gz (int16, scaled × 100)  — deg/s
```

Matches existing format. Direction: STM32 → Pi.

## File Structure

```
apis/can/
├── Inc/
│   ├── can_common.h       # Node IDs, message type IDs, ID builder macros,
│   │                      # init/send/recv transport, CanFrame struct, ring buffer
│   ├── can_imu.h          # can_send_imu_accel(), can_send_imu_gyro(),
│   │                      # can_parse_imu_accel(), can_parse_imu_gyro()
│   ├── can_motor.h        # can_send_torque_cmd(), can_parse_torque_cmd(),
│   │                      # can_send_motor_status(), can_parse_motor_status()
│   └── can_system.h       # can_send_estop(), can_parse_estop()
├── Src/
│   ├── can_common.c       # HAL CAN init, filter config, send/recv, ring buffer, ISR
│   ├── can_imu.c          # IMU message encoding/decoding
│   ├── can_motor.c        # Torque cmd & motor status encoding/decoding
│   └── can_system.c       # ESTOP encoding/decoding
├── python/
│   ├── __init__.py        # Re-exports everything for `from can_api import *`
│   ├── can_common.py      # SocketCAN bus setup, node/message ID constants, send/recv
│   ├── can_imu.py         # send_imu_accel(), parse_imu_accel(), etc.
│   ├── can_motor.py       # send_torque_cmd(), parse_torque_cmd(), etc.
│   └── can_system.py      # send_estop(), parse_estop()
└── CAN_API.md             # Full documentation
```

## C API (STM32 Side)

### can_common.h

```c
/* ── Node IDs ── */
#define CAN_NODE_PI          0
#define CAN_NODE_LEFT_HIP    1
#define CAN_NODE_RIGHT_HIP   2
#define CAN_NODE_LEFT_KNEE   3
#define CAN_NODE_RIGHT_KNEE  4

/* ── Message Type IDs ── */
#define CAN_MSG_ESTOP        0x0
#define CAN_MSG_TORQUE_CMD   0x1
#define CAN_MSG_MOTOR_STATUS 0x2
#define CAN_MSG_IMU_ACCEL    0x3
#define CAN_MSG_IMU_GYRO     0x4
#define CAN_MSG_HEARTBEAT    0x5

/* ── ID Construction / Extraction ── */
#define CAN_BUILD_ID(msg_type, src, dest) \
    (((uint16_t)(msg_type) << 7) | ((uint16_t)(src) << 4) | ((uint16_t)(dest)))

uint8_t can_get_msg_type(uint16_t can_id);  // bits [10:7]
uint8_t can_get_src_node(uint16_t can_id);  // bits [6:4]
uint8_t can_get_dest(uint16_t can_id);      // bits [3:0]

/* ── CAN Frame ── */
typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  is_extended;
    uint8_t  data[8];
} CanFrame;

/* ── Ring Buffer (RX) ── */
#define CAN_RX_BUFFER_CAPACITY 32

typedef struct {
    CanFrame buf[CAN_RX_BUFFER_CAPACITY];
    volatile uint16_t head, tail, count;
} CanRxRingBuffer;

/* ── Transport Functions ── */
int  can_common_init(CAN_HandleTypeDef *hcan, uint8_t my_node_id);
int  can_send_std(uint16_t std_id, const uint8_t *data, uint8_t dlc);
int  can_recv(CanFrame *out);
```

### can_imu.h

```c
int can_send_imu_accel(uint8_t src_node, float ax, float ay, float az);
int can_send_imu_gyro(uint8_t src_node, float gx, float gy, float gz);
int can_parse_imu_accel(const CanFrame *frame, float *ax, float *ay, float *az);
int can_parse_imu_gyro(const CanFrame *frame, float *gx, float *gy, float *gz);
```

### can_motor.h

```c
int can_send_torque_cmd(uint8_t dest_node, float torque_nm);
int can_parse_torque_cmd(const CanFrame *frame, float *torque_nm);
int can_send_motor_status(uint8_t src_node, float position, float speed,
                          float current, int8_t temperature, uint8_t error);
int can_parse_motor_status(const CanFrame *frame, float *position, float *speed,
                           float *current, int8_t *temperature, uint8_t *error);
```

### can_system.h

```c
#define CAN_ESTOP_MANUAL      0
#define CAN_ESTOP_COMM_LOSS   1
#define CAN_ESTOP_MOTOR_ERROR 2
#define CAN_ESTOP_SOFTWARE    3

int can_send_estop(uint8_t src_node, uint8_t reason);
int can_parse_estop(const CanFrame *frame, uint8_t *reason);
```

### Implicit Node ID

`can_common_init(hcan, my_node_id)` stores `my_node_id` internally. Send functions use it as follows:
- `can_send_imu_accel(src_node, ...)` — caller passes `src_node` explicitly (typically `my_node_id`)
- `can_send_torque_cmd(dest_node, ...)` — source is implicitly `my_node_id` (stored at init)
- `can_send_motor_status(src_node, ...)` — caller passes `src_node` explicitly
- `can_send_estop(src_node, ...)` — caller passes `src_node` explicitly

This allows flexibility for testing (send as any node) while keeping the common case simple.

### Return Values

All `can_send_*` and `can_parse_*` functions return:
- `1` on success
- `0` on failure (TX mailbox full, no frame available, invalid argument)

This matches the existing `can_bus.c` convention where `1` = success and `0` = failure.

## Python API (Raspberry Pi Side)

### can_common.py

```python
# Constants mirror C exactly
NODE_PI, NODE_LEFT_HIP, NODE_RIGHT_HIP = 0, 1, 2
NODE_LEFT_KNEE, NODE_RIGHT_KNEE = 3, 4

MSG_ESTOP, MSG_TORQUE_CMD, MSG_MOTOR_STATUS = 0x0, 0x1, 0x2
MSG_IMU_ACCEL, MSG_IMU_GYRO, MSG_HEARTBEAT = 0x3, 0x4, 0x5

def build_can_id(msg_type: int, src_node: int, dest: int) -> int
def parse_can_id(can_id: int) -> tuple[int, int, int]  # (msg_type, src, dest)
def create_bus(channel: str = "can0") -> can.Bus  # bitrate configured at OS level via `ip link`
def recv(bus: can.Bus, timeout: float = 0.01) -> can.Message | None
```

### can_imu.py

```python
def send_imu_accel(bus: can.Bus, src_node: int, ax: float, ay: float, az: float)
def send_imu_gyro(bus: can.Bus, src_node: int, gx: float, gy: float, gz: float)
def parse_imu_accel(msg: can.Message) -> tuple[float, float, float]
def parse_imu_gyro(msg: can.Message) -> tuple[float, float, float]
```

### can_motor.py

```python
def send_torque_cmd(bus: can.Bus, dest_node: int, torque_nm: float)
def parse_torque_cmd(msg: can.Message) -> float
def send_motor_status(bus: can.Bus, src_node: int, position: float, speed: float,
                      current: float, temperature: int, error: int)
def parse_motor_status(msg: can.Message) -> tuple[float, float, float, int, int]  # (position, speed, current, temp, error)
```

### can_system.py

```python
ESTOP_MANUAL, ESTOP_COMM_LOSS, ESTOP_MOTOR_ERROR, ESTOP_SOFTWARE = 0, 1, 2, 3

def send_estop(bus: can.Bus, src_node: int, reason: int = ESTOP_MANUAL)
def parse_estop(msg: can.Message) -> int  # reason code
```

## CAN Filter Configuration

### STM32 (Hardware Filters)

`can_common_init()` auto-configures 3 filter banks based on `my_node_id`:

| Filter Bank | Mode | Scale | FIFO | ID Register | Mask Register | Accepts |
|-------------|------|-------|------|-------------|---------------|---------|
| 0 | Mask | 32-bit | FIFO0 | `0x000 << 5` (ESTOP type) | `0x780 << 5` (type bits only) | All ESTOP messages (highest priority, FIFO0) |
| 1 | Mask | 32-bit | FIFO0 | `(0x080 \| my_node_id) << 5` | `0x78F << 5` (type + dest bits) | TORQUE_CMD where dest = my_node_id |
| 2 | Mask | 32-bit | FIFO1 | IDE bit set | IDE bit set, ID mask = 0 | All extended frames (VESC motor feedback) |

**Notes:**
- STM32 bxCAN filter registers require the 11-bit standard ID to be left-shifted by 5 bits
- Filter 2 must set the IDE bit (bit 2 of `FilterIdLow`) to match only extended frames
- ESTOP and TORQUE_CMD use FIFO0 (priority traffic). Extended frames (motor feedback) use FIFO1 to prevent overflow under high bus load
- `SlaveStartFilterBank` = 14 (CAN1-only configuration)
- Both FIFO0 and FIFO1 RX interrupts are enabled; both feed into the same ring buffer

### Raspberry Pi (Software Filtering)

The Pi accepts all standard frames and dispatches by message type in software:
```python
msg_type, src, dest = can_common.parse_can_id(msg.arbitration_id)
# switch on msg_type to call appropriate parse function
```

## Documentation (CAN_API.md)

The documentation file will contain:

1. **What is CAN?** — Plain-English explanation for beginners. What it is, why robotics uses it, how messages work (ID + data), bus arbitration basics. No deep electrical theory.
2. **Our Network** — Topology diagram, node IDs, roles, data flow.
3. **Message Reference** — All message types with IDs, payloads, byte layouts.
4. **C API Reference** — Every function signature, one-line description, minimal usage example.
5. **Python API Reference** — Same format, Python equivalents.
6. **Quick Start Guides:**
   - Pi to STM32 (send torque, receive on STM32)
   - STM32 to Pi (send IMU data, receive on Pi)
   - Pi to STM32 to Motor (full chain)
   - Multi-node filtered (Pi sends 4 different torques, each STM32 only receives its own)
   - STM32 to STM32 (direct inter-node communication)
7. **Hardware Setup** — MCP2515 HAT config, CAN wiring, termination resistors, `ip link` commands.
8. **Troubleshooting** — Common issues and solutions.

## Timing and Rates

| Message | Recommended Rate | Notes |
|---------|-----------------|-------|
| IMU_ACCEL | 500 Hz (every 2 ms) | Matches current firmware loop rate |
| IMU_GYRO | 500 Hz (every 2 ms) | Sent in same loop iteration as accel |
| MOTOR_STATUS | 100-500 Hz | Matches VESC feedback rate (configurable on motor) |
| TORQUE_CMD | Matches IMU rate | Pi sends after each ML inference cycle |
| ESTOP | On-demand | Sent immediately when triggered |
| HEARTBEAT | Future use | Not yet implemented |

**Bus bandwidth check:** At 1 Mbit/s, a standard CAN frame takes ~76-130 us depending on DLC (DLC 2 ≈ 76 us, DLC 6 ≈ 114 us, DLC 8 ≈ 130 us). With 4 nodes each sending IMU_ACCEL (DLC 6) + IMU_GYRO (DLC 6) + MOTOR_STATUS (DLC 8) at 500 Hz, plus 4 TORQUE_CMD (DLC 2) at 500 Hz, total bus utilization is approximately 85-90%. This leaves very little margin for retransmissions or bus errors. Consider reducing IMU/motor status rate to 200-250 Hz for a safer ~40-50% bus utilization.

**VESC command timeout:** The VESC firmware has a ~1-2 second internal command timeout. The application layer (not the CAN API) is responsible for re-sending torque commands to keep motors active. The existing `uart_cmd_refresh_tick()` pattern (re-send every 50 ms) should be adapted for CAN torque commands.

## Coexistence with Existing Code

- `can_common.c` **supersedes** the existing `can_bus.c` in `apis/motor/Src/can/`. The new API handles all CAN init, filtering, send, recv, and ISR callbacks.
- The existing `ak70_9.c` motor API will be updated to call `can_send_std()` / `can_recv()` from `can_common.c` instead of the old `can_bus_send_ext()` / `can_bus_recv()`.
- The `CanFrame` struct in `can_common.h` is identical to the one in `can_frame.h`. The old `can_frame.h` and `ring_buffer.h` are deprecated — their contents are absorbed into `can_common.h`.
- Since the STM32 HAL only allows one `HAL_CAN_RxFifo0MsgPendingCallback` per compilation unit, `can_common.c` owns all CAN ISR callbacks. The motor API receives its extended frames through `can_recv()` like any other consumer.

## Constraints and Decisions

- **Language:** Pure C for STM32 (matches existing codebase, no C++ runtime overhead)
- **Error handling:** Minimal — return codes only, no built-in safety primitives
- **Byte order:** Little-endian for all inter-node messages (ARM native). VESC motor protocol remains big-endian (handled by ak70_9.c)
- **CAN bitrate:** 1 Mbit/s (matches existing configuration)
- **Ring buffer capacity:** 32 frames (matches existing implementation)
- **Standard vs Extended IDs:** Standard 11-bit for inter-node, Extended 29-bit for VESC motor protocol only
- **No dependencies beyond:** STM32 HAL (C side), python-can (Python side)
