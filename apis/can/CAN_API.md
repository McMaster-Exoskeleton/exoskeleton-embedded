# CAN API Documentation

## What is CAN?

CAN (Controller Area Network) is a communication protocol designed for embedded systems. It was originally developed for cars, but it's used everywhere multiple microcontrollers need to talk to each other — robots, medical devices, industrial automation, and (in our case) exoskeletons.

### How CAN Works

- **Messages, not addresses:** CAN doesn't send data "to" a specific device. Instead, a node broadcasts a message with an **ID** and a **data payload**. Every node on the bus sees every message and decides whether to process it.
- **ID = priority:** Lower IDs win bus access first. If two nodes try to send at the same time, the one with the lower ID automatically wins (this is called *arbitration*). This means you should assign lower IDs to more important messages.
- **Data payload:** Each message carries 0-8 bytes of data.
- **Two wires:** CAN uses two wires — CAN_H and CAN_L — with differential signaling. This makes it very resistant to electrical noise.
- **Termination:** The bus needs a 120-ohm resistor at each physical end to work properly.
- **No master:** Any node can send at any time. There's no polling or token-passing.

### Standard vs Extended IDs

CAN supports two ID formats:
- **Standard (11-bit):** IDs from 0x000 to 0x7FF. We use these for all inter-node communication.
- **Extended (29-bit):** IDs from 0x00000000 to 0x1FFFFFFF. The AK70-9 motor uses these for its VESC protocol.

Our hardware filters separate the two: standard frames = network messages between nodes, extended frames = motor driver communication.

---

## Our Network

### Topology

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

### Node IDs

| Node | ID | Description |
|------|----|-------------|
| Pi | 0 | Central ML controller |
| STM32 #1 | 1 | Left Hip joint controller |
| STM32 #2 | 2 | Right Hip joint controller |
| STM32 #3 | 3 | Left Knee joint controller |
| STM32 #4 | 4 | Right Knee joint controller |
| *Reserved* | 5-7 | Future expansion |

### Data Flow

1. Each STM32 reads its local IMU (I2C) and motor feedback (VESC extended CAN)
2. Each STM32 broadcasts IMU data and motor status to the Pi via the CAN API
3. The Pi collects 28 inputs (7 per joint: 3 accel + 3 gyro + 1 motor position)
4. The Pi's ML model produces 4 torque values
5. The Pi sends individual torque commands to each STM32
6. Each STM32 receives its torque command and drives its motor

---

## CAN ID Scheme

### 11-bit Standard ID Layout

```
Bit:  10  9  8  7  │  6  5  4  │  3  2  1  0
      ─────────────┼───────────┼─────────────
      Message Type │ Src Node  │ Dest/Context
      (4 bits)     │ (3 bits)  │ (4 bits)
```

### Message Types

| Type | Value | Direction | DLC | Description |
|------|-------|-----------|-----|-------------|
| ESTOP | 0x0 | Any → All | 1 | Emergency stop (highest priority) |
| TORQUE_CMD | 0x1 | Pi → STM32 | 2 | Torque command from ML model |
| MOTOR_STATUS | 0x2 | STM32 → Pi | 8 | Motor position, speed, current, temp, error |
| IMU_ACCEL | 0x3 | STM32 → Pi | 6 | Accelerometer XYZ |
| IMU_GYRO | 0x4 | STM32 → Pi | 6 | Gyroscope XYZ |
| HEARTBEAT | 0x5 | Any → All | 0 | Node alive signal (future use) |

### Example CAN IDs

| Scenario | Type | Src | Dest | CAN ID (hex) |
|----------|------|-----|------|--------------|
| ESTOP from Pi | 0x0 | 0 | 0 | 0x000 |
| Pi → Left Hip torque | 0x1 | 0 | 1 | 0x081 |
| Pi → Right Knee torque | 0x1 | 0 | 4 | 0x084 |
| Left Hip motor status | 0x2 | 1 | 0 | 0x110 |
| Left Hip IMU accel | 0x3 | 1 | 0 | 0x190 |
| Left Hip IMU gyro | 0x4 | 1 | 0 | 0x210 |
| Right Knee IMU accel | 0x3 | 4 | 0 | 0x1C0 |

---

## Message Reference

All CAN API messages use **little-endian** byte order (ARM native).

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
Bytes 0-1: torque (int16, scaled x 1000)
  e.g., 5500 = 5.5 Nm
  Range: +/-32,000 (+/-32 Nm, matching AK70-9 limits)
```

Direction: Pi → specific STM32.

### MOTOR_STATUS (DLC: 8)

```
Bytes 0-1: position    (int16, scaled x 10)    — degrees
Bytes 2-3: speed       (int16, raw x 10 = ERPM) — ERPM
Bytes 4-5: current     (int16, scaled x 100)   — amps
Byte  6:   temperature (int8)                  — degrees C
Byte  7:   error_code  (uint8)                 — MotorErrorCode enum
```

Re-encoded from VESC big-endian feedback into little-endian. Direction: STM32 → Pi.

### IMU_ACCEL (DLC: 6)

```
Bytes 0-1: ax (int16, scaled x 100)  — m/s^2
Bytes 2-3: ay (int16, scaled x 100)  — m/s^2
Bytes 4-5: az (int16, scaled x 100)  — m/s^2
```

Direction: STM32 → Pi.

### IMU_GYRO (DLC: 6)

```
Bytes 0-1: gx (int16, scaled x 100)  — deg/s
Bytes 2-3: gy (int16, scaled x 100)  — deg/s
Bytes 4-5: gz (int16, scaled x 100)  — deg/s
```

Direction: STM32 → Pi.

---

## C API Reference (STM32)

### can_common.h — Transport Layer

#### `can_common_init(hcan, my_node_id)`

Initialize CAN with filters auto-configured for this node. Call after `MX_CAN1_Init()`.

```c
// In main.c, after MX_CAN1_Init():
can_common_init(&hcan1, CAN_NODE_LEFT_HIP);
```

#### `can_send_std(std_id, data, dlc)`

Send a standard-ID CAN frame. Used internally by domain modules.

```c
uint8_t payload[2] = {0x10, 0x27};
can_send_std(0x081, payload, 2);
```

#### `can_send_ext(ext_id, data, dlc)`

Send an extended-ID CAN frame. Used by the motor API (ak70_9) for VESC protocol.

```c
uint8_t cmd[4] = {0x00, 0x00, 0x15, 0x7C};
can_send_ext(0x00000168, cmd, 4);
```

#### `can_recv(out)`

Pop the next frame from the receive ring buffer. Returns 1 if a frame was retrieved, 0 if empty.

```c
CanFrame frame;
if (can_recv(&frame)) {
    // process frame
}
```

#### `can_get_my_node_id()`

Returns the node ID set during `can_common_init()`.

#### ID Macros

```c
// Build a CAN ID
uint16_t id = CAN_BUILD_ID(CAN_MSG_IMU_ACCEL, CAN_NODE_LEFT_HIP, CAN_NODE_PI);
// id = 0x190

// Extract fields
uint8_t type = can_get_msg_type(0x190);  // 3 (IMU_ACCEL)
uint8_t src  = can_get_src_node(0x190);  // 1 (LEFT_HIP)
uint8_t dest = can_get_dest(0x190);      // 0 (PI)
```

### can_system.h — ESTOP

#### `can_send_estop(src_node, reason)`

Send an ESTOP broadcast.

```c
can_send_estop(CAN_NODE_LEFT_HIP, CAN_ESTOP_MOTOR_ERROR);
```

#### `can_parse_estop(frame, &reason)`

Parse an ESTOP frame. Returns 1 on success.

```c
uint8_t reason;
if (can_parse_estop(&frame, &reason)) {
    // handle estop
}
```

### can_imu.h — IMU Data

#### `can_send_imu_accel(src_node, ax, ay, az)`

Send accelerometer data in m/s^2.

```c
can_send_imu_accel(CAN_NODE_LEFT_HIP, 0.12f, -9.81f, 0.05f);
```

#### `can_send_imu_gyro(src_node, gx, gy, gz)`

Send gyroscope data in deg/s.

```c
can_send_imu_gyro(CAN_NODE_LEFT_HIP, 1.5f, -0.3f, 0.8f);
```

#### `can_parse_imu_accel(frame, &ax, &ay, &az)` / `can_parse_imu_gyro(frame, &gx, &gy, &gz)`

Parse IMU frames. Returns 1 on success.

```c
float ax, ay, az;
if (can_parse_imu_accel(&frame, &ax, &ay, &az)) {
    // use ax, ay, az
}
```

### can_motor.h — Motor Messages

#### `can_send_torque_cmd(dest_node, torque_nm)`

Send a torque command. Source is implicitly `my_node_id`.

```c
can_send_torque_cmd(CAN_NODE_LEFT_HIP, 5.5f);  // 5.5 Nm
```

#### `can_parse_torque_cmd(frame, &torque_nm)`

Parse a torque command. Returns 1 on success.

```c
float torque;
if (can_parse_torque_cmd(&frame, &torque)) {
    comm_can_set_current(MOTOR_CAN_ID, torque);
}
```

#### `can_send_motor_status(src_node, position, speed, current, temperature, error)`

Send motor status to the Pi. Values re-encoded from VESC big-endian.

```c
can_send_motor_status(CAN_NODE_LEFT_HIP,
    status.position, status.speed, status.current,
    status.temperature, status.error_code);
```

#### `can_parse_motor_status(frame, &position, &speed, &current, &temperature, &error)`

Parse motor status. Returns 1 on success.

```c
float pos, spd, cur;
int8_t temp;
uint8_t err;
if (can_parse_motor_status(&frame, &pos, &spd, &cur, &temp, &err)) {
    // use motor data
}
```

---

## Python API Reference (Raspberry Pi)

### can_common — Transport Layer

```python
from can_api import can_common

bus = can_common.create_bus("can0")

# Build/parse IDs
can_id = can_common.build_can_id(can_common.MSG_TORQUE_CMD, can_common.NODE_PI, can_common.NODE_LEFT_HIP)
msg_type, src, dest = can_common.parse_can_id(can_id)

# Receive
msg = can_common.recv(bus, timeout=0.1)
```

### can_system — ESTOP

```python
from can_api import can_common, can_system

bus = can_common.create_bus()

# Send
can_system.send_estop(bus, can_common.NODE_PI, can_system.ESTOP_MANUAL)

# Parse
reason = can_system.parse_estop(msg)
```

### can_imu — IMU Data

```python
from can_api import can_common, can_imu

bus = can_common.create_bus()

# Send (for testing — normally STM32 sends these)
can_imu.send_imu_accel(bus, can_common.NODE_LEFT_HIP, 0.12, -9.81, 0.05)
can_imu.send_imu_gyro(bus, can_common.NODE_LEFT_HIP, 1.5, -0.3, 0.8)

# Parse
ax, ay, az = can_imu.parse_imu_accel(msg)
gx, gy, gz = can_imu.parse_imu_gyro(msg)
```

### can_motor — Motor Messages

```python
from can_api import can_common, can_motor

bus = can_common.create_bus()

# Send torque command
can_motor.send_torque_cmd(bus, can_common.NODE_LEFT_HIP, 5.5)

# Parse torque command
torque = can_motor.parse_torque_cmd(msg)

# Send motor status (for testing — normally STM32 sends this)
can_motor.send_motor_status(bus, can_common.NODE_LEFT_HIP, 45.0, 1000.0, 2.5, 35, 0)

# Parse motor status
position, speed, current, temperature, error = can_motor.parse_motor_status(msg)
```

---

## Quick Start Guides

### Guide 1: Pi Sends Torque, STM32 Receives

**STM32 side (main.c):**

```c
#include "can_common.h"
#include "can_motor.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_CAN1_Init();

    can_common_init(&hcan1, CAN_NODE_LEFT_HIP);

    while (1) {
        CanFrame frame;
        if (can_recv(&frame)) {
            float torque;
            if (can_parse_torque_cmd(&frame, &torque)) {
                // Drive motor with received torque
                comm_can_set_current(MOTOR_CAN_ID, torque);
            }
        }
    }
}
```

**Pi side:**

```python
from can_api import can_common, can_motor

bus = can_common.create_bus("can0")
can_motor.send_torque_cmd(bus, can_common.NODE_LEFT_HIP, 5.5)
print("Sent 5.5 Nm to Left Hip")
```

### Guide 2: STM32 Sends IMU, Pi Receives

**STM32 side (main.c):**

```c
#include "can_common.h"
#include "can_imu.h"
#include "lsm6ds3tr.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_CAN1_Init();
    MX_I2C1_Init();

    can_common_init(&hcan1, CAN_NODE_LEFT_HIP);
    lsm6ds3tr_init(&hi2c1);

    while (1) {
        float ax, ay, az, gx, gy, gz;
        lsm6ds3tr_read_accel(&ax, &ay, &az);
        lsm6ds3tr_read_gyro(&gx, &gy, &gz);

        can_send_imu_accel(CAN_NODE_LEFT_HIP, ax, ay, az);
        can_send_imu_gyro(CAN_NODE_LEFT_HIP, gx, gy, gz);

        HAL_Delay(2);  // 500 Hz
    }
}
```

**Pi side:**

```python
from can_api import can_common, can_imu

bus = can_common.create_bus("can0")

while True:
    msg = can_common.recv(bus, timeout=0.1)
    if msg is None:
        continue

    msg_type, src, dest = can_common.parse_can_id(msg.arbitration_id)

    if msg_type == can_common.MSG_IMU_ACCEL:
        ax, ay, az = can_imu.parse_imu_accel(msg)
        print(f"Node {src} accel: ({ax:.2f}, {ay:.2f}, {az:.2f}) m/s^2")

    elif msg_type == can_common.MSG_IMU_GYRO:
        gx, gy, gz = can_imu.parse_imu_gyro(msg)
        print(f"Node {src} gyro: ({gx:.2f}, {gy:.2f}, {gz:.2f}) deg/s")
```

### Guide 3: Full Chain — Pi to STM32 to Motor

**STM32 side (main.c):**

```c
#include "can_common.h"
#include "can_motor.h"
#include "can/ak70_9.h"

#define MOTOR_CAN_ID 104

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_CAN1_Init();

    can_common_init(&hcan1, CAN_NODE_LEFT_HIP);

    while (1) {
        CanFrame frame;
        if (can_recv(&frame)) {
            if (frame.is_extended) {
                // Motor feedback — parse and relay to Pi
                MotorStatus status;
                motor_receive(&status, frame.data);
                can_send_motor_status(CAN_NODE_LEFT_HIP,
                    status.position, status.speed, status.current,
                    status.temperature, status.error_code);
            } else {
                // Network message — check for torque command
                float torque;
                if (can_parse_torque_cmd(&frame, &torque)) {
                    comm_can_set_current(MOTOR_CAN_ID, torque);
                }
            }
        }
    }
}
```

**Pi side:**

```python
from can_api import can_common, can_motor
import time

bus = can_common.create_bus("can0")

# Send torque
can_motor.send_torque_cmd(bus, can_common.NODE_LEFT_HIP, 3.0)

# Receive motor status
while True:
    msg = can_common.recv(bus, timeout=0.1)
    if msg is None:
        continue

    msg_type, src, dest = can_common.parse_can_id(msg.arbitration_id)

    if msg_type == can_common.MSG_MOTOR_STATUS:
        pos, spd, cur, temp, err = can_motor.parse_motor_status(msg)
        print(f"Motor {src}: pos={pos:.1f}deg spd={spd:.0f}ERPM "
              f"cur={cur:.2f}A temp={temp}C err={err}")
```

### Guide 4: Multi-Node Filtered

Each STM32 initializes with its own node ID. The hardware filter (bank 1) ensures each STM32 only receives torque commands addressed to it.

**STM32 #1 — Left Hip:**

```c
can_common_init(&hcan1, CAN_NODE_LEFT_HIP);   // Filters accept TORQUE_CMD to node 1
```

**STM32 #3 — Left Knee:**

```c
can_common_init(&hcan1, CAN_NODE_LEFT_KNEE);  // Filters accept TORQUE_CMD to node 3
```

**Pi sends 4 different torques:**

```python
from can_api import can_common, can_motor

bus = can_common.create_bus("can0")

# Each STM32 only receives its own command — hardware filtering
can_motor.send_torque_cmd(bus, can_common.NODE_LEFT_HIP,   5.0)
can_motor.send_torque_cmd(bus, can_common.NODE_RIGHT_HIP,  4.5)
can_motor.send_torque_cmd(bus, can_common.NODE_LEFT_KNEE,  6.0)
can_motor.send_torque_cmd(bus, can_common.NODE_RIGHT_KNEE, 5.5)
```

### Guide 5: STM32 to STM32

Direct inter-node communication is possible but requires adding a filter for the specific message type. By default, STM32 filters only accept ESTOP, TORQUE_CMD (to self), and extended frames.

To receive IMU data from another STM32, you would need to add an additional filter bank in `can_common_init()` or modify the existing filter configuration.

---

## Hardware Setup

### MCP2515 HAT (Raspberry Pi)

Add to `/boot/config.txt`:

```
dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25
```

Reboot after adding.

### Bring Up CAN Interface

```bash
sudo ip link set can0 up type can bitrate 1000000
```

To check status:

```bash
ip -details link show can0
```

### CAN Bus Wiring

- Connect CAN_H to CAN_H on all nodes
- Connect CAN_L to CAN_L on all nodes
- **Do not** cross CAN_H and CAN_L

### Termination

Place a 120-ohm resistor between CAN_H and CAN_L at each physical end of the bus. For a short bus (< 1m), one termination resistor on the Pi's HAT may be sufficient for testing, but production should have both ends terminated.

### Debugging with can-utils

```bash
# Install
sudo apt install can-utils

# Monitor all traffic
candump can0

# Send a test frame (Pi sends torque 5.5 Nm to Left Hip)
# ID 0x081, data: 0x7C15 (5500 in little-endian)
cansend can0 081#7C15

# Monitor with decoded IDs
candump can0 -c -t a
```

---

## Troubleshooting

### "No messages received"

- **Is the bus up?** Run `ip link show can0` — state should be "UP".
- **Bitrate match?** All nodes must use 1 Mbit/s. STM32 config: prescaler 6, BS1 11TQ, BS2 2TQ.
- **Wiring:** Check CAN_H to CAN_H, CAN_L to CAN_L. No crossover.
- **Termination:** At least one 120-ohm resistor between CAN_H and CAN_L.
- **Power:** All nodes must be powered and running.

### "TX mailbox full" (can_send returns 0)

- Sending too fast — the 3 TX mailboxes are all occupied.
- Add a small delay between sends, or check `HAL_CAN_GetTxMailboxesFreeLevel()`.
- Could indicate bus errors preventing transmission — check CAN error counters.

### "Wrong data values"

- **Endianness:** All CAN API messages are little-endian. VESC motor protocol is big-endian.
- **Scaling:** Check the scaling factor — torque is x1000, IMU is x100, position is x10.
- **Byte order:** Use `memcpy` for multi-byte values, not pointer casts (avoids alignment issues).

### "Filter not matching"

- Verify the node ID passed to `can_common_init()` matches what the sender is targeting.
- ESTOP uses filter bank 0 (matches any ESTOP regardless of source/dest).
- TORQUE_CMD uses filter bank 1 (matches type + dest only, ignores source).
- All extended frames are accepted by filter bank 2.

### "VESC motor not responding"

- Extended frames use filter bank 2 (FIFO1). Make sure `can_common_init()` was called.
- Motor CAN ID must match what's configured in the VESC firmware (default: 104).
- VESC has a ~1-2 second command timeout. Re-send commands every 50 ms.

### "Old firmware compatibility"

The old IDs (0x123, 0x124) conflict with the new ID scheme. **All nodes must run the new firmware.** Do not mix old and new firmware on the same bus.
