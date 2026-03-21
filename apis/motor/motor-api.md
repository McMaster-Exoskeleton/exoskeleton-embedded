# Motor API — CubeMars AK70-9 KV60 Driver

Hardware API reference for the **CubeMars AK70-9 KV60** brushless motor driver. This API provides servo mode control (7 modes), MIT force/impedance control, and feedback parsing over CAN bus using STM32 HAL.

**Source files:**

| File | Path |
|---|---|
| Motor header | `Inc/can/ak70_9.h` |
| Motor implementation | `Src/can/ak70_9.c` |
| UART command handler header | `Inc/can/uart_cmd.h` |
| UART command handler implementation | `Src/can/uart_cmd.c` |

**Dependencies:**

| File | Path | Description |
|---|---|---|
| CAN bus driver header | `Inc/can/can_bus.h` | CAN TX/RX abstraction (documented separately) |
| CAN bus driver impl | `Src/can/can_bus.c` | |
| CAN frame types | `Inc/can/can_frame.h` | `CanFrame` struct and ring buffer |
| Ring buffer | `Inc/can/ring_buffer.h` | CAN RX ring buffer (header-only) |

---

## Table of Contents

1. [Motor Overview](#motor-overview)
2. [Architecture](#architecture)
3. [Data Structures & Enums](#data-structures--enums)
4. [Servo Mode API](#servo-mode-api)
5. [MIT Force Control API](#mit-force-control-api)
6. [Motor Feedback API](#motor-feedback-api)
7. [UART Command Interface](#uart-command-interface)
8. [CAN Frame Format](#can-frame-format)
9. [Integration Guide](#integration-guide)
10. [Motor Error Codes](#motor-error-codes)
11. [Recommendations for Generalization](#recommendations-for-generalization)

---

## Motor Overview

| Parameter | Value |
|---|---|
| Motor | CubeMars AK70-9 KV60 |
| Interface | CAN bus (29-bit extended identifiers) |
| Default motor CAN ID | `104` (set in `MOTOR_CAN_ID`) |
| Max current | $\pm \ 60 \ A$ |
| Max speed | $\pm \ 100{,}000 \ ERPM$ (electrical RPM) |
| Position range | $\pm \ 36{,}000$ degrees ($\pm \ 100$ full rotations) |
| Torque range | $\pm \ 32 \ Nm$ |
| Temperature feedback | $-20$ to $127$ $\degree C$ |
| Feedback rate | Configurable ($1$–$500 \ Hz$, set on motor) |
| Firmware | VESC-based (has internal command timeout of ~1–2 s) |

---

## Architecture

```
 Host (PC / Python)
    │  UART 115200 8N1
    ▼
 uart_cmd module ──── parses ASCII commands, clamps to test-safe limits
    │
    ├── Read commands ◄── motor_status (cached MotorStatus struct)
    │
    └── Motor commands ──► ak70_9 API ──► can_bus_send_ext() ──► CAN TX
                                                                    │
                                                                    ▼
                                                              AK70-9 Motor
                                                                    │
                                                              CAN feedback
                                                                    │
                                                                    ▼
                               HAL_CAN_RxFifo0MsgPendingCallback()
                                    │
                                    ▼
                              can_bus ring buffer ──► can_bus_recv()
                                                         │
                                                         ▼
                                                   motor_receive()
                                                         │
                                                         ▼
                                                   motor_status (updated)
```

**Two layers of access:**

| Layer | Functions | Use case |
|---|---|---|
| Direct API | `comm_can_set_*()`, `pack_cmd()`, `motor_receive()` | Embedded control loops, custom applications |
| UART command layer | `uart_cmd_process()` | Host-side testing, debugging, Python scripts |

Teams building their own control systems should use the **direct API** functions. The UART command layer is a reference implementation for testing and can be adapted or replaced.

---

## Data Structures & Enums

### `MotorStatus`

Parsed motor feedback. Updated each time a CAN feedback frame is received.

```c
typedef struct {
    float    position;     // Motor position in degrees
    float    speed;        // Motor speed in electrical RPM
    float    current;      // Motor current in amps
    int8_t   temperature;  // Driver board temperature in degrees C
    uint8_t  error;        // Error code (see MotorErrorCode)
} MotorStatus;
```

### `CAN_PACKET_ID`

Control mode identifiers placed in bits `[28:8]` of the extended CAN identifier.

```c
typedef enum {
    CAN_PACKET_SET_DUTY          = 0,  // Duty cycle mode
    CAN_PACKET_SET_CURRENT       = 1,  // Current/torque mode
    CAN_PACKET_SET_CURRENT_BRAKE = 2,  // Current brake mode
    CAN_PACKET_SET_RPM           = 3,  // Velocity (RPM) mode
    CAN_PACKET_SET_POS           = 4,  // Position mode
    CAN_PACKET_SET_ORIGIN_HERE   = 5,  // Set zero position
    CAN_PACKET_SET_POS_SPD       = 6,  // Position-velocity loop
    CAN_PACKET_SET_MIT           = 8,  // MIT force control
} CAN_PACKET_ID;
```

### `MotorErrorCode`

Error codes reported in byte 7 of the motor feedback message.

```c
typedef enum {
    MOTOR_ERROR_NONE              = 0,  // No fault
    MOTOR_ERROR_OVER_TEMP         = 1,  // Motor over-temperature
    MOTOR_ERROR_OVER_CURRENT      = 2,  // Over-current fault
    MOTOR_ERROR_OVER_VOLTAGE      = 3,  // Over-voltage fault
    MOTOR_ERROR_UNDER_VOLTAGE     = 4,  // Under-voltage fault
    MOTOR_ERROR_ENCODER           = 5,  // Encoder fault
    MOTOR_ERROR_MOSFET_OVER_TEMP  = 6,  // MOSFET/driver board over-temperature
    MOTOR_ERROR_STALL             = 7,  // Motor stall / lock-up
} MotorErrorCode;
```

---

## Servo Mode API

All servo mode functions transmit a CAN frame with an extended 29-bit identifier. The control mode ID is placed in bits `[28:8]` and the motor driver ID in bits `[7:0]`. Each function handles the scaling and byte packing internally.

### `comm_can_set_duty` — Duty Cycle Control (Mode 0)

```c
void comm_can_set_duty(uint8_t controller_id, float duty);
```

Drives the motor with a voltage proportional to the duty cycle. This is open-loop control — the motor speed will depend on the load.

| Parameter | Type | Range | Unit |
|---|---|---|---|
| `controller_id` | `uint8_t` | Motor CAN ID | — |
| `duty` | `float` | $-1.0$ to $1.0$ | fraction ($-100\%$ to $+100\%$) |

**CAN payload:** 4 bytes, `duty * 100000` as big-endian `int32`.

**Example:**
```c
comm_can_set_duty(MOTOR_CAN_ID, 0.15f);  // 15% forward duty
comm_can_set_duty(MOTOR_CAN_ID, 0.0f);   // stop (coast)
```

---

### `comm_can_set_current` — Current/Torque Control (Mode 1)

```c
void comm_can_set_current(uint8_t controller_id, float current);
```

Commands a target Iq current. Motor output torque is proportional to current: $\tau = I_q \times K_T$.

| Parameter | Type | Range | Unit |
|---|---|---|---|
| `controller_id` | `uint8_t` | Motor CAN ID | — |
| `current` | `float` | $-60.0$ to $60.0$ | amps |

**CAN payload:** 4 bytes, `current * 1000` as big-endian `int32`.

**Example:**
```c
comm_can_set_current(MOTOR_CAN_ID, 2.5f);   // 2.5 A forward torque
comm_can_set_current(MOTOR_CAN_ID, -1.0f);  // 1.0 A reverse torque
```

---

### `comm_can_set_cb` — Current Brake (Mode 2)

```c
void comm_can_set_cb(uint8_t controller_id, float current);
```

Applies a braking current to resist motion and hold the motor at its current position. **Monitor motor temperature** — sustained braking generates heat.

| Parameter | Type | Range | Unit |
|---|---|---|---|
| `controller_id` | `uint8_t` | Motor CAN ID | — |
| `current` | `float` | $0.0$ to $60.0$ | amps |

**CAN payload:** 4 bytes, `current * 1000` as big-endian `int32`.

**Example:**
```c
comm_can_set_cb(MOTOR_CAN_ID, 3.0f);  // 3 A brake hold
```

---

### `comm_can_set_rpm` — Velocity Control (Mode 3)

```c
void comm_can_set_rpm(uint8_t controller_id, float rpm);
```

Commands the motor to spin at a target electrical RPM. The motor's internal PID controller handles acceleration. Acceleration defaults to the maximum value.

| Parameter | Type | Range | Unit |
|---|---|---|---|
| `controller_id` | `uint8_t` | Motor CAN ID | — |
| `rpm` | `float` | $-100{,}000$ to $100{,}000$ | electrical RPM |

**CAN payload:** 4 bytes, `rpm` cast to big-endian `int32`.

**Note:** Electrical RPM = mechanical RPM $\times$ number of pole pairs. The AK70-9 has 21 pole pairs, so $1{,}000$ ERPM $\approx 47.6$ mechanical RPM.

**Example:**
```c
comm_can_set_rpm(MOTOR_CAN_ID, 5000.0f);  // 5000 ERPM forward
```

---

### `comm_can_set_pos` — Position Control (Mode 4)

```c
void comm_can_set_pos(uint8_t controller_id, float pos);
```

Commands the motor to move to an absolute angular position. Speed and acceleration default to maximum. The position is relative to the current zero/origin point.

| Parameter | Type | Range | Unit |
|---|---|---|---|
| `controller_id` | `uint8_t` | Motor CAN ID | — |
| `pos` | `float` | $-36{,}000$ to $36{,}000$ | degrees |

**CAN payload:** 4 bytes, `pos * 10000` as big-endian `int32`.

**Example:**
```c
comm_can_set_pos(MOTOR_CAN_ID, 90.0f);    // move to 90 degrees
comm_can_set_pos(MOTOR_CAN_ID, -180.0f);  // move to -180 degrees
```

---

### `comm_can_set_origin` — Set Zero Position (Mode 5)

```c
void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode);
```

Sets the current position as the zero/origin reference point. All subsequent position commands will be relative to this origin.

| Parameter | Type | Value | Description |
|---|---|---|---|
| `controller_id` | `uint8_t` | Motor CAN ID | — |
| `set_origin_mode` | `uint8_t` | `0` | Temporary origin (lost on power cycle) |
| | | `1` | Permanent origin (saved to flash) |

**CAN payload:** 1 byte, the mode value.

**Example:**
```c
comm_can_set_origin(MOTOR_CAN_ID, 0);  // temporary zero
comm_can_set_origin(MOTOR_CAN_ID, 1);  // permanent zero (flash write)
```

---

### `comm_can_set_pos_spd` — Position + Velocity + Acceleration (Mode 6)

```c
void comm_can_set_pos_spd(uint8_t controller_id, float pos, int16_t spd, int16_t rpa);
```

Smooth trajectory control with explicit velocity and acceleration limits. Use this instead of `comm_can_set_pos` when you need controlled motion profiles.

| Parameter | Type | Range | Unit | Encoding |
|---|---|---|---|---|
| `controller_id` | `uint8_t` | Motor CAN ID | — | — |
| `pos` | `float` | $-36{,}000$ to $36{,}000$ | degrees | `pos * 10000` → int32 |
| `spd` | `int16_t` | $-32{,}768$ to $32{,}767$ | raw ($\times 10 = ERPM$) | `spd / 10` → int16 |
| `rpa` | `int16_t` | $0$ to $32{,}767$ | raw ($1 \ unit = 10 \ ERPM/s^2$) | `rpa / 10` → int16 |

**CAN payload:** 8 bytes — 4 bytes position + 2 bytes speed + 2 bytes acceleration (all big-endian).

**Example:**
```c
// Move to 180 deg, max 2000 ERPM, acceleration 500 ERPM/s^2
comm_can_set_pos_spd(MOTOR_CAN_ID, 180.0f, 2000, 500);
```

---

## MIT Force Control API

### `pack_cmd` — Impedance/Force Control (Mode 8)

```c
void pack_cmd(uint8_t controller_id, float p_des, float v_des,
              float kp, float kd, float t_ff);
```

Sends a combined position-velocity-torque command. The motor-side control law is:

$$\tau = K_p \cdot (p_{des} - p_{actual}) + K_d \cdot (v_{des} - v_{actual}) + \tau_{ff}$$

This enables compliant force control and impedance control for the exoskeleton. By tuning $K_p$, $K_d$, and $\tau_{ff}$, you can create behaviors ranging from stiff position hold to compliant backdrivability.

| Parameter | Type | Range | Unit | Bit width |
|---|---|---|---|---|
| `p_des` | `float` | $-12.56$ to $12.56$ | radians | 16 bits |
| `v_des` | `float` | $-30.0$ to $30.0$ | rad/s | 12 bits |
| `kp` | `float` | $0$ to $500$ | position gain | 12 bits |
| `kd` | `float` | $0$ to $5.0$ | velocity/damping gain | 12 bits |
| `t_ff` | `float` | $-32.0$ to $32.0$ | Nm (torque feedforward) | 12 bits |

All inputs are clamped to their valid ranges internally before packing.

**CAN payload — bit packing layout (8 bytes, 64 bits):**

```
Byte 0:    Kp[11:4]
Byte 1:    Kp[3:0]  | Kd[11:8]
Byte 2:    Kd[7:0]
Byte 3:    Position[15:8]
Byte 4:    Position[7:0]
Byte 5:    Velocity[11:4]
Byte 6:    Velocity[3:0] | Torque[11:8]
Byte 7:    Torque[7:0]
```

**Common usage patterns:**

```c
// Pure position hold (stiff spring)
pack_cmd(MOTOR_CAN_ID,
    1.57f,   // target: 90 degrees (pi/2 rad)
    0.0f,    // zero velocity
    100.0f,  // high Kp (stiff)
    2.0f,    // moderate damping
    0.0f);   // no feedforward

// Gravity compensation (torque feedforward only)
pack_cmd(MOTOR_CAN_ID,
    0.0f,    // position ignored (Kp=0)
    0.0f,    // velocity ignored (Kd=0)
    0.0f,    // no position gain
    0.0f,    // no damping
    5.0f);   // 5 Nm feedforward to hold against gravity

// Compliant backdrivable mode (low impedance)
pack_cmd(MOTOR_CAN_ID,
    0.0f,    // nominal position
    0.0f,    // zero velocity
    5.0f,    // low Kp (soft)
    0.5f,    // light damping
    0.0f);   // no feedforward

// Emergency stop via MIT mode
pack_cmd(MOTOR_CAN_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
```

### `float_to_uint`

```c
int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
```

Maps a float from `[x_min, x_max]` into an unsigned integer of the specified bit width. Used internally by `pack_cmd()`. Exposed publicly for teams that need custom packing.

---

## Motor Feedback API

The motor periodically transmits an 8-byte CAN feedback message. These functions parse the raw bytes into usable values.

### `motor_receive` — Parse Full Feedback Message

```c
void motor_receive(MotorStatus* status, const uint8_t* data);
```

Parses all 5 fields from the 8-byte CAN payload into a `MotorStatus` struct in a single call.

| Parameter | Type | Description |
|---|---|---|
| `status` | `MotorStatus*` | Struct to populate |
| `data` | `const uint8_t*` | 8-byte CAN data payload |

**Feedback message byte layout:**

| Bytes | Field | Raw type | Conversion | Unit | Range |
|---|---|---|---|---|---|
| 0–1 | Position | `int16` | $raw \div 10$ | degrees | $\pm \ 3{,}200$ |
| 2–3 | Speed | `int16` | $raw \times 10$ | ERPM | $\pm \ 320{,}000$ |
| 4–5 | Current | `int16` | $raw \div 100$ | amps | $\pm \ 60$ |
| 6 | Temperature | `int8` | direct | $\degree C$ | $-20$ to $127$ |
| 7 | Error code | `uint8` | direct | — | $0$–$7$ |

**Example:**
```c
CanFrame rx_frame;
MotorStatus status;

while (can_bus_recv(&rx_frame)) {
    motor_receive(&status, rx_frame.data);
    // status.position, status.speed, etc. are now populated
}
```

### Individual Field Parsers

For cases where you only need a single field from the feedback message:

```c
float   motor_read_position(const uint8_t* data);     // degrees
float   motor_read_speed(const uint8_t* data);         // ERPM
float   motor_read_current(const uint8_t* data);       // amps
int8_t  motor_read_temperature(const uint8_t* data);   // degrees C
uint8_t motor_read_error(const uint8_t* data);         // error code
```

Each function parses only its respective bytes from the 8-byte payload.

### `motor_error_to_string`

```c
const char* motor_error_to_string(uint8_t error_code);
```

Converts an error code to a human-readable string. Returns a static string (e.g., `"OVER_CURRENT"`, `"MOTOR_STALL"`). Returns `"UNKNOWN"` for unrecognized codes.

---

## UART Command Interface

The `uart_cmd` module provides a text-based UART protocol for testing the motor API from a host (e.g., Python script). Commands are newline-terminated ASCII strings at 115200 baud.

### Setup Functions

```c
void uart_cmd_init(UART_HandleTypeDef* huart);
```
Initialize the command handler and begin interrupt-driven UART RX. Call once after `HAL_UART_Init()`.

```c
void uart_cmd_rx_callback(UART_HandleTypeDef* huart);
```
Call from `HAL_UART_RxCpltCallback()`. Accumulates bytes and flags when a complete command is ready.

```c
void uart_cmd_error_callback(UART_HandleTypeDef* huart);
```
Call from `HAL_UART_ErrorCallback()`. Re-arms UART RX after overrun/framing/noise errors.

```c
void uart_cmd_process(const MotorStatus* status);
```
Process any pending command. Call from the main loop. Reads the cached `MotorStatus` for feedback commands.

```c
void uart_cmd_refresh_tick(void);
```
Re-sends the active motor command every 50 ms to prevent the VESC firmware's internal timeout (~1–2 s) from stopping the motor. Call from the main loop.

```c
void uart_cmd_send_error(uint8_t error_code);
```
Sends a proactive error notification (`!ERR:<code>:<description>\n`) to the host when the motor's error state changes.

### Command Reference

**Read commands** (immediate response, no motor action):

| Command | Response | Description |
|---|---|---|
| `PING` | `PONG` | Connection check |
| `READ_ALL` | `ALL:POS=<deg>,SPD=<rpm>,CUR=<A>,TEMP=<C>,ERR=<code>` | All feedback fields |
| `READ_POS` | `POS:<degrees>` | Position only |
| `READ_SPD` | `SPD:<erpm>` | Speed only |
| `READ_CUR` | `CUR:<amps>` | Current only |
| `READ_TEMP` | `TEMP:<celsius>` | Temperature only |
| `READ_ERR` | `ERR:<code>:<description>` | Error code + string |

**Motor control commands** (all values clamped to test-safe limits):

| Command | Response | Description |
|---|---|---|
| `ESTOP` | `OK:ESTOP` | Immediate stop (sends zero duty) |
| `SET_DUTY <duty>` | `OK:SET_DUTY:<duty>` | Duty cycle ($\pm 0.9$ test limit) |
| `SET_CURRENT <amps>` | `OK:SET_CURRENT:<amps>` | Current/torque ($\pm 5 \ A$ test limit) |
| `SET_BRAKE <amps>` | `OK:SET_BRAKE:<amps>` | Current brake ($0$–$5 \ A$ test limit) |
| `SET_RPM <rpm>` | `OK:SET_RPM:<rpm>` | Velocity ($\pm 5{,}000$ ERPM test limit) |
| `SET_POS <degrees>` | `OK:SET_POS:<degrees>` | Position ($\pm 360\degree$ test limit) |
| `SET_ORIGIN <0\|1>` | `OK:SET_ORIGIN:<mode>` | Set zero position |
| `SET_POS_SPD <deg> <spd> <acc>` | `OK:SET_POS_SPD:<deg>,<spd>,<acc>` | Position with limits |
| `SET_MIT <p> <v> <kp> <kd> <t>` | `OK:SET_MIT:<p>,<v>,<kp>,<kd>,<t>` | MIT force control |

**Error responses:**

| Response | Meaning |
|---|---|
| `ERR:PARSE:<command>` | Could not parse parameters |
| `UNKNOWN_CMD:<command>` | Unrecognized command |
| `!ERR:<code>:<description>` | Proactive motor error notification (sent asynchronously) |

### Test-Safe Limits

The UART command handler clamps all values to conservative limits before calling the motor API, preventing accidental damage during bench testing:

| Parameter | Test limit | Motor maximum |
|---|---|---|
| Duty cycle | $\pm 0.9$ | $\pm 1.0$ |
| Current | $\pm 5 \ A$ | $\pm 60 \ A$ |
| Brake current | $5 \ A$ | $60 \ A$ |
| RPM | $\pm 5{,}000$ ERPM | $\pm 100{,}000$ ERPM |
| Position | $\pm 360\degree$ | $\pm 36{,}000\degree$ |
| MIT position | $\pm 3.14 \ rad$ | $\pm 12.56 \ rad$ |
| MIT velocity | $\pm 5 \ rad/s$ | $\pm 30 \ rad/s$ |
| MIT torque | $\pm 5 \ Nm$ | $\pm 32 \ Nm$ |
| MIT Kp | $50$ | $500$ |
| MIT Kd | $2.5$ | $5.0$ |

These limits are defined as `TEST_*` macros in `ak70_9.h` and are **only enforced by the UART command layer**. Direct API calls (`comm_can_set_*`, `pack_cmd`) clamp only to the motor's physical limits.

### Command Refresh Mechanism

The VESC firmware on the AK70-9 has an internal command timeout (~1–2 seconds). If no new command is received within this window, the motor stops. The `uart_cmd_refresh_tick()` function automatically re-sends the last active motor command every 50 ms to keep the motor running.

- `ESTOP` clears the active command (no refresh)
- `SET_ORIGIN` is a one-shot command (no refresh)
- All other motor commands are refreshed until a new command or `ESTOP` is received

---

## CAN Frame Format

All motor commands use **extended 29-bit CAN identifiers**:

```
 ┌─────────────────────────┬──────────┐
 │  Bits [28:8]            │ Bits[7:0]│
 │  Control Mode ID        │ Motor ID │
 │  (CAN_PACKET_ID enum)   │  (e.g.104│)
 └─────────────────────────┴──────────┘
```

**Constructed as:**
```c
uint32_t ext_id = controller_id | ((uint32_t)mode_id << 8);
```

All data payloads are 8 bytes maximum, packed in **big-endian** byte order.

### Servo Mode Payload Encoding

| Mode | Payload size | Encoding |
|---|---|---|
| Duty (0) | 4 bytes | `(int32_t)(duty * 100000)` |
| Current (1) | 4 bytes | `(int32_t)(current * 1000)` |
| Brake (2) | 4 bytes | `(int32_t)(current * 1000)` |
| RPM (3) | 4 bytes | `(int32_t)(rpm)` |
| Position (4) | 4 bytes | `(int32_t)(pos * 10000)` |
| Origin (5) | 1 byte | `mode` (0 or 1) |
| Pos+Spd (6) | 8 bytes | `int32 pos*10000` + `int16 spd/10` + `int16 rpa/10` |

---

## Integration Guide

### Minimal Setup (Direct API)

```c
#include "can/ak70_9.h"
#include "can/can_bus.h"
#include "can/can_frame.h"

#define MY_MOTOR_ID 104

// Initialization (after MX_CAN1_Init):
can_bus_init(&hcan1);

// Main loop:
MotorStatus status = {0};

while (1) {
    // 1. Poll CAN for motor feedback
    CanFrame rx_frame;
    while (can_bus_recv(&rx_frame)) {
        motor_receive(&status, rx_frame.data);
    }

    // 2. Send motor commands
    if (status.error == MOTOR_ERROR_NONE) {
        comm_can_set_pos(MY_MOTOR_ID, target_angle);
    }
}
```

### Setup with UART Command Layer

```c
#include "can/uart_cmd.h"
#include "can/can_frame.h"

MotorStatus motor_status = {0};
uint8_t prev_error = 0;

// Initialization:
can_bus_init(&hcan1);
uart_cmd_init(&huart2);

// Main loop:
while (1) {
    // Poll CAN and update status
    CanFrame rx_frame;
    while (can_bus_recv(&rx_frame)) {
        motor_receive(&motor_status, rx_frame.data);

        if (motor_status.error != prev_error &&
            motor_status.error != MOTOR_ERROR_NONE) {
            uart_cmd_send_error(motor_status.error);
        }
        prev_error = motor_status.error;
    }

    // Process UART commands
    uart_cmd_process(&motor_status);

    // Re-send active command to prevent timeout
    uart_cmd_refresh_tick();
}

// In your HAL callbacks:
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    uart_cmd_rx_callback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    uart_cmd_error_callback(huart);
}
```

### STM32CubeMX Peripheral Requirements

| Peripheral | Configuration |
|---|---|
| CAN1 | Normal mode, prescaler and time quanta for 1 Mbit/s (motor default) |
| USART2 (for UART commands) | 115200 baud, 8N1, TX+RX |
| NVIC | CAN1 RX0 interrupt enabled, USART2 global interrupt enabled |

### Linker Flags (STM32CubeIDE)

The UART command handler uses `snprintf` with `%f` and `sscanf` with `%f`. Add these linker flags:

```
-u _printf_float
-u _scanf_float
```

**Path:** Project > Properties > C/C++ Build > Settings > MCU GCC Linker > Miscellaneous > Other flags.

---

## Motor Error Codes

| Code | Enum | String | Description | Recommended action |
|---|---|---|---|---|
| 0 | `MOTOR_ERROR_NONE` | `"NONE"` | No fault | — |
| 1 | `MOTOR_ERROR_OVER_TEMP` | `"MOTOR_OVER_TEMP"` | Motor winding over-temperature | Stop motor, allow cooling |
| 2 | `MOTOR_ERROR_OVER_CURRENT` | `"OVER_CURRENT"` | Instantaneous current exceeded limit | Reduce load or current command |
| 3 | `MOTOR_ERROR_OVER_VOLTAGE` | `"OVER_VOLTAGE"` | Bus voltage too high (regenerative braking) | Reduce deceleration rate |
| 4 | `MOTOR_ERROR_UNDER_VOLTAGE` | `"UNDER_VOLTAGE"` | Bus voltage too low | Check power supply |
| 5 | `MOTOR_ERROR_ENCODER` | `"ENCODER_FAULT"` | Encoder communication lost | Check encoder wiring |
| 6 | `MOTOR_ERROR_MOSFET_OVER_TEMP` | `"MOSFET_OVER_TEMP"` | Driver board over-temperature | Stop motor, allow cooling |
| 7 | `MOTOR_ERROR_STALL` | `"MOTOR_STALL"` | Motor locked / unable to move | Check for mechanical obstruction |

---

## Recommendations for Generalization

The current API is well-structured and functional for single-motor bench testing. Below are concrete recommendations for making it more reusable across teams with different motor configurations and multi-motor setups.

### 1. Multi-Motor Instance Support

The current API uses `MOTOR_CAN_ID` as a global constant and passes `controller_id` to every function. This works but the UART command layer and feedback polling are hardcoded to a single motor. Teams with multiple joints need per-motor state.

**Recommended pattern:**

```c
typedef struct {
    uint8_t      can_id;         // CAN ID of this motor
    MotorStatus  status;         // latest feedback
    uint8_t      prev_error;     // for change detection
} MotorInstance;

// Initialize an array for your joint count
MotorInstance motors[4] = {
    { .can_id = 101 },  // hip
    { .can_id = 102 },  // knee
    { .can_id = 103 },  // ankle
    { .can_id = 104 },  // spare
};

// Route feedback by matching the CAN ID in the received frame
void motor_dispatch_feedback(CanFrame* frame, MotorInstance* motors, size_t count);
```

### 2. Feedback CAN ID Routing

Currently, `motor_receive()` parses any 8-byte payload regardless of CAN ID. In a multi-motor system, you need to match the motor ID from the received frame to dispatch feedback to the correct `MotorStatus`.

**Recommended function:**

```c
// Extract the motor driver ID from a received extended CAN frame
uint8_t motor_id_from_can(uint32_t ext_id);

// Or a higher-level dispatcher
int motor_dispatch(MotorInstance* motors, size_t count,
                   uint32_t ext_id, const uint8_t* data);
```

### 3. Command Timeout / Watchdog Integration

The VESC firmware has a ~1–2 second command timeout, but the current API has no built-in watchdog. If the main loop stalls or a command fails to send, the motor could stop unexpectedly with no notification.

**Recommended additions:**

```c
// Track last successful command time per motor
typedef void (*MotorTimeoutCallback)(uint8_t motor_id);

void motor_set_timeout_callback(MotorTimeoutCallback cb);
void motor_check_timeouts(void);  // call from main loop
```

### 4. Configurable Command Refresh Interval

The 50 ms refresh interval is hardcoded in `uart_cmd.c`. Different applications may need faster refresh (for smooth control) or slower refresh (to reduce bus load).

**Recommended function:**

```c
void uart_cmd_set_refresh_interval(uint32_t ms);  // default: 50
```

### 5. Return Status from Command Functions

The `comm_can_set_*` functions currently return `void`. Teams need to know if the CAN transmission succeeded (e.g., all mailboxes full, bus-off state).

**Recommended change:**

```c
// Return 1 on success, 0 on CAN TX failure
int comm_can_set_duty(uint8_t controller_id, float duty);
int comm_can_set_current(uint8_t controller_id, float current);
// ... etc.
```

This propagates the return value from `can_bus_send_ext()` which already returns success/failure.

### 6. Separate Test Limits from Motor Limits

Test-safe limits (`TEST_*`) and motor hardware limits (`AK70_9_*`) are both defined in `ak70_9.h`. Teams deploying to production will want to remove or raise the test limits without modifying the header.

**Recommended approach:**

```c
// In ak70_9.h — motor hardware limits only (never change)
#define AK70_9_CURRENT_MAX  60.0f

// In a separate project-level config header or build flags:
#ifndef MOTOR_CURRENT_LIMIT
#define MOTOR_CURRENT_LIMIT AK70_9_CURRENT_MAX  // default: use full range
#endif
```

This lets teams override limits via their project's build configuration without touching shared API files.

### 7. Mechanical Unit Conversion Helpers

The API uses electrical RPM, but teams think in mechanical units (RPM, rad/s, degrees/s). Conversion depends on pole pairs (21 for the AK70-9).

**Recommended functions:**

```c
#define AK70_9_POLE_PAIRS 21

float erpm_to_mech_rpm(float erpm);       // erpm / pole_pairs
float mech_rpm_to_erpm(float mech_rpm);   // mech_rpm * pole_pairs
float erpm_to_rad_per_sec(float erpm);    // convert through mech RPM
float deg_to_rad(float degrees);
float rad_to_deg(float radians);
```

### 8. Emergency Stop as a First-Class API Function

Currently, emergency stop is implemented in `uart_cmd.c` by sending zero duty. It should be a standalone motor API function that any code path can call without going through the UART layer.

**Recommended function:**

```c
void motor_emergency_stop(uint8_t controller_id);  // sends 0 duty immediately
```

### 9. Feedback Data Timestamp

`MotorStatus` doesn't track when the feedback was received. Teams running control loops need to know if the feedback is stale.

**Recommended addition:**

```c
typedef struct {
    float    position;
    float    speed;
    float    current;
    int8_t   temperature;
    uint8_t  error;
    uint32_t last_update_tick;  // HAL_GetTick() when last received
} MotorStatus;
```

### Summary of Recommended Additions

| Function / Change | Purpose |
|---|---|
| `MotorInstance` struct | Multi-motor state management |
| `motor_id_from_can()` | Route feedback to correct motor |
| `motor_check_timeouts()` | Watchdog for stalled communication |
| `uart_cmd_set_refresh_interval()` | Configurable refresh rate |
| Return `int` from `comm_can_set_*()` | Propagate CAN TX success/failure |
| Separate test limits from API | Production deployment flexibility |
| `erpm_to_mech_rpm()`, etc. | Mechanical unit conversions |
| `motor_emergency_stop()` | First-class E-stop in motor API |
| `last_update_tick` in `MotorStatus` | Feedback staleness detection |
