# AK70-9 Motor CAN API

Hardware API reference for the CubeMars AK70-9 KV60 brushless motor driver used in the **motor-api-tests** project.

**Source files:**
- Motor API: `Core/Inc/can/ak70_9.h` / `Core/Src/can/ak70_9.c`
- CAN bus driver: `Core/Inc/can/can_bus.h` / `Core/Src/can/can_bus.c`
- UART command handler: `Core/Inc/can/uart_cmd.h` / `Core/Src/can/uart_cmd.c`

---

## Motor Overview

| Parameter | Value |
|---|---|
| Motor | CubeMars AK70-9 KV60 |
| Interface | CAN bus (29-bit extended identifiers) |
| Max current | +/- 60 A |
| Max speed | +/- 100,000 ERPM (electrical RPM) |
| Position range | +/- 36,000 degrees (+/- 100 rotations) |
| Torque range | +/- 32 Nm |
| Temperature feedback | -20 to 127 C |

---

## CAN Frame Format

All commands use **extended 29-bit CAN identifiers**:

```
Bits [28:8] = Control Mode ID
Bits  [7:0] = Motor Driver ID
```

- Data length (DLC): 8 bytes
- Frame type: DATA
- Frame format: Extended

The motor driver ID is set on the motor itself (default is typically `1`).

---

## Control Modes

The API supports two control interfaces: **Servo Mode** (7 modes) and **MIT Force Control Mode**.

### Servo Mode Functions

#### `comm_can_set_duty` -- Duty Cycle Control (Mode 0)

```c
void comm_can_set_duty(uint8_t controller_id, float duty);
```

Drives the motor with a voltage proportional to the duty cycle.

| Parameter | Range | Unit |
|---|---|---|
| `duty` | -1.0 to 1.0 | fraction (-100% to +100%) |

---

#### `comm_can_set_current` -- Current/Torque Control (Mode 1)

```c
void comm_can_set_current(uint8_t controller_id, float current);
```

Commands a target Iq current. Motor torque = Iq * KT.

| Parameter | Range | Unit |
|---|---|---|
| `current` | -60.0 to 60.0 | amps |

---

#### `comm_can_set_cb` -- Current Brake (Mode 2)

```c
void comm_can_set_cb(uint8_t controller_id, float current);
```

Applies a braking current to hold the motor at its current position. Monitor motor temperature when using this mode.

| Parameter | Range | Unit |
|---|---|---|
| `current` | 0.0 to 60.0 | amps |

---

#### `comm_can_set_rpm` -- Velocity Control (Mode 3)

```c
void comm_can_set_rpm(uint8_t controller_id, float rpm);
```

Commands the motor to spin at a specified electrical RPM. Acceleration defaults to maximum.

| Parameter | Range | Unit |
|---|---|---|
| `rpm` | -100,000 to 100,000 | electrical RPM |

---

#### `comm_can_set_pos` -- Position Control (Mode 4)

```c
void comm_can_set_pos(uint8_t controller_id, float pos);
```

Commands the motor to move to an absolute angular position. Speed and acceleration default to maximum.

| Parameter | Range | Unit |
|---|---|---|
| `pos` | -36,000 to 36,000 | degrees |

---

#### `comm_can_set_origin` -- Set Zero Position (Mode 5)

```c
void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode);
```

Sets the current position as the zero/origin point.

| Parameter | Value | Description |
|---|---|---|
| `set_origin_mode` | `0` | Temporary origin (lost on power cycle) |
| `set_origin_mode` | `1` | Permanent origin (saved to flash) |

---

#### `comm_can_set_pos_spd` -- Position + Velocity + Acceleration (Mode 6)

```c
void comm_can_set_pos_spd(uint8_t controller_id, float pos, int16_t spd, int16_t rpa);
```

Smooth trajectory control with velocity and acceleration limits.

| Parameter | Range | Unit |
|---|---|---|
| `pos` | -36,000 to 36,000 | degrees |
| `spd` | -32,768 to 32,767 | raw value (*10 = ERPM) |
| `rpa` | 0 to 32,767 | raw acceleration (1 unit = 10 ERPM/s^2) |

---

### MIT Force Control Mode

#### `pack_cmd` -- Impedance/Force Control (Mode 8)

```c
void pack_cmd(uint8_t controller_id, float p_des, float v_des, float kp, float kd, float t_ff);
```

Sends a combined position-velocity-torque command. The motor-side control law is:

```
torque = Kp * (p_des - p_actual) + Kd * (v_des - v_actual) + t_ff
```

This enables compliant force control and impedance control for the exoskeleton.

| Parameter | Range | Unit |
|---|---|---|
| `p_des` | -12.56 to 12.56 | radians |
| `v_des` | -30.0 to 30.0 | rad/s |
| `kp` | 0 to 500 | position gain |
| `kd` | 0 to 5.0 | velocity/damping gain |
| `t_ff` | -32.0 to 32.0 | Nm (torque feedforward) |

**Bit packing layout (8 bytes):**
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

---

## Motor Feedback

The motor periodically sends an 8-byte CAN feedback message.

### Feedback Message Format

| Bytes | Field | Conversion | Unit |
|---|---|---|---|
| 0-1 | Position | int16 / 10 | degrees |
| 2-3 | Speed | int16 * 10 | ERPM |
| 4-5 | Current | int16 / 100 | amps |
| 6 | Temperature | int8 | degrees C |
| 7 | Error code | uint8 | see error table |

### `MotorStatus` Struct

```c
typedef struct {
    float    position;     // degrees
    float    speed;        // ERPM
    float    current;      // amps
    int8_t   temperature;  // degrees C
    uint8_t  error;        // error code
} MotorStatus;
```

### Feedback Functions

```c
// Parse all fields at once
void motor_receive(MotorStatus* status, const uint8_t* data);

// Parse individual fields
float   motor_read_position(const uint8_t* data);    // degrees
float   motor_read_speed(const uint8_t* data);        // ERPM
float   motor_read_current(const uint8_t* data);      // amps
int8_t  motor_read_temperature(const uint8_t* data);  // degrees C
uint8_t motor_read_error(const uint8_t* data);        // error code

// Convert error code to string
const char* motor_error_to_string(uint8_t error_code);
```

---

## Motor Error Codes

| Code | Name | Description |
|---|---|---|
| 0 | `NONE` | No fault |
| 1 | `MOTOR_OVER_TEMP` | Motor over-temperature |
| 2 | `OVER_CURRENT` | Over-current fault |
| 3 | `OVER_VOLTAGE` | Over-voltage fault |
| 4 | `UNDER_VOLTAGE` | Under-voltage fault |
| 5 | `ENCODER_FAULT` | Encoder fault |
| 6 | `MOSFET_OVER_TEMP` | MOSFET/driver board over-temperature |
| 7 | `MOTOR_STALL` | Motor stall / lock-up |

---

## CAN Bus Driver

The motor API uses the `can_bus` driver for all CAN transmission and reception.

### Key Functions

```c
// Initialize CAN peripheral with accept-all filter and RX interrupts
int can_bus_init(CAN_HandleTypeDef* hcan);

// Transmit with 29-bit extended ID (used by AK70-9 protocol)
int can_bus_send_ext(uint32_t ext_id, const uint8_t* data, uint8_t dlc);

// Transmit with 11-bit standard ID
int can_bus_send_std(uint16_t std_id, const uint8_t* data, uint8_t dlc);

// Pop next received frame from ring buffer (non-blocking)
int can_bus_recv(CanFrame* out);
```

### CAN Frame Struct

```c
typedef struct {
    uint32_t id;          // 11-bit or 29-bit identifier
    uint8_t  dlc;         // Data length code (0-8)
    uint8_t  is_extended; // 1 = extended, 0 = standard
    uint8_t  data[8];     // Payload
} CanFrame;
```

The receive path uses an interrupt-driven **ring buffer** (32-frame capacity) so no frames are lost during processing.

---

## Utility Functions

### Byte Packing (Big-Endian)

```c
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index);
```

### Float-to-Uint Conversion (MIT Mode)

```c
int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
```

Maps a float from `[x_min, x_max]` into an unsigned integer of the specified bit width. Used internally by `pack_cmd()`.

---

## Usage in `main.c`

```c
// Initialization
can_bus_init(&hcan1);
uart_cmd_init(&huart2);

// Main loop
while (1) {
    // Poll CAN for motor feedback
    CanFrame rx_frame;
    while (can_bus_recv(&rx_frame)) {
        motor_receive(&motor_status, rx_frame.data);

        // Notify host on error state change
        if (motor_status.error != prev_error && motor_status.error != MOTOR_ERROR_NONE) {
            uart_cmd_send_error(motor_status.error);
        }
        prev_error = motor_status.error;
    }

    // Process UART commands from host
    uart_cmd_process(&motor_status);
}
```
