# Nucleo-Pi CAN Protocol

CAN bus protocol reference for the **nucleo-pi-can** project. The Nucleo-F446RE reads IMU data from the LSM6DS3TR-C IMU and broadcasts it over CAN1 at ~50 Hz.

**Source files:**
- Main application: `Core/Src/main.c` (`CAN_Send_IMU_Data`)
- IMU driver: `Core/Inc/lsm6ds3tr.h` / `Core/Src/lsm6ds3tr.c`

---

## CAN Bus Configuration

| Parameter | Value |
|---|---|
| Peripheral | CAN1 |
| Baud rate | 1 Mbit/s |
| Frame format | Standard (11-bit ID) |
| Prescaler | 6 |
| Time segment 1 | 11 TQ |
| Time segment 2 | 2 TQ |
| TX pins | PA12 (CAN1_TX) |
| RX pins | PA11 (CAN1_RX) |

> The baud rate for a bxCAN is derived from an 42 MHz APB1 clock: 
> 
> $\text{Baud Rate} = \frac{f_{APB1}}{Prescaler \times (1 \ + \ TS1 \ + \ TS2)}= \frac{42 MHz}{6\times(1  \ + \ 11 \ + \ 2)}= 500 kbps$

---

## CAN Frame Format

Because a standard CAN frame has a maximum payload of 8 bytes, the 12 bytes of IMU data (3 axes of acceleration + 3 axes of gyroscope) are split across two sequential frames.

### 1. IMU Accelerometer Frame (ID: `0x123`)

| Field | Value |
|---|---|
| CAN ID | `0x123` (standard, 11-bit) |
| DLC | 6 bytes |
| Rate | ~500 Hz (every 2 ms) |

The frame carries the three filtered accelerometer axes packed as **little-endian signed 16-bit integers**, scaled by **100** (i.e., `int16 = float_m_s2 * 100`).

| Bytes | Field | Encoding | Unit |
|---|---|---|---|
| 0-1 | Accel X | int16, little-endian, / 100 | m/s² |
| 2-3 | Accel Y | int16, little-endian, / 100 | m/s² |
| 4-5 | Accel Z | int16, little-endian, / 100 | m/s² |

**Example frame on the bus:**
```
can0  123   [6]  00 0C FF D3 03 D2
```
Decoded:
- Bytes 0-1: `0x000C` → 12 → `0.12 m/s²` (AX)
- Bytes 2-3: `0xFFD3` → -45 → `-0.45 m/s²` (AY)
- Bytes 4-5: `0x03D2` → 978 → `9.78 m/s²` (AZ)

### 2. IMU Gyroscope Frame (ID: `0x124`)

| Field | Value |
|---|---|
| CAN ID | `0x124` (standard, 11-bit) |
| DLC | 6 bytes |
| Rate | ~500 Hz (every 2 ms) |

The frame carries the three filtered gyroscope axes packed as **little-endian signed 16-bit integers**, scaled by **100** (i.e., `int16 = float_m_s2 * 100`).

| Bytes | Field | Encoding | Unit |
|---|---|---|---|
| 0-1 | Gryo X | int16, little-endian, / 100 | deg/s |
| 2-3 | Gryo Y | int16, little-endian, / 100 | deg/s |
| 4-5 | Gryo Z | int16, little-endian, / 100 | deg/s |

**Example frame on the bus:**
```
can0  124   [6]  E2 04 0C FE 2C 1A
```
Decoded:
- Bytes 0-1: `0x04E2` → 1250 → `12.50 deg/s` (GX)
- Bytes 2-3: `0xFE0C` → -500 → `-5.00 deg/s` (GY)
- Bytes 4-5: `0x1A2C` → 6700 → `67.00 deg/s` (GZ)

---

## Transmission Behavior

- CAN frames are sent sequentially in the main loop in a non-blocking DMA pipeline
    1. First broadcats the data processed from the previous cycle
    2. Triggers `lsm6ds3tr_init_dma_read()` to fetch the next back in the background
- Transmission is **skipped** if the IMU is in `SENSOR_STATE_LOST` (the `CAN_Send_IMU_Data` function returns early).
- The transmit mailbox is checked before queuing (`HAL_CAN_GetTxMailboxesFreeLevel > 0`). If all mailboxes are full, the frame is dropped rather than blocking.

---

## Receiving on a Raspberry Pi

If using a Raspberry Pi with a CAN interface (e.g., MCP2515 HAT configured for 1 Mbit/s):

```bash
# Bring up the CAN interface
sudo ip link set can0 up type can bitrate 1000000

# Monitor raw frames
candump can0

# Filter to only IMU frames (IDs 0x123 and 0x124)
candump can0,123:7FF,124:7FF
```

**Python CAN frame receive example using python-can:**
```python
import can
import struct

bus = can.interface.Bus(channel='can0', bustype='socketcan')
telemetry = {"ax": 0.0, "ay": 0.0, "az": 0.0, "gx": 0.0, "gy": 0.0, "gz": 0.0}

for msg in bus:
    if msg.arbitration_id == 0x123:
        # Note the '<' for little-endian unpacking
        ax, ay, az = struct.unpack('<hhh', msg.data[:6])
        telemetry["ax"] = ax / 100.0
        telemetry["ay"] = ay / 100.0
        telemetry["az"] = az / 100.0

    elif msg.arbitration_id == 0x124:
        gx, gy, gz = struct.unpack('<hhh', msg.data[:6])
        telemetry["gx"] = gx / 100.0
        telemetry["gy"] = gy / 100.0
        telemetry["gz"] = gz / 100.0
        
        # Print when both frames are captured
        print(f"Accel: X={telemetry['ax']:.2f} Y={telemetry['ay']:.2f} Z={telemetry['az']:.2f} m/s²")
        print(f"Gyro:  X={telemetry['gx']:.2f} Y={telemetry['gy']:.2f} Z={telemetry['gz']:.2f} deg/s")
```

---

## IMU Driver Summary

The CAN data originates from the LSM6DS3TR-C driver. Key parameters:

| Parameter | Value |
|---|---|
| Sensor | LSM6DS3TR-C |
| I2C address | `0x6B` (SA0 pin HIGH) |
| I2C Read Method | Non-blocking DMA (12-byte burst) |
| Accelerometer range | +/- 4g |
| Gryoscope range | +/- 500 dps |
| Output data rate | 104 Hz |
| Filter | Low-pass, alpha = 0.5 |
| Accel unit conversion | $raw\times 0.000122\times 9.80665 = m/s^2$ |
| Gyro unit conversion | $raw\times (\pi/180) = deg/s$ |
| Calibration | 100-sample startup zero-rate offset (Gryo only) |

LSM6DS3TR-C datasheet: https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf

---

## Usage in `main.c`

The main loop transmission uses a non-blocking loop (DMA controlled I2C), using `HAL_GetTick()` to manage timing, allowing UART commands to be processed without delaying CAN/I2C.

```c
// Initialization
HAL_CAN_Start(&hcan1);
lsm6ds3tr_init_driver(&hi2c3);

// Main loop
while (1) {
    if (cmd_ready) {
        cmd_ready = 0;
        process_command();   // Handle UART debug commands
    }

    if ((HAL_GetTick() - last_tick) >= 2)
    {
        last_tick = HAL_GetTick();   // Update Clock
        CAN_Send_IMU_Data();   // Broadcast previous cycle of IMU data on CAN
        lsm6ds3tr_init_dma_read();   // Fetch new batch of sensor data
    }
}
```
