# Nucleo-Pi CAN Protocol

CAN bus protocol reference for the **nucleo-pi-can** project. The Nucleo-F446RE reads accelerometer data from the LSM6DS3TR-C IMU and broadcasts it over CAN1 at ~50 Hz.

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

> The baud rate is derived from an 84 MHz APB1 clock: 84 MHz / 6 / (1 + 11 + 2) = 1 Mbit/s.

---

## CAN Frame Format

### IMU Accelerometer Frame

| Field | Value |
|---|---|
| CAN ID | `0x123` (standard, 11-bit) |
| DLC | 6 bytes |
| Rate | ~50 Hz (every 20 ms) |

The frame carries the three filtered accelerometer axes packed as **big-endian signed 16-bit integers**, scaled by **100** (i.e., `int16 = float_m_s2 * 100`).

| Bytes | Field | Encoding | Unit |
|---|---|---|---|
| 0-1 | Accel X | int16, big-endian, / 100 | m/s² |
| 2-3 | Accel Y | int16, big-endian, / 100 | m/s² |
| 4-5 | Accel Z | int16, big-endian, / 100 | m/s² |

**Decoding example (Python):**
```python
import struct

def decode_imu_frame(data: bytes):
    ax_raw, ay_raw, az_raw = struct.unpack('>hhh', data[:6])
    ax = ax_raw / 100.0  # m/s²
    ay = ay_raw / 100.0
    az = az_raw / 100.0
    return ax, ay, az
```

**Example frame on the bus:**
```
can0  123   [6]  00 0C FF D3 03 D2
```
Decoded:
- Bytes 0-1: `0x000C` → 12 → `0.12 m/s²` (AX)
- Bytes 2-3: `0xFFD3` → -45 → `-0.45 m/s²` (AY)
- Bytes 4-5: `0x03D2` → 978 → `9.78 m/s²` (AZ)

---

## Transmission Behavior

- CAN frames are sent in the main loop immediately after `lsm6ds3tr_read()` completes.
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

# Filter to only IMU frames (ID 0x123)
candump can0,123:7FF
```

**Python receive example using python-can:**
```python
import can
import struct

bus = can.interface.Bus(channel='can0', bustype='socketcan')

for msg in bus:
    if msg.arbitration_id == 0x123 and len(msg.data) >= 6:
        ax, ay, az = struct.unpack('>hhh', bytes(msg.data[:6]))
        print(f"AX={ax/100:.2f} AY={ay/100:.2f} AZ={az/100:.2f} m/s²")
```

---

## IMU Driver Summary

The CAN data originates from the LSM6DS3TR-C driver. Key parameters:

| Parameter | Value |
|---|---|
| Sensor | LSM6DS3TR-C |
| I2C address | `0x6B` (SA0 pin HIGH) |
| Accelerometer range | +/- 4g |
| Output data rate | 104 Hz |
| Filter | Low-pass, alpha = 0.5 |
| Accel unit conversion | raw * 0.000122 * 9.80665 = m/s² |

Only accelerometer data is transmitted over CAN. Gyroscope data is available via the UART command interface (`READ` command) but is not included in the CAN frame.

---

## Usage in `main.c`

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

    lsm6ds3tr_read();        // Update filtered IMU data
    CAN_Send_IMU_Data();     // Broadcast accel data on CAN ID 0x123

    HAL_Delay(20);           // ~50 Hz loop
}
```
