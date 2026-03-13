# LSM6DS3TR-C IMU Driver API

Hardware API reference for the LSM6DS3TR-C 6-axis IMU (accelerometer + gyroscope) driver used in the **sensor-testing** project.

**Source files:**
- Header: `Core/Inc/lsm6ds3tr.h`
- Implementation: `Core/Src/lsm6ds3tr.c`
- Datasheet: [LSM6DS3TR-C (ST)](https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf)

---

## Sensor Overview

| Parameter | Value |
|---|---|
| Sensor | LSM6DS3TR-C (6-axis IMU) |
| Interface | I2C (7-bit address `0x6A`, SA0 pin LOW) |
| Accelerometer range | +/- 4g |
| Gyroscope range | +/- 500 dps (degrees per second) |
| Output data rate (ODR) | 104 Hz (both accel and gyro) |
| Data format | Little-endian (low byte first) |

---

## Data Structures

### `SensorState_t`

Tracks whether the IMU is communicating over I2C.

```c
typedef enum {
    SENSOR_STATE_CONNECTED,
    SENSOR_STATE_LOST
} SensorState_t;
```

### `AxisData_t`

Holds raw and filtered data for a 3-axis measurement.

```c
typedef struct {
    int16_t x, y, z;              // Raw sensor values (LSBs)
    float   filt_x, filt_y, filt_z; // Filtered values (real units)
} AxisData_t;
```

- **Accel filtered values** are in **m/s^2** (with gravity offset applied).
- **Gyro filtered values** are in **dps** (degrees per second).

### `LSM6DS3TR_Data_t`

Top-level struct containing all IMU state.

```c
typedef struct {
    SensorState_t state;       // CONNECTED or LOST
    AxisData_t    accel;       // Accelerometer data
    AxisData_t    gyro;        // Gyroscope data
    uint8_t       gyro_config; // CTRL2_G register value
    uint8_t       accel_config;// CTRL1_XL register value
    uint8_t       power_config;// Power config value
} LSM6DS3TR_Data_t;
```

---

## API Functions

### `lsm6ds3tr_init_driver`

```c
void lsm6ds3tr_init_driver(I2C_HandleTypeDef *hi2c);
```

Initialize the driver with the I2C peripheral handle. Must be called once before any other function. Sets the sensor state to `SENSOR_STATE_LOST` and zeros all data fields.

**Example:**
```c
lsm6ds3tr_init_driver(&hi2c3);
```

---

### `lsm6ds3tr_check_connection`

```c
uint8_t lsm6ds3tr_check_connection(void);
```

Reads the `WHO_AM_I` register (`0x0F`) and verifies the response is `0x6A`. Updates the internal sensor state.

**Returns:** `1` if connected, `0` if not.

---

### `lsm6ds3tr_configure`

```c
uint8_t lsm6ds3tr_configure(void);
```

Writes configuration to the control registers:
- `CTRL1_XL` (`0x10`): 104 Hz ODR, +/- 4g full-scale
- `CTRL2_G` (`0x11`): 104 Hz ODR, +/- 500 dps full-scale

**Returns:** `1` on success, `0` on I2C failure.

---

### `lsm6ds3tr_read`

```c
uint8_t lsm6ds3tr_read(void);
```

Reads accelerometer and gyroscope data from the sensor. This is the main function to call in your loop.

**Behavior:**
1. If the sensor state is `LOST`, attempts to reconnect and reconfigure automatically.
2. Reads 6 bytes of accelerometer data (registers `0x28`-`0x2D`) and 6 bytes of gyroscope data (registers `0x22`-`0x27`).
3. On the **first successful call**, runs a 100-sample calibration routine (takes ~300 ms). The sensor must be **stationary** during this time.
4. Applies unit conversion:
   - Accel: raw * 0.000122 * 9.80665 = m/s^2
   - Gyro: raw * 0.0175 = dps
5. Subtracts calibration offsets.
6. Applies a low-pass filter (alpha = 0.5): `filtered = 0.5 * new + 0.5 * previous`
7. Updates both raw (`x`, `y`, `z`) and filtered (`filt_x`, `filt_y`, `filt_z`) fields.

**Returns:** `1` on success, `0` on I2C failure.

---

### `lsm6ds3tr_get_data`

```c
LSM6DS3TR_Data_t* lsm6ds3tr_get_data(void);
```

Returns a pointer to the internal `LSM6DS3TR_Data_t` struct. Use this to access the latest sensor readings.

**Example:**
```c
LSM6DS3TR_Data_t *imu = lsm6ds3tr_get_data();
float accel_x = imu->accel.filt_x; // m/s^2
float gyro_z  = imu->gyro.filt_z;  // dps
```

---

## Register Map (Key Registers)

| Address | Name | Description |
|---|---|---|
| `0x0F` | WHO_AM_I | Device ID (expected: `0x6A`) |
| `0x10` | CTRL1_XL | Accelerometer control (ODR + full-scale) |
| `0x11` | CTRL2_G | Gyroscope control (ODR + full-scale) |
| `0x22`-`0x27` | OUTX_L_G - OUTZ_H_G | Gyroscope output (X, Y, Z) |
| `0x28`-`0x2D` | OUTX_L_XL - OUTZ_H_XL | Accelerometer output (X, Y, Z) |

---

## Sensitivity Constants

| Parameter | Value | Unit |
|---|---|---|
| Accel sensitivity (+/- 4g) | 0.000122 | g/LSB |
| Gyro sensitivity (+/- 500 dps) | 0.0175 | dps/LSB |
| Gravity constant | 9.80665 | m/s^2 |

---

## Usage in `main.c`

The sensor-testing project uses the driver in a UART command loop:

```c
// Initialization
lsm6ds3tr_init_driver(&hi2c3);

// Main loop
while (1) {
    lsm6ds3tr_read();       // Always read to keep data fresh
    process_command();       // Handle UART commands (READ, STATUS, etc.)
    HAL_Delay(10);
}
```

The UART command handler reads filtered data via `lsm6ds3tr_get_data()` and formats it for serial output.
