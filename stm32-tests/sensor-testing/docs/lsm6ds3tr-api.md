# LSM6DS3TR-C IMU Driver API

Hardware API reference for the LSM6DS3TR-C 6-axis IMU (accelerometer + gyroscope) driver used in the **sensor-testing** project. This driver is optimized for the Nucleo F446RE, utilizing a non-blocking I2C DMA pipeline for high-speed data acquisition.

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
| Accelerometer range | $\pm \ 4 \ g$ |
| Gyroscope range | $\pm \ 500 \ dps$ (degrees per second) |
| Output data rate (ODR) | 104 Hz (both accel and gyro) |
| Data format | Little-endian (low byte first) |
| Acquisition Rate | 500 Hz (every 2 ms) |
| Block Data Update | Enabled |

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

- **Accel filtered values** are in **$m/s^2$** (with gravity offset applied).
- **Gyro filtered values** are in **$dps$** (degrees per second).

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

- [lsm6ds3tr_init_driver](#lsm6ds3tr_init_driver)
- [lsm6ds3tr_check_connection](#lsm6ds3tr_check_connection)
- [lsm6ds3tr_configure](#lsm6ds3tr_configure)
- [lsm6ds3tr_calibrate](#lsm6ds3tr_calibrate)
- [lsm6ds3tr_read](#lsm6ds3tr_read)
- [lsm6ds3tr_init_dma_read](#lsm6ds3tr_init_dma_read)
- [HAL_I2C_MemRxCpltCallback](#hal_i2c_memrxcpltcallback)
- [lsm6ds3tr_get_data](#lsm6ds3tr_get_data)

---

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

Reads the `WHO_AM_I` register (`0x0F`) and verifies if the I2C bus is busy with a DMA transfer. If the bus is free, the response is `0x6A`. Updates the internal sensor state.

**Returns:** `1` if connected, `0` if not.

---

### `lsm6ds3tr_configure`

```c
uint8_t lsm6ds3tr_configure(void);
```

Writes configuration to the control registers:
- `CTRL1_XL` (`0x10`): 104 Hz ODR, +/- 4g full-scale
- `CTRL2_G` (`0x11`): 104 Hz ODR, +/- 500 dps full-scale
- `CTRL3_C` (`0x12`): Block Data Update enabled to prevent data tearing during multi-byte reads

**Returns:** `1` on success, `0` on I2C failure.

---

### `lsm6ds3tr_calibrate`

```c
void lsm6ds3tr_calibrate(void);
```

Performs a 100-sample offset calculation to establish a baseline for the gyroscope. The sensor must remain stationary while this routine runs. Takes ~ 2 seconds.

---

### `lsm6ds3tr_read`

```c
uint8_t lsm6ds3tr_read(void);
```

Standard blocking read fallback function.
1. Checks connection state; reconfigures if lost
2. Fetches all 12 bytes via standard I2C read
3. Applies conversions and offsets
4. Updates the struct

**Returns:** `1` if I2C read request was sucessful, `0` if the request failed or the sensor was lost.

---

### `lsm6ds3tr_init_dma_read`

```c
uint8_t lsm6ds3tr_init_dma_read(void);
```

DMA pipeline trigger. This function when called fetches the next batch of IMU data.
1. Checks connection state; reconfigures if lost
2. Initiates a non-blocking 12-byte burst read starting at `OUTX_L_G` register (`0x22`)
3. Returns read without any processing

**Returns:** `1` if DMA request was successful, `0` if the request failed or the sensor was lost

---

### `HAL_I2C_MemRxCpltCallback`

```c
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
```

Primary data acquistion pipeline handling data processing asynchronously. This is the hardware interrupt callback, which is automatically called by the HAL once `lsm6ds3tr_init_dma_read()`successfully completes its 12-byte transfer into the background buffer.
1. Parses the raw bytes into signed 16-bit integers (Little-Endian format)
2. Applies unit conversion:
   - Accel: $raw\times 0.000122\times 9.80665 = m/s^2$
   - Gyro: $raw\times 0.0175 = dps$
3. Subtracts calibration offsets.
4. Updates internal `LSM6DS3TR_Data_t` structure. As of `v0.2.0`, the IMU raw data relies on sensor's internal hardware filter

---

### `lsm6ds3tr_get_data`

```c
LSM6DS3TR_Data_t* lsm6ds3tr_get_data(void);
```

Returns a pointer to the internal `LSM6DS3TR_Data_t` struct. Use this to access the latest DMA sensor readings.

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
| `0x12` | CTRL3_C | Control register 3 (BDU configuration) |
| `0x22`-`0x27` | OUTX_L_G - OUTZ_H_G | Gyroscope output (X, Y, Z) |
| `0x28`-`0x2D` | OUTX_L_XL - OUTZ_H_XL | Accelerometer output (X, Y, Z) |

THe latter two can be combined as mentioned above to make a 12-byte continuous block for burst read.

---

## Sensitivity Constants

| Parameter | Value | Unit |
|---|---|---|
| Accel sensitivity ($\pm \ 4 \ g$) | 0.000122 | $g/LSB$ |
| Gyro sensitivity ($\pm \ 500 \ dps$) | 0.0175 | $dps/LSB$ |
| Gravity constant | 9.80665 | $m/s^2$ |

---

## Usage in `main.c`

The driver is configured to continuously pull data via DMA in a timed loop.

```c
// Initialization
lsm6ds3tr_init_driver(&hi2c3);
lsm6ds3tr_configure();
lsm6ds3tr_calibrate(); // 2 second stationary calibration

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

The UART command handler reads filtered data via `lsm6ds3tr_get_data()` and formats it for serial output.
