# Sensor API — LSM6DS3TR-C IMU Driver + Circular Buffer

Hardware API reference for the **LSM6DS3TR-C 6-axis IMU** (accelerometer + gyroscope) driver with integrated **circular buffer** for timestamped data history. Designed for the STM32 Nucleo F446RE using non-blocking I2C DMA.

**Source files:**

| File | Path |
|---|---|
| Driver header | `Inc/lsm6ds3tr.h` |
| Driver implementation | `Src/lsm6ds3tr.c` |
| Buffer header | `Inc/imu_buffer.h` |
| Buffer implementation | `Src/imu_buffer.c` |
| Datasheet | [LSM6DS3TR-C (ST)](https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf) |

---

## Table of Contents

1. [Sensor Overview](#sensor-overview)
2. [Architecture](#architecture)
3. [Data Structures](#data-structures)
4. [Driver API — lsm6ds3tr](#driver-api--lsm6ds3tr)
5. [Buffer API — imu_buffer](#buffer-api--imu_buffer)
6. [Integration Guide](#integration-guide)
7. [Register Map](#register-map)
8. [Sensitivity Constants](#sensitivity-constants)
9. [ISR Safety & Concurrency](#isr-safety--concurrency)
10. [Recommendations for Generalization](#recommendations-for-generalization)

---

## Sensor Overview

| Parameter | Value |
|---|---|
| Sensor | LSM6DS3TR-C (6-axis IMU) |
| Interface | I2C (7-bit address `0x6B`, SA0 pin HIGH) |
| Accelerometer range | $\pm \ 4 \ g$ |
| Gyroscope range | $\pm \ 500 \ dps$ (degrees per second) |
| Output data rate (ODR) | 104 Hz (both accel and gyro) |
| Data format | Little-endian (low byte first) |
| DMA acquisition rate | Every 2 ms from main loop (~500 Hz trigger rate) |
| Buffer downsample | Every 5th DMA sample pushed to buffer (~100 Hz effective) |
| Buffer capacity | 100 readings (~1 second of history at effective rate) |
| Block Data Update | Enabled (prevents data tearing) |

---

## Architecture

The sensor system uses a two-layer design: a **driver layer** that handles hardware I2C/DMA communication, and a **buffer layer** that stores timestamped readings in a circular buffer for retrieval.

```
 Main Loop (2 ms tick)
    │
    ├── CAN_Send_IMU_Data()         ← reads imu_data via lsm6ds3tr_get_data()
    │
    └── lsm6ds3tr_init_dma_read()   ← starts non-blocking 12-byte I2C DMA read
            │
            ▼
    HAL_I2C_MemRxCpltCallback()     ← ISR fires when DMA transfer completes
            │
            ├── Parses raw bytes → physical units (m/s², dps)
            ├── Updates live imu_data struct
            └── imu_buffer_push()   ← stores timestamped reading (every 5th sample)
                    │
                    ▼
            Circular Buffer [100 entries]
                    │
                    ├── imu_buffer_get_latest()  ← most recent reading
                    ├── imu_buffer_get_all()     ← full history (oldest → newest)
                    └── imu_buffer_count()       ← number of valid entries
```

**Two ways to access data:**

| Method | Function | Data | Use case |
|---|---|---|---|
| Live pointer | `lsm6ds3tr_get_data()` | Most recent reading only (no timestamp) | Real-time control loops, CAN transmission |
| Buffer query | `imu_buffer_get_latest()` / `imu_buffer_get_all()` | Timestamped history | Logging, motion analysis, debugging |

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

Holds raw and converted data for a 3-axis measurement.

```c
typedef struct {
    int16_t x, y, z;                // Raw sensor values (LSBs)
    float   filt_x, filt_y, filt_z; // Converted values (real units)
} AxisData_t;
```

- **Accel `filt_*` values** are in **$m/s^2$** (with gravity offset applied).
- **Gyro `filt_*` values** are in **$dps$** (degrees per second).

### `LSM6DS3TR_Data_t`

Top-level struct containing all live IMU state. Accessed via `lsm6ds3tr_get_data()`.

```c
typedef struct {
    SensorState_t state;        // CONNECTED or LOST
    AxisData_t    accel;        // Accelerometer data
    AxisData_t    gyro;         // Gyroscope data
    uint8_t       gyro_config;  // CTRL2_G register value
    uint8_t       accel_config; // CTRL1_XL register value
    uint8_t       power_config; // Power config value
} LSM6DS3TR_Data_t;
```

### `IMUReading`

A single timestamped IMU reading stored in the circular buffer.

```c
typedef struct {
    uint32_t tick;             // HAL_GetTick() timestamp (ms)
    float ax, ay, az;          // Accelerometer (m/s²)
    float gx, gy, gz;          // Gyroscope (dps)
} IMUReading;
```

**Memory**: Each reading is 28 bytes. The buffer holds 100 entries = **2,800 bytes** in `.bss`.

---

## Driver API — lsm6ds3tr

### `lsm6ds3tr_init_driver`

```c
void lsm6ds3tr_init_driver(I2C_HandleTypeDef *hi2c);
```

Initialize the driver with the I2C peripheral handle. Must be called **once before any other driver function**. Sets sensor state to `SENSOR_STATE_LOST` and zeros all data fields.

| Parameter | Type | Description |
|---|---|---|
| `hi2c` | `I2C_HandleTypeDef *` | Pointer to the HAL I2C handle (e.g., `&hi2c3`) |

**Example:**
```c
lsm6ds3tr_init_driver(&hi2c3);
```

---

### `lsm6ds3tr_check_connection`

```c
uint8_t lsm6ds3tr_check_connection(void);
```

Reads the `WHO_AM_I` register (`0x0F`) to verify the sensor is responsive. Expected response is `0x6A`. Safe to call while DMA is active — if the I2C bus is busy, it returns the cached state without initiating a new transfer.

**Returns:** `1` if connected, `0` if not.

---

### `lsm6ds3tr_configure`

```c
uint8_t lsm6ds3tr_configure(void);
```

Writes configuration to the sensor control registers:

| Register | Address | Value | Effect |
|---|---|---|---|
| `CTRL1_XL` | `0x10` | `0x48` | 104 Hz ODR, $\pm \ 4 \ g$ full-scale |
| `CTRL2_G` | `0x11` | `0x44` | 104 Hz ODR, $\pm \ 500 \ dps$ full-scale |
| `CTRL3_C` | `0x12` | `0x44` | BDU enabled, register auto-increment enabled |

**Returns:** `1` on success, `0` on I2C write failure.

---

### `lsm6ds3tr_calibrate`

```c
void lsm6ds3tr_calibrate(void);
```

Performs a **blocking** 100-sample offset calculation for the gyroscope. The sensor must remain **stationary** during this routine. Duration is approximately **0.3 seconds**.

Internally calls `lsm6ds3tr_check_connection()` and `lsm6ds3tr_configure()` before sampling. Accelerometer offsets are set to zero (gravity axis is unknown without orientation information).

**Important:** Call once after `lsm6ds3tr_init_driver()` and before entering the main loop.

---

### `lsm6ds3tr_read` (blocking — legacy)

```c
uint8_t lsm6ds3tr_read(void);
```

Blocking fallback read. Not used in the main loop — retained for diagnostic use.

1. Checks connection state; reconfigures if sensor was lost
2. Burst-reads 12 bytes via standard (blocking) I2C read
3. Converts to physical units and applies calibration offsets
4. Updates the internal `imu_data` struct

**Returns:** `1` on success, `0` if the read failed or sensor is lost.

---

### `lsm6ds3tr_init_dma_read`

```c
uint8_t lsm6ds3tr_init_dma_read(void);
```

**Primary acquisition function.** Initiates a non-blocking 12-byte DMA burst read starting at `OUTX_L_G` (`0x22`). Data processing happens asynchronously in `HAL_I2C_MemRxCpltCallback()`.

1. Checks connection state; reconfigures if sensor was lost
2. If a previous DMA transfer is still in progress, returns `1` (skip, not an error)
3. Issues `HAL_I2C_Mem_Read_DMA()` for 12 bytes
4. On HAL error, marks sensor as `SENSOR_STATE_LOST`

**Returns:** `1` if the DMA request was accepted or skipped (bus busy), `0` if the request failed or sensor is lost.

**Note:** The actual data arrives later via the DMA callback. After this function returns, the latest data is **not yet updated** — it becomes available when the ISR fires.

---

### `HAL_I2C_MemRxCpltCallback` (internal — ISR context)

```c
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
```

**You do not call this function.** It is automatically invoked by the HAL when `lsm6ds3tr_init_dma_read()` completes its 12-byte DMA transfer. Runs in **interrupt context**.

Processing steps:
1. Validates the I2C instance matches the driver's peripheral
2. Parses the 12-byte DMA buffer into signed 16-bit integers (little-endian)
3. Converts to physical units:
   - Accel: $raw \times 0.000122 \times 9.80665 = m/s^2$
   - Gyro: $raw \times 0.0175 = dps$
4. Subtracts calibration offsets
5. Updates the internal `LSM6DS3TR_Data_t` struct
6. Every 5th sample, pushes to the circular buffer via `imu_buffer_push()`

---

### `lsm6ds3tr_get_data`

```c
LSM6DS3TR_Data_t* lsm6ds3tr_get_data(void);
```

Returns a pointer to the internal `LSM6DS3TR_Data_t` struct. This data is updated in ISR context after each DMA transfer completes.

**Example:**
```c
LSM6DS3TR_Data_t *imu = lsm6ds3tr_get_data();

if (imu->state == SENSOR_STATE_CONNECTED) {
    float accel_x = imu->accel.filt_x;  // m/s²
    float gyro_z  = imu->gyro.filt_z;   // dps
    int16_t raw_y = imu->accel.y;        // raw LSB
}
```

---

## Buffer API — imu_buffer

The circular buffer stores up to `IMU_BUFFER_CAPACITY` (100) timestamped IMU readings. When full, the oldest entry is overwritten. The buffer is populated automatically by the DMA callback (every 5th sample).

### `imu_buffer_push`

```c
void imu_buffer_push(uint32_t tick, float ax, float ay, float az,
                     float gx, float gy, float gz);
```

Push one IMU reading into the ring buffer. Called internally from `HAL_I2C_MemRxCpltCallback()`.

| Parameter | Type | Unit |
|---|---|---|
| `tick` | `uint32_t` | `HAL_GetTick()` timestamp (ms) |
| `ax, ay, az` | `float` | Accelerometer ($m/s^2$) |
| `gx, gy, gz` | `float` | Gyroscope ($dps$) |

**Note:** This is called from ISR context. If you call it from non-ISR code, disable the DMA interrupt first to avoid race conditions.

---

### `imu_buffer_get_latest`

```c
int imu_buffer_get_latest(IMUReading *out);
```

Copies the most recent reading into `*out`.

**Returns:** `1` on success, `0` if the buffer is empty.

**Example:**
```c
IMUReading latest;
if (imu_buffer_get_latest(&latest)) {
    printf("t=%lu ax=%.3f gx=%.3f\r\n", latest.tick, latest.ax, latest.gx);
}
```

---

### `imu_buffer_get_all`

```c
size_t imu_buffer_get_all(IMUReading *out);
```

Copies all stored readings (oldest first) into the array pointed to by `out`. The output array must hold at least `IMU_BUFFER_CAPACITY` elements.

**Returns:** Number of entries written (0 if empty).

**Example:**
```c
IMUReading history[IMU_BUFFER_CAPACITY];

__HAL_DMA_DISABLE_IT(hi2c3.hdmarx, DMA_IT_TC);  // disable DMA interrupt
size_t n = imu_buffer_get_all(history);
__HAL_DMA_ENABLE_IT(hi2c3.hdmarx, DMA_IT_TC);    // re-enable

for (size_t i = 0; i < n; ++i) {
    printf("t=%lu ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f\r\n",
           history[i].tick,
           history[i].ax, history[i].ay, history[i].az,
           history[i].gx, history[i].gy, history[i].gz);
}
```

---

### `imu_buffer_count`

```c
size_t imu_buffer_count(void);
```

Returns the number of valid readings currently stored in the buffer.

---

## Integration Guide

### Minimal Setup

```c
#include "lsm6ds3tr.h"
#include "imu_buffer.h"

// In your initialization (after MX_I2C3_Init, MX_DMA_Init):
lsm6ds3tr_init_driver(&hi2c3);
lsm6ds3tr_calibrate();  // ~0.3s, sensor must be stationary

// Main loop:
uint32_t last_tick = 0;
while (1) {
    if ((HAL_GetTick() - last_tick) >= 2) {
        last_tick = HAL_GetTick();
        lsm6ds3tr_init_dma_read();
    }

    // Access live data:
    LSM6DS3TR_Data_t *imu = lsm6ds3tr_get_data();
    if (imu->state == SENSOR_STATE_CONNECTED) {
        // use imu->accel.filt_x, imu->gyro.filt_z, etc.
    }
}
```

### CAN Bus Integration

To transmit IMU data over CAN, read from the live data pointer and pack into CAN frames:

```c
void CAN_Send_IMU_Data(void) {
    LSM6DS3TR_Data_t *imu = lsm6ds3tr_get_data();
    if (imu->state != SENSOR_STATE_CONNECTED) return;

    CAN_TxHeaderTypeDef header;
    header.StdId = 0x123;   // your CAN ID for accel
    header.DLC = 6;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;

    // Scale floats to int16 (×100 for 2 decimal places)
    int16_t ax = (int16_t)(imu->accel.filt_x * 100);
    int16_t ay = (int16_t)(imu->accel.filt_y * 100);
    int16_t az = (int16_t)(imu->accel.filt_z * 100);

    uint8_t data[6];
    data[0] = ax & 0xFF; data[1] = (ax >> 8) & 0xFF;
    data[2] = ay & 0xFF; data[3] = (ay >> 8) & 0xFF;
    data[4] = az & 0xFF; data[5] = (az >> 8) & 0xFF;

    uint32_t mailbox;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
        HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);
    }
}
```

### Retrieving Buffered History

When retrieving buffered data from non-ISR code, you **must** disable the DMA transfer-complete interrupt to prevent the ISR from modifying the buffer mid-copy:

```c
// Disable DMA TC interrupt to get a consistent snapshot
__HAL_DMA_DISABLE_IT(hi2c3.hdmarx, DMA_IT_TC);
IMUReading latest;
int ok = imu_buffer_get_latest(&latest);
__HAL_DMA_ENABLE_IT(hi2c3.hdmarx, DMA_IT_TC);
```

### STM32CubeMX Peripheral Requirements

| Peripheral | Configuration |
|---|---|
| I2C (e.g., I2C3) | Standard mode (100 kHz), 7-bit addressing |
| DMA | Stream linked to I2Cx_RX, normal mode, byte-width, memory increment enabled |
| NVIC | I2Cx event interrupt enabled, DMA stream interrupt enabled |

---

## Register Map (Key Registers)

| Address | Name | Description |
|---|---|---|
| `0x0F` | WHO_AM_I | Device ID (expected: `0x6A`) |
| `0x10` | CTRL1_XL | Accelerometer control (ODR + full-scale) |
| `0x11` | CTRL2_G | Gyroscope control (ODR + full-scale) |
| `0x12` | CTRL3_C | Control register 3 (BDU, IF_INC) |
| `0x22`–`0x27` | OUTX_L_G – OUTZ_H_G | Gyroscope output (X, Y, Z) |
| `0x28`–`0x2D` | OUTX_L_XL – OUTZ_H_XL | Accelerometer output (X, Y, Z) |

Registers `0x22`–`0x2D` form a contiguous 12-byte block, enabling a single burst read for all 6 axes.

---

## Sensitivity Constants

| Parameter | Value | Unit | Source |
|---|---|---|---|
| Accel sensitivity ($\pm \ 4 \ g$) | 0.000122 | $g/LSB$ | Datasheet Table 2 |
| Gyro sensitivity ($\pm \ 500 \ dps$) | 0.0175 | $dps/LSB$ | Datasheet Table 4 |
| Gravity constant | 9.80665 | $m/s^2$ | SI standard |

**Conversions applied in the DMA callback:**
- Accelerometer: $\text{value} = raw \times 0.000122 \times 9.80665 - \text{offset} \quad [m/s^2]$
- Gyroscope: $\text{value} = raw \times 0.0175 - \text{offset} \quad [dps]$

---

## ISR Safety & Concurrency

The DMA callback (`HAL_I2C_MemRxCpltCallback`) runs in **interrupt context**. This has important implications:

| Concern | Current behavior |
|---|---|
| `imu_data` struct | Written from ISR, read from main loop — **no lock**. Reads of individual `float` fields are atomic on Cortex-M4, but reading multiple fields (e.g., `ax` then `ay`) may see values from different samples. |
| Circular buffer | Written from ISR via `imu_buffer_push()`. Reading from main-loop code should **disable the DMA TC interrupt** to get a consistent snapshot. |
| `lsm6ds3tr_get_data()` | Returns a pointer to static data. Safe to call from any context, but the data may update between field accesses. |

**Rule of thumb:** For single-field access (e.g., reading `accel.filt_x` alone), no protection is needed. For multi-field snapshots, disable the DMA interrupt around the read.

---

## Recommendations for Generalization

The current API is functional and well-structured for a single IMU on a fixed configuration. Below are concrete recommendations for making it more reusable across teams with different hardware setups and requirements.

### 1. Runtime-Configurable ODR and Full-Scale Range

The current driver hardcodes the ODR (104 Hz) and full-scale range ($\pm4g$, $\pm500dps$). Teams using different motion profiles (e.g., high-impact or slow rehabilitation) will need different settings.

**Recommended functions:**

```c
typedef enum {
    LSM6DS3TR_ODR_12_5HZ = 0x10,
    LSM6DS3TR_ODR_26HZ   = 0x20,
    LSM6DS3TR_ODR_52HZ   = 0x30,
    LSM6DS3TR_ODR_104HZ  = 0x40,
    LSM6DS3TR_ODR_208HZ  = 0x50,
    LSM6DS3TR_ODR_416HZ  = 0x60,
    LSM6DS3TR_ODR_833HZ  = 0x70,
    LSM6DS3TR_ODR_1660HZ = 0x80,
} LSM6DS3TR_ODR_t;

typedef enum {
    LSM6DS3TR_ACCEL_2G  = 0x00,
    LSM6DS3TR_ACCEL_4G  = 0x08,
    LSM6DS3TR_ACCEL_8G  = 0x0C,
    LSM6DS3TR_ACCEL_16G = 0x04,
} LSM6DS3TR_AccelRange_t;

typedef enum {
    LSM6DS3TR_GYRO_125DPS  = 0x02,
    LSM6DS3TR_GYRO_250DPS  = 0x00,
    LSM6DS3TR_GYRO_500DPS  = 0x04,
    LSM6DS3TR_GYRO_1000DPS = 0x08,
    LSM6DS3TR_GYRO_2000DPS = 0x0C,
} LSM6DS3TR_GyroRange_t;

// Configure with specific settings instead of hardcoded values
uint8_t lsm6ds3tr_set_accel_config(LSM6DS3TR_ODR_t odr, LSM6DS3TR_AccelRange_t range);
uint8_t lsm6ds3tr_set_gyro_config(LSM6DS3TR_ODR_t odr, LSM6DS3TR_GyroRange_t range);
```

The sensitivity constants would need to be updated internally based on the selected range to keep the conversion math correct.

### 2. Configurable Buffer Capacity and Downsample Ratio

The buffer is fixed at 100 entries and the downsample factor (every 5th sample) is hardcoded in the DMA callback. Different teams may need longer history or higher resolution.

**Recommended changes:**

```c
// Allow compile-time override in a project's build flags (-DIMU_BUFFER_CAPACITY=500)
#ifndef IMU_BUFFER_CAPACITY
#define IMU_BUFFER_CAPACITY 100
#endif

// Runtime-configurable downsample ratio
void lsm6ds3tr_set_buffer_downsample(uint8_t ratio);  // default: 5
```

### 3. Buffer Reset Function

There is currently no way to clear the buffer. Teams may need to reset it (e.g., between test runs, after a mode change, or after transmitting all stored data).

**Recommended function:**

```c
void imu_buffer_reset(void);  // sets count = 0, head = 0
```

### 4. Multi-Instance Support

The driver uses static globals (`_hi2c`, `imu_data`, offsets), which limits it to a single sensor. If a team needs multiple IMUs (e.g., one per limb segment), the driver should accept an instance handle.

**Recommended pattern:**

```c
typedef struct {
    I2C_HandleTypeDef *hi2c;
    LSM6DS3TR_Data_t   data;
    uint8_t            dma_rx_buffer[12];
    float              offset_gx, offset_gy, offset_gz;
    float              offset_ax, offset_ay, offset_az;
    float              accel_sensitivity;
    float              gyro_sensitivity;
} LSM6DS3TR_Handle_t;

void    lsm6ds3tr_init(LSM6DS3TR_Handle_t *dev, I2C_HandleTypeDef *hi2c);
uint8_t lsm6ds3tr_configure(LSM6DS3TR_Handle_t *dev);
uint8_t lsm6ds3tr_init_dma_read(LSM6DS3TR_Handle_t *dev);
// ... etc.
```

This passes the handle through every function rather than relying on file-scope statics. The DMA callback would need a lookup mechanism (e.g., matching `hi2c->Instance`) to dispatch to the correct handle.

### 5. Getter Functions for Calibration Offsets and Configuration

Currently there is no way to inspect or override the calibration offsets or confirm what configuration was written to the sensor. This makes debugging harder and prevents teams from saving/restoring calibration across power cycles.

**Recommended functions:**

```c
// Read back current offsets
void lsm6ds3tr_get_offsets(float *ax, float *ay, float *az,
                           float *gx, float *gy, float *gz);

// Set offsets manually (e.g., from EEPROM or flash-stored calibration)
void lsm6ds3tr_set_offsets(float ax, float ay, float az,
                           float gx, float gy, float gz);

// Read back the configuration register values currently stored
uint8_t lsm6ds3tr_get_accel_config(void);
uint8_t lsm6ds3tr_get_gyro_config(void);
```

### 6. Atomic Snapshot of Live Data

Reading multiple fields from `lsm6ds3tr_get_data()` in the main loop may see values from different samples because the ISR can fire between field accesses. A copy function avoids this without requiring the caller to manage interrupts.

**Recommended function:**

```c
// Copies a consistent snapshot of the current data with interrupt protection
void lsm6ds3tr_get_snapshot(LSM6DS3TR_Data_t *out);
```

Implementation would briefly disable the DMA TC interrupt, copy the struct, and re-enable:

```c
void lsm6ds3tr_get_snapshot(LSM6DS3TR_Data_t *out) {
    __disable_irq();
    *out = imu_data;
    __enable_irq();
}
```

### 7. Error Callback / Status Reporting

Teams integrating the sensor into safety-critical applications need to know when the sensor drops off the bus, not just via polling `imu_data.state`. A user-registerable callback would allow immediate response.

**Recommended function:**

```c
typedef void (*LSM6DS3TR_ErrorCallback_t)(SensorState_t new_state);

void lsm6ds3tr_register_error_callback(LSM6DS3TR_ErrorCallback_t cb);
```

### 8. Power Management API

The `power_config` field exists in `LSM6DS3TR_Data_t` but is never written. The LSM6DS3TR-C supports low-power and high-performance modes that teams may want to control.

**Recommended functions:**

```c
uint8_t lsm6ds3tr_set_power_mode(uint8_t mode);
uint8_t lsm6ds3tr_sleep(void);     // enter low-power / power-down
uint8_t lsm6ds3tr_wake(void);      // resume normal operation
```

### Summary of Recommended Additions

| Function | Purpose |
|---|---|
| `lsm6ds3tr_set_accel_config()` | Runtime ODR and full-scale range for accelerometer |
| `lsm6ds3tr_set_gyro_config()` | Runtime ODR and full-scale range for gyroscope |
| `lsm6ds3tr_set_buffer_downsample()` | Control how many DMA samples per buffer entry |
| `imu_buffer_reset()` | Clear the circular buffer |
| `lsm6ds3tr_get_offsets()` | Read calibration offsets |
| `lsm6ds3tr_set_offsets()` | Manually set calibration offsets |
| `lsm6ds3tr_get_snapshot()` | ISR-safe copy of full live data |
| `lsm6ds3tr_register_error_callback()` | Notification on sensor state changes |
| `lsm6ds3tr_set_power_mode()` / `sleep()` / `wake()` | Power management |
| Multi-instance handle pattern | Support multiple sensors on different I2C buses |
