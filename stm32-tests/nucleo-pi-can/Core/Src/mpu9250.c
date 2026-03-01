#include "mpu9250.h"
#include "main.h"
#include <stdio.h>

#define GRAVITY 9.80665f

// Sensitivities for MPU9250
// 4g range -> 8192 LSB/g -> 1/8192 = 0.00012207031
#define LIN_ACCEL_SENSITIVITY_4G    0.00012207f
// 500dps range -> 65.5 LSB/dps -> 1/65.5 = 0.015267175
#define ANG_VEL_SENSITIVITY_500DPS  0.01526718f

static I2C_HandleTypeDef *_hi2c;
static MPU9250_Data_t imu_data;

// MPU9250 contiguous data: 6 bytes Accel, 2 bytes Temp, 6 bytes Gyro = 14 bytes total
static uint8_t dma_rx_buffer[14];

// Variable definitions for offset calculation
static uint16_t calibrated = 0;
static float offset_gx = 0, offset_gy = 0, offset_gz = 0;
static float offset_ax = 0, offset_ay = 0, offset_az = 0;

void mpu9250_init_driver(I2C_HandleTypeDef *hi2c)
{
    _hi2c = hi2c;
    imu_data.state = SENSOR_STATE_LOST;

    // Initialize struct with default values
    imu_data.accel.x = 0;
    imu_data.accel.y = 0;
    imu_data.accel.z = 0;
    imu_data.gyro.x = 0;
    imu_data.gyro.y = 0;
    imu_data.gyro.z = 0;
    imu_data.accel.filt_x = 0;
    imu_data.accel.filt_y = 0;
    imu_data.accel.filt_z = 0;
    imu_data.gyro.filt_x = 0;
    imu_data.gyro.filt_y = 0;
    imu_data.gyro.filt_z = 0;

    // Initialize config values to 0 (not configured)
    imu_data.gyro_config = 0;
    imu_data.accel_config = 0;
    imu_data.power_config = 0;
}

uint8_t mpu9250_check_connection(void)
{
    uint8_t who_am_i;

    // Read WHO_AM_I register (Usually 0x71 for MPU9250)
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(_hi2c, MPU9250_ADDRESS, REG_WHO_AM_I, 1, &who_am_i, 1, 100);

    if (ret == HAL_OK && who_am_i == WHO_AM_I_VAL)
    {
        imu_data.state = SENSOR_STATE_CONNECTED;
        return 1;
    }

    imu_data.state = SENSOR_STATE_LOST;
    return 0;
}

uint8_t mpu9250_configure(void)
{
    uint8_t temp_data;
    HAL_StatusTypeDef ret;

    // 1. Wake up the sensor (Clear SLEEP bit in PWR_MGMT_1)
    temp_data = 0x00;
    ret = HAL_I2C_Mem_Write(_hi2c, MPU9250_ADDRESS, REG_PWR_MGMT_1, 1, &temp_data, 1, 100);
    if (ret != HAL_OK) return 0;
    imu_data.power_config = temp_data;

    HAL_Delay(10); // Wait for sensor to stabilize

    // 2. Configure Accelerometer (4g FS) -> ACCEL_CONFIG register
    temp_data = FS_ACCEL_4G; // Usually 0x08 for 4g
    ret = HAL_I2C_Mem_Write(_hi2c, MPU9250_ADDRESS, REG_ACCEL_CONFIG, 1, &temp_data, 1, 100);
    if (ret != HAL_OK) return 0;
    imu_data.accel_config = temp_data;

    // 3. Configure Gyroscope (500dps FS) -> GYRO_CONFIG register
    temp_data = FS_GYRO_500DPS; // Usually 0x08 for 500dps
    ret = HAL_I2C_Mem_Write(_hi2c, MPU9250_ADDRESS, REG_GYRO_CONFIG, 1, &temp_data, 1, 100);
    if (ret != HAL_OK) return 0;
    imu_data.gyro_config = temp_data;

    return 1;
}

void mpu9250_calibrate(void)
{
    uint8_t data[14];

    long total_off_gx = 0;
    long total_off_gy = 0;
    long total_off_gz = 0;

    long total_off_ax = 0;
    long total_off_ay = 0;
    long total_off_az = 0;

    for (int i = 0; i < 100; ++i)
    {
        // Start reading from ACCEL_XOUT_H
        HAL_I2C_Mem_Read(_hi2c, MPU9250_ADDRESS, REG_ACCEL_XOUT_H, 1, data, 14, 100);

        // MPU9250 is Big Endian (High Byte First)
        // Gyro is at indices 8 to 13 (after Accel and Temp)
        int16_t curr_off_gx = ((int16_t)data[8] << 8) | data[9];
        int16_t curr_off_gy = ((int16_t)data[10] << 8) | data[11];
        int16_t curr_off_gz = ((int16_t)data[12] << 8) | data[13];

        total_off_gx += curr_off_gx;
        total_off_gy += curr_off_gy;
        total_off_gz += curr_off_gz;

        HAL_Delay(3);
    }

    // TODO: Add Accelerometer calibration
    offset_ax = 0;
    offset_ay = 0;
    offset_az = 0;

    offset_gx = (total_off_gx / 100.0f) * ANG_VEL_SENSITIVITY_500DPS;
    offset_gy = (total_off_gy / 100.0f) * ANG_VEL_SENSITIVITY_500DPS;
    offset_gz = (total_off_gz / 100.0f) * ANG_VEL_SENSITIVITY_500DPS;
}

uint8_t mpu9250_read(void)
{
    if (imu_data.state == SENSOR_STATE_LOST)
    {
        if (!mpu9250_check_connection())
            return 0;

        if (!mpu9250_configure())
            return 0;
    }

    uint8_t buffer[14];
    // Start reading from ACCEL_XOUT_H (0x3B)
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(_hi2c, MPU9250_ADDRESS, REG_ACCEL_XOUT_H, 1, buffer, 14, 100);

    if (ret != HAL_OK)
    {
        imu_data.state = SENSOR_STATE_LOST;
        return 0;
    }

    // MPU9250 is Big Endian
    // [0-5]: Accel, [6-7]: Temp, [8-13]: Gyro
    int16_t x_accel = ((int16_t)buffer[0] << 8) | buffer[1];
    int16_t y_accel = ((int16_t)buffer[2] << 8) | buffer[3];
    int16_t z_accel = ((int16_t)buffer[4] << 8) | buffer[5];

    int16_t x_gyro  = ((int16_t)buffer[8] << 8) | buffer[9];
    int16_t y_gyro  = ((int16_t)buffer[10] << 8) | buffer[11];
    int16_t z_gyro  = ((int16_t)buffer[12] << 8) | buffer[13];

    // Unit Conversions + Offset Application
    float x_accel_ms2 = (x_accel * LIN_ACCEL_SENSITIVITY_4G) * GRAVITY - offset_ax;
    float y_accel_ms2 = (y_accel * LIN_ACCEL_SENSITIVITY_4G) * GRAVITY - offset_ay;
    float z_accel_ms2 = (z_accel * LIN_ACCEL_SENSITIVITY_4G) * GRAVITY + offset_az;

    float x_gyro_dps = (x_gyro * ANG_VEL_SENSITIVITY_500DPS) - offset_gx;
    float y_gyro_dps = (y_gyro * ANG_VEL_SENSITIVITY_500DPS) - offset_gy;
    float z_gyro_dps = (z_gyro * ANG_VEL_SENSITIVITY_500DPS) - offset_gz;

    imu_data.accel.filt_x = x_accel_ms2;
    imu_data.accel.filt_y = y_accel_ms2;
    imu_data.accel.filt_z = z_accel_ms2;

    imu_data.gyro.filt_x = x_gyro_dps;
    imu_data.gyro.filt_y = y_gyro_dps;
    imu_data.gyro.filt_z = z_gyro_dps;

    // Store raw data in Struct
    imu_data.accel.x = x_accel;
    imu_data.accel.y = y_accel;
    imu_data.accel.z = z_accel;
    imu_data.gyro.x  = x_gyro;
    imu_data.gyro.y  = y_gyro;
    imu_data.gyro.z  = z_gyro;

    return 1;
}

uint8_t mpu9250_init_dma_read(void)
{
    if (imu_data.state == SENSOR_STATE_LOST)
    {
        if (!mpu9250_check_connection())
            return 0;

        if (!mpu9250_configure())
            return 0;
    }

    // Burst Read 14 Bytes starting from REG_ACCEL_XOUT_H (0x3B)
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read_DMA(
        _hi2c,
        MPU9250_ADDRESS,
        REG_ACCEL_XOUT_H,
        1,
        dma_rx_buffer,
        14); // 14 bytes captures Accel -> Temp -> Gyro

    if (ret != HAL_OK)
    {
        imu_data.state = SENSOR_STATE_LOST;
        return 0;
    }

    return 1;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == _hi2c->Instance)
    {
        // Buffer Layout (Big Endian): [0-5] -> Accel, [6-7] -> Temp, [8-13] -> Gyro
        int16_t x_accel = ((int16_t)dma_rx_buffer[0] << 8) | dma_rx_buffer[1];
        int16_t y_accel = ((int16_t)dma_rx_buffer[2] << 8) | dma_rx_buffer[3];
        int16_t z_accel = ((int16_t)dma_rx_buffer[4] << 8) | dma_rx_buffer[5];

        int16_t x_gyro  = ((int16_t)dma_rx_buffer[8] << 8) | dma_rx_buffer[9];
        int16_t y_gyro  = ((int16_t)dma_rx_buffer[10] << 8) | dma_rx_buffer[11];
        int16_t z_gyro  = ((int16_t)dma_rx_buffer[12] << 8) | dma_rx_buffer[13];

        // Unit Conversions + Offset Application
        float x_accel_ms2 = (x_accel * LIN_ACCEL_SENSITIVITY_4G) * GRAVITY - offset_ax;
        float y_accel_ms2 = (y_accel * LIN_ACCEL_SENSITIVITY_4G) * GRAVITY - offset_ay;
        float z_accel_ms2 = (z_accel * LIN_ACCEL_SENSITIVITY_4G) * GRAVITY + offset_az;

        float x_gyro_dps = (x_gyro * ANG_VEL_SENSITIVITY_500DPS) - offset_gx;
        float y_gyro_dps = (y_gyro * ANG_VEL_SENSITIVITY_500DPS) - offset_gy;
        float z_gyro_dps = (z_gyro * ANG_VEL_SENSITIVITY_500DPS) - offset_gz;

        // Direct assignment
        imu_data.accel.filt_x = x_accel_ms2;
        imu_data.accel.filt_y = y_accel_ms2;
        imu_data.accel.filt_z = z_accel_ms2;

        imu_data.gyro.filt_x = x_gyro_dps;
        imu_data.gyro.filt_y = y_gyro_dps;
        imu_data.gyro.filt_z = z_gyro_dps;

        // Store raw data
        imu_data.accel.x = x_accel;
        imu_data.accel.y = y_accel;
        imu_data.accel.z = z_accel;

        imu_data.gyro.x = x_gyro;
        imu_data.gyro.y = y_gyro;
        imu_data.gyro.z = z_gyro;
    }
}

MPU9250_Data_t *mpu9250_get_data(void)
{
    return &imu_data;
}
