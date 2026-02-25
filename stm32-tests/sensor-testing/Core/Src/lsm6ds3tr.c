/*
 * lsm6ds3tr.c
 *
 * LSM6DS3TR Driver for Nucleo F446RE
 * Referenced from discovery-tests/discovery-imu-test
 */

#include "lsm6ds3tr.h"
#include "main.h"
#include <stdio.h>

#define GRAVITY 9.80665f

// https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf
#define LIN_ACCEL_SENSITIVITY_4G 0.000122f // g/LSB
#define ANG_VEL_SENSITIVITY_500DPS 0.0175f // dps/LSB

// Normal mode for accel+gyro (104Hz ODR) -> Tables 52,55 of lsm6ds3tr-c.pdf
#define ODR_104HZ 0x40

static I2C_HandleTypeDef *_hi2c;
static LSM6DS3TR_Data_t imu_data;

static uint8_t dma_rx_buffer[12];

// Variable definitions for offset calculation
static uint16_t calibrated = 0;
static float offset_gx = 0, offset_gy = 0, offset_gz = 0;
static float offset_ax = 0, offset_ay = 0, offset_az = 0;

void lsm6ds3tr_init_driver(I2C_HandleTypeDef *hi2c)
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

uint8_t lsm6ds3tr_check_connection(void)
{
	uint8_t who_am_i;

	// Read WHO_AM_I register
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(_hi2c, LSM6DS3TR_ADDRESS, REG_WHO_AM_I, 1, &who_am_i, 1, 100);

	if (ret == HAL_OK && who_am_i == WHO_AM_I_VAL)
	{
		imu_data.state = SENSOR_STATE_CONNECTED;

		return 1;
	}

	imu_data.state = SENSOR_STATE_LOST;

	return 0;
}

uint8_t lsm6ds3tr_configure(void)
{
	uint8_t temp_data;
	HAL_StatusTypeDef ret;

	// Turn on Accelerometer -> configure accel (104 Hz ODR, 4g FS)
	temp_data = ODR_104HZ | FS_ACCEL_4G;
	ret = HAL_I2C_Mem_Write(_hi2c, LSM6DS3TR_ADDRESS, REG_CTRL1_XL, 1, &temp_data, 1, 100);
	
	if (ret != HAL_OK)
		return 0;

	imu_data.accel_config = temp_data;

	// Turn on Gyroscope -> configure accel (104 Hz ODR, 500dps FS)
	temp_data = ODR_104HZ | FS_GYRO_500DPS;
	ret = HAL_I2C_Mem_Write(_hi2c, LSM6DS3TR_ADDRESS, REG_CTRL2_G, 1, &temp_data, 1, 100);

	if (ret != HAL_OK)
		return 0;

	imu_data.gyro_config = temp_data;

	// Enable Block Data Update
	temp_data = BDU_ENABLE;
	ret = HAL_I2C_Mem_Write(_hi2c, LSM6DS3TR_ADDRESS, REG_CTRL3_C, 1, &temp_data, 1, 100);

	if (ret != HAL_OK)
		return 0;

	return 1;
}

void lsm6ds3tr_calibrate(void)
{
	HAL_StatusTypeDef ret;

	uint8_t data[12];

	long total_off_gx = 0;
	long total_off_gy = 0;
	long total_off_gz = 0;

	long total_off_ax = 0;
	long total_off_ay = 0;
	long total_off_az = 0;

	for (int i = 0; i < 100; ++i)
	{
		HAL_I2C_Mem_Read(_hi2c, LSM6DS3TR_ADDRESS, REG_OUTX_L_G, 1, data, 12, 100);

		int16_t curr_off_gx = ((int16_t)data[1] << 8) + data[0];
		int16_t curr_off_gy = ((int16_t)data[3] << 8) + data[2];
		int16_t curr_off_gz = ((int16_t)data[5] << 8) + data[4];

		int16_t curr_off_gx = ((int16_t)data[7] << 8) + data[6];
		int16_t curr_off_gy = ((int16_t)data[9] << 8) + data[8];
		int16_t curr_off_gz = ((int16_t)data[11] << 8) + data[10];

		total_off_gx += curr_off_gx;
		total_off_gy += curr_off_gy;
		total_off_gz += curr_off_gz;

		HAL_Delay(3);
	}

	// TODO: Add Accelerometer calibration
	offset_ax = 0;
	offset_ay = 0;
	offset_az = 0;

	offset_gx = (total_off_gx / 100) * ANG_VEL_SENSITIVITY_500DPS;
	offset_gy = (total_off_gy / 100) * ANG_VEL_SENSITIVITY_500DPS;
	offset_gz = (total_off_gz / 100) * ANG_VEL_SENSITIVITY_500DPS;

	return 1;
}

uint8_t lsm6ds3tr_read(void)
{
	if (imu_data.state == SENSOR_STATE_LOST)
	{
		if (!lsm6ds3tr_check_connection())
			return 0;

		if (!lsm6ds3tr_configure())
			return 0;
	}

	uint8_t buffer[12];
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(_hi2c, LSM6DS3TR_ADDRESS, REG_OUTX_L_G, 1, buffer, 12, 100);

	if (ret != HAL_OK)
	{
		imu_data.state = SENSOR_STATE_LOST;

		return 0;
	}

	int16_t x_accel;
	int16_t y_accel;
	int16_t z_accel;

	int16_t x_gyro;
	int16_t y_gyro;
	int16_t z_gyro;

	// Get raw data from IMU (LITTLE_ENDIAN, so LB comes before HB [opposite of mpu9250])
	x_accel = ((int16_t)buffer[1] << 8) + buffer[0];
	y_accel = ((int16_t)buffer[3] << 8) + buffer[2];
	z_accel = ((int16_t)buffer[5] << 8) + buffer[4];

	x_gyro = ((int16_t)buffer[7] << 8) + buffer[6];
	y_gyro = ((int16_t)buffer[9] << 8) + buffer[8];
	z_gyro = ((int16_t)buffer[11] << 8) + buffer[10];

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
	imu_data.gyro.x = x_gyro;
	imu_data.gyro.y = y_gyro;
	imu_data.gyro.z = z_gyro;

	return 1;
}

uint8_t lsm6ds3tr_init_dma_read(void)
{
	if (imu_data.state == SENSOR_STATE_LOST)
	{
		if (!lsm6ds3tr_check_connection())
			return 0;

		if (!lsm6ds3tr_configure())
			return 0;
	}

	// Burst Read 12 Bytes starting from REG_OUTX_L_G (0x22)
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read_DMA(
		_hi2c,
		LSM6DS3TR_ADDRESS,
		REG_OUTX_L_G,
		1,
		dma_rx_buffer,
		12);

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
        // Buffer Layout: [0 - 5] -> Gyroscope, [6 - 11] -> Accelerometer

        int16_t x_gyro = ((int16_t)dma_rx_buffer[1] << 8) + dma_rx_buffer[0];
        int16_t y_gyro = ((int16_t)dma_rx_buffer[3] << 8) + dma_rx_buffer[2];
        int16_t z_gyro = ((int16_t)dma_rx_buffer[5] << 8) + dma_rx_buffer[4];

        int16_t x_accel = ((int16_t)dma_rx_buffer[7] << 8) + dma_rx_buffer[6];
        int16_t y_accel = ((int16_t)dma_rx_buffer[9] << 8) + dma_rx_buffer[8];
        int16_t z_accel = ((int16_t)dma_rx_buffer[11] << 8) + dma_rx_buffer[10];

        // Unit Conversions + Offset Application (MATCHING THE READ FUNCTION)
        float x_accel_ms2 = (x_accel * LIN_ACCEL_SENSITIVITY_4G) * GRAVITY - offset_ax;
        float y_accel_ms2 = (y_accel * LIN_ACCEL_SENSITIVITY_4G) * GRAVITY - offset_ay;
        float z_accel_ms2 = (z_accel * LIN_ACCEL_SENSITIVITY_4G) * GRAVITY + offset_az;

        float x_gyro_dps = (x_gyro * ANG_VEL_SENSITIVITY_500DPS) - offset_gx;
        float y_gyro_dps = (y_gyro * ANG_VEL_SENSITIVITY_500DPS) - offset_gy;
        float z_gyro_dps = (z_gyro * ANG_VEL_SENSITIVITY_500DPS) - offset_gz;

        // Direct assignment (Hardware Filter is doing the work)
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

LSM6DS3TR_Data_t *lsm6ds3tr_get_data(void)
{
	return &imu_data;
}