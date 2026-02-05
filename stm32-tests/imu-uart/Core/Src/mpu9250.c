/*
 * mpu9250.c
 *
 * MPU9250 IMU Driver for Nucleo F446RE
 * Referenced from discovery-tests/discovery-imu-test
 */

#include "mpu9250.h"
#include "main.h"
#include <stdio.h>

// https://invensense.tdk.com/wp-content/uploads/2017/11/RM-MPU-9250A-00-v1.6.pdf
#define REG_CONFIG_GYRO_DLPF 26 // 0x1A (Configuration + Gyroscope DLPF)
#define REG_CONFIG_GYRO 27		// 0x1B (Gyroscope Configuration)
#define REG_CONFIG_ACCEL 28		// 0x1C (Accelerometer Configuration)
#define REG_CONFIG_ACCEL_2 29	// 0x1D (Accelerometer DLPF Configuration)
#define REG_POW_MAN 107			// 0x6B (Power Management 1)

#define GRAVITY 9.80665f

// https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf
#define LIN_ACCEL_SENSITIVITY_4G 0.000122f // g/LSB
#define ANG_VEL_SENSITIVITY_500DPS 0.0175f // dps/LSB

static I2C_HandleTypeDef *_hi2c;
static MPU9250_Data_t imu_data;

// Variable definitions for offset calculation
static uint16_t calibrated = 0;
static float offset_gx = 0, offset_gy = 0, offset_gz = 0;
static float offset_ax = 0, offset_ay = 0, offset_az = 0;

void mpu9250_init_driver(I2C_HandleTypeDef *hi2c)
{
	_hi2c = hi2c;
	imu_data.state = SENSOR_STATE_LOST;
	imu_data.accel.x = 0;
	imu_data.accel.y = 0;
	imu_data.accel.z = 0;
	imu_data.gyro.x = 0;
	imu_data.gyro.y = 0;
	imu_data.gyro.z = 0;
	imu_data.gyro_config = 0;
	imu_data.accel_config = 0;
	imu_data.power_config = 0;
}

uint8_t mpu9250_check_connection(void)
{
	HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(_hi2c, MPU9250_ADDRESS, 1, 100);

	if (ret == HAL_OK)
	{
		imu_data.state = SENSOR_STATE_CONNECTED;

		return 1;
	}

	imu_data.state = SENSOR_STATE_LOST;

	return 0;
}

void mpu9250_calibrate(void) {
	if (!calibrated)
    {
		uint8_t data[14];

		long total_gx_raw = 0;
		long total_gy_raw = 0;
		long total_gz_raw = 0;

		int sample_num = 100;

		for (int i = 0; i < sample_num; ++i)
		{
			HAL_I2C_Mem_Read(_hi2c, MPU9250_ADDRESS, REG_ACCEL_DATA, 1, data, 14, 100);

			total_gx += ((int16_t)data[8] << 8) + data[9];
			total_gy += ((int16_t)data[10] << 8) + data[11];
			total_gz += ((int16_t)data[12] << 8) + data[13];

			HAL_Delay(3);
		}

		offset_gx = (float)(total_gx / sample_num) * ANG_VEL_SENSITIVITY_500DPS;
		offset_gy = (float)(total_gy / sample_num) * ANG_VEL_SENSITIVITY_500DPS;
		offset_gz = (float)(total_gz / sample_num) * ANG_VEL_SENSITIVITY_500DPS;

		// TODO: Add accelerometer calibration, otherwise assume factory

		offset_ax = 0;
		offset_ay = 0;
		offset_az = 0;

		calibrated = 1;
	}
}

uint8_t mpu9250_configure(void)
{
	uint8_t temp_data;
	HAL_StatusTypeDef ret;

	// Configure Power
	temp_data = 0x00;
	ret = HAL_I2C_Mem_Write(_hi2c, MPU9250_ADDRESS, REG_POW_MAN, 1, &temp_data, 1, 100);

	if (ret != HAL_OK)
		return 0;

	imu_data.power_config = temp_data;

	// Configure Gyroscope DLPF - Register 26 (0x1A)
	temp_data = 0x03;
	ret = HAL_I2C_Mem_Write(_hi2c, MPU9250_ADDRESS, REG_CONFIG_GYRO_DLPF, 1, &temp_data, 1, 100);

	if (ret != HAL_OK)
		return 0;

	// Configure Gyroscope - Register 27 (0x1B)
	temp_data = (1 << 3);
	ret = HAL_I2C_Mem_Write(_hi2c, MPU9250_ADDRESS, REG_CONFIG_GYRO, 1, &temp_data, 1, 100);

	if (ret != HAL_OK)
		return 0;

	imu_data.gyro_config = temp_data;

	// Configure Accelerometer DLPF - Register 29 (0x1D)
	temp_data = 0x03;
	ret = HAL_I2C_Mem_Write(_hi2c, MPU9250_ADDRESS, REG_CONFIG_ACCEL_2, 1, &temp_data, 1, 100);

	if (ret != HAL_OK)
		return 0;

	imu_data.accel_config = temp_data;

	// Configure Accelerometer - Register 28 (0x1C)
	temp_data = (1 << 3);
	ret = HAL_I2C_Mem_Write(_hi2c, MPU9250_ADDRESS, REG_CONFIG_ACCEL, 1, &temp_data, 1, 100);

	if (ret != HAL_OK)
		return 0;

	return 1;
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

	// 14-byte buffer: accel(6) + temp(2) + gyro(6)
	uint8_t data[14];
	HAL_StatusTypeDef ret;

	ret = HAL_I2C_Mem_Read(_hi2c, MPU9250_ADDRESS, REG_ACCEL_DATA, 1, data, 14, 100);

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

	// Get raw data from IMU
	x_accel = ((int16_t)data[0] << 8) + data[1];
	y_accel = ((int16_t)data[2] << 8) + data[3];
	z_accel = ((int16_t)data[4] << 8) + data[5];

	x_gyro = ((int16_t)data[8] << 8) + data[9];
	y_gyro = ((int16_t)data[10] << 8) + data[11];
	z_gyro = ((int16_t)data[12] << 8) + data[13];

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

	imu_data.accel.x = x_accel;
	imu_data.accel.y = y_accel;
	imu_data.accel.z = z_accel;

	return 1;
}

MPU9250_Data_t *mpu9250_get_data(void)
{
	return &imu_data;
}