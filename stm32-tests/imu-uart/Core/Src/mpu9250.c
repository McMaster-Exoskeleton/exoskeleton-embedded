/*
 * mpu9250.c
 *
 * MPU9250 IMU Driver for Nucleo F446RE
 * Referenced from discovery-tests/discovery-imu-test
 */

#include "mpu9250.h"
#include "main.h"
#include <stdio.h>

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

uint8_t mpu9250_configure(void)
{
	uint8_t temp_data;
	HAL_StatusTypeDef ret;

	// Configure Accelerometer (4G, matching discovery implementation)
	temp_data = FS_ACCEL_4G;
	ret = HAL_I2C_Mem_Write(_hi2c, MPU9250_ADDRESS, REG_CONFIG_ACCEL, 1, &temp_data, 1, 100);
	if (ret != HAL_OK)
		return 0;
	imu_data.accel_config = temp_data;

	// Configure Gyroscope (500dps, matching discovery implementation)
	temp_data = FS_GYRO_500;
	ret = HAL_I2C_Mem_Write(_hi2c, MPU9250_ADDRESS, REG_CONFIG_GYRO, 1, &temp_data, 1, 100);
	if (ret != HAL_OK)
		return 0;
	imu_data.gyro_config = temp_data;

	// Configure Power (wake up sensor)
	temp_data = 0x00;
	ret = HAL_I2C_Mem_Write(_hi2c, MPU9250_ADDRESS, REG_POW_MAN, 1, &temp_data, 1, 100);
	if (ret != HAL_OK)
		return 0;
	imu_data.power_config = temp_data;

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

	// Calibrate IMU by calculating offset
	if (!calibrated)
	{

		float total_off_gx = 0, total_off_gy = 0, total_off_gz  = 0;
		float total_off_ax = 0, total_off_ay = 0, total_off_az = 0;

		int sample_num = 100;

		for(int i = 0; i<sample_num; ++i)
		{
			HAL_I2C_Mem_Read(_hi2c, MPU9250_ADDRESS, REG_ACCEL_DATA, 1, data, 14, 100);

			int16_t curr_off_ax = ((int16_t)data[0] << 8) + data[1];
			int16_t curr_off_ay = ((int16_t)data[2] << 8) + data[3];
			int16_t curr_off_az = ((int16_t)data[4] << 8) + data[5];

			int16_t curr_off_gx = ((int16_t)data[8] << 8) + data[9];
			int16_t curr_off_gy = ((int16_t)data[10] << 8) + data[11];
			int16_t curr_off_gz = ((int16_t)data[12] << 8) + data[13];


			total_off_ax += (LIN_ACCEL_SENSITIVITY_4G * curr_off_ax) * GRAVITY;
			total_off_ay += (LIN_ACCEL_SENSITIVITY_4G * curr_off_ay) * GRAVITY;
			total_off_az += (LIN_ACCEL_SENSITIVITY_4G * curr_off_az) * GRAVITY;

			total_off_gx += (ANG_VEL_SENSITIVITY_500DPS * curr_off_gx);
			total_off_gy += (ANG_VEL_SENSITIVITY_500DPS * curr_off_gy);
			total_off_gz += (ANG_VEL_SENSITIVITY_500DPS * curr_off_gz);

			HAL_Delay(3);
		}

		offset_ax = total_off_ax / sample_num;
		offset_ay = total_off_ay / sample_num;
		offset_az = GRAVITY - (total_off_az / sample_num);

		offset_gx = total_off_gx / sample_num;
		offset_gy = total_off_gy / sample_num;
		offset_gz = total_off_gz / sample_num;

		calibrated = 1;
	}

	// Unit Conversions + Offset Application
	float x_accel_ms2 = (x_accel * LIN_ACCEL_SENSITIVITY_4G) * GRAVITY - offset_ax;
	float y_accel_ms2 = (y_accel * LIN_ACCEL_SENSITIVITY_4G) * GRAVITY - offset_ay;
	float z_accel_ms2 = (z_accel * LIN_ACCEL_SENSITIVITY_4G) * GRAVITY + offset_az;

	float x_gyro_dps = (x_gyro * ANG_VEL_SENSITIVITY_500DPS) - offset_gx;
	float y_gyro_dps = (y_gyro * ANG_VEL_SENSITIVITY_500DPS) - offset_gy;
	float z_gyro_dps = (z_gyro * ANG_VEL_SENSITIVITY_500DPS) - offset_gz;

	// Low Pass Filter (y[n] = a*x[n] + (1-a)*y[n-1])
	float alpha = 0.5f; // lower alpha = more smoothing (0.0-1.0)

	// Store filter in Struct
	imu_data.accel.filt_x = (alpha * x_accel_ms2) + (1.0f - alpha) * imu_data.accel.filt_x;
	imu_data.accel.filt_y = (alpha * y_accel_ms2) + (1.0f - alpha) * imu_data.accel.filt_y;
	imu_data.accel.filt_z = (alpha * z_accel_ms2) + (1.0f - alpha) * imu_data.accel.filt_z;

	imu_data.gyro.filt_x = (alpha * x_gyro_dps) + (1.0f - alpha) * imu_data.gyro.filt_x;
	imu_data.gyro.filt_y = (alpha * y_gyro_dps) + (1.0f - alpha) * imu_data.gyro.filt_y;
	imu_data.gyro.filt_z = (alpha * z_gyro_dps) + (1.0f - alpha) * imu_data.gyro.filt_z;

	// Store raw data in Struct
	imu_data.accel.x = x_accel; imu_data.accel.y = y_accel; imu_data.accel.z = z_accel;
	imu_data.gyro.x = x_gyro; imu_data.gyro.y = y_gyro; imu_data.gyro.z = z_gyro;

	return 1;
}

MPU9250_Data_t* mpu9250_get_data(void)
{
	return &imu_data;
}