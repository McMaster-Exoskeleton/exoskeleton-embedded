/*
 * mpu9250.c
 *
 * MPU9250 IMU Driver for Nucleo F446RE
 * Referenced from discovery-tests/discovery-imu-test
 */

#include "mpu9250.h"
#include "main.h"
#include <stdio.h>

static I2C_HandleTypeDef *_hi2c;
static MPU9250_Data_t imu_data;

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

	imu_data.accel.x = ((int16_t)data[0] << 8) + data[1];
	imu_data.accel.y = ((int16_t)data[2] << 8) + data[3];
	imu_data.accel.z = ((int16_t)data[4] << 8) + data[5];

	imu_data.gyro.x = ((int16_t)data[8] << 8) + data[9];
	imu_data.gyro.y = ((int16_t)data[10] << 8) + data[11];
	imu_data.gyro.z = ((int16_t)data[12] << 8) + data[13];

	return 1;
}

MPU9250_Data_t* mpu9250_get_data(void)
{
	return &imu_data;
}
