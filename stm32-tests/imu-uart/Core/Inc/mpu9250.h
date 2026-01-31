/*
 * mpu9250.h
 *
 * MPU9250 IMU Driver for Nucleo F446RE
 * Referenced from discovery-tests/discovery-imu-test
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define DEVICE_ADDRESS		0b1101000
#define MPU9250_ADDRESS		(DEVICE_ADDRESS << 1)

#define FS_GYRO_250			0b00000000
#define FS_GYRO_500			0b00001000
#define FS_GYRO_1000		0b00010000
#define FS_GYRO_2000		0b00011000

#define FS_ACCEL_2G			0b00000000
#define FS_ACCEL_4G			0b00001000
#define FS_ACCEL_8G			0b00010000
#define FS_ACCEL_16G		0b00011000

#define REG_CONFIG_GYRO		27
#define REG_CONFIG_ACCEL	28
#define REG_POW_MAN			107

#define REG_ACCEL_DATA		59
#define REG_GYRO_DATA		67

typedef enum
{
	SENSOR_STATE_CONNECTED,
	SENSOR_STATE_LOST
} SensorState_t;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
} AxisData_t;

typedef struct
{
	SensorState_t state;
	AxisData_t accel;
	AxisData_t gyro;
	uint8_t gyro_config;
	uint8_t accel_config;
	uint8_t power_config;
} MPU9250_Data_t;

void mpu9250_init_driver(I2C_HandleTypeDef *hi2c);
uint8_t mpu9250_check_connection(void);
uint8_t mpu9250_configure(void);
uint8_t mpu9250_read(void);
MPU9250_Data_t* mpu9250_get_data(void);

#endif /* INC_MPU9250_H_ */
