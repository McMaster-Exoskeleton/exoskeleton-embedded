/*
 * mpu9250.h
 *
 *  Created on: Jan 8, 2026
 *      Author: Juan Reyes
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

#define DEVICE_ADDRESS		0b1101000

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
//#define REG_DATA

void mpu9250_read(void);

#endif /* INC_MPU9250_H_ */
