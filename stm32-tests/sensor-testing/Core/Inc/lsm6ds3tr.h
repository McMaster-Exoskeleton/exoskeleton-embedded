/*
 * lsm6ds3tr.h
 *
 * LSM6DS3TR-C Driver for Nucleo F446RE
 */

#ifndef INC_LSM6DS3TR_H_
#define INC_LSM6DS3TR_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define DEVICE_ADDRESS 0b1101011 // SA0 pin HIGH
#define LSM6DS3TR_ADDRESS (DEVICE_ADDRESS << 1)

// Scale selections (for configuration)
#define FS_GYRO_500DPS 0b00000100 // 0x04 -> Table 54. of datasheet
#define FS_ACCEL_4G 0b00001000	  // 0x08 -> Table 51. of datasheet

// Check connection (lsm6ds3tr_check_connection)
#define REG_WHO_AM_I 0x0F
#define WHO_AM_I_VAL 0x6A // expected WHO_AM_I response for LSM6DS3TR-C (verify)

// Control registers
#define REG_CTRL1_XL 0x10 // Accelerometer control register
#define REG_CTRL2_G 0x11  // Gyroscope control register
#define REG_CTRL3_C 0x12  // BDU, Reset

// #define REG_POW_MAN			107 (come back to this after rewriting .c)

// Output registers
#define REG_OUTX_L_G 0x22  // gyro x-axis LB (starts at 0x22 and goes to 0x27 for gyro)
#define REG_OUTX_L_XL 0x28 // accel x-axis LB (starts at 0x28 and goes to 0x2D for accel)
#define BDU_ENABLE 0x44	   // Block Data Update = 1, IF_INC = 1

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

	float filt_x;
	float filt_y;
	float filt_z;
} AxisData_t;

typedef struct
{
	SensorState_t state;
	AxisData_t accel;
	AxisData_t gyro;
	uint8_t gyro_config;
	uint8_t accel_config;
	uint8_t power_config;
} LSM6DS3TR_Data_t;

void lsm6ds3tr_init_driver(I2C_HandleTypeDef *hi2c);
uint8_t lsm6ds3tr_check_connection(void);
uint8_t lsm6ds3tr_configure(void);
void lsm6ds3tr_calibrate(void);
uint8_t lsm6ds3tr_read(void);
LSM6DS3TR_Data_t *lsm6ds3tr_get_data(void);

#endif /* INC_LSM6DS3TR_H_ */
