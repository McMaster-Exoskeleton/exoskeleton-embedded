/*
 * mpu9250.c
 *
 *  Created on: Jan 8, 2026
 *      Author: Juan Reyes
 */

#include <mpu9250.h>
#include <main.h>
#include <stdio.h>

#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"

// https://invensense.tdk.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf
#define MPU9250_ADDRESS (DEVICE_ADDRESS << 1)
#define PWR_MGMT_1 0x6B
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C

extern I2C_HandleTypeDef hi2c3;

typedef enum
{
	SENSOR_STATE_CONNECTED,
	SENSOR_STATE_LOST
} SensorState_t;

static SensorState_t currentState = SENSOR_STATE_LOST;

static void LCD_Print(uint16_t y_pos, char *label, uint16_t color, uint8_t status)
{
	if (status == HAL_OK)
	{
		BSP_LCD_SetTextColor(color);
		BSP_LCD_DisplayStringAt(0, y_pos, (uint8_t *)label, RIGHT_MODE);
	}
	else
	{
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_DisplayStringAt(0, y_pos, (uint8_t *)"Error ", RIGHT_MODE);
	}
}

static uint8_t mpu9250_init(void)
{
	uint8_t temp_data;
	HAL_StatusTypeDef ret;

	// Configure Accelerometer
	temp_data = FS_ACCEL_4G;
	ret = HAL_I2C_Mem_Write(&hi2c3, MPU9250_ADDRESS, REG_CONFIG_ACCEL, 1, &temp_data, 1, 100);

	LCD_Print(70, "   4G", ret, LCD_COLOR_BLACK);
	if (ret != HAL_OK)
		return 0;

	// Configure Gyroscope
	temp_data = FS_GYRO_500;
	ret = HAL_I2C_Mem_Write(&hi2c3, MPU9250_ADDRESS, REG_CONFIG_GYRO, 1, &temp_data, 1, 100);

	LCD_Print(90, "500dps", ret, LCD_COLOR_BLACK);
	if (ret != HAL_OK)
		return 0;

	// Configure Power
	temp_data = 0x00;
	ret = HAL_I2C_Mem_Write(&hi2c3, MPU9250_ADDRESS, REG_POW_MAN, 1, &temp_data, 1, 100);

	LCD_Print(110, "On", ret, LCD_COLOR_GREEN);

	return (ret == HAL_OK);
}

void mpu9250_read()
{
	if (currentState == SENSOR_STATE_LOST)
	{
		HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c3, MPU9250_ADDRESS, 1, 100);

		if (ret == HAL_OK && mpu9250_init())
		{
			currentState = SENSOR_STATE_CONNECTED;

			LCD_Print(50, (uint8_t *)"Secure", LCD_COLOR_GREEN);
		}
	}
	else
	{
		// 14-byte buffer
		uint8_t data[14];
		HAL_StatusTypeDef ret;
		char text[20];

		ret = HAL_I2C_Mem_Read(&hi2c3, MPU9250_ADDRESS, REG_ACCEL_DATA, 1, data, 14, 100);

		if (ret != HAL_OK)
		{
			currentState = SENSOR_STATE_LOST;

			return;
		}

		int16_t x_accel;
		int16_t y_accel;
		int16_t z_accel;

		int16_t x_gyro;
		int16_t y_gyro;
		int16_t z_gyro;

		x_accel = ((int16_t)data[0] << 8) + data[1];
		y_accel = ((int16_t)data[2] << 8) + data[3];
		z_accel = ((int16_t)data[4] << 8) + data[5];

		x_gyro = ((int16_t)data[8] << 8) + data[9];
		y_gyro = ((int16_t)data[10] << 8) + data[11];
		z_gyro = ((int16_t)data[12] << 8) + data[13];

		// Print Acceleration Measurements
		sprintf(text, "%6d", x_accel);
		BSP_LCD_DisplayStringAt(0, 165, (uint8_t *)text, RIGHT_MODE);
		sprintf(text, "%6d", y_accel);
		BSP_LCD_DisplayStringAt(0, 185, (uint8_t *)text, RIGHT_MODE);
		sprintf(text, "%6d", z_accel);
		BSP_LCD_DisplayStringAt(0, 205, (uint8_t *)text, RIGHT_MODE);

		// Print Gyroscope Measurements
		sprintf(text, "%6d", x_gyro);
		BSP_LCD_DisplayStringAt(0, 235, (uint8_t *)text, RIGHT_MODE);
		sprintf(text, "%6d", y_gyro);
		BSP_LCD_DisplayStringAt(0, 255, (uint8_t *)text, RIGHT_MODE);
		sprintf(text, "%6d", z_gyro);
		BSP_LCD_DisplayStringAt(0, 275, (uint8_t *)text, RIGHT_MODE);
	}
}
