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

extern I2C_HandleTypeDef hi2c3;

void mpu9250_init()
{
	// Check Connection
	HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c3, (DEVICE_ADDRESS <<1) + 0, 1, 100);
	if (ret == HAL_OK)
	{
		BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
		BSP_LCD_DisplayStringAt(0, 50, (uint8_t*)"Secure", RIGHT_MODE);
	}
	else
	{
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_DisplayStringAt(0, 50, (uint8_t*)"  None", RIGHT_MODE);
	}

	// Configure Accelerometer
	uint8_t temp_data = FS_ACCEL_4G;
	ret = HAL_I2C_Mem_Write(&hi2c3, (DEVICE_ADDRESS <<1) + 0, REG_CONFIG_ACCEL, 1, &temp_data, 1, 100);
	if (ret == HAL_OK)
	{
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_DisplayStringAt(0, 70, (uint8_t*)"   4G", RIGHT_MODE);
	}
	else
	{
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_DisplayStringAt(0, 70, (uint8_t*)"Error", RIGHT_MODE);
	}

	// Configure Gyroscope
	temp_data = FS_GYRO_500;
	ret = HAL_I2C_Mem_Write(&hi2c3, (DEVICE_ADDRESS <<1) + 0, REG_CONFIG_GYRO, 1, &temp_data, 1, 100);
	if (ret == HAL_OK)
	{
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_DisplayStringAt(0, 90, (uint8_t*)"500dps", RIGHT_MODE);
	}
	else
	{
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_DisplayStringAt(0, 90, (uint8_t*)" Error", RIGHT_MODE);
	}

	// Configure Power
	temp_data = 0;
	ret = HAL_I2C_Mem_Write(&hi2c3, (DEVICE_ADDRESS <<1) + 0, REG_POW_MAN, 1, &temp_data, 1, 100);
	if (ret == HAL_OK)
	{
		BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
		BSP_LCD_DisplayStringAt(0, 110, (uint8_t*)"On", RIGHT_MODE);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	}
	else
	{
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_DisplayStringAt(0, 110, (uint8_t*)" Error", RIGHT_MODE);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	}

}

void mpu9250_read()
{
	// READ ACCEL
	uint8_t data[6];
	char xText[20];
	char yText[20];
	char zText[20];

	int16_t x_accel;
	int16_t y_accel;
	int16_t z_accel;

	int16_t x_gyro;
	int16_t y_gyro;
	int16_t z_gyro;

	// Get Accel Data
	HAL_I2C_Mem_Read(&hi2c3, (DEVICE_ADDRESS <<1) + 1, REG_ACCEL_DATA, 1, data, 6, 100);
	x_accel = ((int16_t)data[0] << 8) + data[1];
	y_accel = ((int16_t)data[2] << 8) + data[3];
	z_accel = ((int16_t)data[4] << 8) + data[5];

	// Get Gyro Data
	HAL_I2C_Mem_Read(&hi2c3, (DEVICE_ADDRESS <<1) + 1, REG_GYRO_DATA, 1, data, 6, 100);
	x_gyro = ((int16_t)data[0] << 8) + data[1];
	y_gyro = ((int16_t)data[2] << 8) + data[3];
	z_gyro = ((int16_t)data[4] << 8) + data[5];

	sprintf(xText, "%d", x_accel);
	sprintf(yText, "%d", y_accel);
	sprintf(zText, "%d", z_accel);

	BSP_LCD_DisplayStringAt(0, 165, (uint8_t*)"        ", RIGHT_MODE);
	BSP_LCD_DisplayStringAt(0, 185, (uint8_t*)"        ", RIGHT_MODE);
	BSP_LCD_DisplayStringAt(0, 205, (uint8_t*)"        ", RIGHT_MODE);

	BSP_LCD_DisplayStringAt(0, 165, (uint8_t*)xText, RIGHT_MODE);
	BSP_LCD_DisplayStringAt(0, 185, (uint8_t*)yText, RIGHT_MODE);
	BSP_LCD_DisplayStringAt(0, 205, (uint8_t*)zText, RIGHT_MODE);

	sprintf(xText, "%d", x_gyro);
	sprintf(yText, "%d", y_gyro);
	sprintf(zText, "%d", z_gyro);

	BSP_LCD_DisplayStringAt(0, 235, (uint8_t*)"        ", RIGHT_MODE);
	BSP_LCD_DisplayStringAt(0, 255, (uint8_t*)"        ", RIGHT_MODE);
	BSP_LCD_DisplayStringAt(0, 275, (uint8_t*)"        ", RIGHT_MODE);

	BSP_LCD_DisplayStringAt(0, 235, (uint8_t*)xText, RIGHT_MODE);
	BSP_LCD_DisplayStringAt(0, 255, (uint8_t*)yText, RIGHT_MODE);
	BSP_LCD_DisplayStringAt(0, 275, (uint8_t*)zText, RIGHT_MODE);
}
