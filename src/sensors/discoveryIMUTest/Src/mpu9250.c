/*
 * mpu9250.c
 *
 *  Created on: Jan 8, 2026
 *      Author: Juan Reyes
 * 
 * Modified on: Jan 23, 2026
 * 		By: Majock Bim, Hao Yan
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

#define GRAVITY 9.80665f

// https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf
#define LIN_ACCEL_SENSITIVITY_4G 0.000122f // g/LSB
#define ANG_VEL_SENSITIVITY_500DPS 0.0175f // dps/LSB

extern I2C_HandleTypeDef hi2c3;

// Variable definitions for offset calculation
static uint16_t calibrated = 0;
static float offset_gx = 0, offset_gy = 0, offset_gz = 0;
static float offset_ax = 0, offset_ay, offset_az = 0;

typedef enum
{
	SENSOR_STATE_CONNECTED,
	SENSOR_STATE_LOST
} SensorState_t;

static SensorState_t currentState = SENSOR_STATE_LOST;

static void LCD_Print(uint16_t y_pos, char *label, uint8_t status, uint32_t color)
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

			LCD_Print(50, "Secure", HAL_OK, LCD_COLOR_GREEN);
		}
		else
		{
			LCD_Print(50, "Check", HAL_ERROR, LCD_COLOR_RED);

			HAL_Delay(50);
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

		// Calibrate IMU by calculating offset
		if (!calibrated)
		{

			float total_off_gx = 0, total_off_gy = 0, total_off_gz  = 0;
			float total_off_ax = 0, total_off_ay = 0, total_off_az = 0;

			int sample_num = 100;

			for(int i = 0; i<sample_num; ++i)
			{
				HAL_I2C_Mem_Read(&hi2c3, MPU9250_ADDRESS, REG_ACCEL_DATA, 1, data, 14, 100);

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

		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

		// Unit Conversions + Offset Application
		float x_accel_ms2 = (x_accel * LIN_ACCEL_SENSITIVITY_4G) * GRAVITY - offset_ax;
		float y_accel_ms2 = (y_accel * LIN_ACCEL_SENSITIVITY_4G) * GRAVITY - offset_ay;
		float z_accel_ms2 = (z_accel * LIN_ACCEL_SENSITIVITY_4G) * GRAVITY + offset_az;

		float x_gyro_dps = (x_gyro * ANG_VEL_SENSITIVITY_500DPS) - offset_gx;
		float y_gyro_dps = (y_gyro * ANG_VEL_SENSITIVITY_500DPS) - offset_gy;
		float z_gyro_dps = (z_gyro * ANG_VEL_SENSITIVITY_500DPS) - offset_gz;

		// Low Pass Filter (y[n] = a*x[n] + (1-a)*y[n-1])
		float alpha = 0.1f; // lower alpha = more smoothing (0.0-1.0)
		static float filt_ax = 0, filt_ay = 0, filt_az = 0;
		static float filt_gx = 0, filt_gy = 0, filt_gz = 0;

		filt_ax = (alpha * x_accel_ms2) + (1.0f - alpha) * filt_ax;
		filt_ay = (alpha * y_accel_ms2) + (1.0f - alpha) * filt_ay;
		filt_az = (alpha * z_accel_ms2) + (1.0f - alpha) * filt_az;

		filt_gx = (alpha * x_gyro_dps) + (1.0f - alpha) * filt_gx;
		filt_gy = (alpha * y_gyro_dps) + (1.0f - alpha) * filt_gy;
		filt_gz = (alpha * z_gyro_dps) + (1.0f - alpha) * filt_gz;

		// Print Acceleration Measurements (m/s^2)
		sprintf(text, "%6.2f", filt_ax);
		BSP_LCD_DisplayStringAt(0, 165, (uint8_t *)text, RIGHT_MODE);
		sprintf(text, "%6.2f", filt_ay);
		BSP_LCD_DisplayStringAt(0, 185, (uint8_t *)text, RIGHT_MODE);
		sprintf(text, "%6.2f", filt_az);
		BSP_LCD_DisplayStringAt(0, 205, (uint8_t *)text, RIGHT_MODE);

		// Print Gyroscope Measurements (rad/s)
		sprintf(text, "%6.2f", filt_gx);
		BSP_LCD_DisplayStringAt(0, 235, (uint8_t *)text, RIGHT_MODE);
		sprintf(text, "%6.2f", filt_gy);
		BSP_LCD_DisplayStringAt(0, 255, (uint8_t *)text, RIGHT_MODE);
		sprintf(text, "%6.2f", filt_gz);
		BSP_LCD_DisplayStringAt(0, 275, (uint8_t *)text, RIGHT_MODE);
	}
}