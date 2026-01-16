/*
 * mpu9250.c
 *
 *  Created on: Jan 8, 2026
 *      Author: Juan Reyes
 * 
 * Jan 16, modifed changes:
 * Made the code a bit more modular, still need to test code to ensure it works.
 * 	- Added helper functions for I2C read/write
 * 	- Added helper function for printing integers to LCD
 *  Changes by: yug
 */
#include "mpu9250.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"

extern I2C_HandleTypeDef hi2c3;

#define MPU_I2C_ADDR   (DEVICE_ADDRESS << 1)
#define I2C_TIMEOUT_MS 100

static inline int16_t be16_to_i16(const uint8_t msb, const uint8_t lsb)
{
    return (int16_t)((msb << 8) | lsb);
}

static void lcd_status(uint16_t y, const char *ok_text, const char *err_text, HAL_StatusTypeDef st,
                       uint32_t ok_color, uint32_t err_color)
{
    BSP_LCD_SetTextColor((st == HAL_OK) ? ok_color : err_color);
    BSP_LCD_DisplayStringAt(0, y, (uint8_t *)((st == HAL_OK) ? ok_text : err_text), RIGHT_MODE);
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
}

static HAL_StatusTypeDef i2c_write_u8(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c3, MPU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef i2c_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c3, MPU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, I2C_TIMEOUT_MS);
}

static void lcd_print_int_at(uint16_t y, int16_t value)
{
    char s[16];
    snprintf(s, sizeof(s), "%d", (int)value);

    BSP_LCD_DisplayStringAt(0, y, (uint8_t*)"        ", RIGHT_MODE);
    BSP_LCD_DisplayStringAt(0, y, (uint8_t*)s, RIGHT_MODE);
}

void mpu9250_init(void)
{
    HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(&hi2c3, MPU_I2C_ADDR, 1, I2C_TIMEOUT_MS);
    lcd_status(50, "Secure", "  None", st, LCD_COLOR_GREEN, LCD_COLOR_RED);

    st = i2c_write_u8(REG_CONFIG_ACCEL, FS_ACCEL_4G);
    lcd_status(70, "   4G", "Error", st, LCD_COLOR_BLACK, LCD_COLOR_RED);

    st = i2c_write_u8(REG_CONFIG_GYRO, FS_GYRO_500);
    lcd_status(90, "500dps", " Error", st, LCD_COLOR_BLACK, LCD_COLOR_RED);

    st = i2c_write_u8(REG_POW_MAN, 0x00);
    lcd_status(110, "On", " Error", st, LCD_COLOR_GREEN, LCD_COLOR_RED);
}

void mpu9250_read(void)
{
    uint8_t buf[6];
    HAL_StatusTypeDef st;

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

	st = i2c_read(REG_ACCEL_DATA, buf, 6);
    if (st != HAL_OK) {
        lcd_status(165, "ACC OK", "ACC ERR", st, LCD_COLOR_BLACK, LCD_COLOR_RED);
        return;
    }

    ax = be16_to_i16(buf[0], buf[1]);
    ay = be16_to_i16(buf[2], buf[3]);
    az = be16_to_i16(buf[4], buf[5]);

    lcd_print_int_at(165, ax);
    lcd_print_int_at(185, ay);
    lcd_print_int_at(205, az);

    
    st = i2c_read(REG_GYRO_DATA, buf, 6);
    if (st != HAL_OK) {
        lcd_status(235, "GYR OK", "GYR ERR", st, LCD_COLOR_BLACK, LCD_COLOR_RED);
        return;
    }

    gx = be16_to_i16(buf[0], buf[1]);
    gy = be16_to_i16(buf[2], buf[3]);
    gz = be16_to_i16(buf[4], buf[5]);

    lcd_print_int_at(235, gx);
    lcd_print_int_at(255, gy);
    lcd_print_int_at(275, gz);
}