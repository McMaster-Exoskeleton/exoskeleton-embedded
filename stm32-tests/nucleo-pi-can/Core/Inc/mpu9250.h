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

// MPU9250 Default I2C address is 0x68 (when AD0 pin is LOW)
// If AD0 is HIGH, the address is 0x69.
#define DEVICE_ADDRESS 0x68
#define MPU9250_ADDRESS (DEVICE_ADDRESS << 1)

// Scale selections (for configuration)
// Gyroscope FS_SEL = 1 (500 dps) -> 0x08
#define FS_GYRO_500DPS 0x08
// Accelerometer AFS_SEL = 1 (4g) -> 0x08
#define FS_ACCEL_4G 0x08

// Check connection (mpu9250_check_connection)
#define REG_WHO_AM_I 0x75
#define WHO_AM_I_VAL 0x71 // Expected WHO_AM_I response for MPU9250

// Control registers
#define REG_PWR_MGMT_1   0x6B // Power management register (wake up)
#define REG_ACCEL_CONFIG 0x1C // Accelerometer configuration
#define REG_GYRO_CONFIG  0x1B // Gyroscope configuration

// Output registers (Start of contiguous data block)
// Burst reading 14 bytes from here gets Accel -> Temp -> Gyro
#define REG_ACCEL_XOUT_H 0x3B

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
} MPU9250_Data_t;

void mpu9250_init_driver(I2C_HandleTypeDef *hi2c);
uint8_t mpu9250_check_connection(void);
uint8_t mpu9250_configure(void);
void mpu9250_calibrate(void);
uint8_t mpu9250_read(void);
uint8_t mpu9250_init_dma_read(void);
MPU9250_Data_t *mpu9250_get_data(void);

#endif /* INC_MPU9250_H_ */
