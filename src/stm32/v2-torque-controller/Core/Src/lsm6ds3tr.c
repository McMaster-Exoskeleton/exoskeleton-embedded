/*
 * lsm6ds3tr.c
 *
 * LSM6DS3TR Driver for Nucleo F446RE — with DMA read support.
 *
 * DMA read flow:
 *   1. lsm6ds3tr_init_dma_read() is called from the main loop each cycle.
 *      It issues a non-blocking HAL_I2C_Mem_Read_DMA() that reads 12 bytes
 *      starting at REG_OUTX_L_G (0x22):
 *        bytes  0–5  : gyro  X,Y,Z (little-endian int16)
 *        bytes  6–11 : accel X,Y,Z (little-endian int16)
 *   2. DMA1_Stream1 fires an interrupt when the transfer is complete.
 *   3. HAL dispatches to HAL_I2C_MemRxCpltCallback() (defined here), which
 *      converts the raw bytes to physical units and pushes the result to the
 *      IMU circular buffer via imu_buffer_push().
 *
 * The blocking lsm6ds3tr_read() is kept for the calibration routine and
 * initial sensor checks, but is not used in the main loop.
 */

#include "lsm6ds3tr.h"
#include "imu_buffer.h"
#include "main.h"
#include <stdio.h>

#define GRAVITY 9.80665f

// https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf
#define LIN_ACCEL_SENSITIVITY_4G   0.000122f // g/LSB
#define ANG_VEL_SENSITIVITY_500DPS 0.0175f   // dps/LSB

// high performance mode for accel+gyro (416Hz ODR) -> Tables 52,55 of lsm6ds3tr-c.pdf
#define ODR_416HZ 0x60

// Single-pole IIR lowpass on the filt_* fields. With the IMU read at 500 Hz,
// alpha = 0.2 gives an effective cutoff around 18 Hz — suppresses high-freq
// vibration / motor switching noise while keeping <10 ms response time.
#define IMU_LPF_ALPHA              0.2f

static I2C_HandleTypeDef *_hi2c;
static LSM6DS3TR_Data_t   imu_data;

// 12-byte DMA receive buffer:  [0-5] gyro, [6-11] accel (little-endian)
static uint8_t dma_rx_buffer[12];
static uint8_t filter_initialized = 0;

// Calibration offsets (computed once in lsm6ds3tr_calibrate)
static float offset_gx = 0.0f, offset_gy = 0.0f, offset_gz = 0.0f;
static float offset_ax = 0.0f, offset_ay = 0.0f, offset_az = 0.0f;

/* --------------------------------------------------------------------------
 * Initialisation
 * ----------------------------------------------------------------------- */

void lsm6ds3tr_init_driver(I2C_HandleTypeDef *hi2c)
{
	_hi2c = hi2c;
	imu_data.state = SENSOR_STATE_LOST;
	filter_initialized = 0;

	imu_data.accel.x = 0; imu_data.accel.y = 0; imu_data.accel.z = 0;
	imu_data.gyro.x  = 0; imu_data.gyro.y  = 0; imu_data.gyro.z  = 0;
	imu_data.accel.filt_x = 0; imu_data.accel.filt_y = 0; imu_data.accel.filt_z = 0;
	imu_data.gyro.filt_x  = 0; imu_data.gyro.filt_y  = 0; imu_data.gyro.filt_z  = 0;

	imu_data.gyro_config  = 0;
	imu_data.accel_config = 0;
	imu_data.power_config = 0;
}

/* --------------------------------------------------------------------------
 * Connection check — guarded against calling while DMA is in progress
 * ----------------------------------------------------------------------- */

uint8_t lsm6ds3tr_check_connection(void)
{
	// Do not interrupt a live DMA transfer on the I2C bus
	if (HAL_I2C_GetState(_hi2c) != HAL_I2C_STATE_READY) {
		return (imu_data.state == SENSOR_STATE_CONNECTED) ? 1 : 0;
	}

	uint8_t who_am_i;
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(
		_hi2c, LSM6DS3TR_ADDRESS, REG_WHO_AM_I, 1, &who_am_i, 1, 100);

	if (ret == HAL_OK && WHO_AM_I_VALID(who_am_i))
	{
		imu_data.state = SENSOR_STATE_CONNECTED;
		return 1;
	}

	imu_data.state = SENSOR_STATE_LOST;
	return 0;
}

/* --------------------------------------------------------------------------
 * Configuration (ODR + full-scale + Block Data Update)
 * ----------------------------------------------------------------------- */

uint8_t lsm6ds3tr_configure(void)
{
	uint8_t temp_data;
	HAL_StatusTypeDef ret;

	// Accelerometer: 416 Hz ODR, +/-4g
	temp_data = ODR_416HZ | FS_ACCEL_4G;
	ret = HAL_I2C_Mem_Write(_hi2c, LSM6DS3TR_ADDRESS, REG_CTRL1_XL, 1, &temp_data, 1, 100);
	if (ret != HAL_OK) return 0;
	imu_data.accel_config = temp_data;

	// Gyroscope: 416 Hz ODR, +/-500 dps
	temp_data = ODR_416HZ | FS_GYRO_500DPS;
	ret = HAL_I2C_Mem_Write(_hi2c, LSM6DS3TR_ADDRESS, REG_CTRL2_G, 1, &temp_data, 1, 100);
	if (ret != HAL_OK) return 0;
	imu_data.gyro_config = temp_data;

	// Enable Block Data Update (BDU) + register address auto-increment (IF_INC)
	// BDU prevents reading high/low bytes from different samples during a burst.
	temp_data = BDU_ENABLE;
	ret = HAL_I2C_Mem_Write(_hi2c, LSM6DS3TR_ADDRESS, REG_CTRL3_C, 1, &temp_data, 1, 100);
	if (ret != HAL_OK) return 0;

	return 1;
}

/* --------------------------------------------------------------------------
 * Calibration — blocking, call once before the main loop
 *
 * Reads 100 samples with short delays and averages the gyroscope bias.
 * The sensor must be stationary during this call (~0.3 s).
 * Accelerometer calibration is deferred (gravity axis unknown without
 * orientation information).
 * ----------------------------------------------------------------------- */

void lsm6ds3tr_calibrate(void)
{
	// Ensure the sensor is connected and configured before reading samples
	if (!lsm6ds3tr_check_connection()) return;
	if (!lsm6ds3tr_configure())        return;

	uint8_t data[12];

	long total_off_gx = 0, total_off_gy = 0, total_off_gz = 0;

	for (int i = 0; i < 100; ++i)
	{
		HAL_I2C_Mem_Read(_hi2c, LSM6DS3TR_ADDRESS, REG_OUTX_L_G, 1, data, 12, 100);

		int16_t raw_gx = ((int16_t)data[1] << 8) + data[0];
		int16_t raw_gy = ((int16_t)data[3] << 8) + data[2];
		int16_t raw_gz = ((int16_t)data[5] << 8) + data[4];

		total_off_gx += raw_gx;
		total_off_gy += raw_gy;
		total_off_gz += raw_gz;

		HAL_Delay(3);
	}

	offset_ax = 0.0f;
	offset_ay = 0.0f;
	offset_az = 0.0f;

	offset_gx = (total_off_gx / 100) * ANG_VEL_SENSITIVITY_500DPS;
	offset_gy = (total_off_gy / 100) * ANG_VEL_SENSITIVITY_500DPS;
	offset_gz = (total_off_gz / 100) * ANG_VEL_SENSITIVITY_500DPS;
}

/* --------------------------------------------------------------------------
 * Blocking read (retained for diagnostic use; not called in the main loop)
 * ----------------------------------------------------------------------- */

uint8_t lsm6ds3tr_read(void)
{
	if (imu_data.state == SENSOR_STATE_LOST)
	{
		if (!lsm6ds3tr_check_connection()) return 0;
		if (!lsm6ds3tr_configure())       return 0;
	}

	uint8_t buffer[12];
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(
		_hi2c, LSM6DS3TR_ADDRESS, REG_OUTX_L_G, 1, buffer, 12, 100);

	if (ret != HAL_OK)
	{
		imu_data.state = SENSOR_STATE_LOST;
		return 0;
	}

	int16_t x_gyro  = ((int16_t)buffer[1] << 8) + buffer[0];
	int16_t y_gyro  = ((int16_t)buffer[3] << 8) + buffer[2];
	int16_t z_gyro  = ((int16_t)buffer[5] << 8) + buffer[4];

	int16_t x_accel = ((int16_t)buffer[7] << 8) + buffer[6];
	int16_t y_accel = ((int16_t)buffer[9] << 8) + buffer[8];
	int16_t z_accel = ((int16_t)buffer[11] << 8) + buffer[10];

	float x_accel_ms2 = (x_accel * LIN_ACCEL_SENSITIVITY_4G) * GRAVITY - offset_ax;
	float y_accel_ms2 = (y_accel * LIN_ACCEL_SENSITIVITY_4G) * GRAVITY - offset_ay;
	float z_accel_ms2 = (z_accel * LIN_ACCEL_SENSITIVITY_4G) * GRAVITY + offset_az;

	float x_gyro_dps = (x_gyro * ANG_VEL_SENSITIVITY_500DPS) - offset_gx;
	float y_gyro_dps = (y_gyro * ANG_VEL_SENSITIVITY_500DPS) - offset_gy;
	float z_gyro_dps = (z_gyro * ANG_VEL_SENSITIVITY_500DPS) - offset_gz;

	imu_data.accel.filt_x = x_accel_ms2;
	imu_data.accel.filt_y = y_accel_ms2;
	imu_data.accel.filt_z = z_accel_ms2;
	imu_data.gyro.filt_x  = x_gyro_dps;
	imu_data.gyro.filt_y  = y_gyro_dps;
	imu_data.gyro.filt_z  = z_gyro_dps;

	imu_data.accel.x = x_accel; imu_data.accel.y = y_accel; imu_data.accel.z = z_accel;
	imu_data.gyro.x  = x_gyro;  imu_data.gyro.y  = y_gyro;  imu_data.gyro.z  = z_gyro;

	return 1;
}

/* --------------------------------------------------------------------------
 * DMA read — non-blocking, call each main-loop iteration
 * ----------------------------------------------------------------------- */

uint8_t lsm6ds3tr_init_dma_read(void)
{
	if (imu_data.state == SENSOR_STATE_LOST)
	{
		if (!lsm6ds3tr_check_connection()) return 0;
		if (!lsm6ds3tr_configure())        return 0;
	}

	// Skip if the previous DMA transfer hasn't finished yet.
	// This is normal — HAL_Delay(20) in the main loop is shorter than
	// worst-case I2C transfer time under bus contention.
	if (HAL_I2C_GetState(_hi2c) != HAL_I2C_STATE_READY) {
		return 1;
	}

	// Burst-read 12 bytes starting at REG_OUTX_L_G (0x22).
	// The DMA completion callback (HAL_I2C_MemRxCpltCallback) will process
	// the result asynchronously.
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read_DMA(
		_hi2c,
		LSM6DS3TR_ADDRESS,
		REG_OUTX_L_G,
		1,
		dma_rx_buffer,
		12);

	if (ret != HAL_OK)
	{
		imu_data.state = SENSOR_STATE_LOST;
		return 0;
	}

	return 1;
}

/* --------------------------------------------------------------------------
 * DMA completion callback (called from ISR context by HAL)
 *
 * Converts the 12-byte DMA buffer to physical units, updates imu_data, and
 * pushes the new reading into the circular buffer for UART retrieval.
 * ----------------------------------------------------------------------- */
static int temp_count = 0;
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{

	if (hi2c->Instance != _hi2c->Instance) return;

	// Buffer layout: bytes [0-5] gyro, bytes [6-11] accel (little-endian)
	int16_t x_gyro  = ((int16_t)dma_rx_buffer[1] << 8) + dma_rx_buffer[0];
	int16_t y_gyro  = ((int16_t)dma_rx_buffer[3] << 8) + dma_rx_buffer[2];
	int16_t z_gyro  = ((int16_t)dma_rx_buffer[5] << 8) + dma_rx_buffer[4];
	int16_t x_accel = ((int16_t)dma_rx_buffer[7] << 8) + dma_rx_buffer[6];
	int16_t y_accel = ((int16_t)dma_rx_buffer[9] << 8) + dma_rx_buffer[8];
	int16_t z_accel = ((int16_t)dma_rx_buffer[11] << 8) + dma_rx_buffer[10];

	float ax = (x_accel * LIN_ACCEL_SENSITIVITY_4G) * GRAVITY - offset_ax;
	float ay = (y_accel * LIN_ACCEL_SENSITIVITY_4G) * GRAVITY - offset_ay;
	float az = (z_accel * LIN_ACCEL_SENSITIVITY_4G) * GRAVITY + offset_az;

	float gx = (x_gyro * ANG_VEL_SENSITIVITY_500DPS) - offset_gx;
	float gy = (y_gyro * ANG_VEL_SENSITIVITY_500DPS) - offset_gy;
	float gz = (z_gyro * ANG_VEL_SENSITIVITY_500DPS) - offset_gz;

	// Single-pole IIR low-pass filter: y[n] = alpha*x[n] + (1 - alpha)*y[n-1].
	// Seed with the first raw sample to avoid a ramp-up transient from 0.
	if (!filter_initialized)
	{
		imu_data.accel.filt_x = ax;
		imu_data.accel.filt_y = ay;
		imu_data.accel.filt_z = az;
		imu_data.gyro.filt_x  = gx;
		imu_data.gyro.filt_y  = gy;
		imu_data.gyro.filt_z  = gz;
		filter_initialized = 1;
	}
	else
	{
		imu_data.accel.filt_x = (IMU_LPF_ALPHA * ax) + ((1.0f - IMU_LPF_ALPHA) * imu_data.accel.filt_x);
		imu_data.accel.filt_y = (IMU_LPF_ALPHA * ay) + ((1.0f - IMU_LPF_ALPHA) * imu_data.accel.filt_y);
		imu_data.accel.filt_z = (IMU_LPF_ALPHA * az) + ((1.0f - IMU_LPF_ALPHA) * imu_data.accel.filt_z);
		imu_data.gyro.filt_x  = (IMU_LPF_ALPHA * gx) + ((1.0f - IMU_LPF_ALPHA) * imu_data.gyro.filt_x);
		imu_data.gyro.filt_y  = (IMU_LPF_ALPHA * gy) + ((1.0f - IMU_LPF_ALPHA) * imu_data.gyro.filt_y);
		imu_data.gyro.filt_z  = (IMU_LPF_ALPHA * gz) + ((1.0f - IMU_LPF_ALPHA) * imu_data.gyro.filt_z);
	}

	imu_data.accel.x = x_accel; imu_data.accel.y = y_accel; imu_data.accel.z = z_accel;
	imu_data.gyro.x  = x_gyro;  imu_data.gyro.y  = y_gyro;  imu_data.gyro.z  = z_gyro;

	// Push to the circular buffer with the filtered readings
	if(temp_count % 5 == 0) {
	    imu_buffer_push(HAL_GetTick(),
	                    imu_data.accel.filt_x, imu_data.accel.filt_y, imu_data.accel.filt_z,
	                    imu_data.gyro.filt_x, imu_data.gyro.filt_y, imu_data.gyro.filt_z);
	}
	temp_count++;
}

/* --------------------------------------------------------------------------
 * Accessor
 * ----------------------------------------------------------------------- */

LSM6DS3TR_Data_t* lsm6ds3tr_get_data(void)
{
	return &imu_data;
}
