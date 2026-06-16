/*
 * can_imu.h
 *
 * IMU CAN messages: accelerometer and gyroscope data.
 * All values are little-endian int16 scaled by 100.
 */

#ifndef CAN_IMU_H
#define CAN_IMU_H

#include "can_common.h"

/*
 * Send IMU accelerometer data (DLC 6).
 * ax, ay, az in m/s^2. Encoded as int16 * 100.
 * Returns 1 on success, 0 on failure.
 */
int can_send_imu_accel(uint8_t src_node, float ax, float ay, float az);

/*
 * Send IMU gyroscope data (DLC 6).
 * gx, gy, gz in deg/s. Encoded as int16 * 100.
 * Returns 1 on success, 0 on failure.
 */
int can_send_imu_gyro(uint8_t src_node, float gx, float gy, float gz);

/*
 * Parse an IMU accelerometer frame.
 * Returns 1 on success, 0 if the frame is not an IMU_ACCEL message.
 */
int can_parse_imu_accel(const CanFrame *frame, float *ax, float *ay, float *az);

/*
 * Parse an IMU gyroscope frame.
 * Returns 1 on success, 0 if the frame is not an IMU_GYRO message.
 */
int can_parse_imu_gyro(const CanFrame *frame, float *gx, float *gy, float *gz);

#endif /* CAN_IMU_H */
