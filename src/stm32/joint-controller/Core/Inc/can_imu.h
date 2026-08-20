/*
 * can_imu.h
 *
 * IMU CAN messages: accelerometer and gyroscope+position data.
 *
 * Wire format:
 *   IMU_ACCEL (DLC 6): ax, ay, az  -> int16 * 100  (m/s^2)
 *   IMU_GYRO  (DLC 8): gx, gy, gz  -> int16 * 100  (deg/s)
 *                      motor_pos   -> int16 *  10  (degrees, +/-3276.7)
 *
 * The motor position is co-located in the gyro frame (bytes 6-7) so the
 * IMU sample and the joint angle share the same timestamp on the wire
 * without adding a third frame per IMU tick.
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
 * Send IMU gyroscope data + motor position (DLC 8).
 * gx, gy, gz in deg/s, encoded as int16 * 100.
 * motor_position in degrees, encoded as int16 * 10 (use 0 if no motor).
 * Returns 1 on success, 0 on failure.
 */
int can_send_imu_gyro(uint8_t src_node, float gx, float gy, float gz,
                      float motor_position);

/*
 * Parse an IMU accelerometer frame.
 * Returns 1 on success, 0 if the frame is not an IMU_ACCEL message.
 */
int can_parse_imu_accel(const CanFrame *frame, float *ax, float *ay, float *az);

/*
 * Parse an IMU gyroscope frame, optionally extracting motor position.
 * If motor_position is non-NULL: set to bytes 6-7 / 10 if DLC >= 8, else 0.
 * Returns 1 on success, 0 if the frame is not an IMU_GYRO message.
 */
int can_parse_imu_gyro(const CanFrame *frame, float *gx, float *gy, float *gz,
                       float *motor_position);

#endif /* CAN_IMU_H */
