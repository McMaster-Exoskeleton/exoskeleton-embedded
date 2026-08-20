/*
 * can_imu.c
 *
 * IMU message encoding/decoding.
 * Little-endian int16, scaled by 100.
 */

#include "can_imu.h"
#include <string.h>

int can_send_imu_accel(uint8_t src_node, float ax, float ay, float az) {
    uint16_t id = CAN_BUILD_ID(CAN_MSG_IMU_ACCEL, src_node, CAN_NODE_PI);
    int16_t raw_ax = (int16_t)(ax * 100.0f);
    int16_t raw_ay = (int16_t)(ay * 100.0f);
    int16_t raw_az = (int16_t)(az * 100.0f);

    uint8_t data[6];
    memcpy(&data[0], &raw_ax, 2);
    memcpy(&data[2], &raw_ay, 2);
    memcpy(&data[4], &raw_az, 2);

    return can_send_std(id, data, 6);
}

int can_send_imu_gyro(uint8_t src_node, float gx, float gy, float gz,
                      float motor_position) {
    uint16_t id = CAN_BUILD_ID(CAN_MSG_IMU_GYRO, src_node, CAN_NODE_PI);
    int16_t raw_gx  = (int16_t)(gx * 100.0f);
    int16_t raw_gy  = (int16_t)(gy * 100.0f);
    int16_t raw_gz  = (int16_t)(gz * 100.0f);
    int16_t raw_pos = (int16_t)(motor_position * 10.0f);

    uint8_t data[8];
    memcpy(&data[0], &raw_gx,  2);
    memcpy(&data[2], &raw_gy,  2);
    memcpy(&data[4], &raw_gz,  2);
    memcpy(&data[6], &raw_pos, 2);

    return can_send_std(id, data, 8);
}

int can_parse_imu_accel(const CanFrame *frame, float *ax, float *ay, float *az) {
    if (frame->is_extended) return 0;
    if (can_get_msg_type((uint16_t)frame->id) != CAN_MSG_IMU_ACCEL) return 0;
    if (frame->dlc < 6) return 0;

    int16_t raw;
    memcpy(&raw, &frame->data[0], 2); *ax = raw / 100.0f;
    memcpy(&raw, &frame->data[2], 2); *ay = raw / 100.0f;
    memcpy(&raw, &frame->data[4], 2); *az = raw / 100.0f;

    return 1;
}

int can_parse_imu_gyro(const CanFrame *frame, float *gx, float *gy, float *gz,
                       float *motor_position) {
    if (frame->is_extended) return 0;
    if (can_get_msg_type((uint16_t)frame->id) != CAN_MSG_IMU_GYRO) return 0;
    if (frame->dlc < 6) return 0;

    int16_t raw;
    memcpy(&raw, &frame->data[0], 2); *gx = raw / 100.0f;
    memcpy(&raw, &frame->data[2], 2); *gy = raw / 100.0f;
    memcpy(&raw, &frame->data[4], 2); *gz = raw / 100.0f;

    if (motor_position != NULL) {
        if (frame->dlc >= 8) {
            memcpy(&raw, &frame->data[6], 2); *motor_position = raw / 10.0f;
        } else {
            *motor_position = 0.0f;
        }
    }

    return 1;
}
