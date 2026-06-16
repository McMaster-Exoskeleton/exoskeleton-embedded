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
    uint16_t ts    = (uint16_t)(HAL_GetTick() & 0xFFFF); /* ms, wraps ~65s */

    uint8_t data[8];
    memcpy(&data[0], &raw_ax, 2);
    memcpy(&data[2], &raw_ay, 2);
    memcpy(&data[4], &raw_az, 2);
    memcpy(&data[6], &ts,     2);

    return can_send_std(id, data, 8);
}

int can_send_imu_gyro(uint8_t src_node, float gx, float gy, float gz) {
    uint16_t id = CAN_BUILD_ID(CAN_MSG_IMU_GYRO, src_node, CAN_NODE_PI);
    int16_t raw_gx = (int16_t)(gx * 100.0f);
    int16_t raw_gy = (int16_t)(gy * 100.0f);
    int16_t raw_gz = (int16_t)(gz * 100.0f);
    uint16_t ts    = (uint16_t)(HAL_GetTick() & 0xFFFF); /* ms, wraps ~65s */

    uint8_t data[8];
    memcpy(&data[0], &raw_gx, 2);
    memcpy(&data[2], &raw_gy, 2);
    memcpy(&data[4], &raw_gz, 2);
    memcpy(&data[6], &ts,     2);

    return can_send_std(id, data, 8);
}

