/*
 * can_motor.c — TORQUE_CMD and MOTOR_STATUS encode/decode (little-endian).
 */

#include "can_motor.h"
#include <string.h>

int can_send_torque_cmd(uint8_t dest_node, float torque_nm) {
    uint8_t src = can_get_my_node_id();
    uint16_t id = CAN_BUILD_ID(CAN_MSG_TORQUE_CMD, src, dest_node);
    int16_t raw = (int16_t)(torque_nm * 1000.0f);

    uint8_t data[2];
    memcpy(&data[0], &raw, 2);

    return can_send_std(id, data, 2);
}

int can_parse_torque_cmd(const CanFrame *frame, float *torque_nm) {
    if (frame->is_extended) return 0;
    if (can_get_msg_type((uint16_t)frame->id) != CAN_MSG_TORQUE_CMD) return 0;
    if (frame->dlc < 2) return 0;

    int16_t raw;
    memcpy(&raw, &frame->data[0], 2);
    *torque_nm = raw / 1000.0f;

    return 1;
}

int can_send_motor_status(uint8_t src_node, float position, float speed,
                          float current, int8_t temperature, uint8_t error) {
    uint16_t id = CAN_BUILD_ID(CAN_MSG_MOTOR_STATUS, src_node, CAN_NODE_PI);

    int16_t raw_pos = (int16_t)(position * 10.0f);
    int16_t raw_spd = (int16_t)(speed / 10.0f);
    int16_t raw_cur = (int16_t)(current * 100.0f);

    uint8_t data[8];
    memcpy(&data[0], &raw_pos, 2);
    memcpy(&data[2], &raw_spd, 2);
    memcpy(&data[4], &raw_cur, 2);
    data[6] = (uint8_t)temperature;
    data[7] = error;

    return can_send_std(id, data, 8);
}

int can_parse_motor_status(const CanFrame *frame, float *position, float *speed,
                           float *current, int8_t *temperature, uint8_t *error) {
    if (frame->is_extended) return 0;
    if (can_get_msg_type((uint16_t)frame->id) != CAN_MSG_MOTOR_STATUS) return 0;
    if (frame->dlc < 8) return 0;

    int16_t raw;
    memcpy(&raw, &frame->data[0], 2); *position = raw / 10.0f;
    memcpy(&raw, &frame->data[2], 2); *speed = raw * 10.0f;
    memcpy(&raw, &frame->data[4], 2); *current = raw / 100.0f;
    *temperature = (int8_t)frame->data[6];
    *error = frame->data[7];

    return 1;
}
