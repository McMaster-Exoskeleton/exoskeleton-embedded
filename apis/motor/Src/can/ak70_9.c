/*
 * ak70_9.c
 *
 *  Created on: Feb 4, 2026
 *      Author: Juan Reyes
 *
 *  CubeMars AK70-9 KV60 Motor CAN API implementation.
 *  Uses the can_bus driver for all CAN transmissions via STM32 HAL.
 */

#include "can/ak70_9.h"
#include "can/can_bus.h"

/* =========================================================================
 * Internal helper: clamp a float to [min_val, max_val]
 * ========================================================================= */
static float clampf(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

/* =========================================================================
 * Internal helper: transmit an extended-ID CAN frame
 * Builds the extended CAN ID from control mode and driver ID, then sends.
 * ========================================================================= */
static void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
    if (len > 8) len = 8;
    can_bus_send_ext(id, data, len);
}

/* =========================================================================
 * Byte packing utilities
 * ========================================================================= */

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
    buffer[(*index)++] = (uint8_t)(number >> 24);
    buffer[(*index)++] = (uint8_t)(number >> 16);
    buffer[(*index)++] = (uint8_t)(number >> 8);
    buffer[(*index)++] = (uint8_t)(number);
}

void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index) {
    buffer[(*index)++] = (uint8_t)(number >> 8);
    buffer[(*index)++] = (uint8_t)(number);
}

/* =========================================================================
 * Servo Mode Control Functions
 *
 * All servo mode functions encode the control mode ID in bits [28:8] and
 * the motor driver ID in bits [7:0] of the extended CAN identifier.
 * ========================================================================= */

void comm_can_set_duty(uint8_t controller_id, float duty) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(duty * 100000.0f), &send_index);
    comm_can_transmit_eid(
        controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8),
        buffer, (uint8_t)send_index);
}

void comm_can_set_current(uint8_t controller_id, float current) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &send_index);
    comm_can_transmit_eid(
        controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8),
        buffer, (uint8_t)send_index);
}

void comm_can_set_cb(uint8_t controller_id, float current) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &send_index);
    comm_can_transmit_eid(
        controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8),
        buffer, (uint8_t)send_index);
}

void comm_can_set_rpm(uint8_t controller_id, float rpm) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)rpm, &send_index);
    comm_can_transmit_eid(
        controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8),
        buffer, (uint8_t)send_index);
}

void comm_can_set_pos(uint8_t controller_id, float pos) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(pos * 10000.0f), &send_index);
    comm_can_transmit_eid(
        controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8),
        buffer, (uint8_t)send_index);
}

void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode) {
    uint8_t buffer[1];
    buffer[0] = set_origin_mode;
    comm_can_transmit_eid(
        controller_id | ((uint32_t)CAN_PACKET_SET_ORIGIN_HERE << 8),
        buffer, 1);
}

void comm_can_set_pos_spd(uint8_t controller_id, float pos, int16_t spd, int16_t rpa) {
    int32_t send_index = 0;
    uint8_t buffer[8];
    buffer_append_int32(buffer, (int32_t)(pos * 10000.0f), &send_index);
    buffer_append_int16(buffer, (int16_t)(spd / 10), &send_index);
    buffer_append_int16(buffer, (int16_t)(rpa / 10), &send_index);
    comm_can_transmit_eid(
        controller_id | ((uint32_t)CAN_PACKET_SET_POS_SPD << 8),
        buffer, (uint8_t)send_index);
}

/* =========================================================================
 * MIT Force Control Mode
 *
 * Bit packing layout (8 bytes, 64 bits total):
 *   Kp      (12 bits) | Kd      (12 bits) |
 *   Position (16 bits) | Velocity (12 bits) | Torque (12 bits)
 * ========================================================================= */

int float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
    float span = x_max - x_min;
    x = clampf(x, x_min, x_max);
    return (int)((x - x_min) * ((float)((1 << bits) - 1) / span));
}

void pack_cmd(uint8_t controller_id, float p_des, float v_des, float kp, float kd, float t_ff) {
    uint8_t buffer[8];

    /* Clamp all inputs to valid ranges */
    p_des = clampf(p_des, AK70_9_MIT_P_MIN, AK70_9_MIT_P_MAX);
    v_des = clampf(v_des, AK70_9_MIT_V_MIN, AK70_9_MIT_V_MAX);
    kp    = clampf(kp,    AK70_9_MIT_KP_MIN, AK70_9_MIT_KP_MAX);
    kd    = clampf(kd,    AK70_9_MIT_KD_MIN, AK70_9_MIT_KD_MAX);
    t_ff  = clampf(t_ff,  AK70_9_MIT_T_MIN,  AK70_9_MIT_T_MAX);

    /* Convert floats to unsigned integers for bit packing */
    int p_int  = float_to_uint(p_des, AK70_9_MIT_P_MIN,  AK70_9_MIT_P_MAX,  16);
    int v_int  = float_to_uint(v_des, AK70_9_MIT_V_MIN,  AK70_9_MIT_V_MAX,  12);
    int kp_int = float_to_uint(kp,    AK70_9_MIT_KP_MIN, AK70_9_MIT_KP_MAX, 12);
    int kd_int = float_to_uint(kd,    AK70_9_MIT_KD_MIN, AK70_9_MIT_KD_MAX, 12);
    int t_int  = float_to_uint(t_ff,  AK70_9_MIT_T_MIN,  AK70_9_MIT_T_MAX,  12);

    /* Pack integers into the CAN buffer (per AK70-9 manual) */
    buffer[0] = (uint8_t)(kp_int >> 4);                                /* Kp high 8 bits */
    buffer[1] = (uint8_t)(((kp_int & 0xF) << 4) | (kd_int >> 8));     /* Kp low 4, Kd high 4 */
    buffer[2] = (uint8_t)(kd_int & 0xFF);                              /* Kd low 8 bits */
    buffer[3] = (uint8_t)(p_int >> 8);                                 /* Position high 8 bits */
    buffer[4] = (uint8_t)(p_int & 0xFF);                               /* Position low 8 bits */
    buffer[5] = (uint8_t)(v_int >> 4);                                 /* Velocity high 8 bits */
    buffer[6] = (uint8_t)(((v_int & 0xF) << 4) | (t_int >> 8));       /* Velocity low 4, Torque high 4 */
    buffer[7] = (uint8_t)(t_int & 0xFF);                               /* Torque low 8 bits */

    comm_can_transmit_eid(
        controller_id | ((uint32_t)CAN_PACKET_SET_MIT << 8),
        buffer, 8);
}

/* =========================================================================
 * Motor Feedback Parsing
 *
 * The motor uploads 8 bytes of status data at a configurable rate:
 *   Bytes 0-1: position (int16, /10 = degrees)
 *   Bytes 2-3: speed    (int16, *10 = ERPM)
 *   Bytes 4-5: current  (int16, /100 = amps)
 *   Byte  6:   temperature (int8, degrees C)
 *   Byte  7:   error code  (uint8)
 * ========================================================================= */

void motor_receive(MotorStatus* status, const uint8_t* data) {
    status->position    = motor_read_position(data);
    status->speed       = motor_read_speed(data);
    status->current     = motor_read_current(data);
    status->temperature = motor_read_temperature(data);
    status->error       = motor_read_error(data);
}

/* =========================================================================
 * Modular Read Functions
 *
 * Each function parses a single field from the raw 8-byte motor feedback.
 * ========================================================================= */

float motor_read_position(const uint8_t* data) {
    int16_t raw = (int16_t)((data[0] << 8) | data[1]);
    return (float)(raw) * 0.1f;
}

float motor_read_speed(const uint8_t* data) {
    int16_t raw = (int16_t)((data[2] << 8) | data[3]);
    return (float)(raw) * 10.0f;
}

float motor_read_current(const uint8_t* data) {
    int16_t raw = (int16_t)((data[4] << 8) | data[5]);
    return (float)(raw) * 0.01f;
}

int8_t motor_read_temperature(const uint8_t* data) {
    return (int8_t)data[6];
}

uint8_t motor_read_error(const uint8_t* data) {
    return data[7];
}

const char* motor_error_to_string(uint8_t error_code) {
    switch (error_code) {
        case MOTOR_ERROR_NONE:             return "NONE";
        case MOTOR_ERROR_OVER_TEMP:        return "MOTOR_OVER_TEMP";
        case MOTOR_ERROR_OVER_CURRENT:     return "OVER_CURRENT";
        case MOTOR_ERROR_OVER_VOLTAGE:     return "OVER_VOLTAGE";
        case MOTOR_ERROR_UNDER_VOLTAGE:    return "UNDER_VOLTAGE";
        case MOTOR_ERROR_ENCODER:          return "ENCODER_FAULT";
        case MOTOR_ERROR_MOSFET_OVER_TEMP: return "MOSFET_OVER_TEMP";
        case MOTOR_ERROR_STALL:            return "MOTOR_STALL";
        default:                           return "UNKNOWN";
    }
}
