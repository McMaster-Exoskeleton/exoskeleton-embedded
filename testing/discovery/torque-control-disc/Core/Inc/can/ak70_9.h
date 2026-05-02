/*
 * ak70_9.h
 *
 *  CubeMars AK70-9 KV60 Motor CAN API
 *
 *  Two control interfaces:
 *  1. Servo Mode (control IDs 0-6): duty cycle, current, brake, RPM, position, set origin, pos-vel
 *  2. MIT Force Control Mode (control ID 8): position + velocity + Kp + Kd + torque feedforward
 *
 *  CAN frame: extended 29-bit IDs, big-endian payload, DLC 8.
 *  Identifier layout: bits [28:8] = control mode, bits [7:0] = motor driver ID.
 */

#ifndef INC_CAN_AK70_9_H_
#define INC_CAN_AK70_9_H_

#include <stdint.h>

#define MOTOR_CAN_ID               104

#define TEST_DUTY_MAX              (0.9f)
#define TEST_CURRENT_MAX           (5.0f)
#define TEST_BRAKE_CURRENT_MAX     (5.0f)
#define TEST_RPM_MAX               (5000.0f)
#define TEST_POS_DEG_MAX           (360.0f)
#define TEST_MIT_P_MAX             (3.14f)
#define TEST_MIT_V_MAX             (5.0f)
#define TEST_MIT_T_MAX             (5.0f)
#define TEST_MIT_KP_MAX            (50.0f)
#define TEST_MIT_KD_MAX            (2.5f)

#define AK70_9_DUTY_MIN            (-1.0f)
#define AK70_9_DUTY_MAX            (1.0f)
#define AK70_9_CURRENT_MIN         (-60.0f)
#define AK70_9_CURRENT_MAX         (60.0f)
#define AK70_9_BRAKE_CURRENT_MIN   (0.0f)
#define AK70_9_BRAKE_CURRENT_MAX   (60.0f)
#define AK70_9_RPM_MIN             (-100000.0f)
#define AK70_9_RPM_MAX             (100000.0f)
#define AK70_9_POS_DEG_MIN         (-36000.0f)
#define AK70_9_POS_DEG_MAX         (36000.0f)
#define AK70_9_POS_SPD_MIN         (-32768)
#define AK70_9_POS_SPD_MAX         (32767)
#define AK70_9_POS_ACC_MIN         (0)
#define AK70_9_POS_ACC_MAX         (32767)

#define AK70_9_MIT_P_MIN           (-12.56f)
#define AK70_9_MIT_P_MAX           (12.56f)
#define AK70_9_MIT_V_MIN           (-30.0f)
#define AK70_9_MIT_V_MAX           (30.0f)
#define AK70_9_MIT_T_MIN           (-32.0f)
#define AK70_9_MIT_T_MAX           (32.0f)
#define AK70_9_MIT_KP_MIN          (0.0f)
#define AK70_9_MIT_KP_MAX          (500.0f)
#define AK70_9_MIT_KD_MIN          (0.0f)
#define AK70_9_MIT_KD_MAX          (5.0f)

typedef enum {
    CAN_PACKET_SET_DUTY          = 0,
    CAN_PACKET_SET_CURRENT       = 1,
    CAN_PACKET_SET_CURRENT_BRAKE = 2,
    CAN_PACKET_SET_RPM           = 3,
    CAN_PACKET_SET_POS           = 4,
    CAN_PACKET_SET_ORIGIN_HERE   = 5,
    CAN_PACKET_SET_POS_SPD       = 6,
    CAN_PACKET_SET_MIT           = 8,
} CAN_PACKET_ID;

typedef enum {
    MOTOR_ERROR_NONE              = 0,
    MOTOR_ERROR_OVER_TEMP         = 1,
    MOTOR_ERROR_OVER_CURRENT      = 2,
    MOTOR_ERROR_OVER_VOLTAGE      = 3,
    MOTOR_ERROR_UNDER_VOLTAGE     = 4,
    MOTOR_ERROR_ENCODER           = 5,
    MOTOR_ERROR_MOSFET_OVER_TEMP  = 6,
    MOTOR_ERROR_STALL             = 7,
} MotorErrorCode;

typedef struct {
    float    position;
    float    speed;
    float    current;
    int8_t   temperature;
    uint8_t  error;
} MotorStatus;

void comm_can_set_duty(uint8_t controller_id, float duty);
void comm_can_set_current(uint8_t controller_id, float current);
void comm_can_set_cb(uint8_t controller_id, float current);
void comm_can_set_rpm(uint8_t controller_id, float rpm);
void comm_can_set_pos(uint8_t controller_id, float pos);
void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode);
void comm_can_set_pos_spd(uint8_t controller_id, float pos, int16_t spd, int16_t rpa);

void pack_cmd(uint8_t controller_id, float p_des, float v_des, float kp, float kd, float t_ff);
int float_to_uint(float x, float x_min, float x_max, unsigned int bits);

void motor_receive(MotorStatus* status, const uint8_t* data);
float motor_read_position(const uint8_t* data);
float motor_read_speed(const uint8_t* data);
float motor_read_current(const uint8_t* data);
int8_t motor_read_temperature(const uint8_t* data);
uint8_t motor_read_error(const uint8_t* data);
const char* motor_error_to_string(uint8_t error_code);

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index);

#endif /* INC_CAN_AK70_9_H_ */
