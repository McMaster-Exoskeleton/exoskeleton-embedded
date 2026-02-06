/*
 * ak70_9.h
 *
 *  Created on: Feb 4, 2026
 *      Author: Juan Reyes
 *
 *  CubeMars AK70-9 KV60 Motor CAN API
 *
 *  This API provides control functions for the AK70-9 motor over CAN bus
 *  using the STM32 HAL. Two control interfaces are supported:
 *
 *  1. Servo Mode (7 modes via control IDs 0-6):
 *     Duty cycle, current (torque), current brake, RPM, position,
 *     set origin, and position-velocity loop.
 *
 *  2. MIT Force Control Mode (control ID 8):
 *     Combines position, velocity, Kp, Kd, and torque feedforward into
 *     a single 8-byte CAN message for compliant force control.
 *
 *  CAN Frame Format (both modes):
 *    Identifier: bits [28:8] = Control Mode ID, bits [7:0] = Motor Driver ID
 *    Frame Type: Extended (29-bit)
 *    Frame Format: DATA
 *    DLC: 8 bytes
 */

#ifndef INC_CAN_AK70_9_H_
#define INC_CAN_AK70_9_H_

#include <stdint.h>

/* ======================== Motor Parameter Limits ========================= */

/* Servo mode limits */
#define AK70_9_DUTY_MIN            (-1.0f)       /* Duty cycle min (-100%) */
#define AK70_9_DUTY_MAX            (1.0f)        /* Duty cycle max (+100%) */
#define AK70_9_CURRENT_MIN         (-60.0f)      /* Current min (A) */
#define AK70_9_CURRENT_MAX         (60.0f)       /* Current max (A) */
#define AK70_9_BRAKE_CURRENT_MIN   (0.0f)        /* Brake current min (A) */
#define AK70_9_BRAKE_CURRENT_MAX   (60.0f)       /* Brake current max (A) */
#define AK70_9_RPM_MIN             (-100000.0f)  /* Electrical RPM min */
#define AK70_9_RPM_MAX             (100000.0f)   /* Electrical RPM max */
#define AK70_9_POS_DEG_MIN         (-36000.0f)   /* Position min (degrees) */
#define AK70_9_POS_DEG_MAX         (36000.0f)    /* Position max (degrees) */
#define AK70_9_POS_SPD_MIN         (-32768)      /* Pos-vel speed min (raw, *10 = ERPM) */
#define AK70_9_POS_SPD_MAX         (32767)       /* Pos-vel speed max (raw) */
#define AK70_9_POS_ACC_MIN         (0)           /* Pos-vel accel min (raw, *10 = ERPM/s^2) */
#define AK70_9_POS_ACC_MAX         (32767)       /* Pos-vel accel max (raw) */

/* MIT force control mode limits */
#define AK70_9_MIT_P_MIN           (-12.56f)     /* Position min (rad) */
#define AK70_9_MIT_P_MAX           (12.56f)      /* Position max (rad) */
#define AK70_9_MIT_V_MIN           (-30.0f)      /* Velocity min (rad/s) */
#define AK70_9_MIT_V_MAX           (30.0f)       /* Velocity max (rad/s) */
#define AK70_9_MIT_T_MIN           (-32.0f)      /* Torque min (Nm) */
#define AK70_9_MIT_T_MAX           (32.0f)       /* Torque max (Nm) */
#define AK70_9_MIT_KP_MIN          (0.0f)        /* Kp gain min */
#define AK70_9_MIT_KP_MAX          (500.0f)      /* Kp gain max */
#define AK70_9_MIT_KD_MIN          (0.0f)        /* Kd gain min */
#define AK70_9_MIT_KD_MAX          (5.0f)        /* Kd gain max */

/* ======================== Control Mode IDs =============================== */

/*
 * CAN control mode IDs placed in bits [28:8] of the extended CAN identifier.
 * The motor driver ID occupies bits [7:0].
 */
typedef enum {
    CAN_PACKET_SET_DUTY          = 0,  /* Duty cycle mode */
    CAN_PACKET_SET_CURRENT       = 1,  /* Current loop (torque) mode */
    CAN_PACKET_SET_CURRENT_BRAKE = 2,  /* Current brake mode */
    CAN_PACKET_SET_RPM           = 3,  /* Velocity (RPM) mode */
    CAN_PACKET_SET_POS           = 4,  /* Position mode */
    CAN_PACKET_SET_ORIGIN_HERE   = 5,  /* Set origin (zero) position */
    CAN_PACKET_SET_POS_SPD       = 6,  /* Position-velocity loop mode */
    CAN_PACKET_SET_MIT           = 8,  /* MIT force control mode */
} CAN_PACKET_ID;

/* ======================== Motor Error Codes ============================== */

/*
 * Error codes reported by the motor in byte 7 of the feedback message.
 */
typedef enum {
    MOTOR_ERROR_NONE              = 0,  /* No fault */
    MOTOR_ERROR_OVER_TEMP         = 1,  /* Motor over-temperature */
    MOTOR_ERROR_OVER_CURRENT      = 2,  /* Over-current fault */
    MOTOR_ERROR_OVER_VOLTAGE      = 3,  /* Over-voltage fault */
    MOTOR_ERROR_UNDER_VOLTAGE     = 4,  /* Under-voltage fault */
    MOTOR_ERROR_ENCODER           = 5,  /* Encoder fault */
    MOTOR_ERROR_MOSFET_OVER_TEMP  = 6,  /* MOSFET/driver board over-temperature */
    MOTOR_ERROR_STALL             = 7,  /* Motor stall / lock-up */
} MotorErrorCode;

/* ======================== Motor Feedback Data ============================ */

/*
 * Parsed motor feedback from the CAN upload message.
 * The motor transmits this data periodically (configurable 1-500 Hz).
 *
 * Raw message format (8 bytes):
 *   Bytes 0-1: Position (int16, value/10 = degrees, range +/-3200 deg)
 *   Bytes 2-3: Speed    (int16, value*10 = ERPM, range +/-320000 ERPM)
 *   Bytes 4-5: Current  (int16, value/100 = amps, range +/-60 A)
 *   Byte  6:   Temperature (int8, degrees C, range -20 to 127)
 *   Byte  7:   Error code  (uint8, see MotorErrorCode)
 */
typedef struct {
    float    position;     /* Motor position in degrees */
    float    speed;        /* Motor speed in electrical RPM */
    float    current;      /* Motor current in amps */
    int8_t   temperature;  /* Driver board temperature in degrees C */
    uint8_t  error;        /* Error code (see MotorErrorCode) */
} MotorStatus;

/* ======================== Servo Mode Functions =========================== */

/*
 * Set the motor duty cycle.
 * Drives the motor with a voltage proportional to the duty cycle.
 *
 * duty: -1.0 to 1.0 (representing -100% to +100%)
 */
void comm_can_set_duty(uint8_t controller_id, float duty);

/*
 * Set the motor current (torque control).
 * Commands a target Iq current. Motor torque = Iq * KT.
 *
 * current: -60.0 to 60.0 amps
 */
void comm_can_set_current(uint8_t controller_id, float current);

/*
 * Set the motor brake current.
 * Applies a braking current to hold the motor at its current position.
 * Monitor motor temperature when using this mode.
 *
 * current: 0.0 to 60.0 amps
 */
void comm_can_set_cb(uint8_t controller_id, float current);

/*
 * Set the motor speed (velocity mode).
 * Commands the motor to spin at a specified electrical RPM.
 * Acceleration defaults to the maximum value.
 *
 * rpm: -100000 to 100000 electrical RPM
 */
void comm_can_set_rpm(uint8_t controller_id, float rpm);

/*
 * Set the motor position.
 * Commands the motor to move to an absolute angular position.
 * Speed and acceleration default to maximum values.
 *
 * pos: -36000.0 to 36000.0 degrees
 */
void comm_can_set_pos(uint8_t controller_id, float pos);

/*
 * Set the motor origin (zero position).
 *
 * set_origin_mode: 0 = temporary origin (lost on power cycle)
 *                  1 = permanent origin (saved to flash)
 */
void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode);

/*
 * Set motor position with velocity and acceleration limits.
 * Provides smooth trajectory control.
 *
 * pos: -36000.0 to 36000.0 degrees
 * spd: -32768 to 32767 (raw value, multiplied by 10 = ERPM)
 * rpa: 0 to 32767 (raw acceleration, 1 unit = 10 ERPM/s^2)
 */
void comm_can_set_pos_spd(uint8_t controller_id, float pos, int16_t spd, int16_t rpa);

/* ======================== MIT Force Control ============================== */

/*
 * Send an MIT force control command.
 * Packs position, velocity, Kp, Kd, and torque feedforward into an 8-byte
 * CAN message. The motor-side control law is:
 *   torque = Kp * (p_des - p_actual) + Kd * (v_des - v_actual) + t_ff
 *
 * p_des: desired position in radians   (-12.56 to 12.56)
 * v_des: desired velocity in rad/s     (-30.0 to 30.0)
 * kp:    position gain                 (0 to 500)
 * kd:    velocity/damping gain         (0 to 5.0)
 * t_ff:  torque feedforward in Nm      (-32.0 to 32.0)
 */
void pack_cmd(uint8_t controller_id, float p_des, float v_des, float kp, float kd, float t_ff);

/*
 * Convert a float to an unsigned integer for CAN transmission.
 * Maps a value from [x_min, x_max] into a (bits)-wide unsigned integer.
 */
int float_to_uint(float x, float x_min, float x_max, unsigned int bits);

/* ======================== Motor Feedback ================================= */

/*
 * Parse an 8-byte motor feedback CAN message into a MotorStatus struct.
 * Populates all fields at once. For individual field parsing, use the
 * motor_read_* functions below.
 *
 * status: pointer to MotorStatus struct to populate
 * data:   pointer to the 8-byte CAN data payload
 */
void motor_receive(MotorStatus* status, const uint8_t* data);

/* ======================== Modular Read Functions ========================== */

/*
 * Parse individual motor feedback fields from the raw 8-byte CAN payload.
 * These allow reading a single value without parsing the entire message.
 */

/* Read motor position in degrees from feedback bytes 0-1. */
float motor_read_position(const uint8_t* data);

/* Read motor speed in electrical RPM from feedback bytes 2-3. */
float motor_read_speed(const uint8_t* data);

/* Read motor current in amps from feedback bytes 4-5. */
float motor_read_current(const uint8_t* data);

/* Read driver board temperature in degrees C from feedback byte 6. */
int8_t motor_read_temperature(const uint8_t* data);

/* Read motor error code from feedback byte 7. */
uint8_t motor_read_error(const uint8_t* data);

/*
 * Convert a motor error code to a human-readable string.
 * Returns a static string describing the error (e.g. "OVER_CURRENT").
 */
const char* motor_error_to_string(uint8_t error_code);

/* ======================== Byte Packing Utilities ========================= */

/* Append a 32-bit integer to a buffer in big-endian order. */
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);

/* Append a 16-bit integer to a buffer in big-endian order. */
void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index);

#endif /* INC_CAN_AK70_9_H_ */
