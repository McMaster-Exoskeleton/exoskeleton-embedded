/*
 * can_motor.h
 *
 * Motor CAN messages: torque commands (Pi -> STM32) and
 * motor status feedback (STM32 -> Pi).
 */

#ifndef CAN_MOTOR_H
#define CAN_MOTOR_H

#include "can_common.h"

int can_send_torque_cmd(uint8_t dest_node, float torque_nm);
int can_parse_torque_cmd(const CanFrame *frame, float *torque_nm);

int can_send_motor_status(uint8_t src_node, float position, float speed,
                          float current, int8_t temperature, uint8_t error);
int can_parse_motor_status(const CanFrame *frame, float *position, float *speed,
                           float *current, int8_t *temperature, uint8_t *error);

#endif /* CAN_MOTOR_H */
