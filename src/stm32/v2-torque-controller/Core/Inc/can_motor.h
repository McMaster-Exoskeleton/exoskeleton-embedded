/*
 * can_motor.h
 *
 * Motor CAN messages: torque commands (Pi -> STM32) and
 * motor status feedback (STM32 -> Pi).
 *
 * These are inter-node messages, NOT the VESC motor protocol.
 * VESC communication is handled by ak70_9.c using extended CAN frames.
 */

#ifndef CAN_MOTOR_H
#define CAN_MOTOR_H

#include "can_common.h"

/*
 * Send a torque command to a specific STM32 node (DLC 2).
 * torque_nm in Nm. Encoded as int16 * 1000. Range: +/-32 Nm.
 * Source node is implicitly my_node_id (set during can_common_init).
 * Returns 1 on success, 0 on failure.
 */
int can_send_torque_cmd(uint8_t dest_node, float torque_nm);

/*
 * Parse a torque command frame.
 * Returns 1 on success, 0 if the frame is not a TORQUE_CMD message.
 */
int can_parse_torque_cmd(const CanFrame *frame, float *torque_nm);

/*
 * Send motor status to the Pi (DLC 8).
 * Values are re-encoded from VESC big-endian into little-endian.
 *   position: degrees (int16 * 10)
 *   speed: ERPM (int16, raw * 10 = ERPM)
 *   current: amps (int16 * 100)
 *   temperature: degrees C (int8)
 *   error: MotorErrorCode (uint8)
 * Returns 1 on success, 0 on failure.
 */
int can_send_motor_status(uint8_t src_node, float position, float speed,
                          float current, int8_t temperature, uint8_t error);

/*
 * Parse a motor status frame.
 * Returns 1 on success, 0 if the frame is not a MOTOR_STATUS message.
 */
int can_parse_motor_status(const CanFrame *frame, float *position, float *speed,
                           float *current, int8_t *temperature, uint8_t *error);

#endif /* CAN_MOTOR_H */
