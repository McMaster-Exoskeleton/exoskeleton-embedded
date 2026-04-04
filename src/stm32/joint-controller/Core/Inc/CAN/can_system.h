/*
 * can_system.h
 *
 * System-level CAN messages: ESTOP.
 */

#ifndef CAN_SYSTEM_H
#define CAN_SYSTEM_H

#include "can_common.h"

/* ── ESTOP Reason Codes ── */
#define CAN_ESTOP_MANUAL      0
#define CAN_ESTOP_COMM_LOSS   1
#define CAN_ESTOP_MOTOR_ERROR 2
#define CAN_ESTOP_SOFTWARE    3

/*
 * Send an ESTOP broadcast. dest is set to 0 (broadcast).
 * Returns 1 on success, 0 on failure.
 */
int can_send_estop(uint8_t src_node, uint8_t reason);

/*
 * Parse an ESTOP frame. Extracts the reason code.
 * Returns 1 on success, 0 if the frame is not an ESTOP message.
 */
int can_parse_estop(const CanFrame *frame, uint8_t *reason);

#endif /* CAN_SYSTEM_H */
