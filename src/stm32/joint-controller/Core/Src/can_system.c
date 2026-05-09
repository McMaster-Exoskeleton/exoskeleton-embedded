/*
 * can_system.c
 *
 * ESTOP message encoding/decoding.
 */

#include "can_system.h"

int can_send_estop(uint8_t src_node, uint8_t reason) {
    uint16_t id = CAN_BUILD_ID(CAN_MSG_ESTOP, src_node, 0);
    uint8_t data[1] = { reason };
    return can_send_std(id, data, 1);
}

int can_parse_estop(const CanFrame *frame, uint8_t *reason) {
    if (frame->is_extended) return 0;
    if (can_get_msg_type((uint16_t)frame->id) != CAN_MSG_ESTOP) return 0;
    if (frame->dlc < 1) return 0;
    *reason = frame->data[0];
    return 1;
}
