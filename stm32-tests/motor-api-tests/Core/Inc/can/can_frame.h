/*
 * can_frame.h
 *
 * CAN frame data structure for use across the CAN driver and motor API.
 */

#ifndef CAN_FRAME_H
#define CAN_FRAME_H

#include <stdint.h>

typedef struct {
    uint32_t id;          /* CAN identifier (11-bit standard or 29-bit extended) */
    uint8_t  dlc;         /* Data length code (0-8) */
    uint8_t  is_extended; /* 1 = extended (29-bit) ID, 0 = standard (11-bit) ID */
    uint8_t  data[8];     /* Message data payload */
} CanFrame;

#endif /* CAN_FRAME_H */
