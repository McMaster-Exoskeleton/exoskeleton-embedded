/*
 * can_common.h
 *
 * CAN API transport layer for the McMaster Exoskeleton network.
 * Provides node/message constants, CAN ID construction/extraction,
 * frame structure, ring buffer, and low-level send/recv functions.
 */

#ifndef CAN_COMMON_H
#define CAN_COMMON_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ── Node IDs ── */
#define CAN_NODE_PI          0
#define CAN_NODE_LEFT_HIP    1
#define CAN_NODE_RIGHT_HIP   2
#define CAN_NODE_LEFT_KNEE   3
#define CAN_NODE_RIGHT_KNEE  4

/* ── Message Type IDs (ordered by CAN priority) ── */
#define CAN_MSG_ESTOP        0x0
#define CAN_MSG_TORQUE_CMD   0x1
#define CAN_MSG_MOTOR_STATUS 0x2
#define CAN_MSG_IMU_ACCEL    0x3
#define CAN_MSG_IMU_GYRO     0x4
#define CAN_MSG_HEARTBEAT    0x5

/* ── CAN ID Construction ──
 * 11-bit standard ID layout:
 *   Bits [10:7] = Message Type (4 bits)
 *   Bits [6:4]  = Source Node  (3 bits)
 *   Bits [3:0]  = Dest/Context (4 bits)
 */
#define CAN_BUILD_ID(msg_type, src, dest) \
    ((uint16_t)(((uint16_t)(msg_type) << 7) | ((uint16_t)(src) << 4) | ((uint16_t)(dest))))

/* ── CAN ID Extraction ── */
static inline uint8_t can_get_msg_type(uint16_t can_id) {
    return (uint8_t)((can_id >> 7) & 0x0F);
}

static inline uint8_t can_get_src_node(uint16_t can_id) {
    return (uint8_t)((can_id >> 4) & 0x07);
}

static inline uint8_t can_get_dest(uint16_t can_id) {
    return (uint8_t)(can_id & 0x0F);
}

/* ── CAN Frame ── */
typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  is_extended;
    uint8_t  data[8];
} CanFrame;

/* ── Ring Buffer (RX) ── */
#define CAN_RX_BUFFER_CAPACITY 32

typedef struct {
    CanFrame buf[CAN_RX_BUFFER_CAPACITY];
    volatile uint16_t head, tail, count;
} CanRxRingBuffer;

/* ── Transport Functions ── */
int can_common_init(CAN_HandleTypeDef *hcan, uint8_t my_node_id);
int can_send_std(uint16_t std_id, const uint8_t *data, uint8_t dlc);
int can_send_ext(uint32_t ext_id, const uint8_t *data, uint8_t dlc);
int can_recv(CanFrame *out);
uint8_t can_get_my_node_id(void);
int can_set_motor_filter(uint8_t motor_id);

#endif /* CAN_COMMON_H */
