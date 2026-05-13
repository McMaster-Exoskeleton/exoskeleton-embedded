/*
 * can_common.h
 *
 * CAN API transport layer for the McMaster Exoskeleton network.
 * Provides node/message constants, CAN ID construction/extraction,
 * frame structure, ring buffer, and low-level send/recv functions.
 *
 * This header supersedes the old can_frame.h, ring_buffer.h, and can_bus.h.
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

/*
 * Initialize the CAN peripheral with filters configured for this node.
 * Must be called after HAL_CAN_Init() (i.e., after MX_CAN1_Init()).
 *
 * Filter setup:
 *   Bank 0 (FIFO0): Accept all ESTOP messages
 *   Bank 1 (FIFO0): Accept TORQUE_CMD addressed to my_node_id
 *   Bank 2 (FIFO1): Accept all extended frames (VESC motor feedback)
 *
 * Returns 1 on success, 0 on failure.
 */
int can_common_init(CAN_HandleTypeDef *hcan, uint8_t my_node_id);

/*
 * Transmit a CAN frame using an 11-bit standard identifier.
 * Returns 1 on success, 0 on failure (mailbox full or invalid args).
 */
int can_send_std(uint16_t std_id, const uint8_t *data, uint8_t dlc);

/*
 * Transmit a CAN frame using a 29-bit extended identifier.
 * Used by the motor API (ak70_9) for VESC protocol.
 * Returns 1 on success, 0 on failure.
 */
int can_send_ext(uint32_t ext_id, const uint8_t *data, uint8_t dlc);

/*
 * Pop the next received CAN frame from the internal ring buffer.
 * Returns 1 if a frame was retrieved, 0 if the buffer is empty.
 */
int can_recv(CanFrame *out);

/*
 * Get the node ID set during can_common_init().
 */
uint8_t can_get_my_node_id(void);

/*
 * Reconfigure Filter 2 (FIFO1) to accept only extended frames whose low
 * 8 bits match motor_id. Call this after can_common_init() to avoid
 * receiving feedback from other motors on a shared bus.
 *
 * Returns 1 on success, 0 on failure.
 */
int can_set_motor_filter(uint8_t motor_id);

#endif /* CAN_COMMON_H */
