/*
 * ring_buffer.h
 *
 * Circular buffer for CAN RX frames. Header-only with static inline functions.
 * Used by the CAN bus driver to queue received messages from the ISR.
 */

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include "can/can_frame.h"
#include <stdint.h>
#include <string.h>

#define CAN_RX_BUFFER_CAPACITY 32

typedef struct {
    CanFrame buf[CAN_RX_BUFFER_CAPACITY];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
} CanRxRingBuffer;

static inline void can_rx_rb_init(CanRxRingBuffer* rb) {
    rb->head  = 0;
    rb->tail  = 0;
    rb->count = 0;
}

/* Returns 1 on success, 0 if buffer is full (frame dropped) */
static inline int can_rx_rb_push(CanRxRingBuffer* rb, const CanFrame* f) {
    if (rb->count >= CAN_RX_BUFFER_CAPACITY) return 0;
    rb->buf[rb->head] = *f;
    rb->head = (rb->head + 1) % CAN_RX_BUFFER_CAPACITY;
    rb->count++;
    return 1;
}

/* Returns 1 on success, 0 if buffer is empty */
static inline int can_rx_rb_pop(CanRxRingBuffer* rb, CanFrame* out) {
    if (rb->count == 0) return 0;
    *out = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1) % CAN_RX_BUFFER_CAPACITY;
    rb->count--;
    return 1;
}

static inline uint16_t can_rx_rb_size(const CanRxRingBuffer* rb) {
    return rb->count;
}

#endif /* RING_BUFFER_H */
