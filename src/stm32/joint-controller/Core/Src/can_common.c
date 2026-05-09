/*
 * can_common.c
 *
 * CAN API transport layer implementation.
 * Handles initialization, filter config, transmit, receive, and ISR callbacks.
 * Supersedes the old can_bus.c.
 */

#include "can_common.h"
#include "error_log.h"
#include <string.h>

/* ── Internal State ── */
static CAN_HandleTypeDef *g_hcan = NULL;
static uint8_t g_my_node_id = 0;
static CanRxRingBuffer g_rxq;

/* ── Ring Buffer Implementation ── */

static void rb_init(CanRxRingBuffer *rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

static int rb_push(CanRxRingBuffer *rb, const CanFrame *frame) {
    if (rb->count >= CAN_RX_BUFFER_CAPACITY) {
        log_can_rx_overflow_count++;
        return 0;
    }
    rb->buf[rb->head] = *frame;
    rb->head = (rb->head + 1) % CAN_RX_BUFFER_CAPACITY;
    rb->count++;
    return 1;
}

static int rb_pop(CanRxRingBuffer *rb, CanFrame *out) {
    if (rb->count == 0) return 0;
    *out = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1) % CAN_RX_BUFFER_CAPACITY;
    rb->count--;
    return 1;
}

/* ── Init ── */

int can_common_init(CAN_HandleTypeDef *hcan, uint8_t my_node_id) {
    g_hcan = hcan;
    g_my_node_id = my_node_id;
    rb_init(&g_rxq);

    CAN_FilterTypeDef f;

    /* Filter 0: Accept all ESTOP messages (FIFO0).
     * Mask must include IDE=0 — without it, every extended frame whose
     * ExtID[28:25] = 0 (which is essentially every VESC packet, since their
     * packet IDs stay below ~30) accidentally matches the type=0 condition
     * here and leaks into FIFO0. */
    memset(&f, 0, sizeof(f));
    f.FilterBank           = 0;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = (CAN_BUILD_ID(CAN_MSG_ESTOP, 0, 0) << 5);
    f.FilterIdLow          = 0x0000;        /* IDE bit (bit 2) = 0 -> standard frame */
    f.FilterMaskIdHigh     = (0x0780 << 5);  /* match type bits [10:7] */
    f.FilterMaskIdLow      = 0x0004;        /* require IDE = 0 */
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(hcan, &f) != HAL_OK) return 0;

    /* Filter 1: Accept TORQUE_CMD for my node (FIFO0).
     * Same IDE=0 requirement as Filter 0 — without it the filter rejects
     * VESC frames only by accidental bit alignment, which is fragile. */
    uint16_t torque_id = CAN_BUILD_ID(CAN_MSG_TORQUE_CMD, 0, my_node_id);
    memset(&f, 0, sizeof(f));
    f.FilterBank           = 1;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = (torque_id << 5);
    f.FilterIdLow          = 0x0000;        /* IDE = 0 -> standard frame */
    f.FilterMaskIdHigh     = (0x078F << 5);  /* match type + dest bits */
    f.FilterMaskIdLow      = 0x0004;        /* require IDE = 0 */
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(hcan, &f) != HAL_OK) return 0;

    /* Filter 2: Accept all extended frames — VESC motor feedback (FIFO1) */
    memset(&f, 0, sizeof(f));
    f.FilterBank           = 2;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = 0x0004;  /* IDE bit (bit 2) set */
    f.FilterMaskIdHigh     = 0x0000;
    f.FilterMaskIdLow      = 0x0004;  /* Only check IDE bit */
    f.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(hcan, &f) != HAL_OK) return 0;

    /* Start CAN peripheral */
    if (HAL_CAN_Start(hcan) != HAL_OK) return 0;

    /* Enable NVIC for both FIFOs */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);

    /* Enable SCE interrupt */
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);

    /* Activate RX and error notifications */
    uint32_t notif = CAN_IT_RX_FIFO0_MSG_PENDING |
                     CAN_IT_RX_FIFO1_MSG_PENDING |
                     CAN_IT_ERROR |
                     CAN_IT_BUSOFF |
                     CAN_IT_LAST_ERROR_CODE;
    if (HAL_CAN_ActivateNotification(hcan, notif) != HAL_OK) return 0;

    return 1;
}

/* ── Send ── */

int can_send_std(uint16_t std_id, const uint8_t *data, uint8_t dlc) {
    if (!g_hcan) return 0;
    if (std_id > 0x7FF) return 0;
    if (dlc > 8) return 0;
    if (HAL_CAN_GetTxMailboxesFreeLevel(g_hcan) == 0) return 0;

    CAN_TxHeaderTypeDef hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.IDE   = CAN_ID_STD;
    hdr.RTR   = CAN_RTR_DATA;
    hdr.StdId = std_id;
    hdr.DLC   = dlc;
    hdr.TransmitGlobalTime = DISABLE;

    uint32_t mailbox = 0;
    return (HAL_CAN_AddTxMessage(g_hcan, &hdr, (uint8_t *)data, &mailbox) == HAL_OK);
}

int can_send_ext(uint32_t ext_id, const uint8_t *data, uint8_t dlc) {
    if (!g_hcan) return 0;
    if (ext_id > 0x1FFFFFFFU) return 0;
    if (dlc > 8) return 0;
    if (HAL_CAN_GetTxMailboxesFreeLevel(g_hcan) == 0) return 0;

    CAN_TxHeaderTypeDef hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.IDE   = CAN_ID_EXT;
    hdr.RTR   = CAN_RTR_DATA;
    hdr.ExtId = ext_id;
    hdr.DLC   = dlc;
    hdr.TransmitGlobalTime = DISABLE;

    uint32_t mailbox = 0;
    return (HAL_CAN_AddTxMessage(g_hcan, &hdr, (uint8_t *)data, &mailbox) == HAL_OK);
}

/* ── Recv ── */

int can_recv(CanFrame *out) {
    return rb_pop(&g_rxq, out);
}

uint8_t can_get_my_node_id(void) {
    return g_my_node_id;
}

int can_set_motor_filter(uint8_t motor_id) {
    if (!g_hcan) return 0;

    /* Extended filter in 32-bit mode:
     *   FR = (ExtID << 3) | (IDE << 2) | (RTR << 1)
     * Match bits [7:0] of ExtID = motor_id, and require IDE = 1.
     *   FilterID   = (motor_id << 3) | 0x04
     *   FilterMask = (0xFF << 3) | 0x04  = 0x7FC
     */
    CAN_FilterTypeDef f;
    memset(&f, 0, sizeof(f));
    f.FilterBank           = 2;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = ((uint16_t)motor_id << 3) | 0x0004;
    f.FilterMaskIdHigh     = 0x0000;
    f.FilterMaskIdLow      = 0x07FC;
    f.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;
    return (HAL_CAN_ConfigFilter(g_hcan, &f) == HAL_OK);
}

/* ── ISR Callbacks ── */

static void can_rx_handler(CAN_HandleTypeDef *hcan, uint32_t fifo) {
    CAN_RxHeaderTypeDef hdr;
    uint8_t data[8];

    memset(&hdr, 0, sizeof(hdr));
    memset(data, 0, sizeof(data));

    if (HAL_CAN_GetRxMessage(hcan, fifo, &hdr, data) != HAL_OK) return;

    CanFrame frame;
    memset(&frame, 0, sizeof(frame));

    if (hdr.IDE == CAN_ID_EXT) {
        frame.id = hdr.ExtId;
        frame.is_extended = 1;
    } else {
        frame.id = hdr.StdId;
        frame.is_extended = 0;
    }

    frame.dlc = hdr.DLC;
    memcpy(frame.data, data, 8);

    rb_push(&g_rxq, &frame);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    can_rx_handler(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    can_rx_handler(hcan, CAN_RX_FIFO1);
}
