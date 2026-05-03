/*
 * can_common.c
 *
 * CAN API transport layer implementation.
 * Handles initialization, filter config, transmit, receive, and ISR callbacks.
 * Supersedes the old can_bus.c.
 */

#include "can_common.h"
#include <string.h>

/* ── Internal State ── */
static CAN_HandleTypeDef *g_hcan = NULL;
static uint8_t g_my_node_id = 0;
static uint8_t g_my_motor_can_id = 0;

static CanRxRingBuffer g_rxq;

//Global variables for debugging
volatile uint32_t g_fifo0_cb_count = 0;
volatile uint32_t g_fifo0_get_ok = 0;
volatile uint32_t g_fifo0_get_fail = 0;
volatile uint32_t g_fifo0_last_id = 0;
volatile uint8_t  g_fifo0_last_ext = 0;
volatile uint8_t  g_fifo0_last_dlc = 0;
volatile uint32_t g_fifo0_fill_at_entry = 0;


/* ── Ring Buffer Implementation ── */

static void rb_init(CanRxRingBuffer *rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

static int rb_push(CanRxRingBuffer *rb, const CanFrame *frame) {
    if (rb->count >= CAN_RX_BUFFER_CAPACITY) return 0;
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

//static uint32_t pack_ext_filter_id32(uint32_t ext_id)
//{
//    /*
//     * bxCAN 32-bit filter layout for extended data frames:
//     * bits 31:3 = 29-bit extended ID
//     * bit  2    = IDE
//     * bit  1    = RTR
//     * bit  0    = reserved
//     *
//     * For extended data frame:
//     * IDE = 1
//     * RTR = 0
//     */
//    return ((ext_id & 0x1FFFFFFFU) << 3) | 0x00000004U;
//}

static uint32_t pack_ext_filter32(uint32_t ext_id)
{
    /*
     * bxCAN 32-bit filter layout for extended data frames:
     * bits 31:3 = 29-bit extended ID
     * bit  2    = IDE = 1
     * bit  1    = RTR = 0
     */
    return ((ext_id & 0x1FFFFFFFU) << 3) | 0x00000004U;
}


/* ── Init ── */

int can_common_init(CAN_HandleTypeDef *hcan, uint8_t my_node_id, uint8_t my_motor_can_id) {
    g_hcan = hcan;
    g_my_node_id = my_node_id;
    g_my_motor_can_id = my_motor_can_id;
    rb_init(&g_rxq);
    uint16_t estop_id = CAN_BUILD_ID(CAN_MSG_ESTOP, CAN_NODE_PI, 0);


    CAN_FilterTypeDef f;

    /* Filter 0: Accept all standard ESTOP messages (FIFO0) */
    memset(&f, 0, sizeof(f));
    f.FilterBank           = 0;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = (estop_id << 5);
    f.FilterIdLow          = 0x0000;        // IDE = 0, standard frame
    f.FilterMaskIdHigh     = (0x07F0 << 5); // match type bits + source bits
    f.FilterMaskIdLow      = 0x0004;        // require standard frame
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(hcan, &f) != HAL_OK) return 0;

    /* Filter 1: Accept standard TORQUE_CMD for my node (FIFO0) */
    uint16_t torque_id = CAN_BUILD_ID(CAN_MSG_TORQUE_CMD, CAN_NODE_PI, my_node_id);

    memset(&f, 0, sizeof(f));
    f.FilterBank           = 1;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = (torque_id << 5);
    f.FilterIdLow          = 0x0000;        // IDE = 0, standard frame
    f.FilterMaskIdHigh     = (0x07FF << 5); // match type + source + destination
    f.FilterMaskIdLow      = 0x0004;        // require standard frame
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(hcan, &f) != HAL_OK) return 0;

    /* Filter 2: Accept only this MCU's motor feedback frame */
    uint32_t motor_feedback_ext_id = (0x29U << 8) | ((uint32_t)my_motor_can_id & 0xFFU);

    uint32_t id32 = pack_ext_filter32(motor_feedback_ext_id);

    memset(&f, 0, sizeof(f));
    f.FilterBank           = 2;
    f.FilterMode           = CAN_FILTERMODE_IDLIST;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;

    /* In 32-bit IDLIST mode, one bank can hold two exact IDs.
       We put the same exact ID in both slots. */
    f.FilterIdHigh         = (uint16_t)((id32 >> 16) & 0xFFFFU);
    f.FilterIdLow          = (uint16_t)( id32        & 0xFFFFU);
    f.FilterMaskIdHigh     = (uint16_t)((id32 >> 16) & 0xFFFFU);
    f.FilterMaskIdLow      = (uint16_t)( id32        & 0xFFFFU);

    f.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan, &f) != HAL_OK) return 0;

    /* Start CAN peripheral */
    HAL_CAN_Stop(hcan);
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

    if (HAL_CAN_GetTxMailboxesFreeLevel(g_hcan) == 0) {
        return 0;
    }

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

    if (HAL_CAN_GetTxMailboxesFreeLevel(g_hcan) == 0) {
        return 0;
    }

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

// PRIMASK preserves the previous interrupt state instead of blindly enabling interrupts.
int can_recv(CanFrame *out) {
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    int ok = rb_pop(&g_rxq, out);

    if (!primask) {
        __enable_irq();
    }

    return ok;
}

uint8_t can_get_my_node_id(void) {
    return g_my_node_id;
}

/* ── ISR Callbacks ── */

static int can_rx_handler(CAN_HandleTypeDef *hcan, uint32_t fifo)
{
    CAN_RxHeaderTypeDef hdr;
    uint8_t data[8];

    memset(&hdr, 0, sizeof(hdr));
    memset(data, 0, sizeof(data));

    if (HAL_CAN_GetRxMessage(hcan, fifo, &hdr, data) != HAL_OK) {
        return 0;
    }

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

    return 1;
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1) return;

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        if (!can_rx_handler(hcan, CAN_RX_FIFO0)) {
            break;
        }
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1) return;

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) > 0)
    {
        if (!can_rx_handler(hcan, CAN_RX_FIFO1)) {
            break;
        }
    }
}

volatile uint32_t g_can_error_count = 0;
volatile uint32_t g_last_can_error = 0;
volatile uint32_t g_last_can_esr = 0;
volatile uint8_t g_can_busoff_seen = 0;

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1) return;

    g_can_error_count++;
    g_last_can_error = HAL_CAN_GetError(hcan);
    g_last_can_esr = CAN1->ESR;

    if (g_last_can_error & HAL_CAN_ERROR_BOF) {
        g_can_busoff_seen = 1;
    }
}
