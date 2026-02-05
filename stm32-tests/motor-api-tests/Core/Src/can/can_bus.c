/*
 * can_bus.c
 *
 * CAN bus driver implementation for STM32 HAL.
 * Handles initialization, transmit, and interrupt-driven receive.
 */

#include "can/can_bus.h"
#include "can/ring_buffer.h"
#include <string.h>

/* Internal state */
static CAN_HandleTypeDef* g_hcan = NULL;
static CanRxRingBuffer g_rxq;

int can_bus_init(CAN_HandleTypeDef* hcan) {
    g_hcan = hcan;
    can_rx_rb_init(&g_rxq);

    /* Accept-all filter: mask = 0 means all bits are don't-care */
    CAN_FilterTypeDef filter;
    memset(&filter, 0, sizeof(filter));
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(g_hcan, &filter) != HAL_OK) return 0;
    if (HAL_CAN_Start(g_hcan) != HAL_OK) return 0;

    /* Enable NVIC for CAN1 RX FIFO 0 */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    /* Activate RX and error notifications */
    uint32_t notif = CAN_IT_RX_FIFO0_MSG_PENDING |
                     CAN_IT_ERROR |
                     CAN_IT_BUSOFF |
                     CAN_IT_LAST_ERROR_CODE;

    if (HAL_CAN_ActivateNotification(g_hcan, notif) != HAL_OK) return 0;

    return 1;
}

int can_bus_send_ext(uint32_t ext_id, const uint8_t* data, uint8_t dlc) {
    if (!g_hcan) return 0;
    if (ext_id > 0x1FFFFFFFU) return 0;
    if (dlc > 8) return 0;

    CAN_TxHeaderTypeDef hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.IDE   = CAN_ID_EXT;
    hdr.RTR   = CAN_RTR_DATA;
    hdr.ExtId = ext_id;
    hdr.DLC   = dlc;
    hdr.TransmitGlobalTime = DISABLE;

    uint32_t mailbox = 0;
    return (HAL_CAN_AddTxMessage(g_hcan, &hdr, (uint8_t*)data, &mailbox) == HAL_OK);
}

int can_bus_send_std(uint16_t std_id, const uint8_t* data, uint8_t dlc) {
    if (!g_hcan) return 0;
    if (std_id > 0x7FF) return 0;
    if (dlc > 8) return 0;

    CAN_TxHeaderTypeDef hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.IDE   = CAN_ID_STD;
    hdr.RTR   = CAN_RTR_DATA;
    hdr.StdId = std_id;
    hdr.DLC   = dlc;
    hdr.TransmitGlobalTime = DISABLE;

    uint32_t mailbox = 0;
    return (HAL_CAN_AddTxMessage(g_hcan, &hdr, (uint8_t*)data, &mailbox) == HAL_OK);
}

int can_bus_recv(CanFrame* out) {
    return can_rx_rb_pop(&g_rxq, out);
}

/*
 * HAL callback: called from HAL_CAN_IRQHandler when a message arrives
 * in RX FIFO 0. Reads the message and pushes it into the ring buffer.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef hdr;
    uint8_t data[8];

    memset(&hdr, 0, sizeof(hdr));
    memset(data, 0, sizeof(data));

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, data) != HAL_OK) {
        return;
    }

    CanFrame f;
    memset(&f, 0, sizeof(f));

    if (hdr.IDE == CAN_ID_EXT) {
        f.id = hdr.ExtId;
        f.is_extended = 1;
    } else {
        f.id = hdr.StdId;
        f.is_extended = 0;
    }

    f.dlc = hdr.DLC;
    memcpy(f.data, data, 8);

    /* If buffer is full the frame is silently dropped */
    can_rx_rb_push(&g_rxq, &f);
}
