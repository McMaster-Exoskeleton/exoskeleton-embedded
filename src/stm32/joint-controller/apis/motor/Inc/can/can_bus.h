/*
 * can_bus.h
 *
 * CAN bus driver for STM32 HAL. Provides initialization, transmit (standard
 * and extended IDs), and interrupt-driven receive with an internal ring buffer.
 */

#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "stm32f4xx_hal.h"
#include "can/can_frame.h"
#include <stdint.h>

/*
 * Initialize the CAN peripheral: configure an accept-all filter, start the
 * peripheral, enable the CAN1 RX FIFO 0 interrupt in the NVIC, and activate
 * RX/error notifications.
 *
 * Must be called after HAL_CAN_Init() (i.e. after MX_CAN1_Init()).
 *
 * Returns 1 on success, 0 on failure.
 */
int can_bus_init(CAN_HandleTypeDef* hcan);

/*
 * Transmit a CAN frame using a 29-bit extended identifier.
 * This is the frame type used by the AK70-9 motor protocol.
 *
 * Returns 1 on success, 0 on failure.
 */
int can_bus_send_ext(uint32_t ext_id, const uint8_t* data, uint8_t dlc);

/*
 * Transmit a CAN frame using an 11-bit standard identifier.
 *
 * Returns 1 on success, 0 on failure.
 */
int can_bus_send_std(uint16_t std_id, const uint8_t* data, uint8_t dlc);

/*
 * Pop the next received CAN frame from the internal ring buffer.
 *
 * Returns 1 if a frame was retrieved, 0 if the buffer is empty.
 */
int can_bus_recv(CanFrame* out);

#endif /* CAN_BUS_H */
