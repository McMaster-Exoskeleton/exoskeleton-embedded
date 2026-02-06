/*
 * uart_cmd.h
 *
 * Text-based UART command protocol for testing the AK70-9 motor API.
 *
 * Commands are newline-terminated ASCII strings sent from a host (Python script).
 * Responses are newline-terminated ASCII strings sent back to the host.
 *
 * Receive-only commands (current phase):
 *   PING       -> PONG
 *   READ_ALL   -> ALL:POS=<deg>,SPD=<rpm>,CUR=<A>,TEMP=<C>,ERR=<code>
 *   READ_POS   -> POS:<degrees>
 *   READ_SPD   -> SPD:<erpm>
 *   READ_CUR   -> CUR:<amps>
 *   READ_TEMP  -> TEMP:<celsius>
 *   READ_ERR   -> ERR:<code>:<description>
 *
 * Proactive error notifications (sent when motor error state changes):
 *   !ERR:<code>:<description>
 *
 * Unknown commands:
 *   UNKNOWN_CMD:<original_command>
 *
 * NOTE: Float formatting with snprintf requires the "-u _printf_float"
 *       linker flag in STM32CubeIDE (Project > Properties > C/C++ Build >
 *       Settings > MCU GCC Linker > Miscellaneous > Other flags).
 */

#ifndef UART_CMD_H
#define UART_CMD_H

#include "stm32f4xx_hal.h"
#include "can/ak70_9.h"

#define UART_CMD_BUF_SIZE 64

/*
 * Initialize the UART command handler and begin receiving bytes via interrupt.
 * Call once after HAL_UART_Init().
 */
void uart_cmd_init(UART_HandleTypeDef* huart);

/*
 * Process any pending UART commands. Call this from the main loop.
 * Reads the cached MotorStatus to respond to READ_* commands.
 */
void uart_cmd_process(const MotorStatus* status);

/*
 * UART RX complete callback. Call this from HAL_UART_RxCpltCallback().
 * Accumulates received bytes and flags when a complete command is ready.
 */
void uart_cmd_rx_callback(UART_HandleTypeDef* huart);

/*
 * Send a proactive motor error notification to the host.
 * Call when the motor's error code changes to a non-zero value.
 * Sends: "!ERR:<code>:<description>\n"
 */
void uart_cmd_send_error(uint8_t error_code);

#endif /* UART_CMD_H */
