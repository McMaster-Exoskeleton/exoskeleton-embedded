/*
 * uart_cmd.h
 *
 * Text-based UART command protocol for testing the AK70-9 motor API.
 *
 * Commands are newline-terminated ASCII strings sent from a host (Python script).
 * Responses are newline-terminated ASCII strings sent back to the host.
 *
 * Read commands:
 *   PING       -> PONG
 *   READ_ALL   -> ALL:POS=<deg>,SPD=<rpm>,CUR=<A>,TEMP=<C>,ERR=<code>
 *   READ_POS   -> POS:<degrees>
 *   READ_SPD   -> SPD:<erpm>
 *   READ_CUR   -> CUR:<amps>
 *   READ_TEMP  -> TEMP:<celsius>
 *   READ_ERR   -> ERR:<code>:<description>
 *
 * Motor control commands (all values clamped to test-safe limits):
 *   ESTOP                          -> OK:ESTOP
 *   SET_DUTY <duty>                -> OK:SET_DUTY:<duty>
 *   SET_CURRENT <amps>             -> OK:SET_CURRENT:<amps>
 *   SET_BRAKE <amps>               -> OK:SET_BRAKE:<amps>
 *   SET_RPM <rpm>                  -> OK:SET_RPM:<rpm>
 *   SET_POS <degrees>              -> OK:SET_POS:<degrees>
 *   SET_ORIGIN <0|1>               -> OK:SET_ORIGIN:<mode>
 *   SET_POS_SPD <deg> <spd> <acc>  -> OK:SET_POS_SPD:<deg>,<spd>,<acc>
 *   SET_MIT <p> <v> <kp> <kd> <t>  -> OK:SET_MIT:<p>,<v>,<kp>,<kd>,<t>
 *
 * Error responses:
 *   ERR:PARSE:<command>            -- could not parse parameters
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

#define UART_CMD_BUF_SIZE 128

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

/*
 * UART error recovery callback. Call from HAL_UART_ErrorCallback().
 * Re-arms UART RX after overrun, framing, or noise errors so the
 * command handler keeps receiving.
 */
void uart_cmd_error_callback(UART_HandleTypeDef* huart);

/*
 * Command refresh tick. Call from the main loop.
 * Re-sends the last motor command every ~50ms to keep the VESC firmware's
 * internal timeout from stopping the motor. Runs until ESTOP or a new
 * command is received.
 */
void uart_cmd_refresh_tick(void);

#endif /* UART_CMD_H */
