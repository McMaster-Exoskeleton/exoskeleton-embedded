/*
 * error_log.h
 *
 * Real-time fault logging over USART for the joint controller.
 *
 * Design notes:
 * - Edge-triggered: most events log once on state change, not per occurrence.
 *   At 500 Hz IMU and 20 Hz motor refresh, level-triggered logging would
 *   saturate the 115200 baud UART and stall the main loop.
 * - ISR-safe paths just bump a volatile counter. The main loop calls
 *   log_periodic_check() which drains the counters into one line per
 *   LOG_PERIODIC_INTERVAL_MS, and only when a delta is non-zero.
 * - Single-bit CAN errors and momentary mailbox-full are normal on a real
 *   bus; we log threshold crossings (bus-off, error-passive) and aggregate
 *   counts, not raw error-counter increments.
 */

#ifndef ERROR_LOG_H
#define ERROR_LOG_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define LOG_PERIODIC_INTERVAL_MS  1000

/*
 * Bind UART for human-readable lines and CAN for ESR introspection.
 * Safe to call once at boot. After this, log_print/log_printf are usable.
 */
void log_init(UART_HandleTypeDef *huart, CAN_HandleTypeDef *hcan);

/* Print a fixed string. Blocks on UART up to 100 ms. No-op if uninitialized. */
void log_print(const char *msg);

/* printf-style. Truncates at 128 chars. Blocks on UART. */
void log_printf(const char *fmt, ...);

/*
 * ISR-safe event counters. Bump from any context; log_periodic_check()
 * drains the deltas into a single line at most once per
 * LOG_PERIODIC_INTERVAL_MS.
 */
extern volatile uint32_t log_can_rx_overflow_count;
extern volatile uint32_t log_can_tx_drop_count;
extern volatile uint32_t log_can_err_isr_count;
extern volatile uint32_t log_i2c_err_count;

/*
 * LEC (Last Error Code) histogram, indexed 1-6 per the bxCAN ESR layout:
 *   1=Stuff, 2=Form, 3=Ack, 4=BitRecessive, 5=BitDominant, 6=CRC.
 * Index 0 (NoError) and 7 (Set-by-software) are unused. Populated from
 * HAL_CAN_ErrorCallback by reading hcan->ErrorCode bits and drained at 1 Hz.
 *
 * The dominant LEC type points at the root cause:
 *   Stuff/BitR/BitD heavy -> bitrate mismatch or noise/termination
 *   CRC heavy             -> noise / signal integrity
 *   Ack heavy             -> no other node ACKing your frames (lone talker)
 *   Form heavy            -> malformed frames somewhere on the bus
 */
extern volatile uint32_t log_can_lec_counts[8];

/*
 * Cheap to call every loop iteration. Internally throttles to 1 Hz and
 * only emits a line when something non-zero or state-changing occurred.
 * Reads CAN ESR for bus-off / error-passive edge transitions.
 */
void log_periodic_check(void);

#endif /* ERROR_LOG_H */
