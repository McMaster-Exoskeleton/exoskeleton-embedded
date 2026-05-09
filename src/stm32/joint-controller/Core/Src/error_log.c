/*
 * error_log.c — see error_log.h for design notes.
 */

#include "error_log.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef *g_huart = NULL;
static CAN_HandleTypeDef  *g_hcan  = NULL;

volatile uint32_t log_can_rx_overflow_count = 0;
volatile uint32_t log_can_tx_drop_count     = 0;
volatile uint32_t log_can_err_isr_count     = 0;
volatile uint32_t log_i2c_err_count         = 0;
volatile uint32_t log_can_lec_counts[8]     = {0};

static uint32_t g_last_periodic_tick = 0;
static uint32_t g_last_rx_overflow   = 0;
static uint32_t g_last_tx_drop       = 0;
static uint32_t g_last_can_err       = 0;
static uint32_t g_last_i2c_err       = 0;
static uint32_t g_last_lec_counts[8] = {0};
static uint8_t  g_prev_bus_off       = 0;
static uint8_t  g_prev_err_passive   = 0;

static const char *const LEC_NAMES[8] = {
    "NoErr", "Stuff", "Form", "Ack", "BitR", "BitD", "CRC", "SwSet"
};

void log_init(UART_HandleTypeDef *huart, CAN_HandleTypeDef *hcan)
{
  g_huart = huart;
  g_hcan  = hcan;
  g_last_periodic_tick = HAL_GetTick();
}

void log_print(const char *msg)
{
  if (!g_huart || !msg) return;
  HAL_UART_Transmit(g_huart, (uint8_t *)msg, (uint16_t)strlen(msg), 100);
}

void log_printf(const char *fmt, ...)
{
  if (!g_huart) return;
  char buf[128];
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  if (n <= 0) return;
  if (n > (int)sizeof(buf) - 1) n = (int)sizeof(buf) - 1;
  HAL_UART_Transmit(g_huart, (uint8_t *)buf, (uint16_t)n, 100);
}

void log_periodic_check(void)
{
  if (!g_huart) return;

  uint32_t now = HAL_GetTick();
  if ((now - g_last_periodic_tick) < LOG_PERIODIC_INTERVAL_MS) return;
  g_last_periodic_tick = now;

  // Snapshot ISR-bumped counters. Reads are 32-bit aligned and atomic on
  // Cortex-M4, so no critical section needed for the snapshot itself.
  uint32_t rx_ov = log_can_rx_overflow_count;
  uint32_t tx_dr = log_can_tx_drop_count;
  uint32_t can_e = log_can_err_isr_count;
  uint32_t i2c_e = log_i2c_err_count;

  uint32_t d_rx = rx_ov - g_last_rx_overflow;
  uint32_t d_tx = tx_dr - g_last_tx_drop;
  uint32_t d_ce = can_e - g_last_can_err;
  uint32_t d_ie = i2c_e - g_last_i2c_err;

  if (d_rx) log_printf("[CAN] RX ring overflow +%lu (total %lu)\r\n",
                       (unsigned long)d_rx, (unsigned long)rx_ov);
  if (d_tx) log_printf("[CAN] TX dropped +%lu (total %lu)\r\n",
                       (unsigned long)d_tx, (unsigned long)tx_dr);
  if (d_ce) log_printf("[CAN] error ISR +%lu (total %lu)\r\n",
                       (unsigned long)d_ce, (unsigned long)can_e);
  if (d_ie) log_printf("[I2C] error ISR +%lu (total %lu)\r\n",
                       (unsigned long)d_ie, (unsigned long)i2c_e);

  g_last_rx_overflow = rx_ov;
  g_last_tx_drop     = tx_dr;
  g_last_can_err     = can_e;
  g_last_i2c_err     = i2c_e;

  if (g_hcan) {
    uint32_t esr = g_hcan->Instance->ESR;
    uint8_t  bus_off     = (esr & CAN_ESR_BOFF) ? 1 : 0;
    uint8_t  err_passive = (esr & CAN_ESR_EPVF) ? 1 : 0;

    if (bus_off != g_prev_bus_off)
      log_printf("[CAN] bus_off=%u (TEC=%lu REC=%lu)\r\n",
                 bus_off,
                 (unsigned long)((esr >> 16) & 0xFF),
                 (unsigned long)((esr >> 24) & 0xFF));
    if (err_passive != g_prev_err_passive)
      log_printf("[CAN] error_passive=%u (TEC=%lu REC=%lu)\r\n",
                 err_passive,
                 (unsigned long)((esr >> 16) & 0xFF),
                 (unsigned long)((esr >> 24) & 0xFF));

    g_prev_bus_off     = bus_off;
    g_prev_err_passive = err_passive;
  }

  // LEC histogram: build a single line of nonzero deltas. Skip indices 0
  // (NoErr) and 7 (Set-by-software, only set in test mode). One snprintf'd
  // line keeps it readable when several error types coexist.
  uint32_t lec_total_delta = 0;
  for (int i = 1; i <= 6; i++) {
    lec_total_delta += log_can_lec_counts[i] - g_last_lec_counts[i];
  }
  if (lec_total_delta > 0) {
    char line[160];
    int n = snprintf(line, sizeof(line), "[CAN] LEC(1s):");
    for (int i = 1; i <= 6; i++) {
      uint32_t delta = log_can_lec_counts[i] - g_last_lec_counts[i];
      if (delta > 0 && n < (int)sizeof(line) - 24) {
        n += snprintf(line + n, sizeof(line) - n, " %s=%lu",
                      LEC_NAMES[i], (unsigned long)delta);
      }
      g_last_lec_counts[i] = log_can_lec_counts[i];
    }
    if (n < (int)sizeof(line) - 3)
      n += snprintf(line + n, sizeof(line) - n, "\r\n");
    log_print(line);
  }
}
