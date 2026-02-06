/*
 * uart_cmd.c
 *
 * Text-based UART command protocol implementation.
 * Receives newline-terminated commands via interrupt-driven UART RX,
 * processes them in the main loop, and sends responses back to the host.
 */

#include "can/uart_cmd.h"
#include <string.h>
#include <stdio.h>

/* Internal state */
static UART_HandleTypeDef* g_huart = NULL;
static uint8_t rx_byte;
static char cmd_buf[UART_CMD_BUF_SIZE];
static uint8_t cmd_len = 0;
static volatile uint8_t cmd_ready = 0;

/* Send a null-terminated string over UART (blocking) */
static void uart_send(const char* str) {
    HAL_UART_Transmit(g_huart, (uint8_t*)str, (uint16_t)strlen(str), HAL_MAX_DELAY);
}

void uart_cmd_init(UART_HandleTypeDef* huart) {
    g_huart = huart;
    cmd_len = 0;
    cmd_ready = 0;
    /* Start receiving the first byte via interrupt */
    HAL_UART_Receive_IT(g_huart, &rx_byte, 1);
}

void uart_cmd_rx_callback(UART_HandleTypeDef* huart) {
    if (huart != g_huart) return;

    if (rx_byte == '\n' || rx_byte == '\r') {
        /* End of command - null-terminate and flag as ready */
        if (cmd_len > 0) {
            cmd_buf[cmd_len] = '\0';
            cmd_ready = 1;
        }
    } else if (cmd_len < UART_CMD_BUF_SIZE - 1) {
        /* Accumulate character into command buffer */
        cmd_buf[cmd_len++] = (char)rx_byte;
    }

    /* Re-arm for the next byte */
    HAL_UART_Receive_IT(g_huart, &rx_byte, 1);
}

void uart_cmd_process(const MotorStatus* status) {
    if (!cmd_ready) return;

    char resp[128];

    if (strcmp(cmd_buf, "PING") == 0) {
        uart_send("PONG\n");

    } else if (strcmp(cmd_buf, "READ_ALL") == 0) {
        snprintf(resp, sizeof(resp),
            "ALL:POS=%.2f,SPD=%.1f,CUR=%.2f,TEMP=%d,ERR=%d\n",
            status->position, status->speed, status->current,
            (int)status->temperature, (int)status->error);
        uart_send(resp);

    } else if (strcmp(cmd_buf, "READ_POS") == 0) {
        snprintf(resp, sizeof(resp), "POS:%.2f\n", status->position);
        uart_send(resp);

    } else if (strcmp(cmd_buf, "READ_SPD") == 0) {
        snprintf(resp, sizeof(resp), "SPD:%.1f\n", status->speed);
        uart_send(resp);

    } else if (strcmp(cmd_buf, "READ_CUR") == 0) {
        snprintf(resp, sizeof(resp), "CUR:%.2f\n", status->current);
        uart_send(resp);

    } else if (strcmp(cmd_buf, "READ_TEMP") == 0) {
        snprintf(resp, sizeof(resp), "TEMP:%d\n", (int)status->temperature);
        uart_send(resp);

    } else if (strcmp(cmd_buf, "READ_ERR") == 0) {
        snprintf(resp, sizeof(resp), "ERR:%d:%s\n",
            (int)status->error, motor_error_to_string(status->error));
        uart_send(resp);

    } else {
        snprintf(resp, sizeof(resp), "UNKNOWN_CMD:%s\n", cmd_buf);
        uart_send(resp);
    }

    /* Reset for the next command */
    cmd_len = 0;
    cmd_ready = 0;
}

void uart_cmd_send_error(uint8_t error_code) {
    if (error_code == 0 || !g_huart) return;
    char resp[128];
    snprintf(resp, sizeof(resp), "!ERR:%d:%s\n",
        (int)error_code, motor_error_to_string(error_code));
    uart_send(resp);
}
