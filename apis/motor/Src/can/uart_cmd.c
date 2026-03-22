/*
 * uart_cmd.c
 *
 * Text-based UART command protocol implementation.
 * Receives newline-terminated commands via interrupt-driven UART RX,
 * processes them in the main loop, and sends responses back to the host.
 *
 * Supports both read-only feedback commands and motor control commands.
 * All motor control values are clamped to test-safe limits defined in ak70_9.h.
 *
 * The VESC firmware on the AK70-9 has an internal command timeout (~1-2s).
 * To keep the motor running, uart_cmd_refresh_tick() re-sends the last
 * motor command every CMD_REFRESH_MS milliseconds until ESTOP or a new
 * command is received.
 *
 * NOTE: Float formatting with snprintf requires "-u _printf_float" and
 *       float parsing with sscanf requires "-u _scanf_float" linker flags.
 */

#include "can/uart_cmd.h"
#include "can/can_bus.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Internal state */
static UART_HandleTypeDef* g_huart = NULL;
static uint8_t rx_byte;
static char cmd_buf[UART_CMD_BUF_SIZE];
static uint8_t cmd_len = 0;
static volatile uint8_t cmd_ready = 0;

/* =========================================================================
 * Command refresh: re-send the last motor command periodically so the
 * VESC firmware's internal timeout doesn't kill the motor.
 * ========================================================================= */

#define CMD_REFRESH_MS 50

typedef enum {
    ACTIVE_CMD_NONE = 0,
    ACTIVE_CMD_DUTY,
    ACTIVE_CMD_CURRENT,
    ACTIVE_CMD_BRAKE,
    ACTIVE_CMD_RPM,
    ACTIVE_CMD_POS,
    ACTIVE_CMD_POS_SPD,
    ACTIVE_CMD_MIT,
} ActiveCmdType;

static ActiveCmdType active_cmd = ACTIVE_CMD_NONE;
static float    active_f[5];     /* float params (up to 5 for MIT) */
static int16_t  active_i[2];    /* int params (spd, acc for POS_SPD) */
static uint32_t last_refresh_tick = 0;

/* Re-send the currently active motor command */
static void resend_active_cmd(void) {
    switch (active_cmd) {
        case ACTIVE_CMD_DUTY:
            comm_can_set_duty(MOTOR_CAN_ID, active_f[0]);
            break;
        case ACTIVE_CMD_CURRENT:
            comm_can_set_current(MOTOR_CAN_ID, active_f[0]);
            break;
        case ACTIVE_CMD_BRAKE:
            comm_can_set_cb(MOTOR_CAN_ID, active_f[0]);
            break;
        case ACTIVE_CMD_RPM:
            comm_can_set_rpm(MOTOR_CAN_ID, active_f[0]);
            break;
        case ACTIVE_CMD_POS:
            comm_can_set_pos(MOTOR_CAN_ID, active_f[0]);
            break;
        case ACTIVE_CMD_POS_SPD:
            comm_can_set_pos_spd(MOTOR_CAN_ID, active_f[0], active_i[0], active_i[1]);
            break;
        case ACTIVE_CMD_MIT:
            pack_cmd(MOTOR_CAN_ID, active_f[0], active_f[1], active_f[2], active_f[3], active_f[4]);
            break;
        default:
            break;
    }
}

/* =========================================================================
 * Internal helpers
 * ========================================================================= */

/* Send a null-terminated string over UART (blocking) */
static void uart_send(const char* str) {
    HAL_UART_Transmit(g_huart, (uint8_t*)str, (uint16_t)strlen(str), HAL_MAX_DELAY);
}

/* Clamp a float to [-limit, +limit] */
static float clamp_sym(float val, float limit) {
    if (val < -limit) return -limit;
    if (val >  limit) return  limit;
    return val;
}

/* Clamp a float to [0, limit] */
static float clamp_pos(float val, float limit) {
    if (val < 0.0f)  return 0.0f;
    if (val > limit)  return limit;
    return val;
}

/* Clamp a float to [min_val, max_val] */
static float clamp_range(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

/*
 * Send a zero-duty CAN frame to stop the motor and clear the active command.
 */
static void motor_stop(void) {
    active_cmd = ACTIVE_CMD_NONE;
    comm_can_set_duty(MOTOR_CAN_ID, 0.0f);
}

/* =========================================================================
 * Motor control command handlers
 * ========================================================================= */

static void handle_estop(void) {
    motor_stop();
    uart_send("OK:ESTOP\n");
}

static void handle_set_duty(const char* args) {
    float duty;
    if (sscanf(args, "%f", &duty) != 1) {
        uart_send("ERR:PARSE:SET_DUTY\n");
        return;
    }
    duty = clamp_sym(duty, TEST_DUTY_MAX);
    comm_can_set_duty(MOTOR_CAN_ID, duty);

    if (duty != 0.0f) {
        active_cmd = ACTIVE_CMD_DUTY;
        active_f[0] = duty;
        last_refresh_tick = HAL_GetTick();
    } else {
        active_cmd = ACTIVE_CMD_NONE;
    }

    char resp[64];
    snprintf(resp, sizeof(resp), "OK:SET_DUTY:%.4f\n", duty);
    uart_send(resp);
}

static void handle_set_current(const char* args) {
    float current;
    if (sscanf(args, "%f", &current) != 1) {
        uart_send("ERR:PARSE:SET_CURRENT\n");
        return;
    }
    current = clamp_sym(current, TEST_CURRENT_MAX);
    comm_can_set_current(MOTOR_CAN_ID, current);

    if (current != 0.0f) {
        active_cmd = ACTIVE_CMD_CURRENT;
        active_f[0] = current;
        last_refresh_tick = HAL_GetTick();
    } else {
        active_cmd = ACTIVE_CMD_NONE;
    }

    char resp[64];
    snprintf(resp, sizeof(resp), "OK:SET_CURRENT:%.2f\n", current);
    uart_send(resp);
}

static void handle_set_brake(const char* args) {
    float current;
    if (sscanf(args, "%f", &current) != 1) {
        uart_send("ERR:PARSE:SET_BRAKE\n");
        return;
    }
    current = clamp_pos(current, TEST_BRAKE_CURRENT_MAX);
    comm_can_set_cb(MOTOR_CAN_ID, current);

    if (current != 0.0f) {
        active_cmd = ACTIVE_CMD_BRAKE;
        active_f[0] = current;
        last_refresh_tick = HAL_GetTick();
    } else {
        active_cmd = ACTIVE_CMD_NONE;
    }

    char resp[64];
    snprintf(resp, sizeof(resp), "OK:SET_BRAKE:%.2f\n", current);
    uart_send(resp);
}

static void handle_set_rpm(const char* args) {
    float rpm;
    if (sscanf(args, "%f", &rpm) != 1) {
        uart_send("ERR:PARSE:SET_RPM\n");
        return;
    }
    rpm = clamp_sym(rpm, TEST_RPM_MAX);
    comm_can_set_rpm(MOTOR_CAN_ID, rpm);

    if (rpm != 0.0f) {
        active_cmd = ACTIVE_CMD_RPM;
        active_f[0] = rpm;
        last_refresh_tick = HAL_GetTick();
    } else {
        active_cmd = ACTIVE_CMD_NONE;
    }

    char resp[64];
    snprintf(resp, sizeof(resp), "OK:SET_RPM:%.1f\n", rpm);
    uart_send(resp);
}

static void handle_set_pos(const char* args) {
    float pos;
    if (sscanf(args, "%f", &pos) != 1) {
        uart_send("ERR:PARSE:SET_POS\n");
        return;
    }
    pos = clamp_sym(pos, TEST_POS_DEG_MAX);
    comm_can_set_pos(MOTOR_CAN_ID, pos);

    active_cmd = ACTIVE_CMD_POS;
    active_f[0] = pos;
    last_refresh_tick = HAL_GetTick();

    char resp[64];
    snprintf(resp, sizeof(resp), "OK:SET_POS:%.2f\n", pos);
    uart_send(resp);
}

static void handle_set_origin(const char* args) {
    int mode;
    if (sscanf(args, "%d", &mode) != 1 || (mode != 0 && mode != 1)) {
        uart_send("ERR:PARSE:SET_ORIGIN\n");
        return;
    }
    comm_can_set_origin(MOTOR_CAN_ID, (uint8_t)mode);
    /* SET_ORIGIN is a one-shot command, no refresh needed */

    char resp[64];
    snprintf(resp, sizeof(resp), "OK:SET_ORIGIN:%d\n", mode);
    uart_send(resp);
}

static void handle_set_pos_spd(const char* args) {
    float pos;
    int spd, acc;
    if (sscanf(args, "%f %d %d", &pos, &spd, &acc) != 3) {
        uart_send("ERR:PARSE:SET_POS_SPD\n");
        return;
    }
    pos = clamp_sym(pos, TEST_POS_DEG_MAX);
    if (spd < -32768) spd = -32768;
    if (spd >  32767) spd =  32767;
    if (acc < 0)      acc = 0;
    if (acc > 32767)  acc = 32767;

    comm_can_set_pos_spd(MOTOR_CAN_ID, pos, (int16_t)spd, (int16_t)acc);

    active_cmd = ACTIVE_CMD_POS_SPD;
    active_f[0] = pos;
    active_i[0] = (int16_t)spd;
    active_i[1] = (int16_t)acc;
    last_refresh_tick = HAL_GetTick();

    char resp[96];
    snprintf(resp, sizeof(resp), "OK:SET_POS_SPD:%.2f,%d,%d\n", pos, spd, acc);
    uart_send(resp);
}

static void handle_set_mit(const char* args) {
    float p, v, kp, kd, t;
    if (sscanf(args, "%f %f %f %f %f", &p, &v, &kp, &kd, &t) != 5) {
        uart_send("ERR:PARSE:SET_MIT\n");
        return;
    }
    p  = clamp_sym(p,  TEST_MIT_P_MAX);
    v  = clamp_sym(v,  TEST_MIT_V_MAX);
    kp = clamp_range(kp, 0.0f, TEST_MIT_KP_MAX);
    kd = clamp_range(kd, 0.0f, TEST_MIT_KD_MAX);
    t  = clamp_sym(t,  TEST_MIT_T_MAX);

    pack_cmd(MOTOR_CAN_ID, p, v, kp, kd, t);

    active_cmd = ACTIVE_CMD_MIT;
    active_f[0] = p;
    active_f[1] = v;
    active_f[2] = kp;
    active_f[3] = kd;
    active_f[4] = t;
    last_refresh_tick = HAL_GetTick();

    char resp[128];
    snprintf(resp, sizeof(resp), "OK:SET_MIT:%.3f,%.3f,%.1f,%.2f,%.3f\n", p, v, kp, kd, t);
    uart_send(resp);
}

/* =========================================================================
 * Public API
 * ========================================================================= */

void uart_cmd_init(UART_HandleTypeDef* huart) {
    g_huart = huart;
    cmd_len = 0;
    cmd_ready = 0;
    active_cmd = ACTIVE_CMD_NONE;
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

void uart_cmd_error_callback(UART_HandleTypeDef* huart) {
    if (huart != g_huart) return;
    /*
     * Re-arm UART RX after any error (overrun, framing, noise, etc.).
     * Without this, a UART error permanently kills RX because the HAL
     * calls ErrorCallback instead of RxCpltCallback, so our rx_callback
     * never re-arms the interrupt.
     */
    HAL_UART_Receive_IT(g_huart, &rx_byte, 1);
}

void uart_cmd_process(const MotorStatus* status) {
    if (!cmd_ready) return;

    char resp[128];

    /* --- Read / utility commands --- */

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

    /* --- Motor control commands --- */

    } else if (strcmp(cmd_buf, "ESTOP") == 0) {
        handle_estop();

    } else if (strncmp(cmd_buf, "SET_DUTY ", 9) == 0) {
        handle_set_duty(cmd_buf + 9);

    } else if (strncmp(cmd_buf, "SET_CURRENT ", 12) == 0) {
        handle_set_current(cmd_buf + 12);

    } else if (strncmp(cmd_buf, "SET_BRAKE ", 10) == 0) {
        handle_set_brake(cmd_buf + 10);

    } else if (strncmp(cmd_buf, "SET_RPM ", 8) == 0) {
        handle_set_rpm(cmd_buf + 8);

    } else if (strncmp(cmd_buf, "SET_POS ", 8) == 0) {
        handle_set_pos(cmd_buf + 8);

    } else if (strncmp(cmd_buf, "SET_ORIGIN ", 11) == 0) {
        handle_set_origin(cmd_buf + 11);

    } else if (strncmp(cmd_buf, "SET_POS_SPD ", 12) == 0) {
        handle_set_pos_spd(cmd_buf + 12);

    } else if (strncmp(cmd_buf, "SET_MIT ", 8) == 0) {
        handle_set_mit(cmd_buf + 8);

    } else {
        snprintf(resp, sizeof(resp), "UNKNOWN_CMD:%s\n", cmd_buf);
        uart_send(resp);
    }

    /* Reset for the next command */
    cmd_len = 0;
    cmd_ready = 0;
}

void uart_cmd_refresh_tick(void) {
    if (active_cmd == ACTIVE_CMD_NONE) return;

    uint32_t now = HAL_GetTick();
    if ((now - last_refresh_tick) < CMD_REFRESH_MS) return;
    last_refresh_tick = now;

    resend_active_cmd();
}

void uart_cmd_send_error(uint8_t error_code) {
    if (error_code == 0 || !g_huart) return;
    char resp[128];
    snprintf(resp, sizeof(resp), "!ERR:%d:%s\n",
        (int)error_code, motor_error_to_string(error_code));
    uart_send(resp);
}
