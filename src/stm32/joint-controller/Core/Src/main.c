/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lsm6ds3tr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "can_common.h"
#include "can_imu.h"
#include "can_motor.h"
#include "can/ak70_9.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * Per-board configuration. Change before flashing each joint.
 *   Left Hip = 1, Right Hip = 2, Left Knee = 3, Right Knee = 4
 */
#define MY_NODE_ID          2
#define MY_MOTOR_CAN_ID     106

/* AK70-9 KV60: torque = Kt * gear_ratio * Iq -> Iq = torque / KT_EFFECTIVE */
#define AK70_9_KT           0.159f
#define AK70_9_GEAR_RATIO   9.0f
#define KT_EFFECTIVE        (AK70_9_KT * AK70_9_GEAR_RATIO)

#define CURRENT_REFRESH_MS  50      /* prevents VESC current-loop timeout */
#define CURRENT_LIMIT       5.0f    /* test-safe current clamp (A) */
#define IMU_PERIOD_MS       5       /* 200 Hz IMU send */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c3_rx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static MotorStatus motor_status      = {0};
static float      active_current     = 0.0f;
static uint8_t    motor_active       = 0;
static uint8_t    estop_active       = 0;
static uint32_t   last_refresh_tick  = 0;
static uint32_t   ext_rx_count       = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */
static void CAN_Send_IMU_Data(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float clampf(float val, float lo, float hi)
{
  if (val < lo) return lo;
  if (val > hi) return hi;
  return val;
}

static void debug_print(const char *msg)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), 50);
}

static void debug_printf(const char *fmt, ...)
{
  char buf[128];
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  if (n > 0) HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 50);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_I2C3_Init();

  /* USER CODE BEGIN 2 */
  if (!can_common_init(&hcan1, MY_NODE_ID))
  {
    debug_print("CAN INIT FAILED\r\n");
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    while (1) {}
  }
  can_set_motor_filter(MY_MOTOR_CAN_ID);
  debug_printf("CAN OK node=%u motor=%u\r\n",
               (unsigned)MY_NODE_ID, (unsigned)MY_MOTOR_CAN_ID);

  lsm6ds3tr_init_driver(&hi2c3);
  if (!lsm6ds3tr_check_connection())
  {
    debug_print("IMU NOT FOUND\r\n");
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    while (1) {}
  }
  lsm6ds3tr_calibrate();
  debug_print("READY\r\n");

  uint32_t last_imu_tick    = HAL_GetTick();
  uint32_t last_status_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN WHILE */

    /* ── Drain CAN RX ──
     * NO per-frame logging. At two nodes with VESC traffic, anything
     * faster than 1 Hz over UART blocks the main loop and breaks the
     * 50 ms motor refresh.
     */
    CanFrame rx_frame;
    while (can_recv(&rx_frame))
    {
      if (rx_frame.is_extended)
      {
        ext_rx_count++;
        motor_receive(&motor_status, rx_frame.data);
      }
      else
      {
        uint8_t msg_type = can_get_msg_type((uint16_t)rx_frame.id);

        if (msg_type == CAN_MSG_ESTOP)
        {
          active_current = 0.0f;
          motor_active   = 0;
          estop_active   = 1;
          comm_can_set_current(MY_MOTOR_CAN_ID, 0.0f);
        }
        else if (msg_type == CAN_MSG_TORQUE_CMD && !estop_active)
        {
          float torque_nm;
          if (can_parse_torque_cmd(&rx_frame, &torque_nm))
          {
            float current = torque_nm / KT_EFFECTIVE;
            active_current = clampf(current, -CURRENT_LIMIT, CURRENT_LIMIT);
            motor_active = 1;
            comm_can_set_current(MY_MOTOR_CAN_ID, active_current);
            last_refresh_tick = HAL_GetTick();
          }
        }
      }
    }

    /* ── Refresh: re-send current command every 50 ms to prevent VESC timeout ── */
    if (motor_active &&
        (HAL_GetTick() - last_refresh_tick >= CURRENT_REFRESH_MS))
    {
      comm_can_set_current(MY_MOTOR_CAN_ID, active_current);
      last_refresh_tick = HAL_GetTick();
    }

    /* ── 200 Hz IMU send + DMA read ── */
    if ((HAL_GetTick() - last_imu_tick) >= IMU_PERIOD_MS)
    {
      last_imu_tick = HAL_GetTick();
      CAN_Send_IMU_Data();
      lsm6ds3tr_init_dma_read();
    }

    /* ── 1 Hz status dump (mirrors torque-controller) ── */
    if (HAL_GetTick() - last_status_tick >= 1000)
    {
      debug_printf("[STATUS] active=%d estop=%d cmd=%.3fA motor_rx=%lu | "
                   "pos=%.1f spd=%.0f cur=%.2fA temp=%dC err=%d\r\n",
                   motor_active, estop_active, active_current,
                   (unsigned long)ext_rx_count,
                   motor_status.position, motor_status.speed,
                   motor_status.current, motor_status.temperature,
                   motor_status.error);
      last_status_tick = HAL_GetTick();
    }

    /* USER CODE END WHILE */
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief CAN1 Initialization Function
 */
static void MX_CAN1_Init(void)
{
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief I2C3 Initialization Function
 */
static void MX_I2C3_Init(void)
{
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief USART2 Initialization Function
 */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

/**
 * @brief GPIO Initialization Function
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/**
 * @brief Encode + transmit the latest IMU reading. Motor position rides
 *        along in the gyro frame's spare 2 bytes (DLC 8) for shared timestamp.
 */
static void CAN_Send_IMU_Data(void)
{
  LSM6DS3TR_Data_t *imu = lsm6ds3tr_get_data();

  if (imu->state != SENSOR_STATE_CONNECTED) return;
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) < 2) return;

  HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);
  float ax = imu->accel.filt_x;
  float ay = imu->accel.filt_y;
  float az = imu->accel.filt_z;
  float gx = imu->gyro.filt_x;
  float gy = imu->gyro.filt_y;
  float gz = imu->gyro.filt_z;
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

  can_send_imu_accel(MY_NODE_ID, ax, ay, az);
  can_send_imu_gyro(MY_NODE_ID, gx, gy, gz, motor_status.position);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
