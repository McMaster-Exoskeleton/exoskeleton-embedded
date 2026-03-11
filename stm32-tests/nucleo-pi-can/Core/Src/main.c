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
#include <lsm6ds3tr.h>
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "imu_buffer.h"
#include "can/can_bus.h"
#include "can/ak70_9.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_CAN_ID  104   /* CAN driver ID for the AK70-9 motor */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef  hcan1;
DMA_HandleTypeDef  hdma_i2c3_rx;
I2C_HandleTypeDef  hi2c3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t          rx_data[1];
uint8_t          rx_buffer[100];
uint8_t          rx_index = 0;
uint8_t          tx_buffer[256];
volatile uint8_t cmd_ready = 0;

/* Latest motor feedback — updated each loop from CAN RX ring buffer.
 * g_motor_position_deg is read by the I2C DMA callback (ISR) when it
 * pushes a new entry to the circular buffer, so it must be volatile. */
MotorStatus      g_motor_status;
volatile float   g_motor_position_deg = 0.0f;
static float     g_motor_target_current = 0.0f;  /* set via UART SET_CUR command */

/* VESC command refresh — re-send active command every 50ms to prevent
 * the VESC firmware's internal timeout from stopping the motor. */
#define CMD_REFRESH_MS 50
static uint32_t  last_refresh_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
static void process_command(void);
static void poll_motor_can(void);
void CAN_Send_IMU_Data(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();          // Must be called before MX_I2C3_Init
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  MX_CAN1_Init();

  /* USER CODE BEGIN 2 */

  /* Initialize the CAN bus driver (accept-all filter, RX interrupt, ring buffer).
   * This replaces the manual HAL_CAN_Start + filter setup. */
  can_bus_init(&hcan1);

  lsm6ds3tr_init_driver(&hi2c3);
  HAL_UART_Receive_IT(&huart2, rx_data, 1);

  // Blocking calibration (~0.3 s). Keep the sensor stationary.
  lsm6ds3tr_calibrate();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (cmd_ready)
    {
      cmd_ready = 0;
      process_command();
    }

    // Non-blocking DMA read; result is processed in HAL_I2C_MemRxCpltCallback
    // which also pushes to the circular buffer and updates imu_data.
    lsm6ds3tr_init_dma_read();

    // Re-send active motor command every CMD_REFRESH_MS to prevent
    // VESC internal timeout from stopping the motor.
    {
      uint32_t now = HAL_GetTick();
      if ((now - last_refresh_tick) >= CMD_REFRESH_MS) {
        last_refresh_tick = now;
        comm_can_set_current(MOTOR_CAN_ID, g_motor_target_current);
      }
    }

    // Drain CAN RX ring buffer and parse any motor feedback messages.
    poll_motor_can();

    // Broadcast latest accel data on CAN standard ID 0x123.
    CAN_Send_IMU_Data();

    HAL_Delay(20);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
  * @brief DMA Initialization Function
  * @retval None
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init — DMA1_Stream1 (I2C3 RX) */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;                 /* 42 MHz / 3 / (1+10+3) = 1 Mbps */
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;       /* sample point at 78.6% */
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
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
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
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Drain CAN RX ring buffer and parse motor feedback messages.
 *
 * The AK70-9 motor responds with an extended-ID CAN frame where
 * bits [7:0] = motor driver ID. We check for our motor's ID and
 * parse the 8-byte feedback payload.
 */
static void poll_motor_can(void)
{
  CanFrame rx_frame;
  while (can_bus_recv(&rx_frame))
  {
    /* Motor feedback arrives as an extended CAN frame.
     * The driver ID is in bits [7:0] of the extended identifier. */
    if (rx_frame.is_extended && (rx_frame.id & 0xFF) == MOTOR_CAN_ID)
    {
      motor_receive(&g_motor_status, rx_frame.data);
      g_motor_position_deg = g_motor_status.position;
    }
  }
}

/**
 * @brief Encode and transmit the latest accel reading on CAN ID 0x123.
 *
 * Packs AX, AY, AZ as little-endian signed 16-bit integers scaled by 100.
 * Skips silently if the IMU is not connected or CAN mailboxes are full.
 */
void CAN_Send_IMU_Data(void)
{
  LSM6DS3TR_Data_t *imu = lsm6ds3tr_get_data();

  if (imu->state != SENSOR_STATE_CONNECTED) return;

  int16_t ax = (int16_t)(imu->accel.filt_x * 100);
  int16_t ay = (int16_t)(imu->accel.filt_y * 100);
  int16_t az = (int16_t)(imu->accel.filt_z * 100);

  uint8_t data[6];
  data[0] = ax & 0xFF;
  data[1] = (ax >> 8) & 0xFF;
  data[2] = ay & 0xFF;
  data[3] = (ay >> 8) & 0xFF;
  data[4] = az & 0xFF;
  data[5] = (az >> 8) & 0xFF;

  can_bus_send_std(0x123, data, 6);
}

/**
 * @brief UART RX interrupt callback — accumulates bytes into rx_buffer.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  UNUSED(huart);

  if (rx_data[0] == '\n' || rx_data[0] == '\r')
  {
    if (rx_index > 0)
    {
      rx_buffer[rx_index] = '\0';
      cmd_ready = 1;
      rx_index  = 0;
    }
  }
  else if (rx_index < sizeof(rx_buffer) - 1)
  {
    rx_buffer[rx_index++] = rx_data[0];
  }

  HAL_UART_Receive_IT(&huart2, rx_data, 1);
}

/**
 * @brief Process a complete UART command stored in rx_buffer.
 *
 * Commands:
 *   READ       - Latest filtered accel + gyro + motor position
 *   READLATEST - Latest reading from the circular buffer
 *   READALL    - All stored readings, oldest first
 *   STATUS     - IMU connection state + motor error
 *   REGISTER   - I2C device address
 *   CONFIG     - Gyro and accel control register values
 *   POWER      - Power configuration register value
 */
static void process_command(void)
{
  LSM6DS3TR_Data_t *imu = lsm6ds3tr_get_data();
  uint16_t len = 0;

  if (strcmp((char*)rx_buffer, "READ") == 0)
  {
    len = sprintf((char*)tx_buffer,
        "T:%lu MP:%.2f AX:%.2f AY:%.2f AZ:%.2f GX:%.2f GY:%.2f GZ:%.2f\r\n",
        (unsigned long)HAL_GetTick(),
        (double)g_motor_position_deg,
        imu->accel.filt_x, imu->accel.filt_y, imu->accel.filt_z,
        imu->gyro.filt_x,  imu->gyro.filt_y,  imu->gyro.filt_z);
    HAL_UART_Transmit(&huart2, tx_buffer, len, 100);
  }
  else if (strcmp((char*)rx_buffer, "READLATEST") == 0)
  {
    SensorReading reading;

    HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);
    int ok = imu_buffer_get_latest(&reading);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

    if (ok)
    {
      len = sprintf((char*)tx_buffer,
          "LATEST T:%lu MP:%.2f AX:%.2f AY:%.2f AZ:%.2f GX:%.2f GY:%.2f GZ:%.2f\r\n",
          (unsigned long)reading.timestamp_ms,
          reading.motor_position_deg,
          reading.ax, reading.ay, reading.az,
          reading.gx, reading.gy, reading.gz);
    }
    else
    {
      len = sprintf((char*)tx_buffer, "LATEST:EMPTY\r\n");
    }
    HAL_UART_Transmit(&huart2, tx_buffer, len, 200);
  }
  else if (strcmp((char*)rx_buffer, "READALL") == 0)
  {
    SensorReading snapshot[IMU_BUFFER_CAPACITY];
    size_t count;

    HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);
    count = imu_buffer_get_all(snapshot);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

    if (count == 0)
    {
      len = sprintf((char*)tx_buffer, "ALL:EMPTY\r\n");
      HAL_UART_Transmit(&huart2, tx_buffer, len, 100);
    }
    else
    {
      len = sprintf((char*)tx_buffer, "ALL:COUNT=%u\r\n", (unsigned)count);
      HAL_UART_Transmit(&huart2, tx_buffer, len, 100);

      for (size_t i = 0; i < count; ++i)
      {
        len = sprintf((char*)tx_buffer,
            "[%u] T:%lu MP:%.2f AX:%.2f AY:%.2f AZ:%.2f GX:%.2f GY:%.2f GZ:%.2f\r\n",
            (unsigned)i,
            (unsigned long)snapshot[i].timestamp_ms,
            snapshot[i].motor_position_deg,
            snapshot[i].ax, snapshot[i].ay, snapshot[i].az,
            snapshot[i].gx, snapshot[i].gy, snapshot[i].gz);
        HAL_UART_Transmit(&huart2, tx_buffer, len, 200);
      }
    }
  }
  else if (strcmp((char*)rx_buffer, "STATUS") == 0)
  {
    lsm6ds3tr_check_connection();
    const char *imu_state = (imu->state == SENSOR_STATE_CONNECTED) ? "CONNECTED" : "LOST";
    const char *motor_err = motor_error_to_string(g_motor_status.error);
    len = sprintf((char*)tx_buffer,
        "IMU:%s MOTOR_POS:%.2f MOTOR_ERR:%s\r\n",
        imu_state, g_motor_status.position, motor_err);
    HAL_UART_Transmit(&huart2, tx_buffer, len, 100);
  }
  else if (strcmp((char*)rx_buffer, "REGISTER") == 0)
  {
    len = sprintf((char*)tx_buffer, "REGISTER:0x%02X\r\n", DEVICE_ADDRESS);
    HAL_UART_Transmit(&huart2, tx_buffer, len, 100);
  }
  else if (strcmp((char*)rx_buffer, "CONFIG") == 0)
  {
    len = sprintf((char*)tx_buffer,
        "GYRO_CFG:0x%02X ACCEL_CFG:0x%02X\r\n",
        imu->gyro_config, imu->accel_config);
    HAL_UART_Transmit(&huart2, tx_buffer, len, 100);
  }
  else if (strcmp((char*)rx_buffer, "POWER") == 0)
  {
    len = sprintf((char*)tx_buffer, "POWER_CFG:0x%02X\r\n", imu->power_config);
    HAL_UART_Transmit(&huart2, tx_buffer, len, 100);
  }
  else if (strcmp((char*)rx_buffer, "SET_ORIGIN") == 0)
  {
    comm_can_set_origin(MOTOR_CAN_ID, 1);
    g_motor_position_deg = 0.0f;
    len = sprintf((char*)tx_buffer, "OK:SET_ORIGIN\r\n");
    HAL_UART_Transmit(&huart2, tx_buffer, len, 100);
  }
  else if (strncmp((char*)rx_buffer, "SET_CUR:", 8) == 0)
  {
    float requested = (float)atof((char*)rx_buffer + 8);
    /* Clamp to safe range: -5.0 to +5.0 A */
    if (requested > 5.0f)  requested = 5.0f;
    if (requested < -5.0f) requested = -5.0f;
    g_motor_target_current = requested;
    /* Send immediately and reset refresh tick */
    comm_can_set_current(MOTOR_CAN_ID, g_motor_target_current);
    last_refresh_tick = HAL_GetTick();
    len = sprintf((char*)tx_buffer, "OK:CUR=%.2f\r\n", (double)g_motor_target_current);
    HAL_UART_Transmit(&huart2, tx_buffer, len, 100);
  }
  else
  {
    len = sprintf((char*)tx_buffer, "ERR:UNKNOWN_CMD\r\n");
    HAL_UART_Transmit(&huart2, tx_buffer, len, 100);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
