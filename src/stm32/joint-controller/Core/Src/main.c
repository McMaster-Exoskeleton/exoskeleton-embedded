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
#include "imu_buffer.h"
#include "can_common.h"
#include "can_imu.h"
#include "can_motor.h"
#include "can_system.h"
#include "can/ak70_9.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MY_NODE_ID   CAN_NODE_RIGHT_HIP   // change per board
#define MY_MOTOR_CAN_ID     105        // default 104

/*
 * ── Torque-to-Current Conversion ──
 * AK70-9 KV60 datasheet:
 *   Kt = 0.159 Nm/A (motor shaft torque constant)
 *   Gear ratio = 9:1
 *   Output torque = Kt * gear_ratio * Iq
 *   Iq = desired_torque / (Kt * gear_ratio)
 */
#define AK70_9_KT          0.159f
#define AK70_9_GEAR_RATIO   9.0f
#define KT_EFFECTIVE        (AK70_9_KT * AK70_9_GEAR_RATIO)  /* 1.431 Nm/A */

#define CURRENT_REFRESH_MS  50    /* Re-send interval to prevent AK70-9 timeout */
#define CURRENT_LIMIT       5.0f  //Amps

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
// UART variables
uint8_t             rx_data[1];
uint8_t         rx_buffer[100];
uint8_t           rx_index = 0;
uint8_t         tx_buffer[256];
volatile uint8_t cmd_ready = 0;


//motor variables
static MotorStatus g_motor_status       = {0};
static float       g_active_current     = 0.0f;
static uint8_t     g_motor_active       = 0;
static uint32_t    g_last_refresh_tick  = 0;
static uint32_t    g_last_status_tick   = 0;
static uint32_t    g_motor_rx_count     = 0;


/* TX drop counter — incremented when CAN mailboxes are full */
static uint32_t g_can_tx_dropped = 0;


volatile uint8_t g_estop_active = 0;
volatile uint8_t g_estop_reason = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

// IMU TX
static void CAN_Send_IMU_Data(void);

//UART
static void process_command(void);


// CAN RX processing + handlers
static void CAN_Poll_Incoming(void);
static void handle_estop(uint8_t reason);
static void handle_torque_cmd(float torque_nm);
static void motor_zero(void);


/* Debug helpers */
static void debug_printf(const char *fmt, ...);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static float clampf(float val, float lo, float hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

static void debug_printf(const char *fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), 50);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void){
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_I2C3_Init();

  /* USER CODE BEGIN 2 */
  /*
   * can_common_init() configures all hardware filters, starts the CAN
   * peripheral, enables both FIFO IRQs, and activates notifications.
   */

  debug_printf("before pre-can delay\r\n");
  HAL_Delay(150);
  debug_printf("after pre-can delay\r\n");

  	if (can_common_init(&hcan1, MY_NODE_ID, MY_MOTOR_CAN_ID)) {
		debug_printf("CAN OK node=%d motor_id=%d\r\n", MY_NODE_ID, MY_MOTOR_CAN_ID);
//		HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
//		HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);		/*Three quick blinks = CAN init success */
//		debug_printf("Before delay test, tick=%lu primask=%lu\r\n",
//		             HAL_GetTick(),
//		             __get_PRIMASK());
//
//		__enable_irq();
//
//		debug_printf("After enable irq, tick=%lu primask=%lu\r\n",
//		             HAL_GetTick(),
//		             __get_PRIMASK());
//		for (int i = 0; i < 3; i++) {
//		        debug_printf("In for loop of 3 blinks\r\n");
//		        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//		        debug_printf("after first toggle before delay\r\n");
//		        HAL_Delay(150);
//		        debug_printf("after first toggle and delay\r\n");
//		        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//		        HAL_Delay(150);
//		        debug_printf("after second toggle\r\n");
//		    }
	}else{
		debug_printf("CAN INIT FAILED\r\n");
		Error_Handler();
	}
  	debug_printf("before lsm6ds3tr init\r\n");
  	lsm6ds3tr_init_driver(&hi2c3);
  	debug_printf("after lsm6ds3tr init\r\n");
  	HAL_UART_Receive_IT(&huart2, rx_data, 1);
  	debug_printf("after hal_uart_receive_it \r\n");

  // Blocking calibration (~0.3 s). Keep the sensor stationary.
  	debug_printf("before calibrate\r\n");
  	lsm6ds3tr_calibrate();
  	debug_printf("after calibrate\r\n");

  uint32_t last_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN 3 */
//	  debug_printf("in while loop\r\n");
    if (cmd_ready){
      cmd_ready = 0;
//      debug_printf("about to process command\r\n");
      process_command();
//      debug_printf("command processed\r\n");
    }

     /*
     * CAN TX: send IMU data at 500 Hz (every 2 ms).
     * Suppressed during ESTOP so the Pi sees the data stream stop —
     * this is a secondary signal that something is wrong, on top of
     * the motor_status acknowledgement sent by handle_estop().
     */
    if (!g_estop_active && (HAL_GetTick() - last_tick) >= 2){
//    	debug_printf("about to send imu data\r\n");
      last_tick = HAL_GetTick();
      CAN_Send_IMU_Data();
      lsm6ds3tr_init_dma_read();
//      debug_printf("imu data sent\r\n");
    }

    /*
     * CAN RX: drain one frame per loop iteration.
     * One frame per iteration keeps this from blocking UART and IMU
     * work during a burst of CAN traffic. At 500 Hz TX and low bus
     * load, the ring buffer never accumulates more than a few frames.
     */
//    debug_printf("BEFORE can poll incoming\r\n");
    CAN_Poll_Incoming();
//    debug_printf("AFTER can poll incoming\r\n");


    /*
     * ── Motor current refresh every 50 ms ──
     * Re-sends the last commanded current to prevent the AK70-9 watchdog
     * from cutting power. Only runs when motor is active and ESTOP is clear.
     */
    if (g_motor_active && !g_estop_active &&
        (HAL_GetTick() - g_last_refresh_tick) >= CURRENT_REFRESH_MS)
    {
      comm_can_set_current(MY_MOTOR_CAN_ID, g_active_current);
      g_last_refresh_tick = HAL_GetTick();
    }
 

    // Periodic status dump every 1s
    if (HAL_GetTick() - g_last_status_tick >= 10000){
    	debug_printf("fifo0_cb=%lu\r\n", g_fifo0_cb_count);
    	debug_printf("[FIFO0] cb=%lu ok=%lu fail=%lu fill=%lu id=0x%lX ext=%u dlc=%u\r\n",
    	             g_fifo0_cb_count,
    	             g_fifo0_get_ok,
    	             g_fifo0_get_fail,
    	             g_fifo0_fill_at_entry,
    	             g_fifo0_last_id,
    	             g_fifo0_last_ext,
    	             g_fifo0_last_dlc);
      debug_printf("[STATUS] estop=%d motor_active=%d cmd=%.3fA rx=%lu | "
                   "pos=%.1f spd=%.0f cur=%.2fA temp=%dC err=%d(%s)\r\n",
                   g_estop_active, g_motor_active, g_active_current,
                   g_motor_rx_count,
                   g_motor_status.position, g_motor_status.speed,
                   g_motor_status.current,  g_motor_status.temperature,
                   g_motor_status.error,
                   motor_error_to_string(g_motor_status.error));
      g_last_status_tick = HAL_GetTick();
    }

//    if (g_can_recover_requested)
//    {
//        g_can_recover_requested = 0;
//
//        HAL_CAN_Stop(&hcan1);
//        HAL_Delay(100);
//        HAL_CAN_Start(&hcan1);
//
//        HAL_CAN_ActivateNotification(&hcan1,
//            CAN_IT_RX_FIFO0_MSG_PENDING |
//            CAN_IT_RX_FIFO1_MSG_PENDING |
//            CAN_IT_ERROR |
//            CAN_IT_BUSOFF |
//            CAN_IT_LAST_ERROR_CODE);
//    }


      /* USER CODE END 3 */
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
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
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

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2) return;

  uint32_t err = HAL_UART_GetError(huart);

  rx_index = 0;
  rx_buffer[0] = '\0';

  // Restart RX after error
  HAL_UART_Receive_IT(&huart2, rx_data, 1);

  // Optional debug indicator
  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

  (void)err;
}

/**
 * @brief Transmit latest IMU accel + gyro frames to Pi via CAN
 *
 * Frame IDs are built by CAN_BUILD_ID() inside can_send_imu_accel() and
 * can_send_imu_gyro(). Payloads are little-endian int16 values scaled by 100.
 * Called every loop cycle; returns immediately if IMU is not connected.
 */
static void CAN_Send_IMU_Data(void)
{
  LSM6DS3TR_Data_t *imu = lsm6ds3tr_get_data();
  if (imu->state != SENSOR_STATE_CONNECTED) return;

  int accel_ok = can_send_imu_accel(MY_NODE_ID,
                                    imu->accel.filt_x,
                                    imu->accel.filt_y,
                                    imu->accel.filt_z);
  int gyro_ok = can_send_imu_gyro(MY_NODE_ID,
                                  imu->gyro.filt_x,
                                  imu->gyro.filt_y,
                                  imu->gyro.filt_z);

  if (!accel_ok) g_can_tx_dropped++;
  if (!gyro_ok)  g_can_tx_dropped++;
  if (!accel_ok || !gyro_ok)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // Transmission failed
  }
}

/**
 * @brief Drain one frame from the ring buffer and route it to the right handler.
 *
 * Called every main loop iteration. The ISR (HAL_CAN_RxFifo0MsgPendingCallback
 * in can_common.c) already pushed the frame into g_rxq; we just pop and dispatch.
 *
 * Priority order:
 *   1. ESTOP      — checked first, always acted on
 *   2. TORQUE_CMD — only accepted when ESTOP is not active
 */
static void CAN_Poll_Incoming(void)
{
  CanFrame frame;
  if (!can_recv(&frame)) return;   /* ring buffer empty */

/* ── Motor feedback (extended CAN frame from AK70-9) ── */
  if (frame.is_extended){
    g_motor_rx_count++;
    motor_receive(&g_motor_status, frame.data);
    debug_printf("MOTOR pos=%.1f spd=%.0f cur=%.2f temp=%d err=%d(%s)\r\n",
                 g_motor_status.position, g_motor_status.speed,
                 g_motor_status.current,  g_motor_status.temperature,
                 g_motor_status.error,
                 motor_error_to_string(g_motor_status.error));
    // If the motor driver is reporting a fault, zero it immediately.
    if (g_motor_status.error != MOTOR_ERROR_NONE && !g_estop_active){
            debug_printf("FAULT: %s — zeroing motor\r\n",
                         motor_error_to_string(g_motor_status.error));
            handle_estop(g_motor_status.error);
        }
    return;
  }

  /* ── Standard frames: check message type ── */
  uint8_t msg_type = can_get_msg_type((uint16_t)frame.id);


  /* 1. ESTOP */
  uint8_t estop_reason = 0;
  if (can_parse_estop(&frame, &estop_reason))
  {
    handle_estop(estop_reason);
    return;
  }

  /*
   * 2. TORQUE_CMD
   *
   * Silently discarded during ESTOP — motor stays at zero until board reset.
   * Hardware filter in can_common_init() already verified this frame is
   * addressed to MY_NODE_ID, so no destination check needed here.
   */
  float torque_nm = 0.0f;
  if (msg_type == CAN_MSG_TORQUE_CMD && !g_estop_active)
  {
    if (can_parse_torque_cmd(&frame, &torque_nm))
    {
      handle_torque_cmd(torque_nm);
    }
    return;
  }

  /* Any other frame that passed the hardware filter lands here.
   * Add more handlers above this line as the protocol grows.
   */
}

static void handle_estop(uint8_t reason)
{
  g_estop_active = 1;
  g_estop_reason = reason;

  /* Zero motor */
  motor_zero();
  g_motor_active = 0;
  g_active_current = 0.0f;

  /* Acknowledge back to Pi */
  can_send_motor_status(MY_NODE_ID, 0.0f, 0.0f, 0.0f, 0, reason);

  /* LED solid on — distinct from the TX-error blink */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

  debug_printf("ESTOP reason=0x%02X\r\n", reason);
}

/**
 * @brief Handle an incoming TORQUE_CMD frame from the Pi.
 *
 * Converts torque (N·m) to current (A) using the AK70-9 effective torque
 * constant, clamps to CURRENT_LIMIT, then sends to the motor driver via
 * comm_can_set_current() from ak70_9.c.
 *
 * Also resets the refresh timer so the motor doesn't time out.
 */
static void handle_torque_cmd(float torque_nm)
{
  /*
   * Torque → current conversion using AK70-9 datasheet values.
   * KT_EFFECTIVE = Kt * gear_ratio = 0.159 * 9 = 1.431 N·m/A
   */
  float current = torque_nm / KT_EFFECTIVE;

  /* Clamp to test-safe limit before sending anything to the motor */
  g_active_current = clampf(current, -CURRENT_LIMIT, CURRENT_LIMIT);

  /* Send to AK70-9 driver via extended CAN (ak70_9.c handles the frame format) */
  comm_can_set_current(MY_MOTOR_CAN_ID, g_active_current);

  g_motor_active = 1;
  g_last_refresh_tick = HAL_GetTick();

  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  debug_printf("torque=%.3f Nm -> current=%.3f A\r\n", torque_nm, g_active_current);
}

static void motor_zero(void)
{
  for (int attempt = 0; attempt < 3; attempt++)
  {
    comm_can_set_current(MY_MOTOR_CAN_ID, 0.0f);
    HAL_Delay(1);
  }
}


/**
 * @brief UART RX interrupt callback — accumulates bytes into rx_buffer.
 *
 * One byte arrives per interrupt. A newline / carriage-return terminates the
 * command and sets cmd_ready so process_command() runs next loop cycle.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance != USART2) return;

	char c = (char)rx_data[0];

	if (c == '\n' || c == '\r'){
		if (rx_index > 0){
			rx_buffer[rx_index] = '\0';
			cmd_ready = 1;
			rx_index = 0;
		}
	}
	else if (rx_index < sizeof(rx_buffer) - 1){
		rx_buffer[rx_index++] = (uint8_t)c;
	}else{
		rx_index = 0;
		rx_buffer[0] = '\0';
	}
	HAL_UART_Receive_IT(&huart2, rx_data, 1);
}
/**
 * @brief Process a complete UART command stored in rx_buffer.
 *
 * IMU commands:
 *   READ        - Latest filtered accel + gyro
 *   READLATEST  - Latest reading from the circular buffer
 *   STATUS      - IMU connection state
 *   REGISTER    - I2C device address
 *   CONFIG      - Gyro and accel control register values
 *   POWER       - Power configuration register value
 *
 * CAN diagnostic commands:
 *   CANERR      - CAN peripheral error counters (TEC/REC)
 *   CANDROP     - Cumulative TX drop count since boot
 *
 * Motor commands:
 *   MOTORSTATUS - Latest decoded motor feedback (pos, spd, cur, temp, err)
 *   ZERO        - Command 0 A to the motor (bench testing)
 *
 * ESTOP commands:
 *   ESTOP       - Manually trigger ESTOP (bench testing)
 *   ESTOPSTATE  - Query whether ESTOP is active and the reason code
 */
static void process_command(void)
{
  LSM6DS3TR_Data_t *imu = lsm6ds3tr_get_data();
  uint16_t len = 0;

  /* ── IMU commands ── */
  if (strcmp((char *)rx_buffer, "READ") == 0)
  {
    len = sprintf((char *)tx_buffer,
                  "AX:%.2f AY:%.2f AZ:%.2f GX:%.2f GY:%.2f GZ:%.2f\r\n",
                  imu->accel.filt_x, imu->accel.filt_y, imu->accel.filt_z,
                  imu->gyro.filt_x,  imu->gyro.filt_y,  imu->gyro.filt_z);
    HAL_UART_Transmit(&huart2, tx_buffer, len, 100);
  }
  else if (strcmp((char *)rx_buffer, "READLATEST") == 0)
  {
    IMUReading reading;
    HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);
    int ok = imu_buffer_get_latest(&reading);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

    if (ok)
      len = sprintf((char *)tx_buffer,
                    "LATEST AX:%.2f AY:%.2f AZ:%.2f GX:%.2f GY:%.2f GZ:%.2f\r\n",
                    reading.ax, reading.ay, reading.az,
                    reading.gx, reading.gy, reading.gz);
    else
      len = sprintf((char *)tx_buffer, "LATEST:EMPTY\r\n");
    HAL_UART_Transmit(&huart2, tx_buffer, len, 200);
  }
  else if (strcmp((char *)rx_buffer, "STATUS") == 0)
  {
    lsm6ds3tr_check_connection();
    len = sprintf((char *)tx_buffer,
                  imu->state == SENSOR_STATE_CONNECTED
                  ? "STATUS:CONNECTED\r\n" : "STATUS:LOST\r\n");
    HAL_UART_Transmit(&huart2, tx_buffer, len, 100);
  }
  else if (strcmp((char *)rx_buffer, "REGISTER") == 0)
  {
    len = sprintf((char *)tx_buffer, "REGISTER:0x%02X\r\n", DEVICE_ADDRESS);
    HAL_UART_Transmit(&huart2, tx_buffer, len, 100);
  }
  else if (strcmp((char *)rx_buffer, "CONFIG") == 0)
  {
    len = sprintf((char *)tx_buffer,
                  "GYRO_CFG:0x%02X ACCEL_CFG:0x%02X\r\n",
                  imu->gyro_config, imu->accel_config);
    HAL_UART_Transmit(&huart2, tx_buffer, len, 100);
  }
  else if (strcmp((char *)rx_buffer, "POWER") == 0)
  {
    len = sprintf((char *)tx_buffer, "POWER_CFG:0x%02X\r\n", imu->power_config);
    HAL_UART_Transmit(&huart2, tx_buffer, len, 100);
  }

   /* ── CAN diagnostics ── */
  else if (strcmp((char *)rx_buffer, "CANERR") == 0)
  {
    uint32_t err = HAL_CAN_GetError(&hcan1);
    len = sprintf((char *)tx_buffer, "CAN_ERR:0x%08lX TEC:%lu REC:%lu\r\n",
                  err,
                  (hcan1.Instance->ESR >> 16) & 0xFF,
                  (hcan1.Instance->ESR >> 24) & 0xFF);
    HAL_UART_Transmit(&huart2, tx_buffer, len, 100);
  }
  else if (strcmp((char *)rx_buffer, "CANDROP") == 0)
  {
    len = sprintf((char *)tx_buffer, "DROPPED:%lu\r\n", g_can_tx_dropped);
    HAL_UART_Transmit(&huart2, tx_buffer, len, 100);
  }

  /* ── Motor commands ── */
  else if (strcmp((char *)rx_buffer, "MOTORSTATUS") == 0)
  {
    len = sprintf((char *)tx_buffer,
                  "MOTOR pos=%.1f spd=%.0f cur=%.2fA temp=%dC err=%d(%s) cmd=%.3fA\r\n",
                  g_motor_status.position, g_motor_status.speed,
                  g_motor_status.current,  g_motor_status.temperature,
                  g_motor_status.error,
                  motor_error_to_string(g_motor_status.error),
                  g_active_current);
    HAL_UART_Transmit(&huart2, tx_buffer, len, 100);
  }
  else if (strcmp((char *)rx_buffer, "ZERO") == 0)
  {
    motor_zero();
    g_motor_active   = 0;
    g_active_current = 0.0f;
    len = sprintf((char *)tx_buffer, "MOTOR:ZEROED\r\n");
    HAL_UART_Transmit(&huart2, tx_buffer, len, 100);
  }

  /* ── ESTOP commands ── */
  else if (strcmp((char *)rx_buffer, "ESTOP") == 0)
  {
    handle_estop(0xFF);   /* 0xFF = manual/debug trigger */
    len = sprintf((char *)tx_buffer, "ESTOP:TRIGGERED reason=0xFF\r\n");
    HAL_UART_Transmit(&huart2, tx_buffer, len, 100);
  }
  else if (strcmp((char *)rx_buffer, "ESTOPSTATE") == 0)
  {
    if (g_estop_active)
      len = sprintf((char *)tx_buffer, "ESTOP:ACTIVE reason=0x%02X\r\n", g_estop_reason);
    else
      len = sprintf((char *)tx_buffer, "ESTOP:CLEAR\r\n");
    HAL_UART_Transmit(&huart2, tx_buffer, len, 100);
  }

  else
  {
    len = sprintf((char *)tx_buffer, "ERR:UNKNOWN_CMD\r\n");
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
	debug_printf("In error handler\r\n");
  __disable_irq();
  while (1)
  { HAL_GPIO_TogglePin( LD2_GPIO_Port, LD2_Pin); HAL_Delay(200); }
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
