/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Joint Controller)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "can_common.h"
#include "can_motor.h"
#include "can_system.h"
#include "ak70_9.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
#define MY_NODE_ID        CAN_NODE_LEFT_HIP  // change per board
#define TORQUE_TIMEOUT_MS 500
#define STATUS_PERIOD_MS   20
/* USER CODE END PD */

/* USER CODE BEGIN PM */
/* USER CODE END PM */

CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static volatile uint8_t  estop_active     = 0;
static volatile uint32_t last_torque_tick = 0;
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();

  /* USER CODE BEGIN 2 */
  if (!can_common_init(&hcan1, MY_NODE_ID))
    Error_Handler();

  // Enter MIT mode — send zero command to wake motor
  pack_cmd(MOTOR_CAN_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  uint32_t last_status_tick = HAL_GetTick();
  last_torque_tick          = HAL_GetTick();
  MotorStatus motor         = {0};
  uint8_t prev_error        = 0;
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */

    /* ── 1. Drain CAN RX ring buffer ── */
    CanFrame frame;
    while (can_recv(&frame))
    {
      // Extended frames = motor feedback from VESC
      if (frame.is_extended)
      {
        motor_receive(&motor, frame.data);

        if (motor.error != prev_error &&
            motor.error != MOTOR_ERROR_NONE)
        {
          // Motor fault — zero torque, latch estop
          pack_cmd(MOTOR_CAN_ID, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);
          estop_active = 1;
          HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        }
        prev_error = motor.error;
        continue;
      }

      uint8_t msg_type = can_get_msg_type((uint16_t)frame.id);

      // ESTOP — latch immediately, zero torque
      if (msg_type == CAN_MSG_ESTOP)
      {
        pack_cmd(MOTOR_CAN_ID, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);
        estop_active = 1;
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      }

      // TORQUE_CMD from Pi
      else if (msg_type == CAN_MSG_TORQUE_CMD && !estop_active)
      {
        float torque_nm;
        if (can_parse_torque_cmd(&frame, &torque_nm))
        {
          // Clamp to AK70-9 MIT torque limits (+/-32 Nm)
          if (torque_nm >  AK70_9_MIT_T_MAX) torque_nm =  AK70_9_MIT_T_MAX;
          if (torque_nm < -AK70_9_MIT_T_MAX) torque_nm = -AK70_9_MIT_T_MAX;

          pack_cmd(MOTOR_CAN_ID, 0.0f, 0.0f, 0.0f, 1.0f, torque_nm);
          last_torque_tick = HAL_GetTick();
        }
      }
    }

    /* ── 2. Torque timeout watchdog ── */
    if (!estop_active &&
        (HAL_GetTick() - last_torque_tick) > TORQUE_TIMEOUT_MS)
    {
      pack_cmd(MOTOR_CAN_ID, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);
      last_torque_tick = HAL_GetTick(); // reset to avoid spamming
    }

    /* ── 3. Periodic motor status → Pi ── */
    if ((HAL_GetTick() - last_status_tick) >= STATUS_PERIOD_MS)
    {
      last_status_tick = HAL_GetTick();
      can_send_motor_status(MY_NODE_ID,
                            motor.position,
                            motor.speed,
                            motor.current,
                            motor.temperature,
                            motor.error);
    }

    /* USER CODE END 3 */
  }
}

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_CAN1_Init(void)
{
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK) Error_Handler();
}

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
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

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

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  { HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); HAL_Delay(200); }
}