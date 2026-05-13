/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *
  * v2-torque-controller — joint-controller port for the custom-PCB STM32F446RET6
  * board. Drives one VESC over CAN, streams IMU + motor position back over CAN,
  * and accepts torque commands or an estop from the Pi.
  *
  * Differs from the Nucleo joint-controller mainly in:
  *   - HSE 8 MHz crystal + PLL -> 180 MHz (vs. HSI -> 84 MHz)
  *   - I2C1 on PB8/PB9 (vs. I2C3 on PA8/PC9)
  *   - I2C1_RX DMA on DMA1 Stream 0 (vs. I2C3_RX on DMA1 Stream 1)
  *   - Status LED on PC13 (vs. PA5 on Nucleo)
  *   - No user button
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "lsm6ds3tr.h"
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
#define MY_NODE_ID          3
#define MY_MOTOR_CAN_ID     106

/* AK70-9 KV60: torque = Kt * gear_ratio * Iq -> Iq = torque / KT_EFFECTIVE */
#define AK70_9_KT           0.159f
#define AK70_9_GEAR_RATIO   9.0f
#define KT_EFFECTIVE        (AK70_9_KT * AK70_9_GEAR_RATIO)

#define CURRENT_REFRESH_MS  50      /* prevents VESC current-loop timeout */
#define CURRENT_LIMIT       0.5f    /* test-safe current clamp (A) */
#define IMU_PERIOD_MS       2       /* 500 Hz IMU send */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static MotorStatus motor_status      = {0};
static float      active_current     = 0.0f;
static uint8_t    motor_active       = 0;
static uint8_t    estop_active       = 0;
static uint32_t   last_refresh_tick  = 0;
static uint32_t   ext_rx_count       = 0;
static uint8_t    origin_set         = 0;   /* zeroed on first VESC feedback frame */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_DMA_Init(void);
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

  /* Initialize all configured peripherals.
   *
   * MX_DMA_Init MUST run before MX_I2C1_Init: HAL_I2C_MspInit calls
   * __HAL_LINKDMA which assumes the DMA controller's clock is already on. */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  if (!can_common_init(&hcan1, MY_NODE_ID))
  {
    debug_print("CAN INIT FAILED\r\n");
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    while (1) {}
  }
  can_set_motor_filter(MY_MOTOR_CAN_ID);
  debug_printf("CAN OK node=%u motor=%u\r\n",
               (unsigned)MY_NODE_ID, (unsigned)MY_MOTOR_CAN_ID);

  lsm6ds3tr_init_driver(&hi2c1);
  if (!lsm6ds3tr_check_connection())
  {
    debug_print("IMU NOT FOUND\r\n");
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    while (1) {}
  }
  lsm6ds3tr_calibrate();
  debug_print("READY\r\n");

  uint32_t last_imu_tick    = HAL_GetTick();
  uint32_t last_status_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* -- Drain CAN RX --
     * NO per-frame logging. At two nodes with VESC traffic, anything
     * faster than 1 Hz over UART blocks the main loop and breaks the
     * 50 ms motor refresh. */
    CanFrame rx_frame;
    while (can_recv(&rx_frame))
    {
      if (rx_frame.is_extended)
      {
        ext_rx_count++;
        motor_receive(&motor_status, rx_frame.data);

        /* First time we hear from the VESC = motor is detected. Zero the
         * encoder origin so position is reported relative to whatever pose
         * the joint is in at startup. Mode 0 = temporary (lost on power
         * cycle), so each boot re-zeros without writing to the VESC's flash. */
        if (!origin_set)
        {
          comm_can_set_origin(MY_MOTOR_CAN_ID, 0);
          origin_set = 1;
        }
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

    /* -- Refresh: re-send current command every 50 ms to prevent VESC timeout -- */
    if (motor_active &&
        (HAL_GetTick() - last_refresh_tick >= CURRENT_REFRESH_MS))
    {
      comm_can_set_current(MY_MOTOR_CAN_ID, active_current);
      last_refresh_tick = HAL_GetTick();
    }

    /* -- 500 Hz IMU send + DMA read -- */
    if ((HAL_GetTick() - last_imu_tick) >= IMU_PERIOD_MS)
    {
      last_imu_tick = HAL_GetTick();
      CAN_Send_IMU_Data();
      lsm6ds3tr_init_dma_read();
    }

    /* -- 1 Hz status dump -- */
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

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  *
  * HSE 8 MHz crystal -> PLL -> 180 MHz SYSCLK.
  *   PLLM=8 (1 MHz ref), PLLN=360 (360 MHz VCO), PLLP=2 (180 MHz)
  *   AHB /1 = 180 MHz, APB1 /4 = 45 MHz, APB2 /2 = 90 MHz
  * Over-drive must be enabled before selecting PLL @ 180 MHz.
  *
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DMA controller bring-up.
  *
  * Enables the DMA1 clock and the NVIC line for the I2C1 RX stream
  * (Stream 0). The actual stream config lives in HAL_I2C_MspInit so
  * deinit/reinit of I2C1 also tears down its DMA cleanly.
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
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

  /* Snapshot under DMA-IRQ mask so the filter fields can't update mid-copy. */
  HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
  float ax = imu->accel.filt_x;
  float ay = imu->accel.filt_y;
  float az = imu->accel.filt_z;
  float gx = imu->gyro.filt_x;
  float gy = imu->gyro.filt_y;
  float gz = imu->gyro.filt_z;
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

  can_send_imu_accel(MY_NODE_ID, ax, ay, az);
  can_send_imu_gyro(MY_NODE_ID, gx, gy, gz, motor_status.position);
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
