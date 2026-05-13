/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *
  * v2-torque-controller — joint-controller port for the custom-PCB STM32F446RET6
  * board. Drives one VESC over CAN and streams IMU + motor position back over
  * CAN. Accepts torque commands or an estop from the Pi.
  *
  * No UART on the v2 PCB — all status comes out over CAN, plus a single LED
  * (PC13) used as a coarse boot/run indicator:
  *   - Solid OFF after power-up, briefly:  MCU running but pre-init
  *   - Solid ON for ~0.5 s during boot:    init in progress
  *   - Slow blink (1 Hz, 50% duty):        running normally
  *   - Fast blink (5 Hz):                  IMU not found at boot
  *   - Solid ON forever:                   CAN init failed at boot
  *
  * Differs from the Nucleo joint-controller mainly in:
  *   - HSE 8 MHz crystal + PLL -> 180 MHz (vs. HSI -> 84 MHz)
  *   - I2C1 on PB8/PB9 (vs. I2C3 on PA8/PC9)
  *   - I2C1_RX DMA on DMA1 Stream 0 (vs. I2C3_RX on DMA1 Stream 1)
  *   - Status LED on PC13 (vs. PA5 on Nucleo)
  *   - No user button, no UART debug console
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
#define MY_NODE_ID          1
#define MY_MOTOR_CAN_ID     104

/* AK70-9 KV60: torque = Kt * gear_ratio * Iq -> Iq = torque / KT_EFFECTIVE */
#define AK70_9_KT           0.159f
#define AK70_9_GEAR_RATIO   9.0f
#define KT_EFFECTIVE        (AK70_9_KT * AK70_9_GEAR_RATIO)

#define CURRENT_REFRESH_MS  50      /* prevents VESC current-loop timeout */
#define CURRENT_LIMIT       0.5f    /* test-safe current clamp (A) */
#define IMU_PERIOD_MS       2       /* 500 Hz IMU send */
#define HEARTBEAT_PERIOD_MS 500     /* 1 Hz LED blink, 50% duty */
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
static uint8_t    origin_set         = 0;   /* zeroed on first VESC feedback frame */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void CAN_Send_IMU_Data(void);
static void CAN_Send_Heartbeat(uint32_t counter);
static void fault_blink_forever(uint32_t period_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float clampf(float val, float lo, float hi)
{
  if (val < lo) return lo;
  if (val > hi) return hi;
  return val;
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

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Brief LED-on during init so a quick visual confirms the MCU is alive. */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  if (!can_common_init(&hcan1, MY_NODE_ID))
  {
    /* CAN INIT FAILED -> LED solid ON forever. */
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    while (1) {}
  }
  can_set_motor_filter(MY_MOTOR_CAN_ID);

  lsm6ds3tr_init_driver(&hi2c1);
  if (!lsm6ds3tr_check_connection())
  {
    /* IMU NOT FOUND -> LED fast blink forever (5 Hz, 100 ms toggle). */
    fault_blink_forever(100);
  }
  lsm6ds3tr_calibrate();

  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  uint32_t last_imu_tick       = HAL_GetTick();
  uint32_t last_heartbeat_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* -- Drain CAN RX -- */
    CanFrame rx_frame;
    while (can_recv(&rx_frame))
    {
      if (rx_frame.is_extended)
      {
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

    /* -- 1 Hz heartbeat: toggle LED + send a known CAN frame.
     *    The heartbeat frame is the diagnostic for "is the firmware
     *    actually putting bits on the bus?". It does not depend on the
     *    IMU pipeline, motor state, or CAN RX -- if candump on the Pi
     *    sees the heartbeat ID but not the IMU IDs, the IMU side is
     *    broken; if it sees nothing at all, the bus is broken. -- */
    if (HAL_GetTick() - last_heartbeat_tick >= HEARTBEAT_PERIOD_MS)
    {
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
      static uint32_t heartbeat_counter = 0;
      CAN_Send_Heartbeat(heartbeat_counter++);
      last_heartbeat_tick = HAL_GetTick();
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

/**
  * @brief Diagnostic heartbeat frame, sent at 1 Hz alongside the LED toggle.
  *
  * Wire format (DLC 8, big-endian):
  *   bytes 0-3 : monotonically increasing counter (1 per heartbeat)
  *   bytes 4-7 : HAL_GetTick() snapshot in ms (uptime)
  *
  * Standard ID = (CAN_MSG_HEARTBEAT << 7) | (MY_NODE_ID << 4) | NODE_PI
  *   For MY_NODE_ID=3: id = (5<<7)|(3<<4)|0 = 0x2B0
  *
  * Test on the Pi:
  *   candump can1 -t a | grep 2B0
  * If you see one line every ~500 ms, firmware-to-bus is working and the
  * problem is somewhere downstream (script filter, IMU pipeline, etc.).
  * If you see nothing, the problem is upstream (transceiver, wiring,
  * termination, or bitrate mismatch).
  */
static void CAN_Send_Heartbeat(uint32_t counter)
{
  uint32_t uptime = HAL_GetTick();
  uint8_t  data[8];

  data[0] = (uint8_t)(counter >> 24);
  data[1] = (uint8_t)(counter >> 16);
  data[2] = (uint8_t)(counter >> 8);
  data[3] = (uint8_t)(counter);
  data[4] = (uint8_t)(uptime >> 24);
  data[5] = (uint8_t)(uptime >> 16);
  data[6] = (uint8_t)(uptime >> 8);
  data[7] = (uint8_t)(uptime);

  uint16_t id = CAN_BUILD_ID(CAN_MSG_HEARTBEAT, MY_NODE_ID, CAN_NODE_PI);
  can_send_std(id, data, 8);
}

/**
  * @brief Park here forever, toggling the status LED at a fixed rate.
  *        Used as the fault indicator for boot-time failures so an operator
  *        can identify the failure mode visually without serial.
  */
static void fault_blink_forever(uint32_t period_ms)
{
  while (1)
  {
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    HAL_Delay(period_ms);
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
