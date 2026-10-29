/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for Stepper Motor Motion Controller
  *                   with 1:10 Gearbox, Zero-Jitter Hardware Timing & USB CDC
  ******************************************************************************
  * @attention
  *
  * Hardware Connections:
  *   Primary Outputs (Set 1):
  *     PWM / STEP 1: PA0 (TIM2 Channel 1 Hardware PWM - 0ns jitter)
  *     DIRECTION 1 : PA1 (DIR_PORT: GPIOA, DIR_PIN: GPIO_PIN_1)
  *     ENABLE 1    : PA3 (ENA_PORT: GPIOA, ENA_PIN: GPIO_PIN_3)
  *   Secondary Outputs (Set 2 - Simultaneous):
  *     PWM / STEP 2: PB13 (TIM1 Channel 1N Hardware PWM - Hardware Gated Lockstep)
  *     DIRECTION 2 : PB12 (DIR2_PORT: GPIOB, DIR2_PIN: GPIO_PIN_12)
  *     ENABLE 2    : PA7  (ENA2_PORT: GPIOA, ENA2_PIN: GPIO_PIN_7)
  *   USB CDC:
  *     USB DM      : PA11 (Full Speed USB D-)
  *     USB DP      : PA12 (Full Speed USB D+)
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stepper_config.h"
#include "stepper_control.h"
#include "stepper_direction.h"
#include "usb_cdc.h"
#include "command_parser.h"
#include "closed_loop.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Global Stepper Motor Instance */
StepperMotor g_stepper;

/* Track previous moving state to send completion event */
static uint8_t g_last_moving = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

  /* Configure the system clock: 72 MHz SYSCLK, 48 MHz USB Clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  /* 1. Initialize Stepper Motor Controller (Dual outputs: PA0/PB13 PWM, PA1/PB12 DIR, PA3/PA7 ENA) */
  Stepper_Init(&g_stepper);

  /* 2. Initialize Closed-Loop Controller & AS5600 Sensor (Reads initial position from sensor) */
  ClosedLoop_Init(&g_stepper);

  /* 3. Initialize USB CDC Virtual COM Port (Forces host re-enumeration on PA12) */
  USB_CDC_Init();

  /* 4. Initialize Command Parser */
  CMD_Init();

  /* Brief delay to allow host to finish USB CDC enumeration */
  HAL_Delay(500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 1. Process any incoming bytes from USB CDC Virtual COM Port */
    while (USB_CDC_Available() > 0)
    {
      int16_t b = USB_CDC_Read();
      if (b >= 0)
      {
        CMD_ProcessByte((uint8_t)b, &g_stepper);
      }
    }

    /* Periodic parser poll (auto-executes commands sent without newline) */
    CMD_Poll(&g_stepper);

    /* 2. Non-blocking Closed-Loop PID and AS5600 sensor update (100 Hz fixed interval) */
    if (g_cl_timer_flag)
    {
      g_cl_timer_flag = 0;
      ClosedLoop_Update(&g_stepper);
    }

    /* 3. Check if a motion move has just finished */
    if (g_last_moving && !g_stepper.moving)
    {
      char pos_str[32];
      char done_msg[64];
      CMD_FormatFloat(pos_str, sizeof(pos_str), Stepper_GetPosition(&g_stepper), 3);
      snprintf(done_msg, sizeof(done_msg),
               "DONE: Reached %s DEG\r\n", pos_str);
      USB_CDC_Print(done_msg);
    }
    g_last_moving = g_stepper.moving;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  *        Configures 8 MHz HSE crystal with PLL x9 = 72 MHz SYSCLK,
  *        APB1 timer clock = 72 MHz, USB prescaler /1.5 = 48 MHz.
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9; /* 8 MHz * 9 = 72 MHz */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;  /* 72 MHz HCLK */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;    /* 36 MHz PCLK1 (Timer clk = 2*PCLK1 = 72 MHz) */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;    /* 72 MHz PCLK2 */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the USB clock (48 MHz required for USB FS device)
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5; /* 72 MHz / 1.5 = 48 MHz */
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
