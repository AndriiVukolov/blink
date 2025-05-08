/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "mdf.h"
#include "i2c.h"
#include "icache.h"
#include "memorymap.h"
#include "octospi.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "ucpd.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "loging.h"
#include "microrl.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stm32u5xx_hal.h"
#include "cli_commands.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_DELAY 50 //ms
#define PULSE_STEP  21000L
#define PULSE_MAX  160000L
#define PULSE_MIN       0L


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//TIM_HandleTypeDef htim2;
//TIM_HandleTypeDef htim3;
static TIM_OC_InitTypeDef sConfigOC;
UART_HandleTypeDef * huart;
//static unsigned int timer1ms = 0;
//static GPIO_PinState flagButtonState;
//static int32_t pulseWidth = 0;
//static int direction = 1;
//static uint32_t brightness;

static microrl_t mcon;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
/* USER CODE BEGIN PFP */
//void print (const char * str);
int execute (int argc, const char * const * argv);
void print_help (void);
//uint8_t get_char (void);
void sigint (void);

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

  /* Configure the System Power */
  SystemPower_Config();

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADF1_Init();
  MX_I2C1_Init();
  MX_ICACHE_Init();
  MX_OCTOSPI1_Init();
  MX_OCTOSPI2_Init();
  MX_SPI2_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_UCPD1_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  huart = UartInit(1);
//========================================================================Microrl=============================
  microrl_init(&mcon, print);
  microrl_set_execute_callback (&mcon, execute);
  microrl_set_sigint_callback (&mcon, sigint);
//==========================================================================================================
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
  TIM3_PeriodSet(80000);
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

      uint8_t cChar;
      cChar = get_char();
      //UartSendString(&cChar, huart);
      microrl_insert_char (&mcon, cChar);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{
  HAL_PWREx_EnableVddIO2();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

/* USER CODE BEGIN 4 */
//Timer interrupt every 1 ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
//    if(htim->Instance == TIM2)
//    {
//        if (timer1ms < DEBOUNCE_DELAY) timer1ms++;
//    }
//    if(htim->Instance == TIM3)
//    {
//        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
//    }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM3)
        {
            HAL_GPIO_WritePin(ILED_Port, ILED_Pin, GPIO_PIN_RESET);
        }
}
//
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM3)
    {
        HAL_GPIO_WritePin(ILED_Port, ILED_Pin, GPIO_PIN_SET);
    }
}
//Button interrupt
//void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
//{
//    if(GPIO_Pin == USER_Button_Pin)
//        {
//            HAL_NVIC_DisableIRQ(USER_Button_EXTI_IRQn);
//            HAL_TIM_Base_Start_IT(&htim2);
//        }
//}

//==========================================================================================================
void print (const char * str)
{
    if (HAL_UART_Transmit(huart, str, strlen(str), 100) != HAL_OK)
    {
        Error_Handler();
    }
}
int execute (int argc, const char * const * argv)
{
    int i = 0;

    // just iterate through argv word and compare it with your commands
    while (i < argc) {
        if (strcmp (argv[i], _CMD_HELP) == 0)
        {
            cmdHelp();
        }
        else if (strcmp (argv[i], _CMD_CLEAR) == 0)
        {
            cmdClear();
        }
        else if (strcmp (argv[i], _CMD_LED_ON) == 0)
        {
            cmdLedon();
        }
        else if (strcmp (argv[i], _CMD_LED_OFF) == 0)
        {
            cmdLedoff();
        }
        else if (strcmp (argv[i], _CMD_SET_BRIGHTNESS) == 0)
        {
            cmdSetBrightness(argv[++i]);
        }
        else
        {
            print ("command: '");
            print ((char*)argv[i]);
            print ("' Not found.\n\r");
        }
        i++;
    }
    return 0;
}
void print_help (void)
{
    print ("Use TAB key for completion\n\rCommand:\n\r");
    print ("\tclear               - clear screen\n\r");
    print ("\tledon               - turns LED on\n\r");
    print ("\tledoff              - turns LED off\n\r");
    print ("\tled_set <brightness> - sets LED brightness 0..100\n\r");
}
unsigned char get_char (void)
{
    uint8_t symb;
    if (HAL_UART_Receive(huart, &symb, 1, 0) != HAL_OK)
    {
        Error_Handler();
    }
    return (symb);
}
void sigint (void)
{
    print ("^C catched!\n\r");
}

//==========================================================================================================
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
