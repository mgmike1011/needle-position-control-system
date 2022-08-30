/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// Motor drive
#include "A4988_Drive.h"
// Sensor
#include "VL6180X.h"
// Screen
//#include
// Temperature sensor
//#include
// Standard C library
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FORWARD_NEEDLE 	RIGHT_DIR
#define BACK_NEEDLE 	LEFT_DIR
#define SUCK_SYRINGE	RIGHT_DIR
#define BLOW_SYRINGE	LEFT_DIR
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//
// Motor controllers
//
A4988_Drive Syringe = {	.NAME = "SYRINGE",
						.STEPS = 200,
						.RESOLUTION = 1,
						.PORT_DIR = DIR_SYRINGE_GPIO_Port,
						.PIN_DIR = DIR_SYRINGE_Pin,
						.PORT_ENABLE = ENABLE_SYRINGE_GPIO_Port,
						.PIN_ENABLE = ENABLE_SYRINGE_Pin,
						.PORT_MS1 = MS1_SYRINGE_GPIO_Port,
						.PIN_MS1 = MS1_SYRINGE_Pin,
						.PORT_MS2 = MS2_SYRINGE_GPIO_Port,
						.PIN_MS2 = MS2_SYRINGE_Pin,
						.PORT_MS3 = MS3_SYRINGE_GPIO_Port,
						.PIN_MS3 = MS3_SYRINGE_Pin,
						.STEP_mm_RESOLUTION = 0.01
						,.PORT_RESET = RESET_SYRINGE_GPIO_Port,
						.PIN_RESET = RESET_SYRINGE_Pin,
						.PORT_SLEEP = SLEEP_SYRINGE_GPIO_Port,
						.PIN_SLEEP = SLEEP_SYRINGE_Pin,
						.TIM_STEP = &htim2,
						.TIM_STEP_CHANNEL = TIM_CHANNEL_1,
						.TIM_COUNTER_SLAVE = &htim3
						};
A4988_Drive Needle = {	.NAME = "NEEDLE",
						.STEPS = 200,
						.RESOLUTION = 1,
						.STEP_mm_RESOLUTION = 0.005
						,.PORT_DIR = DIR_NEEDLE_GPIO_Port,
						.PIN_DIR = DIR_NEEDLE_Pin,
						.PORT_ENABLE = ENABLE_NEEDLE_GPIO_Port,
						.PIN_ENABLE = ENABLE_NEEDLE_Pin,
						.PORT_MS1 = MS1_NEEDLE_GPIO_Port,
						.PIN_MS1 = MS1_NEEDLE_Pin,
						.PORT_MS2 = MS2_NEEDLE_GPIO_Port,
						.PIN_MS2 = MS2_NEEDLE_Pin,
						.PORT_MS3 = MS3_NEEDLE_GPIO_Port,
						.PIN_MS3 = MS3_NEEDLE_Pin,
						.PORT_RESET = RESET_NEEDLE_GPIO_Port,
						.PIN_RESET = RESET_NEEDLE_Pin,
						.PORT_SLEEP = SLEEP_NEEDLE_GPIO_Port,
						.PIN_SLEEP = SLEEP_NEEDLE_Pin,
						.TIM_STEP = &htim4,
						.TIM_STEP_CHANNEL = TIM_CHANNEL_1,
						.TIM_COUNTER_SLAVE = &htim5
						};
//
// Distance Sensors
//
VL6180X_ Syringe_sensor;
VL6180X_ Needle_sensor;
volatile uint16_t MESURE_Needle = 0; // distance in mm
volatile uint16_t MESURE_Syringe = 0; // distance in mm
//
// Control
//
volatile uint16_t Set_distance_needle = 0;
volatile uint16_t Set_distance_syringe = 0;
//
// USART communication
//
uint8_t Buffor_USART[128], Buffor_Rx_USART[8];
uint16_t length_Buffor_USART;
//
// OLED screen print
//
char Message_OLED[32];
uint16_t length_Buffor_OLED;
//
// Soft timer
//
uint32_t SoftTimer = 0;
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

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_I2C2_Init();
  MX_I2C4_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  	  // Drive initialization
  Init_A4988(&Syringe);
  Init_A4988(&Needle);
  	  // Sensor initialization
  VL6180X_Init(&Syringe_sensor, &hi2c4);
  VL6180X_Init(&Needle_sensor, &hi2c2);
  configureDefault_VL6180X(&Syringe_sensor);
  configureDefault_VL6180X(&Needle_sensor);
  	  // Measure timer Start
  HAL_TIM_Base_Start_IT(&htim6); // Needle - 5Hz
  HAL_TIM_Base_Start_IT(&htim7); // Syringe - 5Hz
  	  // Initial measurement
  MESURE_Needle = readRangeSingleMillimeters_VL6180X(&Needle_sensor);
  MESURE_Syringe = readRangeSingleMillimeters_VL6180X(&Syringe_sensor);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
