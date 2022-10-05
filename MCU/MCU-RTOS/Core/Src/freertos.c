/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// Motor drive
#include "A4988_Drive.h"
// Sensor
#include "VL6180X.h"
// Screen
#include "SSD1306_OLED.h"
#include "GFX_BW.h"
#include "fonts/fonts.h"
// Temperature sensor
#include "BMPXX80.h"
// printf external library
#include "printf.h"
// MCU libraries
#include "usart.h"
#include "i2c.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//
// Syringe info data structure for queue
//
typedef struct{
	uint16_t MEASURE_Syringe; // distance in mm
	uint16_t Set_distance_syringe; // distance in mm
}Syringe_info;
//
// Needle info data structure for queue
//
typedef struct{
	uint16_t MEASURE_Needle; // distance in mm
	uint16_t Set_distance_needle; // distance in mm
}Needle_info;
//
// Temperature info data structure for queue
//
typedef struct{
	float Temperature;
	uint8_t Fan_info;
}Temperature_info;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//
// Direction control
//
#define FORWARD_NEEDLE 	RIGHT_DIR
#define BACK_NEEDLE 	LEFT_DIR
#define SUCK_SYRINGE	RIGHT_DIR
#define BLOW_SYRINGE	LEFT_DIR
//
// Control type
//
#define OPEN_LOOP 1
#define CLOSE_LOOP 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for HeartBeatTast */
osThreadId_t HeartBeatTastHandle;
const osThreadAttr_t HeartBeatTast_attributes = {
  .name = "HeartBeatTast",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SyringeControlT */
osThreadId_t SyringeControlTHandle;
const osThreadAttr_t SyringeControlT_attributes = {
  .name = "SyringeControlT",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for OLEDTask */
osThreadId_t OLEDTaskHandle;
const osThreadAttr_t OLEDTask_attributes = {
  .name = "OLEDTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for QueueSyringeInfo */
osMessageQueueId_t QueueSyringeInfoHandle;
const osMessageQueueAttr_t QueueSyringeInfo_attributes = {
  .name = "QueueSyringeInfo"
};
/* Definitions for SyringeInfoTimer */
osTimerId_t SyringeInfoTimerHandle;
const osTimerAttr_t SyringeInfoTimer_attributes = {
  .name = "SyringeInfoTimer"
};
/* Definitions for MutexPrintf */
osMutexId_t MutexPrintfHandle;
const osMutexAttr_t MutexPrintf_attributes = {
  .name = "MutexPrintf"
};
/* Definitions for MutexI2C2 */
osMutexId_t MutexI2C2Handle;
const osMutexAttr_t MutexI2C2_attributes = {
  .name = "MutexI2C2"
};
/* Definitions for MutexI2C4 */
osMutexId_t MutexI2C4Handle;
const osMutexAttr_t MutexI2C4_attributes = {
  .name = "MutexI2C4"
};
/* Definitions for SyringeInfoSemaphore */
osSemaphoreId_t SyringeInfoSemaphoreHandle;
const osSemaphoreAttr_t SyringeInfoSemaphore_attributes = {
  .name = "SyringeInfoSemaphore"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//
// printf function prototype
//
void _putchar(char character);
/* USER CODE END FunctionPrototypes */

void StartHeartBeatTast(void *argument);
void StartSyringeControlTask(void *argument);
void StartOLEDTask(void *argument);
void SyringeInfoTimerCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of MutexPrintf */
  MutexPrintfHandle = osMutexNew(&MutexPrintf_attributes);

  /* creation of MutexI2C2 */
  MutexI2C2Handle = osMutexNew(&MutexI2C2_attributes);

  /* creation of MutexI2C4 */
  MutexI2C4Handle = osMutexNew(&MutexI2C4_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of SyringeInfoSemaphore */
  SyringeInfoSemaphoreHandle = osSemaphoreNew(1, 1, &SyringeInfoSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of SyringeInfoTimer */
  SyringeInfoTimerHandle = osTimerNew(SyringeInfoTimerCallback, osTimerPeriodic, NULL, &SyringeInfoTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of QueueSyringeInfo */
  QueueSyringeInfoHandle = osMessageQueueNew (8, sizeof(Syringe_info), &QueueSyringeInfo_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of HeartBeatTast */
  HeartBeatTastHandle = osThreadNew(StartHeartBeatTast, NULL, &HeartBeatTast_attributes);

  /* creation of SyringeControlT */
  SyringeControlTHandle = osThreadNew(StartSyringeControlTask, NULL, &SyringeControlT_attributes);

  /* creation of OLEDTask */
  OLEDTaskHandle = osThreadNew(StartOLEDTask, NULL, &OLEDTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartHeartBeatTast */
/**
  * @brief  Main basic function informing about MCU work.
  * @priority: Low
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartHeartBeatTast */
void StartHeartBeatTast(void *argument)
{
  /* USER CODE BEGIN StartHeartBeatTast */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	  osDelay(1000);
  }
  /* USER CODE END StartHeartBeatTast */
}

/* USER CODE BEGIN Header_StartSyringeControlTask */
/**
* @brief Function implementing the SyringeControlT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSyringeControlTask */
void StartSyringeControlTask(void *argument)
{
  /* USER CODE BEGIN StartSyringeControlTask */
	//
	// Motor controllers
	//
	A4988_Drive Syringe = {	.NAME = "SYRINGE",
							.STEPS = 200,
							.RESOLUTION = HALF_STEP,
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
							.STEP_mm_RESOLUTION = 0.01,
							.PORT_RESET = RESET_SYRINGE_GPIO_Port,
							.PIN_RESET = RESET_SYRINGE_Pin,
							.PORT_SLEEP = SLEEP_SYRINGE_GPIO_Port,
							.PIN_SLEEP = SLEEP_SYRINGE_Pin,
							.TIM_STEP = &htim2,
							.TIM_STEP_CHANNEL = TIM_CHANNEL_1,
							.TIM_COUNTER_SLAVE = &htim3
							};
	//
	// Distance Sensors
	//
	VL6180X_ Syringe_sensor;
	//
	// Queue info
	//
	Syringe_info _Syringe_info;
	//###################################//###################################
	  _Syringe_info.MEASURE_Syringe = 10;
	  _Syringe_info.Set_distance_syringe = 20;
	//###################################//###################################
	//
	// Initialization
	//
//	Init_A4988(&Syringe); // Drive initialization
//	osMutexAcquire(MutexI2C4Handle, osWaitForever);
//	VL6180X_Init(&Syringe_sensor, &hi2c4); // Sensor initialization
//	osMutexRelease(MutexI2C4Handle);
//	configureDefault_VL6180X(&Syringe_sensor); // Sensor initialization
//	osMutexAcquire(MutexI2C4Handle, osWaitForever);
//	MEASURE_Syringe = readRangeSingleMillimeters_VL6180X(&Syringe_sensor); // Initial measurement
//	osMutexRelease(MutexI2C4Handle);
	osTimerStart(SyringeInfoTimerHandle, 500);
  /* Infinite loop */
  for(;;)
  {
	  //###################################//###################################
	  _Syringe_info.MEASURE_Syringe += 1;
	  _Syringe_info.Set_distance_syringe += 2;
	  //###################################//###################################

	  // Read measurement from sensor
//	  osMutexAcquire(MutexI2C4Handle, osWaitForever);
//	  _Syringe_info.MEASURE_Syringe = readRangeSingleMillimeters_VL6180X(&Syringe_sensor); // Measurement
//	  osMutexRelease(MutexI2C4Handle);

	  // Send data to queue
	  if (osOK == osSemaphoreAcquire(SyringeInfoSemaphoreHandle, 0)) {
	  		osMessageQueuePut(QueueSyringeInfoHandle, &_Syringe_info, 0, osWaitForever);
	  	}

	  // Control the motor
//	  if (LEFT_DIR == BLOW_SYRINGE) { // TODO implement
//		  if (MEASURE_Syringe <= Set_distance_syringe) {
//			  HAL_TIM_PWM_Stop(Syringe.TIM_STEP, Syringe.TIM_STEP_CHANNEL); // Stop syringe
//		  }
//	  //			else{
//	  //				HAL_TIM_PWM_Start(Syringe.TIM_STEP, Syringe.TIM_STEP_CHANNEL); // Start syringe
//	  //			}
//	  } else {
//		  if (MEASURE_Syringe >= Set_distance_syringe) { // TODO check if correct
//			  HAL_TIM_PWM_Stop(Syringe.TIM_STEP, Syringe.TIM_STEP_CHANNEL); // Stop syringe
//		  }
//	  //			else{
//	  //				HAL_TIM_PWM_Sart(Syringe.TIM_STEP, Syringe.TIM_STEP_CHANNEL); // Start syringe
//	  //			}
//	  }

	  //###################################//###################################
	  printf("Hello world! %d %d \n\r", _Syringe_info.MEASURE_Syringe , _Syringe_info.Set_distance_syringe);
	  //###################################//###################################
	  // Time interval
	  osDelay(200);
  }
  /* USER CODE END StartSyringeControlTask */
}

/* USER CODE BEGIN Header_StartOLEDTask */
/**
* @brief OLED screen print
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOLEDTask */
void StartOLEDTask(void *argument)
{
  /* USER CODE BEGIN StartOLEDTask */
	// Variables
	char Message_OLED[32]; // Message buffer
	Syringe_info _Syringe_info;
	Needle_info _Needle_info;
	_Needle_info.Set_distance_needle = 0;
	_Needle_info.MEASURE_Needle = 0;
	_Syringe_info.Set_distance_syringe = 0;
	_Syringe_info.MEASURE_Syringe = 0;
	// Screen initialization
	osMutexAcquire(MutexI2C4Handle, osWaitForever);
	SSD1306_Init(&hi2c4);
	osMutexRelease(MutexI2C4Handle);
	GFX_SetFont(font_8x5);
	GFX_SetFontSize(1);
	SSD1306_Clear(BLACK);
	SSD1306_Display();
  /* Infinite loop */
  for(;;)
  {
	  // Get data from Syringe info queue
	  osMessageQueueGet(QueueSyringeInfoHandle, &_Syringe_info, NULL, 0);
	  // Display functions
	  SSD1306_Clear(BLACK);
	  sprintf(Message_OLED, "Needle position");
	  GFX_DrawString(20, 0, Message_OLED, WHITE, 0);
	  GFX_DrawLine(0, 9, 128, 9, WHITE);
	  sprintf(Message_OLED, "Set: %d mm", _Needle_info.Set_distance_needle); //Set_distance_needle
	  GFX_DrawString(0, 12, Message_OLED, WHITE, 0);
	  sprintf(Message_OLED, "Measure: %d mm", _Needle_info.MEASURE_Needle); //MEASURE_Needle
	  GFX_DrawString(0, 22, Message_OLED, WHITE, 0);
	  sprintf(Message_OLED, "Syringe position");
	  GFX_DrawString(15, 32, Message_OLED, WHITE, 0);
	  GFX_DrawLine(0, 41, 128, 41, WHITE);
	  sprintf(Message_OLED, "Set: %d mm", _Syringe_info.Set_distance_syringe); //Set_distance_syringe
	  GFX_DrawString(0, 44, Message_OLED, WHITE, 0);
	  sprintf(Message_OLED, "Measure: %d mm", _Syringe_info.MEASURE_Syringe); //MEASURE_Syringe
	  GFX_DrawString(0, 54, Message_OLED, WHITE, 0);
	  SSD1306_Display();
	  // Time interval
	  osDelay(500);
  }
  /* USER CODE END StartOLEDTask */
}

/* SyringeInfoTimerCallback function */
void SyringeInfoTimerCallback(void *argument)
{
  /* USER CODE BEGIN SyringeInfoTimerCallback */
	osSemaphoreRelease(SyringeInfoSemaphoreHandle);
  /* USER CODE END SyringeInfoTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void _putchar(char character)
{
	osMutexAcquire(MutexPrintfHandle, osWaitForever);
	HAL_UART_Transmit(&huart3, (uint8_t*)&character, 1, 1000);
	osMutexRelease(MutexPrintfHandle);
}
/* USER CODE END Application */

