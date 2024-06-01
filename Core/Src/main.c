/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#define USE_TICKLESS_IDLE 1
#define USE_MY_SUPPRESS_TICKS_AND_SLEEP 0
#define USE_PRANAV 0
// #define configUSE_TICKLESS_IDLE 1
// #define configEXPECTED_IDLE_TIME_BEFORE_SLEEP 50

//PRANAV METHOD
// #define configUSE_IDLE_HOOK                      1

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "FreeRTOS.h"                   // ARM.FreeRTOS::RTOS:Core
#include "task.h"                       // ARM.FreeRTOS::RTOS:Core
#include "event_groups.h"               // ARM.FreeRTOS::RTOS:Event Groups
#include "semphr.h"                     // ARM.FreeRTOS::RTOS:Core

void TaskWait(void *argument);
void TaskToggleFiveTimes(void *argument);
TaskHandle_t TaskWaitHandler, TaskToggleFiveTimesHandler;

SemaphoreHandle_t semaphore;

TickType_t shortTimeOut = 50;		// portMAX_DELAY


#define BLINK_TIME 250
TickType_t blinkTicks = BLINK_TIME / portTICK_PERIOD_MS;





/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#if USE_MY_SUPPRESS_TICKS_AND_SLEEP

	#define portSUPPRESS_TICKS_AND_SLEEP( xIdleTime ) vApplicationSleep( xIdleTime )

	void vApplicationSleep(TickType_t xExpectedIdleTime)
	{
		// Ensure the expected idle time is at least the minimum time before sleep
		if (xExpectedIdleTime > (configEXPECTED_IDLE_TIME_BEFORE_SLEEP  / portTICK_PERIOD_MS))
		{
			// Enter critical section to disable interrupts
			__disable_irq();

			// Stop the SysTick timer or other tick source
			vPortSetupTimerInterruptForSleep(xExpectedIdleTime);

			// Enter low-power mode (e.g., WFI instruction on ARM Cortex-M)
			__WFI();

			// Re-enable interrupts
			__enable_irq();

			// Re-enable the SysTick timer or other tick source
			vPortSetupTimerInterruptAfterSleep();
		}
		else
		{
			// If expected idle time is less than the threshold, do not enter sleep mode
			// Instead, perform normal idle tasks
		}
	}

#endif
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */


void vApplicationIdleHook(void){
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI); // shleep
}





void TaskWait(void *argument)
{
	for(;;)
	{

		xSemaphoreGive(semaphore);
		vTaskDelay(5000/portTICK_PERIOD_MS);

	}
}


void TaskToggleFiveTimes(void *argument)
{
	for(;;)
	{

		if (xSemaphoreTake(semaphore, shortTimeOut) == pdPASS) {

			GPIOA->ODR |= GPIO_ODR_OD5;
			for (int i = 0; i < 9; i++){
				vTaskDelay(blinkTicks);
				GPIOA->ODR ^= GPIO_ODR_OD5;
			}

		}
	}

}

//// Task will toggle PC2 when it receives task3 semaphore
////  if it times out waiting for the semaphore it will toggle PC3
//void Task3(void *argument)
//{
//	for(;;)
//	{								 // portMAX_DELAY
//		if (xSemaphoreTake(task3Sema, shortTimeOut) == pdPASS) {
//			GPIOC->ODR ^= GPIO_ODR_OD2;			// toggle PC2
//		}
//		else {
//			GPIOC->ODR ^= GPIO_ODR_OD3;			// toggle PC3
//		}
//	}
//}




/*
 * Configure PC0-PC3 for GPIO Output
 * push-pull, low speed, no pull-up/pull-down resistors
 * Initialize all to 0s
 */
void PortC_Init(void)
{
	// turn on clock to GPIOC
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	// Configure PC0-3 for GPIO output
	GPIOC->MODER   &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 |
					    GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
	GPIOC->MODER   |=  ((1 << GPIO_MODER_MODE0_Pos) |
					    (1 << GPIO_MODER_MODE1_Pos) |
					    (1 << GPIO_MODER_MODE2_Pos) |
					    (1 << GPIO_MODER_MODE3_Pos));
	GPIOC->OTYPER  &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1 |
					    GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3);
	GPIOC->BSRR    =   (GPIO_BSRR_BR0 | GPIO_BSRR_BR1 |
					    GPIO_BSRR_BR2 | GPIO_BSRR_BR3);

	// Configure PC13 for user button input
	GPIOC->MODER &= ~(GPIO_MODER_MODE13);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD13);

	// Configure PA5 for LED output
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	GPIOA->MODER   &= ~(GPIO_MODER_MODE5);
	GPIOA->MODER   |=  (1 << GPIO_MODER_MODE5_Pos);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5);
	GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT5);
	GPIOA->ODR     &= ~(GPIO_PIN_5);
}

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
  /* USER CODE BEGIN 2 */


  PortC_Init();
  // Create the tasks

  	BaseType_t retVal;
    retVal = xTaskCreate(TaskWait, "TaskWait", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, &TaskWaitHandler);
    if (retVal != pdPASS) { while(1);}	// check if task creation failed

    retVal = xTaskCreate(TaskToggleFiveTimes, "TaskToggleFiveTimes", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 5, &TaskToggleFiveTimesHandler);
    if (retVal != pdPASS) { while(1);}	// check if task creation failed

//    retVal = xTaskCreate(Task3, "task3", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, &task3Handler);
//    if (retVal != pdPASS) { while(1);}	// check if task creation failed

    // Create Semaphores for TaskToggleFiveTimes and task3
    semaphore = xSemaphoreCreateBinary();
    if (semaphore == NULL) { while(1); }

//    task3Sema = xSemaphoreCreateBinary();
//    if (task3Sema == NULL) { while(1); }

    // Start scheduler
    vTaskStartScheduler();
















  //////////////////////////////////////////
  /// WE NEVER HIT THIS POINT, JUST KEEPING IT HERE FOR THE SAKE OF IOC RESETS


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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

  /*Configure GPIO pin : User_Button_EXTI_Pin */
  GPIO_InitStruct.Pin = User_Button_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(User_Button_EXTI_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM16 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM16) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
