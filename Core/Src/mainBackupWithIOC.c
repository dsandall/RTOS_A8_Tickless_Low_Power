///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  * @attention
//  *
//  * Copyright (c) 2024 STMicroelectronics.
//  * All rights reserved.
//  *
//  * This software is licensed under terms that can be found in the LICENSE file
//  * in the root directory of this software component.
//  * If no LICENSE file comes with this software, it is provided AS-IS.
//  *
//  ******************************************************************************
//  */
//
//
///* USER CODE END Header */
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//#include "cmsis_os.h"
//
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//
//#include "FreeRTOS.h"                   // ARM.FreeRTOS::RTOS:Core
//#include "task.h"                       // ARM.FreeRTOS::RTOS:Core
//#include "event_groups.h"               // ARM.FreeRTOS::RTOS:Event Groups
//#include "semphr.h"                     // ARM.FreeRTOS::RTOS:Core
//
//void TaskWait(void *argument);
//void TaskToggleFiveTimes(void *argument);
//TaskHandle_t TaskWaitHandler, TaskToggleFiveTimesHandler;
//
//SemaphoreHandle_t semaphore;
//
//#define BLINK_TIME 128
//#define WAIT_TIME 5000
//
//
//
///* USER CODE END Includes */
//
///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */
//
///* USER CODE END PTD */
//
///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
//
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
//
///* USER CODE END PM */
//
///* Private variables ---------------------------------------------------------*/
///* Definitions for defaultTask */
//osThreadId_t defaultTaskHandle;
//const osThreadAttr_t defaultTask_attributes = {
//  .name = "defaultTask",
//  .stack_size = 128 * 4,
//  .priority = (osPriority_t) osPriorityNormal,
//};
///* USER CODE BEGIN PV */
//
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//void StartDefaultTask(void *argument);
//
///* USER CODE BEGIN PFP */
//
//void TaskWait(void *argument)
//{
//	for(;;)
//	{
//
//		xSemaphoreGive(semaphore);
//		vTaskDelay(WAIT_TIME/portTICK_PERIOD_MS);
//
//	}
//}
//
//
//void TaskToggleFiveTimes(void *argument)
//{
//	for(;;)
//	{
//
//		// block for 150 ms (sleeping for 100ms)
//		if (xSemaphoreTake(semaphore, portMAX_DELAY) == pdPASS) {
//
//			GPIOA->ODR |= GPIO_ODR_OD5;
//			for (int i = 0; i < 9; i++){
//				vTaskDelay(BLINK_TIME / portTICK_PERIOD_MS);
//				GPIOA->ODR ^= GPIO_ODR_OD5;
//			}
//
//		}
//	}
//
//}
//
///* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//
//void EXTI15_10_IRQHandler(void)
//{
//    // Check if the interrupt was triggered by EXTI13
//    if (EXTI->PR1 & EXTI_PR1_PIF13)
//    {
//        // Clear the pending bit by writing 1 to it
//        EXTI->PR1 |= EXTI_PR1_PIF13;
//
//
//    	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//		xSemaphoreGiveFromISR(semaphore, &xHigherPriorityTaskWoken);
//    	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//
//
//        // Your custom interrupt handling code here
//    }
//}
//
///* USER CODE END 0 */
//
///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//  /* USER CODE BEGIN 1 */
//
//  /* USER CODE END 1 */
//
//  /* MCU Configuration--------------------------------------------------------*/
//
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();
//
//  /* USER CODE BEGIN Init */
//
//  /* USER CODE END Init */
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//  /* USER CODE BEGIN SysInit */
//
//  /* USER CODE END SysInit */
//
//  /* Initialize all configured peripherals */
//  /* USER CODE BEGIN 2 */
//
////  PortC_Init();
//
//
//
//  // Enable the clock for GPIOC
//  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
//
//  // Enable the clock for SYSCFG
//  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
//
//  // Set PC13 to input mode
//  GPIOC->MODER &= ~(GPIO_MODER_MODE13);
//
//  // Select the source input for the EXTI13 external interrupt (PC13)
//  SYSCFG->EXTICR[3] &= ~(SYSCFG_EXTICR4_EXTI13);
//  SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;
//
//  // Configure EXTI13 to trigger on rising edge
//  EXTI->RTSR1 |= EXTI_RTSR1_RT13;
//
//  // Enable interrupt request from line 13
//  EXTI->IMR1 |= EXTI_IMR1_IM13;
//
//  // Set priority for EXTI15_10 interrupt (covers EXTI lines 10 to 15)
//  NVIC_SetPriority(EXTI15_10_IRQn, 6);
//
//  // Enable EXTI15_10 interrupt in NVIC
//  NVIC_EnableIRQ(EXTI15_10_IRQn);
//
//
//  // Configure PA5 for LED output
//  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
//  GPIOA->MODER   &= ~(GPIO_MODER_MODE5);
//  GPIOA->MODER   |=  (1 << GPIO_MODER_MODE5_Pos);
//  GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5);
//  GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT5);
//  GPIOA->ODR     &= ~(GPIO_PIN_5);
//
//
//
//  // Create the tasks
//
//  	BaseType_t retVal;
//    retVal = xTaskCreate(TaskWait, "TaskWait", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 6, &TaskWaitHandler);
//    if (retVal != pdPASS) { while(1);}	// check if task creation failed
//
//    retVal = xTaskCreate(TaskToggleFiveTimes, "TaskToggleFiveTimes", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, &TaskToggleFiveTimesHandler);
//    if (retVal != pdPASS) { while(1);}	// check if task creation failed
//
//    // Create Semaphores for TaskToggleFiveTimes and task3
//    semaphore = xSemaphoreCreateBinary();
//    if (semaphore == NULL) { while(1); }
//
//    // Start scheduler
//    vTaskStartScheduler();
//
//
//
//  //////////////////////////////////////////
//  /// WE NEVER HIT THIS POINT, JUST KEEPING IT HERE FOR THE SAKE OF IOC RESETS
//
//
//  /* USER CODE END 2 */
//
//  /* Init scheduler */
//  osKernelInitialize();
//
//  /* USER CODE BEGIN RTOS_MUTEX */
//  /* add mutexes, ... */
//  /* USER CODE END RTOS_MUTEX */
//
//  /* USER CODE BEGIN RTOS_SEMAPHORES */
//  /* add semaphores, ... */
//  /* USER CODE END RTOS_SEMAPHORES */
//
//  /* USER CODE BEGIN RTOS_TIMERS */
//  /* start timers, add new ones, ... */
//  /* USER CODE END RTOS_TIMERS */
//
//  /* USER CODE BEGIN RTOS_QUEUES */
//  /* add queues, ... */
//  /* USER CODE END RTOS_QUEUES */
//
//  /* Create the thread(s) */
//  /* creation of defaultTask */
//  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
//
//  /* USER CODE BEGIN RTOS_THREADS */
//  /* add threads, ... */
//  /* USER CODE END RTOS_THREADS */
//
//  /* USER CODE BEGIN RTOS_EVENTS */
//  /* add events, ... */
//  /* USER CODE END RTOS_EVENTS */
//
//  /* Start scheduler */
//  osKernelStart();
//
//  /* We should never get here as control is now taken by the scheduler */
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//    /* USER CODE END WHILE */
//
//    /* USER CODE BEGIN 3 */
//  }
//  /* USER CODE END 3 */
//}
//
///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
//  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
//  RCC_OscInitStruct.MSICalibrationValue = 0;
//  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///* USER CODE BEGIN 4 */
//
///* USER CODE END 4 */
//
///* USER CODE BEGIN Header_StartDefaultTask */
///**
//  * @brief  Function implementing the defaultTask thread.
//  * @param  argument: Not used
//  * @retval None
//  */
///* USER CODE END Header_StartDefaultTask */
//void StartDefaultTask(void *argument)
//{
//  /* USER CODE BEGIN 5 */
//  /* Infinite loop */
//  for(;;)
//  {
//    osDelay(1);
//  }
//  /* USER CODE END 5 */
//}
//
///**
//  * @brief  Period elapsed callback in non blocking mode
//  * @note   This function is called  when TIM16 interrupt took place, inside
//  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
//  * a global variable "uwTick" used as application time base.
//  * @param  htim : TIM handle
//  * @retval None
//  */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  /* USER CODE BEGIN Callback 0 */
//
//  /* USER CODE END Callback 0 */
//  if (htim->Instance == TIM16) {
//    HAL_IncTick();
//  }
//  /* USER CODE BEGIN Callback 1 */
//
//  /* USER CODE END Callback 1 */
//}
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
