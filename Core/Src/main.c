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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include "mpu6050.h"
#include "orientation.h"

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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* Definitions for DebugTask */
osThreadId_t DebugTaskHandle;
const osThreadAttr_t DebugTask_attributes = {
  .name = "DebugTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SensorReadTask */
osThreadId_t SensorReadTaskHandle;
const osThreadAttr_t SensorReadTask_attributes = {
  .name = "SensorReadTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for OrientationTask */
osThreadId_t OrientationTaskHandle;
const osThreadAttr_t OrientationTask_attributes = {
  .name = "OrientationTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AngularRateQueue */
osMessageQueueId_t AngularRateQueueHandle;
const osMessageQueueAttr_t AngularRateQueue_attributes = {
  .name = "AngularRateQueue"
};
/* USER CODE BEGIN PV */

MPU6050_Handle_t hmpu;
Orientation_Handle_t ho;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
void DebugTask_Start(void *argument);
void SensorReadTask_Start(void *argument);
void OrientationTask_Start(void *argument);

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  //Ensure MPU boots up completely
  HAL_Delay(10);
  //MPU6050 Inits
  if (MPU6050_Init(&hmpu, &hi2c1, MPU6050_I2C_ADDRESS) != MPU6050_OK)
  {
	  Error_Handler();
  }

  if (MPU6050_DisableInterrupt(&hmpu, ALL_INT) != MPU6050_OK)
  {
	  Error_Handler();
  }
  if (MPU6050_InterruptConfig(&hmpu, INT_LEVEL_ACTIVE_HIGH) != MPU6050_OK)
  {
	  Error_Handler();
  }
  if (MPU6050_EnableInterrupt(&hmpu, RAW_RDY_INT) != MPU6050_OK)
  {
	  Error_Handler();
  }
  if (MPU6050_ConfigureLowPassFilter(&hmpu, DLPF_CFG_21HZ) != MPU6050_OK)
  {
	  Error_Handler();
  }
  //to make interrupts less frequent
  if (MPU6050_SetSampleRatePrescaler(&hmpu, 10 - 1) != MPU6050_OK)
  {
	  Error_Handler();
  }

  MPU6050_AccelCalibration(&hmpu, 100);
  MPU6050_GyroCalibration(&hmpu, 100);
  MPU6050_GetAndClearInterruptStatus(&hmpu);

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

  /* Create the queue(s) */
  /* creation of AngularRateQueue */
  AngularRateQueueHandle = osMessageQueueNew (8, 32, &AngularRateQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of DebugTask */
  DebugTaskHandle = osThreadNew(DebugTask_Start, NULL, &DebugTask_attributes);

  /* creation of SensorReadTask */
  SensorReadTaskHandle = osThreadNew(SensorReadTask_Start, NULL, &SensorReadTask_attributes);

  /* creation of OrientationTask */
  OrientationTaskHandle = osThreadNew(OrientationTask_Start, NULL, &OrientationTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */

  //To start handling interrupts
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : MPU6050_INT_Pin */
  GPIO_InitStruct.Pin = MPU6050_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MPU6050_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (GPIO_Pin == MPU6050_INT_Pin)
	{
		if (SensorReadTaskHandle) // Check if MPU is ready and task exists
		{
			// Only notify the task. Do NO I2C operations here.
			xTaskNotifyFromISR((TaskHandle_t)SensorReadTaskHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_DebugTask_Start */
/**
  * @brief  Function implementing the UART_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_DebugTask_Start */
void DebugTask_Start(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	char msg[64];
  for(;;)
  {
    sprintf(msg, "Roll: %0.2f, Pitch: %0.2f\r\n", ho.roll, ho.pitch);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_SensorReadTask_Start */
/**
* @brief Function implementing the SensorReadTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SensorReadTask_Start */
void SensorReadTask_Start(void *argument)
{
  /* USER CODE BEGIN SensorReadTask_Start */
	MPU6050_Data_t MPU_data;
	uint8_t mpu_int_status;
	uint32_t previous_tick = osKernelGetTickCount();
	uint32_t current_tick;
	float dt;

	/* Infinite loop */
	for (;;) {
		// Enter blocked state until direct notification received from ISR
		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
		current_tick = osKernelGetTickCount();
		dt = (current_tick - previous_tick) / 1000.0f;
		previous_tick = current_tick;

		// Now in task context, safe to do I2C read to get interrupt status
		mpu_int_status = MPU6050_GetAndClearInterruptStatus(&hmpu);

		if (mpu_int_status & RAW_RDY_INT) {
			if (MPU6050_Read(&hmpu, &MPU_data) != MPU6050_OK) {
				continue;
			}
			MPU_data.dt = dt;
			//send to orientation task
			osMessageQueuePut(AngularRateQueueHandle, &MPU_data, 0, 0);
		}
		// else if (mpu_int_status & OTHER_INT_FLAG) { handle other interrupt types }
	}
  /* USER CODE END SensorReadTask_Start */
}

/* USER CODE BEGIN Header_OrientationTask_Start */
/**
* @brief Function implementing the OrientationTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OrientationTask_Start */
void OrientationTask_Start(void *argument)
{
  /* USER CODE BEGIN OrientationTask_Start */
  /* Infinite loop */
 	MPU6050_Data_t mpu_data;
  for(;;)
  {
    osMessageQueueGet(AngularRateQueueHandle, &mpu_data, NULL, osWaitForever);
    Orientation_Update(&ho, &mpu_data);
  }
  /* USER CODE END OrientationTask_Start */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
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
