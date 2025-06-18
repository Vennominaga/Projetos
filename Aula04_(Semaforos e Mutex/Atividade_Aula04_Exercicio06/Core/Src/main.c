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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<string.h>
#include<stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*#define TASK_ADC_INIT 0x001*/
#define ADC_SAMPLES 100
#define SIZE_HISTORY_ADC 10



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

/* Definitions for TaskADC */
osThreadId_t TaskADCHandle;
const osThreadAttr_t TaskADC_attributes = {
  .name = "TaskADC",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskCalc */
osThreadId_t TaskCalcHandle;
const osThreadAttr_t TaskCalc_attributes = {
  .name = "TaskCalc",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TaskSave */
osThreadId_t TaskSaveHandle;
const osThreadAttr_t TaskSave_attributes = {
  .name = "TaskSave",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TaskSend */
osThreadId_t TaskSendHandle;
const osThreadAttr_t TaskSend_attributes = {
  .name = "TaskSend",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for semStart */
osSemaphoreId_t semStartHandle;
const osSemaphoreAttr_t semStart_attributes = {
  .name = "semStart"
};
/* Definitions for semCalc */
osSemaphoreId_t semCalcHandle;
const osSemaphoreAttr_t semCalc_attributes = {
  .name = "semCalc"
};
/* Definitions for semSave */
osSemaphoreId_t semSaveHandle;
const osSemaphoreAttr_t semSave_attributes = {
  .name = "semSave"
};
/* Definitions for semSend */
osSemaphoreId_t semSendHandle;
const osSemaphoreAttr_t semSend_attributes = {
  .name = "semSend"
};
/* Definitions for TaskADCInit */
osEventFlagsId_t TaskADCInitHandle;
const osEventFlagsAttr_t TaskADCInit_attributes = {
  .name = "TaskADCInit"
};
/* USER CODE BEGIN PV */
double ADC_Buffer[ADC_SAMPLES] = {0};
double HistoryADCAverage[SIZE_HISTORY_ADC] = {0};
double average;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
void StartTaskADC(void *argument);
void StartTaskCalc(void *argument);
void StartTaskSave(void *argument);
void StartTaskSend(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == B1_Pin)
	{
		/*osEventFlagsSet(TaskADCInitHandle,TASK_ADC_INIT);*/
		osSemaphoreRelease(semStartHandle);

	}

}

double map(uint16_t x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of semStart */
  semStartHandle = osSemaphoreNew(1, 0, &semStart_attributes);

  /* creation of semCalc */
  semCalcHandle = osSemaphoreNew(1, 0, &semCalc_attributes);

  /* creation of semSave */
  semSaveHandle = osSemaphoreNew(1, 0, &semSave_attributes);

  /* creation of semSend */
  semSendHandle = osSemaphoreNew(1, 0, &semSend_attributes);

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
  /* creation of TaskADC */
  TaskADCHandle = osThreadNew(StartTaskADC, NULL, &TaskADC_attributes);

  /* creation of TaskCalc */
  TaskCalcHandle = osThreadNew(StartTaskCalc, NULL, &TaskCalc_attributes);

  /* creation of TaskSave */
  TaskSaveHandle = osThreadNew(StartTaskSave, NULL, &TaskSave_attributes);

  /* creation of TaskSend */
  TaskSendHandle = osThreadNew(StartTaskSend, NULL, &TaskSend_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of TaskADCInit */
  TaskADCInitHandle = osEventFlagsNew(&TaskADCInit_attributes);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart1.Init.Mode = UART_MODE_TX_RX;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTaskADC */
/**
  * @brief  Function implementing the TaskADC thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskADC */
void StartTaskADC(void *argument)
{
  /* USER CODE BEGIN 5 */
	const char *MSG_INIT = "Iniciando a Aquicicao de Dados do ADC ...";
	/*const char *MSG_INIT_ADC = "ADC(Anologic Digitak Converter) Inicializado com Sucesso";*/
	const char *MSG_SUCESS = "Leitura Realizada com Sucesso";
	const char *MSG_ERROR = "OCORREU UM ERRO NA LEITURA";


	uint16_t adc_value;


  /* Infinite loop */
  for(;;)
  {

	  /*uint32_t flag = osEventFlagsWait(TaskADCInitHandle,TASK_ADC_INIT,osFlagsWaitAny,osWaitForever);*/

	  /*if(flag == TASK_ADC_INIT)
	  {
		  HAL_UART_Transmit(&huart1,(uint8_t *)MSG_INIT,strlen(MSG_INIT),100);

		  osEventFlagsClear(TaskADCInitHandle,TASK_ADC_INIT);
	  }*/

	  osSemaphoreAcquire(semStartHandle, osWaitForever);
	  HAL_UART_Transmit(&huart1,(uint8_t *)MSG_INIT,strlen(MSG_INIT),100);

	  for(uint8_t i = 0;i<ADC_SAMPLES;i++)
	  {

		  HAL_ADC_Start(&hadc1);
		  if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
		  {
			  adc_value = HAL_ADC_GetValue(&hadc1);
			  HAL_UART_Transmit(&huart1,(uint8_t *)MSG_SUCESS,strlen(MSG_SUCESS),100);
			  ADC_Buffer[i] = map(adc_value,0,4095,0,3.3);
			  osDelay(100);

		  }

		  else
		  {
			  HAL_UART_Transmit(&huart1,(uint8_t *)MSG_ERROR,strlen(MSG_ERROR),100);
			  --i;

		  }

	  }

	  osSemaphoreRelease(semCalcHandle);





    osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskCalc */
/**
* @brief Function implementing the TaskCalc thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskCalc */
void StartTaskCalc(void *argument)
{
  /* USER CODE BEGIN StartTaskCalc */
	const char *MSG_INIT = "Iniciando o Calculo da Media";
	const char *MSG_SUCESS = "Media Calculada com Sucesso";
	/*const char *MSG_ERROR = "Error no Calculo da Media";*/
  /* Infinite loop */
	double sum  = 0;
  for(;;)
  {
	  osSemaphoreAcquire(semCalcHandle,osWaitForever);
	  HAL_UART_Transmit(&huart1,(uint8_t *)MSG_INIT,strlen(MSG_INIT),100);
	  osDelay(200);

	  for(uint8_t i = 0;i<ADC_SAMPLES;i++)
	  {
		  sum+=ADC_Buffer[i];

	  }
	  average = sum/ADC_SAMPLES;
	  sum = 0;
	  HAL_UART_Transmit(&huart1,(uint8_t *)MSG_SUCESS,strlen(MSG_SUCESS),100);
	  osDelay(200);
	  osSemaphoreRelease(semSaveHandle);


    osDelay(100);
  }
  /* USER CODE END StartTaskCalc */
}

/* USER CODE BEGIN Header_StartTaskSave */
/**
* @brief Function implementing the TaskSave thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskSave */
void StartTaskSave(void *argument)
{
  /* USER CODE BEGIN StartTaskSave */
	const char *MSG_INIT = "Salvando no Historico de Medias";
	const char *MSG_SUCESS = "Media Salva com Sucesso !";
	static uint8_t index = 0;
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(semSaveHandle,osWaitForever);
	  HAL_UART_Transmit(&huart1,(uint8_t *)MSG_INIT,strlen(MSG_INIT),100);
	  osDelay(200);
	  HistoryADCAverage[index] = average;
	  HAL_UART_Transmit(&huart1,(uint8_t *)MSG_SUCESS,strlen(MSG_SUCESS),100);
	  osDelay(200);
	  index = (index + 1) % 10;

	  osSemaphoreRelease(semSendHandle);
      osDelay(100);
  }
  /* USER CODE END StartTaskSave */
}

/* USER CODE BEGIN Header_StartTaskSend */
/**
* @brief Function implementing the TaskSend thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskSend */
void StartTaskSend(void *argument)
{
  /* USER CODE BEGIN StartTaskSend */

	const char* MSG_INIT = "Enviando a Media,Aguarde...";
	char send[50];
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(semSendHandle,osWaitForever);
	  HAL_UART_Transmit(&huart1,(uint8_t *)MSG_INIT,strlen(MSG_INIT),100);
	  osDelay(200);

	  snprintf(send,50,"Media Enviada : %.2lf",average);
	  HAL_UART_Transmit(&huart1,(uint8_t *)send, strlen(send),100);


    osDelay(100);
  }
  /* USER CODE END StartTaskSend */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
