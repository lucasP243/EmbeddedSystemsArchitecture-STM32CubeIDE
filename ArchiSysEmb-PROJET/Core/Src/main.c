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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId COMM_MGRHandle;
osThreadId BT1_MGRHandle;
osThreadId INPUT_MGRHandle;
osMutexId debugLockHandle;
osStaticMutexDef_t debugLockControlBlock;
/* USER CODE BEGIN PV */
role_t role = ROLE_NONE;
cipher_key_t readKey;
cipher_key_t writeKey;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
void startCommMgr(void const * argument);
void startBT1Mgr(void const * argument);
void startInputMgr(void const * argument);

/* USER CODE BEGIN PFP */
void cipher(uint8_t*, size_t, cipher_key_t);
void decipher(uint8_t*, size_t, cipher_key_t);
cipher_key_t generateKey();
void readHeader(frame_t *, msg_type_t*, msg_size_t*);
void writeHeader(frame_t *, msg_type_t, msg_size_t);
bool readUART(UART_HandleTypeDef *, frame_t *, msg_size_t);
bool writeUART(UART_HandleTypeDef *, const frame_t *, msg_size_t);
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of debugLock */
  osMutexStaticDef(debugLock, &debugLockControlBlock);
  debugLockHandle = osMutexCreate(osMutex(debugLock));

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
  /* definition and creation of COMM_MGR */
  osThreadDef(COMM_MGR, startCommMgr, osPriorityAboveNormal, 0, 128);
  COMM_MGRHandle = osThreadCreate(osThread(COMM_MGR), NULL);

  /* definition and creation of BT1_MGR */
  osThreadDef(BT1_MGR, startBT1Mgr, osPriorityLow, 0, 128);
  BT1_MGRHandle = osThreadCreate(osThread(BT1_MGR), NULL);

  /* definition and creation of INPUT_MGR */
  osThreadDef(INPUT_MGR, startInputMgr, osPriorityNormal, 0, 128);
  INPUT_MGRHandle = osThreadCreate(osThread(INPUT_MGR), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	//HAL_UART_Transmit(&huart2, (uint8_t *) "\n\r*** Program Started ***\n\r", 27, 5);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 799;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void cipher(uint8_t *message, size_t messageSize, cipher_key_t key) {
	return;
}
void decipher(uint8_t *message, size_t messageSize, cipher_key_t key) {

}
cipher_key_t generateKey() {
	return 0; // TODO
}
void readHeader(frame_t *header, msg_type_t *msgType, msg_size_t *msgSize) {
	uint16_t header_unified;
	sscanf((char *) header, "%" SCNu16, &header_unified);

	*msgType = (msg_type_t)(header_unified & HEADER_TYPE_MASK);
	*msgSize = (msg_size_t)(header_unified & HEADER_TYPE_MASK);
}
void writeHeader(frame_t *header, msg_type_t msgType, msg_size_t msgSize) {
	uint16_t header_unified = (msgType << 13) + msgSize;

	sprintf((char *) header, "%" PRIu16, header_unified);
}
bool readUART(UART_HandleTypeDef *huart, frame_t *data, msg_size_t size) {

	HAL_StatusTypeDef status = HAL_UART_Receive(huart, data, size, TIMEOUT);

	char *label = "-----";
	if (huart == &huart1) {
		label = "UART1";
	}
	if (huart == &huart2) {
		label = "UART2";
	}
	if (huart == &huart3) {
		label = "UART3";
	}

	char *result = "--";
	if (status == HAL_OK) {
		result = "OK";
	}
	if (status == HAL_TIMEOUT) {
		result = "TO";
	}
	if (status == HAL_BUSY) {
		result = "BS";
	}
	if (status == HAL_ERROR) {
		result = "ER";
	}

	char debugMsg[15];
	sprintf(debugMsg, "[%s] R(%s): ", label, result);

	(void) osMutexWait(debugLockHandle, 0);
	HAL_UART_Transmit(&huart2, (uint8_t *) debugMsg, sizeof (debugMsg), TIMEOUT);
	(void) osMutexRelease(debugLockHandle);

	return (status == HAL_OK);
}
bool writeUART(UART_HandleTypeDef *huart, const frame_t *data, msg_size_t size) {

	HAL_StatusTypeDef status = HAL_UART_Transmit(huart, data, size, TIMEOUT);

	char *label = "-----";
	if (huart == &huart1) {
		label = "UART1";
	}
	if (huart == &huart2) {
		label = "UART2";
	}
	if (huart == &huart3) {
		label = "UART3";
	}

	char *result = "--";
	if (status == HAL_OK) {
		result = "OK";
	}
	if (status == HAL_TIMEOUT) {
		result = "TO";
	}
	if (status == HAL_BUSY) {
		result = "BS";
	}
	if (status == HAL_ERROR) {
		result = "ER";
	}

	char debugMsg[15];
	sprintf(debugMsg, "[%s] W(%s): ", label, result);

	(void) osMutexWait(debugLockHandle, 0);
	HAL_UART_Transmit(&huart2, (uint8_t *) debugMsg, sizeof (debugMsg), TIMEOUT);
	HAL_UART_Transmit(&huart2, (uint8_t *) data, size, TIMEOUT);
	HAL_UART_Transmit(&huart2, (uint8_t *) "\n\r", 2, TIMEOUT);
	(void) osMutexRelease(debugLockHandle);

	return (status == HAL_OK);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_startCommMgr */
/**
 * @brief  Function implementing the COMM_MGR thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_startCommMgr */
void startCommMgr(void const * argument)
{
  /* USER CODE BEGIN 5 */

	//(void) osMutexWait(debugLockHandle, 0);
	HAL_UART_Transmit(&huart2, (uint8_t *) "test\n\r", 6, TIMEOUT);
	//(void) osMutexRelease(debugLockHandle);

	frame_t header[FRAME_HEADER_SIZE];
	frame_t ioBuffer[MAX_FRAME_SIZE];
	msg_type_t msgType;
	msg_size_t msgSize;

	memset(header, 0, FRAME_HEADER_SIZE);
	memset(ioBuffer, 0, MAX_FRAME_SIZE);
	/* Infinite loop */
	commMgr_i:
	while (role != ROLE_INPUT) // If ROLE_INPUT, terminate the thread
	{

		if (!readUART(&huart1, header, FRAME_HEADER_SIZE)) {
			continue;
		}

		readHeader(header, &msgType, &msgSize);

		if (msgType == MSG_TYPE_CLEF) {
			role = ROLE_TRANSIT;

			if (!readUART(&huart1, ioBuffer, msgSize)) {
				continue;
			}

			readKey = ioBuffer[0];
			cipher(ioBuffer, msgSize, readKey);
			writeHeader(header, MSG_TYPE_OKCLEF, 1);

			if (!writeUART(&huart1, header, FRAME_HEADER_SIZE)) {
				continue;
			}

			if (!writeUART(&huart1, ioBuffer, 1)) {
				continue;
			}

			writeKey = generateKey();
			writeHeader(header, MSG_TYPE_CLEF, 1);
			sprintf((char *) ioBuffer, "%" PRIu8, writeKey);

			if (!writeUART(&huart3, header, FRAME_HEADER_SIZE)) {
				continue;
			}

			if (!writeUART(&huart3, ioBuffer, 1)) {
				continue;
			}

			if (!readUART(&huart3, header, FRAME_HEADER_SIZE)) {
				role = ROLE_OUTPUT;
				continue;
			}

			readHeader(header, &msgType, &msgSize);

			if (msgType != MSG_TYPE_OKCLEF) {
				HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // signal error
				continue;
			}

			(void) readUART(&huart3, ioBuffer, msgSize);

		} else if (msgType == MSG_TYPE_MSG) {

			if (!readUART(&huart1, ioBuffer, msgSize)) {
				continue;
			}

			writeHeader(header, MSG_TYPE_OKMSG, 0);

			if (!writeUART(&huart1, header, FRAME_HEADER_SIZE)) {
				continue;
			}

			decipher(ioBuffer, msgSize, readKey);

			if (role == ROLE_OUTPUT) {
				(void) writeUART(&huart2, ioBuffer, msgSize);
				continue;
			}

			cipher(ioBuffer, msgSize, writeKey);
			writeHeader(header, MSG_TYPE_MSG, msgSize);

			for (uint8_t fails = 0; fails < 3; fails++)
			{
				if (!writeUART(&huart3, header, FRAME_HEADER_SIZE)) {
					continue;
				}

				if (!writeUART(&huart3, ioBuffer, msgSize)) {
					continue;
				}

				if (!readUART(&huart3, header, FRAME_HEADER_SIZE)) {
					continue;
				}

				readHeader(header, &msgType, &msgSize);
				(void) readUART(&huart3, ioBuffer, msgSize);

				if (msgType == MSG_TYPE_OKMSG) {
					goto commMgr_i; // Done, proceed to next iteration
				}
			}

			Error_Handler();
		}

		osDelay(1);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startBT1Mgr */
/**
 * @brief Function implementing the BT1_MGR thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startBT1Mgr */
void startBT1Mgr(void const * argument)
{
  /* USER CODE BEGIN startBT1Mgr */
	/* Infinite loop */
	for (;;) {
		osDelay(1000);
	}
  /* USER CODE END startBT1Mgr */
}

/* USER CODE BEGIN Header_startInputMgr */
/**
 * @brief Function implementing the INPUT_MGR thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startInputMgr */
void startInputMgr(void const * argument)
{
  /* USER CODE BEGIN startInputMgr */
	/* Infinite loop */
	for (;;) {
		osDelay(1000);
	}
  /* USER CODE END startInputMgr */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	while (1) {
		osDelay(1);
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
