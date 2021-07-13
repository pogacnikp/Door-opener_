/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
BZ_Def_t buzzer;
LIN_Def_t lin_comm;
MC_Def_t motor_control;
MD_Def_t motor_diag;
PS_Def_t power_supply;
ST_Def_t switch_struct;

/* Machine states initialized with initial state */
uint8_t MainState = MS_MAIN_START;
uint8_t LockingState = MS_LCK_MOTOR;

/* ADC 4 channel buffer */
uint32_t ADCbuff[ADC_CH_NUM];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */
static void HAL_BZ_setPWM(TIM_HandleTypeDef* htim, uint32_t rawFrequency, uint32_t rawDutyCycle);
static void HAL_MC_setPWM(TIM_HandleTypeDef* htim, uint32_t rawFrequency, uint32_t rawDutyCycle);
static void DO_switchSateLocking(bool setCmd);
static void DO_turnOffActuators(void);
static void DO_writeActuatorsOut(void);
static void Modules_Init(void);

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

  /* Init modules */
  Modules_Init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_TIM14_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim14);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &ADCbuff, ADC_CH_NUM);

  HAL_UART_Receive_DMA(&huart1, LIN_getpBuff(&lin_comm), LIN_BUFF_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t *LINdata;
  uint8_t prevMainState = 0;
  while (1){

	  /* On main state change turn actuators OFF */
	  if (prevMainState != MainState){
		  DO_turnOffActuators();
	  }

	  switch(MainState){

		 case MS_MAIN_START:
			if (PS_getSupplyType(&power_supply) == PS_AC){
				MainState = MS_MAIN_S1;
			}
			else if (PS_getSupplyType(&power_supply) == PS_DC){
				MainState = MS_MAIN_S2;
			}
			break;

		 case MS_MAIN_S1:

			/* If period is correct, then unlock door, else lock */
			if (PS_isTcorrect(&power_supply) == true){
				/* Set locking command */
				DO_switchSateLocking(DO_SET_UNLOCK);
			}
			else{
				DO_switchSateLocking(DO_SET_LOCK);
			}

			LINdata = LIN_getpData(&lin_comm);
			//Add data compare function, if data are correct, go to MS_MAIN_S3
			break;

		 case MS_MAIN_S2:

			/* If DC edge is rising, then unlock door, else lock */
			if (PS_getDCEdgeType(&power_supply) == PS_EDGE_RISING){
				/* Set locking command */
				DO_switchSateLocking(DO_SET_UNLOCK);
			}
			else if (PS_getDCEdgeType(&power_supply) == PS_EDGE_FALLING){
				DO_switchSateLocking(DO_SET_LOCK);
			}

			LINdata = LIN_getpData(&lin_comm);
			//Add data compare function, if data are correct, go to MS_MAIN_S3
			break;

		 case MS_MAIN_S3:

			LINdata = LIN_getpData(&lin_comm);
			// If correct data lock, else unlock
			//Add data compare function, if data are correct to go out of comm. mode, go to MS_MAIN_START
			break;

		 default:
			MainState = MS_MAIN_START;
			break;
	  }

	  /* Store previous state */
	  prevMainState = MainState;

	  /* Write in actuators outputs */
	  DO_writeActuatorsOut();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_7CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_2);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_1);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 16-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 16-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 1000-1;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim17, TIM_CHANNEL_1);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_LIN_Init(&huart1, UART_LINBREAKDETECTLENGTH_10B) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */
/**
  * @brief Set PWM duty cycle and frequency
  * @param rawFrequency			Raw frequency value to set PWM frequency
  * @param rawDutyCycle			Raw duty cycle value to set PWM duty cycle
  * @retval None
  */
static void HAL_BZ_setPWM(TIM_HandleTypeDef* htim, uint32_t rawFrequency, uint32_t rawDutyCycle){

	/* Autoreload register for frequency */
	__HAL_TIM_SET_AUTORELOAD(htim, rawFrequency);

	/* Compare register (duty cycle) */
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, rawDutyCycle);
}

/**
  * @brief Set PWM duty cycle and frequency
  * @param rawFrequency			Raw frequency value to set PWM frequency
  * @param rawDutyCycle			Raw duty cycle value to set PWM duty cycle
  * @retval None
  */
static void HAL_MC_setPWM(TIM_HandleTypeDef* htim, uint32_t rawFrequency, uint32_t rawDutyCycle){

	//Invert raw Frequency
	__HAL_TIM_SET_AUTORELOAD(htim, rawFrequency);

	//Compare register (duty cycle)
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, rawDutyCycle);
}


/**
  * @brief Set door command to lock/unclok and switch locking states based on that
  * @param setCmd		Command to lock/unlock door
  * @retval None
  */
static void DO_switchSateLocking(bool setCmd){
	static bool prevCmd;

	/* If command has changed, then reset states */
	if (prevCmd != setCmd){
		LockingState =  MS_LCK_MOTOR;
	}
	switch(LockingState){
		 /* Set motor control properties */
		 case MS_LCK_MOTOR:
			//Correct motor direction according to locking/unlocking
			if (setCmd == DO_SET_UNLOCK){
				MC_setDirection(&motor_control, MC_FORWARD);
			}
			else if (setCmd == DO_SET_LOCK){
				MC_setDirection(&motor_control, MC_BACKWARD);
			}

			/* Start the motor */
			MC_Run(&motor_control, MC_START);

			LockingState = MS_LCK_END_POSIT;
			break;

		 /* Waiting for end position */
		 case MS_LCK_END_POSIT:
			if (MD_isEndPosition(&motor_diag) == true){
				/* Stop the motor */
				MC_Run(&motor_control, MC_STOP);

				if (setCmd == DO_SET_LOCK){
					/* If locking, at end position set buzzer */
					BZ_setOnOff(&buzzer, BZ_ON);
				}

				/* Go to idle state*/
				LockingState = MS_LCK_IDLE;
			}
			break;

		 /* Idle state */
		 case MS_LCK_IDLE:
			/* wait in idle until command change */
			break;

		 default:
			break;
	}

	/* Store previous command */
	prevCmd = setCmd;
}

/**
  * @brief Turn off all actuators
  * @param None
  * @retval None
  */
static void DO_turnOffActuators(void){
	MC_Run(&motor_control, MC_STOP);
	BZ_setOnOff(&buzzer, BZ_OFF);
}

/**
  * @brief Write on actuators outputs
  * @param None
  * @retval None
  */
static void DO_writeActuatorsOut(void){
	/* Motor */
	if (MC_isWriteOutFlgSet(&motor_control) == true){
		MC_clearWriteOutFlg(&motor_control);

		/* Set frequency and duty cycle in PWM channels */
		HAL_MC_setPWM(&htim3, MC_getRawFrequency(&motor_control), MC_getRawSpeedCH1(&motor_control));
		HAL_MC_setPWM(&htim17, MC_getRawFrequency(&motor_control), MC_getRawSpeedCH2(&motor_control));
	}

	/* Buzzer */
	if (BZ_isWriteOutFlgSet(&buzzer) == true){
		BZ_clearWriteOutFlg(&buzzer);

		HAL_BZ_setPWM(&htim2, BZ_getRawFrequency(&buzzer), BZ_getRawDutyCycle(&buzzer));
	}
}

/**
  * @brief Init modules
  * @param None
  * @retval None
  */
static void Modules_Init(void)
{
	BZ_Freq_InitLimits(&buzzer, 10000, 1000000);
	BZ_setDutyCycle(&buzzer, 0);
	BZ_setFrequency(&buzzer, 0);
	BZ_setBeepLength(&buzzer, 1000);

	/* Init current slave ID */
	LIN_Init_SlaveID(&lin_comm, 23);

	MC_Freq_InitLimits(&motor_control, 1000, 1000000);
	MC_Speed_InitLimits(&motor_control, 100);
	MC_setSpeed(&motor_control, 0);
	MC_setFrequency(&motor_control, 0);

	/* 12 bit ADC */
	PS_Init_maxAmplitude(&power_supply, 4095);

	/* Ts = 4 channels * ADC_CLK * ADC cycles = 4 * 1us * 20 = 80 us */
	PS_Init_samplingPeriod(&power_supply, 80);
	/* Td = 20 ms */
	PS_Init_detectionPeriod(&power_supply, 20);

	/* Debounce time = 100 ms */
	ST_Init_debounceTime(&switch_struct, 100);

	/* When out of standby mode, read switch level from flash */
	ST_Init_SwitchLevel(&switch_struct, (uint32_t) FLASH_read8Bytes(SWITCH_LEVEL_ADDR));
}

/* Callback funtion: timer has reset */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	/* If our 1 ms timer has reset */
	if (htim == &htim14){

		/* Switch */

		/* Count 1 ms counters in switch structure */
		ST_countMillis(&switch_struct);

		/* Has counter reached desired value */
		if (ST_isCounted(&switch_struct)){
			ST_disableCounter(&switch_struct);

			/* After debounce time read switch state */
			ST_readSwitchState(&switch_struct, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0));

			/* Write switch level in FLASH */
			FLASH_write8Bytes(SWITCH_LEVEL_ADDR, (uint64_t) ST_getSwitchLevel(&switch_struct));
		}

		/* Buzzer */

		/* Count 1 ms counters in buzzer structure */
		BZ_countMillis(&buzzer);
	}
}

/* Callback funtion for button rising edge */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin){

	/* If ext interrupt occurred on PA0 */
	if (GPIO_Pin == GPIO_PIN_0){

		/* Enable switch counter for debounce */
		ST_enableCounter(&switch_struct);
	}
}

/* Callback funtion for button falling edge */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin){

	/* If ext interrupt occurred on PA0 */
	if (GPIO_Pin == GPIO_PIN_0){

		/* Enable switch counter for debounce */
		ST_enableCounter(&switch_struct);
	}
}

/* Callback funtion for ADC conversion complete */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){

	if (hadc == &hadc1){
		/* Read data from ADC buffer */
		MD_measureVoltage(&motor_diag, ADCbuff[0], ADCbuff[1]);
		MD_measureCurrent(&motor_diag, ADCbuff[2]);
		PS_readSample(&power_supply, ADCbuff[3]);
	}
}

/* Callback funtion for UART LIN reception complete */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	LIN_processBuffer(&lin_comm);

	/* Continue to receive data */
	HAL_UART_Receive_DMA(&huart1, LIN_getpBuff(&lin_comm), LIN_BUFF_SIZE);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
