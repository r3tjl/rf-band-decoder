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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "string.h"
uint32_t freq = 0;
uint32_t current_freq = 0;
int count = 0;
int flag_band = 0;
int previous_flag_band = 0;
int flag_ptt = 0;
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void ResetAllOuts(){
	LL_GPIO_ResetOutputPin(BAND1_GPIO_Port, BAND1_Pin);
	LL_GPIO_ResetOutputPin(BAND2_GPIO_Port, BAND2_Pin);
	LL_GPIO_ResetOutputPin(BAND3_GPIO_Port, BAND3_Pin);
	LL_GPIO_ResetOutputPin(BAND4_GPIO_Port, BAND4_Pin);
	LL_GPIO_ResetOutputPin(BAND5_GPIO_Port, BAND5_Pin);
	LL_GPIO_ResetOutputPin(BAND6_GPIO_Port, BAND6_Pin);
	LL_GPIO_ResetOutputPin(BAND7_GPIO_Port, BAND7_Pin);
	LL_GPIO_ResetOutputPin(BAND8_GPIO_Port, BAND8_Pin);
}

void TestOuts(){
	LL_GPIO_SetOutputPin(BAND1_GPIO_Port, BAND1_Pin);
	LL_mDelay(200);
	LL_GPIO_SetOutputPin(BAND2_GPIO_Port, BAND2_Pin);
	LL_mDelay(200);
	LL_GPIO_SetOutputPin(BAND3_GPIO_Port, BAND3_Pin);
	LL_mDelay(200);
	LL_GPIO_SetOutputPin(BAND4_GPIO_Port, BAND4_Pin);
	LL_mDelay(200);
	LL_GPIO_SetOutputPin(BAND5_GPIO_Port, BAND5_Pin);
	LL_mDelay(200);
	LL_GPIO_SetOutputPin(BAND6_GPIO_Port, BAND6_Pin);
	LL_mDelay(200);
	LL_GPIO_SetOutputPin(BAND7_GPIO_Port, BAND7_Pin);
	LL_mDelay(200);
	LL_GPIO_SetOutputPin(BAND8_GPIO_Port, BAND8_Pin);
	LL_mDelay(200);
	LL_GPIO_SetOutputPin(PTT_OUT_GPIO_Port, PTT_OUT_Pin);
	LL_mDelay(200);
	LL_GPIO_ResetOutputPin(PTT_OUT_GPIO_Port, PTT_OUT_Pin);
	ResetAllOuts();
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if ((htim == &htim16)&&(LL_GPIO_IsInputPinSet(PTT_IN_GPIO_Port, PTT_IN_Pin) == 0)) {
		HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
		uint16_t count_main = __HAL_TIM_GET_COUNTER(&htim1);
		uint16_t count_secondary = __HAL_TIM_GET_COUNTER(&htim3);
		uint16_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
		uint32_t freq_raw = count_main + (count_secondary * (arr + 1));
		current_freq = freq_raw>>2;
		if (current_freq >= 1) count++; else count = 0;
		if (count == 3) {
			freq = current_freq;
			count=0;
		}
		//LL_GPIO_TogglePin(BAND1_GPIO_Port, BAND1_Pin);
		__HAL_TIM_SET_COUNTER(&htim1, 0x0000);
		__HAL_TIM_SET_COUNTER(&htim3, 0x0000);
		 HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	}
}

void SetBand(){
	if ((freq >= 900)&&(freq <= 2900)) {flag_band=1; freq = 0;}
	if ((freq >= 2950)&&(freq <= 4500)) {flag_band=2; freq = 0;}
	if ((freq >= 6500)&&(freq <= 7500)) {flag_band=3; freq = 0;}
	if ((freq >= 9000)&&(freq <= 11000)) {flag_band=4; freq = 0;}
	if ((freq >= 13000)&&(freq <=15000)) {flag_band=5; freq = 0;}
	if ((freq >= 17000)&&(freq <= 19000)) {flag_band=6; freq = 0;}
	if ((freq >= 20000)&&(freq <= 22000)) {flag_band=7; freq = 0;}
	if ((freq >= 23000)&&(freq <= 31000)) {flag_band=8; freq = 0;}
	if (previous_flag_band == flag_band) flag_ptt = 1; else flag_ptt  = 0;
}


void SetOuts() {
	if (flag_band == 1) {
		previous_flag_band = flag_band;
		ResetAllOuts();
		LL_GPIO_SetOutputPin(BAND1_GPIO_Port, BAND1_Pin);
		flag_ptt = 1;
	}
	if (flag_band == 2) {
		previous_flag_band = flag_band;
		ResetAllOuts();
		LL_GPIO_SetOutputPin(BAND2_GPIO_Port, BAND2_Pin);
		flag_ptt = 1;
	}
	if (flag_band == 3) {
		previous_flag_band = flag_band;
		ResetAllOuts();
		LL_GPIO_SetOutputPin(BAND3_GPIO_Port, BAND3_Pin);
		flag_ptt = 1;
	}
	if (flag_band == 4) {
		previous_flag_band = flag_band;
		ResetAllOuts();
		LL_GPIO_SetOutputPin(BAND4_GPIO_Port, BAND4_Pin);
		flag_ptt = 1;
	}
	if (flag_band == 5) {
		previous_flag_band = flag_band;
		ResetAllOuts();
		LL_GPIO_SetOutputPin(BAND5_GPIO_Port, BAND5_Pin);
		flag_ptt = 1;
	}
	if (flag_band == 6) {
		previous_flag_band = flag_band;
		ResetAllOuts();
		LL_GPIO_SetOutputPin(BAND6_GPIO_Port, BAND6_Pin);
		flag_ptt = 1;
	}
	if (flag_band == 7) {
		previous_flag_band = flag_band;
		ResetAllOuts();
		LL_GPIO_SetOutputPin(BAND7_GPIO_Port, BAND7_Pin);
		flag_ptt = 1;
	}
	if (flag_band == 8) {
		previous_flag_band = flag_band;
		ResetAllOuts();
		LL_GPIO_SetOutputPin(BAND8_GPIO_Port, BAND8_Pin);
		flag_ptt = 1;
	}
	if (flag_band == 0) {
		flag_ptt = 0;
		ResetAllOuts();
	}

}

void CheckPTT() {
	if ((LL_GPIO_IsInputPinSet(PTT_IN_GPIO_Port, PTT_IN_Pin) == 0)&&(flag_ptt)) {
		LL_mDelay(10);
		LL_GPIO_SetOutputPin(PTT_OUT_GPIO_Port, PTT_OUT_Pin);
	} else {
		LL_GPIO_ResetOutputPin(PTT_OUT_GPIO_Port, PTT_OUT_Pin);
	}
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  LL_TIM_EnableCounter(TIM17);
  LL_TIM_EnableIT_UPDATE(TIM17);
  //TestOuts();
  HAL_TIM_Base_Start_IT(&htim16);
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  //HAL_TIM_Base_Start(&htim16);
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  SetBand();
	  SetOuts();
	  CheckPTT();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 63999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 5;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 4;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM17);

  /* TIM17 interrupt Init */
  NVIC_SetPriority(TIM17_IRQn, 0);
  NVIC_EnableIRQ(TIM17_IRQn);

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  TIM_InitStruct.Prescaler = 63999;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 499;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM17, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM17);
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_ResetOutputPin(PTT_OUT_GPIO_Port, PTT_OUT_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BAND1_GPIO_Port, BAND1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BAND2_GPIO_Port, BAND2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BAND3_GPIO_Port, BAND3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BAND4_GPIO_Port, BAND4_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BAND5_GPIO_Port, BAND5_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BAND6_GPIO_Port, BAND6_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BAND7_GPIO_Port, BAND7_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BAND8_GPIO_Port, BAND8_Pin);

  /**/
  GPIO_InitStruct.Pin = PTT_IN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(PTT_IN_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = PTT_OUT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(PTT_OUT_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BAND1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BAND1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BAND2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BAND2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BAND3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BAND3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BAND4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BAND4_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BAND5_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BAND5_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BAND6_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BAND6_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BAND7_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BAND7_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BAND8_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BAND8_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
