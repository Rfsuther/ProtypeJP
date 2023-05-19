/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//Used to Control Length of USBbuffer
#define txBuffSize 8
#define rxBuffSize 13

//should match number of channels on ADC1 and ADC2
#define numOfChansADC1 6
#define numOfChansADC2 1
#define numberOfLimbs 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

enum Sensor
{
	LL,
	LU,
	RL,
	RU,
	Bdy,
	Hd,
};


static uint8_t exoCounter[numberOfLimbs] = { 0 };
static uint16_t exoThresh[numberOfLimbs] = { 0 };
static int8_t exoCurrentState[numberOfLimbs] = { 0 };
static uint16_t exoStateCounter[numberOfLimbs] = { 0 };
struct errorCode
{
	uint8_t sensorCode;
	uint8_t comCode;
};

enum Sensor selectADC = LL;

//ADC PVs
volatile uint16_t Adc1Results[numOfChansADC1];
volatile uint16_t ADC2_val = 0;
uint16_t Adc1Loop[numOfChansADC1] = {0};

uint32_t Adc1Filt[numOfChansADC1] = { 0 };
uint16_t ADC2_LoopVal = 0;

//GPIO PVs
uint8_t buttonStatus = 0;
uint8_t buttonPrevious = 0;
//

//UART PVs
//use this to define number of consecutive highs before reset
uint16_t sensrErrCntThresh = 200;
//uint8_t txBuffer[txBuffSize] = { 'H', 'e', 'l', 'l', 'o', ' ', 'W', 'o', 'r', 'l', 'd', '\r', '\n' };
uint8_t USBtxBuffer[txBuffSize] = {1,2,3,4,5,6,7,8};
uint8_t BTtxBuffer[txBuffSize] = { 1, 2, 3, 4, 5, 6, 7, 8 };
uint8_t USBrxBuffer[rxBuffSize];
uint8_t BTrxBuffer[rxBuffSize];
uint8_t rxStorageBuffer[rxBuffSize];
uint8_t BTrxStorageBuffer[rxBuffSize];

//DEBUGING Vars

HAL_StatusTypeDef ADC1Status; //return status of ADC1 for debugging
HAL_StatusTypeDef ADC2Status; //return status of ADC2 for debugging
HAL_StatusTypeDef timer1Status; //return status of timer 1 for debugging
HAL_StatusTypeDef USBtxStatus; //return status on tranmit for debugging
HAL_StatusTypeDef USBrxStatus; //return status on receive for debugging
HAL_StatusTypeDef BTtxStatus; //return status on tranmit for debugging
HAL_StatusTypeDef BTrxStatus; //return status on receive for debugging

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

//Three reset functions for createing structs
void resetExoHitVars();
struct errorCode ReseterrorCode(void);

void calibrateSensor();
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
  MX_DMA_Init();
  MX_ADC2_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	//Start Periferals
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)Adc1Results, sizeof(Adc1Results) / sizeof(Adc1Results[0])) != HAL_OK)
		Error_Handler();	
	if (HAL_ADC_Start(&hadc2) != HAL_OK)
		Error_Handler();
	if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
		Error_Handler();
	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
		Error_Handler();
	
	//ConfigureDMA
	USBrxStatus = HAL_UARTEx_ReceiveToIdle_DMA(&huart3, USBrxBuffer, sizeof(USBrxBuffer) / sizeof(USBrxBuffer[0])); //CHANGE
	if (USBrxStatus != HAL_OK)
		Error_Handler();
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT); //remeber that I set DMA handel to extern in the .h file this may cause bugs
	
	BTrxStatus = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, BTrxBuffer, sizeof(BTrxBuffer) / sizeof(BTrxBuffer[0])); //CHANGE
	if (BTrxStatus != HAL_OK)
		Error_Handler();
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT); //remeber that I set DMA handel to extern in the .h file this may cause bugs
	
	

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
	//Base ADC Threshholds (need to calibrate final version
	exoThresh[0] =  1500; exoThresh[1] =  2450; exoThresh[2] =  950; exoThresh[3] =  250; exoThresh[4] =  3500; exoThresh[5] =  3500; 
	//TODO add auto calibrate
	calibrateSensor();
	
	
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	int loop;
  while (1)
  {
	  //Update State of Controller
	  
	  //Get adc values
	  
	  //No filter loop
	  //	  for (enum Sensor i = 0; i < numOfChansADC1; i++)
//		  {
//			  Adc1Loop[i] = Adc1Results[i];
//		  }
	  for (enum Sensor i = 0; i < numOfChansADC1; i++)
		  {
			  Adc1Filt[i] = Adc1Results[i];
		  }
	  
	  //ADD extra values for averageing filter
	  uint8_t numberOfTaps = 20;
	  for (enum Sensor j = 0; j < numberOfTaps; j++)
	  {
			  for (enum Sensor i = 0; i < numOfChansADC1; i++)
			  {
				  Adc1Filt[i] += Adc1Results[i];
			  }
		  HAL_Delay(2);
	  }
	  //average out values
	  for (enum Sensor i = 0; i < numOfChansADC1; i++)
	  {
		  Adc1Loop[i] = (uint16_t)((double)Adc1Filt[i] / (1.0*numberOfTaps));
	  }
	  

	  
	  ADC2_LoopVal = ADC2_val;
	  //Get PB values
	  
	  //Get UART values
	  buttonStatus = HAL_GPIO_ReadPin(PushB1_GPIO_Port, PushB1_Pin);
	  
	  
	  
	  	  //update ADC threshold values
	  if (buttonStatus == 1)
	  {
		  //increment channel counter
		  if (selectADC >= Hd)
		  {
			  selectADC = LL;
		  } 
		  else
		  {
			  selectADC += 1;
		  }
		  // update threshold value
		  exoThresh[selectADC] = ADC2_LoopVal;
		  buttonStatus = 0;
		  HAL_Delay(100);
		  //TODO reset error state of sensor on value change (alows for retime reset and adjustments
	  }
		//compare ADC to Thresh and count hits
	  for (enum Sensor sensorPos = LL; sensorPos < numberOfLimbs; sensorPos++)
	  {
		  //check if sensor is in error state and if so leave it alone
		  if (exoCurrentState[sensorPos] != -1)
		  {
			  //set flag if threshold is exceded
			  if (Adc1Loop[sensorPos] > exoThresh[sensorPos])
			  {
				  exoCurrentState[sensorPos] = 1;
			  }
			  else
			  {
				  exoCurrentState[sensorPos] = 0;
			  }
		  }
	  }	  
	  
	  
	  
	  uint8_t resetFlag = 0; //repalce this
	  //Reset Condition
	  
	  if (rxStorageBuffer[0] == 'r')
	  {
		  resetFlag = 1;
		  rxStorageBuffer[0] = 0;
	  }
	  if (resetFlag == 1)
	  {
		  //reset exoCounter and state varibles
		  resetExoHitVars();
		  HAL_Delay(500);
		  resetFlag = 0;
	  }
	  else if(0) //error flag goes here
	  {
		  while (1)
		  {
			  //TODO add error Trap UART
			  __NOP();
		  }
	  }
	  else //check State of punches
	  {
		  // iterate across sensors and store valus sensor error flag is -1 on exoCurrent state which is handeled above
		  for (enum Sensor sensorPos = 0; sensorPos < numberOfLimbs; sensorPos++)
		  {
			  if ((exoCurrentState[sensorPos] == 1) && (exoStateCounter[sensorPos] == 0))
			  {
				  //
				  exoCounter[sensorPos] += 1;
				  exoStateCounter[sensorPos] += 1; 
			  }
			  else if ((exoCurrentState[sensorPos] == 1) && (exoStateCounter[sensorPos]> 0) && (sensrErrCntThresh >= exoStateCounter[sensorPos]))
			  {
				  //
				  exoStateCounter[sensorPos] += 1; 
			  }
			  //Entered Error State Must Reset System
			  else if((exoCurrentState[sensorPos] == 1) && (exoStateCounter[sensorPos] > sensrErrCntThresh))
			  {
				  exoCurrentState[sensorPos] = -1;			  
			  }
			  else if(exoCurrentState[sensorPos] == 0)
			  {
				  exoStateCounter[sensorPos] = 0; 
			  }
			  else
			  {
				  __NOP(); //Illegal State;
			  }
		  }
	  }

//	  
//	  
	  //Send results to master
	  HAL_Delay(100);
	  	  for (int i = 0; i < numberOfLimbs; i++)
	  {
		  USBtxBuffer[i] = (uint8_t) exoCounter[i];  //CHANGE
		  BTtxBuffer[i] = (uint8_t) exoCounter[i]; //CHANGE //TODO Combine Buffers for optimizatition
	  }
	  
	  USBtxStatus = HAL_UART_Transmit(&huart3, USBtxBuffer, sizeof(USBtxBuffer) / sizeof(USBtxBuffer[0]), 100); //CHANGE
	  //USBrxStatus = HAL_UART_Receive(&huart3, USBrxBuffer, sizeof(USBrxBuffer) / sizeof(USBrxBuffer[0]), 500);
	if(USBtxStatus != HAL_OK)
		Error_Handler();
	  
	  BTtxStatus = HAL_UART_Transmit(&huart2, BTtxBuffer, sizeof(BTtxBuffer) / sizeof(BTtxBuffer[0]), 100); //CHANGE
//USBrxStatus = HAL_UART_Receive(&huart2, USBrxBuffer, sizeof(USBrxBuffer) / sizeof(USBrxBuffer[0]), 500);
	if (BTtxStatus != HAL_OK)
	  Error_Handler();
	
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 16000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_8;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_ENABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_8;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_ENABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Green_Pin|LED_Red_Pin|LED_Blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE6 PE7 PE8 PE10
                           PE11 PE12 PE13 PE14
                           PE15 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PushB1_Pin */
  GPIO_InitStruct.Pin = PushB1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PushB1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 PC1 PC2
                           PC4 PC5 PC6 PC7
                           PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_1|GPIO_PIN_2
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 PF3
                           PF4 PF5 PF6 PF7
                           PF8 PF9 PF10 PF11
                           PF12 PF13 PF14 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0 PH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA4
                           PA8 PA9 PA10 PA11
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Green_Pin LED_Red_Pin LED_Blue_Pin */
  GPIO_InitStruct.Pin = LED_Green_Pin|LED_Red_Pin|LED_Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB11 PB12
                           PB13 PB15 PB3 PB4
                           PB5 PB6 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 PG2 PG3
                           PG4 PG5 PG6 PG7
                           PG8 PG9 PG10 PG11
                           PG12 PG13 PG14 PG15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD11 PD12 PD13
                           PD14 PD15 PD0 PD1
                           PD2 PD3 PD4 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//functions
struct errorCode ReseterrorCode(void)
{ 
		
};

//function reset exoCounter, exoThresh, exoCurrentState
void resetExoHitVars()
{
	memset(exoCounter, 0, sizeof(exoCounter)*sizeof(exoCounter[0]));
	memset(exoCurrentState, 0, sizeof(exoCurrentState)* sizeof(exoCurrentState[0])); // Set Baseline by adjustment
	memset(exoStateCounter, 0, sizeof(exoStateCounter)*sizeof(exoStateCounter[0]));
}

void calibrateSensor()
{
	uint16_t channelMax[numberOfLimbs] = { 0 };
	uint16_t tempChannelStorage = 0;
	//obtain max values after running loop jn times
	for (uint16_t j = 0; j < 7000; j++)
	{
		for (enum Sensor i = 0; i < numOfChansADC1; i++)
		{
			tempChannelStorage = Adc1Results[i];
			if (tempChannelStorage  > channelMax[i])
			{
				channelMax[i] = tempChannelStorage;
			}
		}
		HAL_Delay(1);
	}
	
	
	//update threshhold
	for (enum Sensor sensorPos = 0; sensorPos < numberOfLimbs; sensorPos++)
	{
	
		exoThresh[sensorPos] = (uint16_t)(((double)channelMax[sensorPos]+20) * 1.3); //added + 20 to overcome noise floor problem
	}	  
}

//Modified HAL functions
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	asm("nop");
	ADC2_val = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Start_IT(&hadc2);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  //USART3 section for host communication  //CHANGE ADD SAME CODE FOR UART2
  if (huart->Instance == USART3)
  {
    memcpy(rxStorageBuffer, USBrxBuffer, Size);
	USBrxStatus = HAL_UARTEx_ReceiveToIdle_DMA(&huart3, USBrxBuffer, sizeof(USBrxBuffer) / sizeof(USBrxBuffer[0])); //CHANGE
	if (USBrxStatus != HAL_OK)
		Error_Handler();
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT); //remeber that I set DMA handel to extern in the .h file this may cause bugs
  }
  
if (huart->Instance == USART2)
{
	memcpy(rxStorageBuffer, BTrxBuffer, Size);
	BTrxStatus = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, BTrxBuffer, sizeof(BTrxBuffer) / sizeof(BTrxBuffer[0])); //CHANGE
	if (BTrxStatus != HAL_OK)
		Error_Handler();
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT); //remeber that I set DMA handel to extern in the .h file this may cause bugs
}
	

	
	//BTrxStatus = HAL_UARTEx_ReceiveToIdle_DMA(&huart3, BTrxBuffer, sizeof(BTrxBuffer) / sizeof(BTrxBuffer[0])); //CHANGE
	//if (BTrxStatus != HAL_OK)
	//	Error_Handler();
	//__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT); //remeber that I set DMA handel to extern in the .h file this may cause bugs
	
  HAL_GPIO_TogglePin(LED_Blue_GPIO_Port, LED_Blue_Pin);
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
