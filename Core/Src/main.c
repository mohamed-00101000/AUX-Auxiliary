/* USER CODE BEGIN Header */
/*******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *****************************************************************************/
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

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
AS5600_TypeDef sensor;

FrontSide_BTNStates   front_state={OFF, OFF};		/* Create struct that holds front-side states */

//change last attribute value here back to OFF
Common_BtnStates 	  common_state={OFF, OFF, OFF, ON};		/* Create struct that holds Common states (available in both front and back sides ) */
BackSide_BTNStates    back_state={OFF, OFF};		/* Create struct that holds back-side  states */
Pedal_Values		  pedal_state={0,0};

typedef struct
{
	uint8_t status0;   // Represents the first byte (e.g., status or flag)
	uint8_t status1;   // Represents the first byte (e.g., status or flag)
	uint8_t status2;   // Represents the first byte (e.g., status or flag)
	uint8_t status3;   // Represents the first byte (e.g., status or flag)
	uint8_t status4;   // Represents the first byte (e.g., status or flag)
	uint8_t status5;   // Represents the first byte (e.g., status or flag)
	uint8_t status6;   // Represents the first byte (e.g., status or flag)
	uint8_t status7;   // Represents the first byte (e.g., status or flag)
} CAN_Message;
uint8_t TxData[8];
CAN_Message RxMessage;
CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
CAN_FilterTypeDef sFilterConfig;
CAN_RxHeaderTypeDef RxHeader;

uint16_t currReading = 0;

uint32_t Adc_lastTime = 0;
uint32_t F_lastTime = 0;
uint32_t Servo_lastTime = 0;
uint16_t current_servo_position = 0;
uint16_t offset = 0;  // Variable to store the offset

uint16_t minThrottle = 3000;
uint16_t maxThrottle = 1;

uint16_t minBrake = 3000;
uint16_t maxBrake = 1;

uint8_t init_counter = 2;
uint16_t init_temp = 0;
uint32_t init_counter_lasttime = 0;
float steering_angle = 0;

/****** CAN SEND VARs ********/
uint16_t steering_angle_send = 0;
uint16_t brake_val_send = 0;
uint16_t throttle_val_send = 0;
uint8_t dummy_receeive = 0;
uint16_t Temp_11 = 0;
uint8_t sine_wave[ARR_SIZE];
float temp = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
void ReadBtn_States(void);
void Write_States(void);

void LightingFunc(short period, char side);
void Turn_Off_All_flashers(void);
void Turn_Off_Right_flashers(void);
void Turn_Off_Left_flashers(void);

void Servomotor_func(char on_off, short period);
uint8_t PWM_Calc_compare_val(float degree_wanted);

void CAN_SEND(uint16_t can_send);

//void ADC_Select_CH0(void);
void ADC_Select_CH5(void);

void AS5600_Read_Position(AS5600_TypeDef *sensor) ;

void Reset_I2C() ;

uint16_t Calc_AS5600_Degree(void);

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan1);
void Rec_float(void);

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
	MX_CAN1_Init();
	MX_I2C1_Init();
//	MX_TIM1_Init();
	MX_ADC1_Init();
//	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	//CAN CODE INIT
	HAL_CAN_Start(&hcan1);
	//  TxHeader.DLC = 8;  // data length
	//  TxHeader.IDE = CAN_ID_STD;
	//  TxHeader.RTR = CAN_RTR_DATA;
	//TxHeader.StdId = 0x677; // ID can be between Hex1 and Hex7FF (1-2047 decimal)

	uint16_t can_send_id = 0x80;
	// Activate the notification on CAN bus
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);

//	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	//Generate array values for the servomotor
	for (int i = 0; i < ARR_SIZE; i++) {
		temp = (i+1) * PI / ARR_SIZE; // Calculate x for each point in the half-cycle
		sine_wave[i] = PEAK_VALUE * sin(temp);
	}

	/* HAL_Delay(2000); */

	/***** read initial Pedal Values *****/
	while(init_counter){
		//read initial minThrottle Val
		ADC_Select_CH5();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		init_temp = HAL_ADC_GetValue(&hadc1);
		if( init_temp < minThrottle ){
			minThrottle = init_temp;
		}
		HAL_ADC_Stop(&hadc1);

	  ADC_Select_CH5();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  init_temp = HAL_ADC_GetValue(&hadc1);
	  if( init_temp > maxBrake){
		  maxThrottle = init_temp;
	  }
	  HAL_ADC_Stop(&hadc1);

		if( HAL_GetTick() - init_counter_lasttime > INIT_COUNTER_PERIOD){

			ADC_Select_CH5();
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 1000);
			init_temp = HAL_ADC_GetValue(&hadc1);
			if( init_temp > maxThrottle){
				maxThrottle = init_temp;
				init_counter++;
			}
			HAL_ADC_Stop(&hadc1);

			init_counter--;
			init_counter_lasttime = HAL_GetTick();
		}
		init_counter=0;//we must init_counter=0 to get out of while(1)??
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	offset = sensor.sensor_Angle;  // Store the initial angle as the offset
	sensor.sensor_Angle =0 ;
	while (1)
	{
		/* USER CODE END WHILE */
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
//		/* USER CODE BEGIN 3 */
//		// CAN Transmit
//		if(  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData , &TxMailbox ) == HAL_OK)
//		{
//			CAN_SEND(can_send_id);
//			if(can_send_id == 0x80){
//				can_send_id = 0x90;
//			}else{
//				can_send_id = 0x80;
//			}
//		}
		ReadBtn_States();
		Write_States();
		HAL_Delay(100);
//		steering_angle = Calc_AS5600_Degree();
//		/* code to  send steering_angle on CAN bus */
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
//	sConfig.Channel = ADC_CHANNEL_1;
//	sConfig.Rank = 2;
//	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
//	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//	{
//		Error_Handler();
//	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. */
	//  sConfig.Channel = ADC_CHANNEL_0;
	//  sConfig.Rank = 1;
	//  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	//  {
	//    Error_Handler();
	//  }

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void)
{

	/* USER CODE BEGIN CAN1_Init 0 */
	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */
	/* USER CODE END CAN1_Init 1 */
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 10;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */
	else{
		sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
		sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterIdHigh = 0x677<<5;
		sFilterConfig.FilterIdLow = 0;
		sFilterConfig.FilterMaskIdHigh = 0x7FF<<5; // SET 0 to unfilter
		sFilterConfig.FilterMaskIdLow = 0;
		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	}
	/* USER CODE END CAN1_Init 2 */
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
	hi2c1.Init.ClockSpeed = 50000;
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
	htim1.Init.Period = 65535;
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
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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
	htim2.Init.Prescaler = 1599;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 200;
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
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

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

	GPIO_InitTypeDef GPIO_InitStructA_Pins = {0};
	GPIO_InitTypeDef GPIO_InitStructA_Btns = {0};

	GPIO_InitTypeDef GPIO_InitStructB_Pins = {0};
	GPIO_InitTypeDef GPIO_InitStructB_Btns = {0};

	GPIO_InitTypeDef GPIO_InitStructC_Pins = {0};
	GPIO_InitTypeDef GPIO_InitStructC_Btns = {0};
	GPIO_InitTypeDef test_struct = {0};
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

	/*Configure GPIO pin : PB2 */
//	GPIO_InitStruct.Pin = GPIO_PIN_2;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	__HAL_RCC_GPIOC_CLK_ENABLE();

//	GPIO_InitStructB_Pins.Pin = FRONT_HORN_PIN | FRONT_BEAM_PIN | FRONT_BLINKER_R_PIN | FRONT_BLINKER_L_PIN;
//	GPIO_InitStructB_Pins.Mode = GPIO_MODE_OUTPUT_PP;   	 /* Set as Push-Pull Output */
//	GPIO_InitStructB_Pins.Pull = GPIO_NOPULL;  		   	 /* No pull-up or pull-down resistor */
//	GPIO_InitStructB_Pins.Speed = GPIO_SPEED_FREQ_LOW;   	 /* Low frequency for GPIO speed */
//	HAL_GPIO_Init(PINS_PORT, &GPIO_InitStructB_Pins);  	 /* Initialize PINS for GPIOB */

	/******* BTNs CONFIG *******/

	/* PORTB BTNs CONFIG */
	test_struct.Pin = HAZARD_BTN;
	test_struct.Mode = GPIO_MODE_INPUT;  		 /* Set as INPUT */
	test_struct.Pull = GPIO_PULLUP;  			 /* pull-up resistor */
	test_struct.Speed = GPIO_SPEED_FREQ_LOW;	 /* Low frequency for GPIO speed */
	HAL_GPIO_Init(GPIOC, &test_struct);     /* Initialize BUTTONS for GPIOB */

	/* PORTA BTNs CONFIG */
	GPIO_InitStructA_Btns.Pin = REVERSE_BTN | BLINKER_R_BTN | LIMIT_SWITCH_BTN;
	GPIO_InitStructA_Btns.Mode = GPIO_MODE_INPUT;  		 /* Set as INPUT */
	GPIO_InitStructA_Btns.Pull = GPIO_PULLUP;  			 /* pull-up resistor */
	GPIO_InitStructA_Btns.Speed = GPIO_SPEED_FREQ_LOW;	 /* Low frequency for GPIO speed */
	HAL_GPIO_Init(REVERSE_BTN_PORT, &GPIO_InitStructA_Btns);     /* Initialize BUTTONS for GPIOA */

	/* PORTB BTNs CONFIG */
	GPIO_InitStructB_Btns.Pin = SERVO_BTN | HORN_BTN | BLINKER_L_BTN  ;
	GPIO_InitStructB_Btns.Mode = GPIO_MODE_INPUT;  		 /* Set as INPUT */
	GPIO_InitStructB_Btns.Pull = GPIO_PULLUP;  			 /* pull-up resistor */
	GPIO_InitStructB_Btns.Speed = GPIO_SPEED_FREQ_LOW;	 /* Low frequency for GPIO speed */
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructB_Btns);     /* Initialize BUTTONS for GPIOB */

	/* PORTC BTNs CONFIG */
	GPIO_InitStructC_Btns.Pin =  BEAM_BTN | HAZARD_BTN;
	GPIO_InitStructC_Btns.Mode = GPIO_MODE_INPUT;  		 /* Set as INPUT */
	GPIO_InitStructC_Btns.Pull = GPIO_PULLUP;  			 /* pull-up resistor */
	GPIO_InitStructC_Btns.Speed = GPIO_SPEED_FREQ_LOW;	 /* Low frequency for GPIO speed */
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructC_Btns);     /* Initialize BUTTONS for GPIOB */




	/******* PINs CONFIG *******/
	/* PORTA PINs CONFIG */
 	GPIO_InitStructA_Pins.Pin =  BACK_BRAKE_PIN | FRONT_HORN_PIN;
	GPIO_InitStructA_Pins.Mode = GPIO_MODE_OUTPUT_PP;    	 /* Set as Push-Pull Output */
	GPIO_InitStructA_Pins.Pull = GPIO_NOPULL;  			 /* No pull-up or pull-down resistor */
	GPIO_InitStructA_Pins.Speed = GPIO_SPEED_FREQ_LOW;  	 /* Low frequency for GPIO speed */
	HAL_GPIO_Init(BACK_BRAKE_PORT, &GPIO_InitStructA_Pins);  	 /* Initialize PINS for GPIOA */

	/* PORTB PINs CONFIG */
 	GPIO_InitStructB_Pins.Pin =  BACK_REVERSE_PIN ;
	GPIO_InitStructB_Pins.Mode = GPIO_MODE_OUTPUT_PP;    	 /* Set as Push-Pull Output */
	GPIO_InitStructB_Pins.Pull = GPIO_NOPULL;  			 /* No pull-up or pull-down resistor */
	GPIO_InitStructB_Pins.Speed = GPIO_SPEED_FREQ_LOW;  	 /* Low frequency for GPIO speed */
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructB_Pins);  	 /* Initialize PINS for GPIOB */

	/*PORTc PINs CONFIG*/
	GPIO_InitStructC_Pins.Pin = BACK_BLINKER_R_PIN | FRONT_BEAM_PIN | BACK_BLINKER_L_PIN | FRONT_BLINKER_R_PIN | FRONT_BLINKER_L_PIN;
	GPIO_InitStructC_Pins.Mode = GPIO_MODE_OUTPUT_PP;    	 /* Set as Push-Pull Output */
	GPIO_InitStructC_Pins.Pull = GPIO_NOPULL;  			 /* No pull-up or pull-down resistor */
	GPIO_InitStructC_Pins.Speed = GPIO_SPEED_FREQ_LOW;  	 /* Low frequency for GPIO speed */
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructC_Pins);  	 /* Initialize PINS for GPIOB */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Write_States(void){
	// 0 1 2 6 8 14
	/* Write values on FRONT Lighting/Horn */
	if(front_state.Front_Horn_state){	/* check Front Beam BTN IS pressed (Normal Logic)*/
		HAL_GPIO_WritePin(FRONT_HORN_PORT, FRONT_HORN_PIN, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(FRONT_HORN_PORT, FRONT_HORN_PIN, GPIO_PIN_RESET);
	}
	if(front_state.Front_Beam_state){ /* check Front Beam BTN WAS pressed (one short click -> change state)*/
		HAL_GPIO_WritePin(FRONT_BEAM_PORT, FRONT_BEAM_PIN, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(FRONT_BEAM_PORT, FRONT_BEAM_PIN, GPIO_PIN_RESET);
	}

	/* Write values on COMMON Lighting */
	if(common_state.Hazard_state || (common_state.Right_Flasher_state && common_state.Left_Flasher_state)){ /* check Front hazard BTN is pressed (Toggling Logic) */
		LightingFunc(BLINKER_PERIOD,'H');
	}
	else if(common_state.Right_Flasher_state && !common_state.Left_Flasher_state){ /* check Front right flasher BTN is pressed (Toggling Logic) */
		/* HAL_GPIO_WritePin(FRONT_PORT, FRONT_BLINKER_R_PIN, GPIO_PIN_SET); */
		LightingFunc(BLINKER_PERIOD,'R');
	}
	else if(common_state.Left_Flasher_state && !common_state.Right_Flasher_state){ /* check Front Left flasher BTN is pressed (Toggling Logic) */
		LightingFunc(BLINKER_PERIOD,'L');
	}else{
		Turn_Off_All_flashers();
	}


	/* Write values on BACK Lighting */
	if(back_state.Back_Reverse_state){
		HAL_GPIO_WritePin(BACK_REVERSE_PORT, BACK_REVERSE_PIN, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(BACK_REVERSE_PORT, BACK_REVERSE_PIN, GPIO_PIN_RESET);
	}
	if(back_state.Back_Brake_state){
		HAL_GPIO_WritePin(BACK_BRAKE_PORT, BACK_BRAKE_PIN, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(BACK_BRAKE_PORT, BACK_BRAKE_PIN, GPIO_PIN_RESET);
	}
//
//	/* Control Servomotor */
	if(common_state.Servo_state){
		Servomotor_func(ON, (uint16_t)SERVO_MOTOR_PERIOD);
	}else{
		Servomotor_func(OFF, (uint16_t)SERVO_MOTOR_PERIOD);
	}
}

void ReadBtn_States(void){
	int testing_val = 0;
	static uint8_t prev_Beam_State 		    =  OFF;
	static uint8_t prev_Hazard_State 		=  OFF;
	static uint8_t prev_R_Flasher_State 	=  OFF;
	static uint8_t prev_L_Flasher_State 	=  OFF;
	static uint8_t prev_Servo_State 		=  OFF;
	/*  static uint8_t prev_Reverse_state 	=  OFF; */

	/****** READ BTN STATES (Buttons are pull up thus used (!) operator for +ve logic) ******/
	front_state.Front_Horn_state  = !HAL_GPIO_ReadPin(HORN_BTN_PORT, HORN_BTN);
	back_state.Back_Reverse_state	= !HAL_GPIO_ReadPin(REVERSE_BTN_PORT, REVERSE_BTN);

	if( OFF == prev_Beam_State &&  ON == !HAL_GPIO_ReadPin(BEAM_BTN_PORT, BEAM_BTN)){
		//		  HAL_Delay(20);
		//		  if( OFF == prev_Beam_State &&  ON == !HAL_GPIO_ReadPin(FRONT_PORT, FRONT_BEAM_BTN)){
		front_state.Front_Beam_state = !front_state.Front_Beam_state;
		//		  }
	}
	//UNCOMMENT
		  if( OFF == prev_Servo_State &&  ON == !HAL_GPIO_ReadPin(SERVO_PORT, SERVO_BTN) ){
	//		  HAL_Delay(20);
	//		  if( OFF == prev_Servo_State &&  ON == !HAL_GPIO_ReadPin(SERVO_PORT, SERVO_BTN) ){
				  common_state.Servo_state = !common_state.Servo_state;
	//		  }
		  }

	//if it holds 1 then Servo_state = 1 (ON) else 0 (OFF)
	testing_val = common_state.Servo_state;

	/* Read States for COMMON Btns */
	if( OFF == prev_R_Flasher_State &&  ON == !HAL_GPIO_ReadPin(BLINKER_R_BTN_PORT, BLINKER_R_BTN)){
		common_state.Right_Flasher_state = !common_state.Right_Flasher_state;
		Turn_Off_Left_flashers();
	}
	if( OFF == prev_L_Flasher_State &&  ON == !HAL_GPIO_ReadPin(BLINKER_L_BTN_PORT, BLINKER_L_BTN)){
		common_state.Left_Flasher_state = !common_state.Left_Flasher_state;
		Turn_Off_Right_flashers();
	}
	if( OFF == prev_Hazard_State &&  ON == !HAL_GPIO_ReadPin(HAZARD_BTN_PORT, HAZARD_BTN)){
		common_state.Hazard_state = !common_state.Hazard_state;
		Turn_Off_All_flashers();
	}

	prev_Beam_State 		=  !HAL_GPIO_ReadPin(BEAM_BTN_PORT, BEAM_BTN);
	prev_Hazard_State  	    =  !HAL_GPIO_ReadPin(HAZARD_BTN_PORT, HAZARD_BTN);
	prev_R_Flasher_State 	=  !HAL_GPIO_ReadPin(BLINKER_R_BTN_PORT, BLINKER_R_BTN);
	prev_L_Flasher_State 	=  !HAL_GPIO_ReadPin(BLINKER_L_BTN_PORT, BLINKER_L_BTN);
	prev_Servo_State		=  !HAL_GPIO_ReadPin(SERVO_PORT, SERVO_BTN);

	/***********     ADC     *************/
//	ADC_Select_CH0();
//	HAL_ADC_Start(&hadc1);
//	HAL_ADC_PollForConversion(&hadc1, 1000);
//	currReading = HAL_ADC_GetValue(&hadc1);
//	if(currReading < maxBrake){
//		pedal_state.Brake_Value = currReading - minBrake;
//	}
//
//	HAL_ADC_Stop(&hadc1);
	pedal_state.Brake_Value = !HAL_GPIO_ReadPin(LIMIT_SWITCH_BTN_PORT, LIMIT_SWITCH_BTN);

	if(pedal_state.Brake_Value){
		back_state.Back_Brake_state = ON;
	}else{
		back_state.Back_Brake_state = OFF;
	}

	/* 50 margin error for unpressed pedal */
//	if( (pedal_state.Brake_Value  + 50) > MIN_BRAKE_VAL){
//		back_state.Back_Brake_state = ON;
//	}else{
//		back_state.Back_Brake_state = OFF;
//	}

	ADC_Select_CH5();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);

	currReading = HAL_ADC_GetValue(&hadc1);
	if(currReading < maxThrottle){
		pedal_state.Throttle_Value = currReading - minThrottle;
	}
	HAL_ADC_Stop(&hadc1);


	/* Old (maybe)
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  pedal_state.Throttle_Value = HAL_ADC_GetValue(&hadc1);

	  // Wait for a short delay before starting the second conversion
	  //HAL_Delay(1); // Adjust delay as needed

	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  pedal_state.Brake_Value = HAL_ADC_GetValue(&hadc1);
	  if(pedal_state.Brake_Value > MIN_BRAKE_VAL){
		  back_state.Back_Brake_state = ON;
	  }else{
		  back_state.Back_Brake_state = OFF;
	  }*/
}

void LightingFunc(short period, char side) {
	//apply this logic each (period) of time
	if ((HAL_GetTick() - F_lastTime) >= period) {
		switch (side) {
		case 'R':
			Turn_Off_Left_flashers();
			/* toggle right blinkers */
			HAL_GPIO_TogglePin(FRONT_BLINKER_R_PORT, FRONT_BLINKER_R_PIN);
			HAL_GPIO_TogglePin(BACK_BLINKER_R_PORT, BACK_BLINKER_R_PIN);
			break;

		case 'L':
			Turn_Off_Right_flashers();
			/* toggle left blinkers */
			HAL_GPIO_TogglePin(FRONT_BLINKER_L_PORT, FRONT_BLINKER_L_PIN);
			HAL_GPIO_TogglePin(BACK_BLINKER_L_PORT,  BACK_BLINKER_L_PIN);
			break;

		case 'H':
			//		  HAL_GPIO_WritePin(PINS_PORT, FRONT_BEAM_PIN, GPIO_PIN_SET); /*Check */
			/* toggle right blinkers */
			HAL_GPIO_TogglePin(FRONT_BLINKER_R_PORT, FRONT_BLINKER_R_PIN);
			HAL_GPIO_TogglePin(BACK_BLINKER_R_PORT,  BACK_BLINKER_R_PIN);
			/* toggle left blinkers */
			HAL_GPIO_TogglePin(FRONT_BLINKER_L_PORT, FRONT_BLINKER_L_PIN);
			HAL_GPIO_TogglePin(BACK_BLINKER_L_PORT,  BACK_BLINKER_L_PIN);
			break;
		default:
			Turn_Off_All_flashers();
			break;
		}
		F_lastTime = HAL_GetTick();
	}
}

void Turn_Off_All_flashers(void){
	HAL_GPIO_WritePin(FRONT_BLINKER_R_PORT, FRONT_BLINKER_R_PIN,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FRONT_BLINKER_L_PORT, FRONT_BLINKER_L_PIN,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BACK_BLINKER_R_PORT,  BACK_BLINKER_R_PIN,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BACK_BLINKER_L_PORT,  BACK_BLINKER_L_PIN,  GPIO_PIN_RESET);
}

void Turn_Off_Right_flashers(void){
	HAL_GPIO_WritePin(FRONT_BLINKER_R_PORT, FRONT_BLINKER_R_PIN,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BACK_BLINKER_R_PORT,  BACK_BLINKER_R_PIN,  GPIO_PIN_RESET);
}

void Turn_Off_Left_flashers(void){
	HAL_GPIO_WritePin(FRONT_BLINKER_L_PORT, FRONT_BLINKER_L_PIN,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BACK_BLINKER_L_PORT,  BACK_BLINKER_L_PIN,  GPIO_PIN_RESET);
}


/* 0 degree   -> compare val = 5
   90 degree  -> compare val = 15
   180 degree -> compare val = 25 */
void Servomotor_func(char on_off, short period){
	uint8_t compare_Val = 0; /* variable that holds calculated value to insert into Timer to rotate the Servomotor */
	//	static char current_state = 0;
	int value_on_curve = 0;
	//	if( HAL_GetTick() - Servo_lastTime > period){
	if(on_off){
		for( ; current_servo_position < ARR_SIZE ; ){
			value_on_curve = sine_wave[current_servo_position];
			compare_Val = PWM_Calc_compare_val(value_on_curve); /* Should carry values between 5 and 25 */
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, compare_Val);
			//				Servo_lastTime = HAL_GetTick();
			current_servo_position++;
			if(current_servo_position == ( ARR_SIZE) ){
				current_servo_position = 0;
			}
			break; // This break line allows Servomotor to be non blocking, make 1 iteration then continue rest of the controller's code
		}
		//			for( ; current_servo_position > 0 ; current_servo_position--){
		//				value_on_curve = sine_wave[current_servo_position];
		//				compare_Val = PWM_Calc_compare_val(value_on_curve);
		//				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, compare_Val);
		//				Servo_lastTime = HAL_GetTick();
		//				//if this logic is added back remember u need to add break;
		//			}
	}else{
		for( ; current_servo_position > 0 ; ){
			value_on_curve = sine_wave[current_servo_position];
			compare_Val = PWM_Calc_compare_val(value_on_curve);
			//				compare_Val = PWM_Calc_compare_val(current_servo_position);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, compare_Val);
			//				Servo_lastTime = HAL_GetTick();
			current_servo_position--;
			break; // This break line allows Servomotor to be non blocking, make 1 iteration then continue rest of the controller's code
		}
	}
	//	}
}

uint8_t PWM_Calc_compare_val(float degree_wanted){
	uint8_t compare_Val = 5 + (uint8_t)(degree_wanted*0.111);
	return compare_Val;
}

//void ADC_Select_CH0(void){
//	ADC_ChannelConfTypeDef sConfig = {0};
//
//	/* Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.*/
//	sConfig.Channel = ADC_CHANNEL_0;
//	sConfig.Rank = 1;
//	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
//	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//	{
//		Error_Handler();
//	}
//}

void ADC_Select_CH5(void){
	ADC_ChannelConfTypeDef sConfig = {0};
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.*/
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

// CAN Receive

void AS5600_Read_Position(AS5600_TypeDef *sensor) {
	uint8_t data[2];

	// Read the angle from the AS5600
	if (HAL_I2C_Mem_Read(&hi2c1, AS5600_SLAVE_ADDRESS, AS5600_REGISTER_ANGLE_HIGH, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY) == HAL_OK) {
		sensor->sensor_Angle = (data[0] << 8) | data[1]; /* Combine high and low byte */
	} else {
		// Handle error and print a message over UART
		sensor->sensor_Angle = 0xFFFF; // Invalid value
		//char errorMsg[] = "I2C Read Error\n";
		//HAL_UART_Transmit(&huart1, (uint8_t*)errorMsg, strlen(errorMsg), HAL_MAX_DELAY);
		Reset_I2C();
	}
}

uint16_t Calc_AS5600_Degree(){
	float angle_degrees = 0;
	AS5600_Read_Position(&sensor);
	angle_degrees = (sensor.sensor_Angle / 4095.0) * 360.0;
	return angle_degrees;
}


void CAN_SEND(uint16_t can_id){

	TxHeader.DLC = 8;  // data length
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	if(can_id == CAN_MSG_1)
	{
		TxHeader.StdId = can_id; // ID can be between Hex1 and Hex7FF (1-2047 decimal)
		/************************************************/
		throttle_val_send  = 1000 *  pedal_state.Throttle_Value ;
		TxData[0]= ( (uint8_t)throttle_val_send ) >> 8;
		TxData[1]=  (uint8_t)throttle_val_send ;
		/************************************************/
		brake_val_send = 1000 * pedal_state.Brake_Value;

		TxData[2]= (uint8_t)(brake_val_send>> 8);
		TxData[3]= (uint8_t)brake_val_send;
		/*************************************************/
		steering_angle_send = 1000 * steering_angle;
		TxData[4]= (uint8_t)(steering_angle_send>>8);
		TxData[5]= (uint8_t)steering_angle_send ;

		TxData[6] = 8;
		/************************************************/
	}
	else if (can_id == CAN_MSG_2)
	{
		TxHeader.StdId = can_id; // ID can be between Hex1 and Hex7FF (1-2047 decimal)
		/*************************************************/
		TxData[0]= common_state.Right_Flasher_state;
		TxData[1]= common_state.Left_Flasher_state;
		/************************************************/
		TxData[2]= front_state.Front_Beam_state;
		TxData[3]= common_state.Hazard_state;
		/************************************************/
		TxData[6] = 8;
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO1, &RxHeader, &RxMessage );
	if (RxHeader.StdId == 0x677){
		Rec_float();
		dummy_receeive = RxMessage.status3;
		if( RxMessage.status3 == 7 )
		{
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0 ) ;
		}
	}
}

void Rec_float(void)
{
	uint16_t Temp_Send_1 = (unsigned short)((RxMessage.status0 << 8) | RxMessage.status1);
	Temp_11= Temp_Send_1/1000.0 ;
}
//alternate implementation for servomotor function
/*
 if(on_off){
	 switch(current_state){
		case 0:
			if( HAL_GetTick() - Servo_lastTime > period){
				compare_Val = PWM_Calc_compare_val(SERVO_POSITION_0);
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, compare_Val);
				Servo_lastTime = HAL_GetTick();
				current_state++;
			}
			break;
		case 1:
			if( HAL_GetTick() - Servo_lastTime > period){
				compare_Val = PWM_Calc_compare_val(SERVO_POSITION_1);
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, compare_Val);
				Servo_lastTime = HAL_GetTick();
				current_state++;
			}
			break;
		case 2:
			if( HAL_GetTick() - Servo_lastTime > period){
				compare_Val = PWM_Calc_compare_val(SERVO_POSITION_2);
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, compare_Val);
				Servo_lastTime = HAL_GetTick();
				current_state++;
			}
			break;
		case 5:
			if( HAL_GetTick() - Servo_lastTime > period){
				compare_Val = PWM_Calc_compare_val(SERVO_POSITION_1); //starting position
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, compare_Val);
				Servo_lastTime = HAL_GetTick();
				current_state++;
			}
			break;
		case 6:
			if( HAL_GetTick() - Servo_lastTime > period){
				compare_Val = PWM_Calc_compare_val(SERVO_POSITION_0); //starting position
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, compare_Val);
				Servo_lastTime = HAL_GetTick();
				current_state = 1;
			}
			break;
		default:
			compare_Val = PWM_Calc_compare_val(SERVO_POSITION_0);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, compare_Val);	// 0 degree (starting position)
			current_state = 0;
			break;
	}
}else{
	switch(current_state){
		case 0:
			if( HAL_GetTick() - Servo_lastTime > period){
				compare_Val = PWM_Calc_compare_val(SERVO_POSITION_0);
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, compare_Val);
				Servo_lastTime = HAL_GetTick();
			}
			break;
		case 1:
			if( HAL_GetTick() - Servo_lastTime > period){
				compare_Val = PWM_Calc_compare_val(SERVO_POSITION_0);
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, compare_Val);
				Servo_lastTime = HAL_GetTick();
				current_state--;
			}
			break;
		case 3:
			if( HAL_GetTick() - Servo_lastTime > period){
				compare_Val = PWM_Calc_compare_val(SERVO_POSITION_2);
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, compare_Val);
				Servo_lastTime = HAL_GetTick();
				current_state--;
			}
			break;
		case 4:
			if( HAL_GetTick() - Servo_lastTime > period){
				compare_Val = PWM_Calc_compare_val(SERVO_POSITION_1);
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, compare_Val);
				Servo_lastTime = HAL_GetTick();
				current_state = 1;
			}
			break;
		case 6:
			if( HAL_GetTick() - Servo_lastTime > period){
				compare_Val = PWM_Calc_compare_val(SERVO_POSITION_0);
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, compare_Val);
				Servo_lastTime = HAL_GetTick();
				current_state = 0;
			}
			break;
	}
} */

void Reset_I2C() {
	HAL_I2C_DeInit(&hi2c1);
	HAL_Delay(100);  // Short delay
	HAL_I2C_Init(&hi2c1);
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
