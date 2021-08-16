/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/**** ESP-01 ****/
#include "ESP8266_HAL.h"
#include "UartRingbuffer_multi.h"

/**** Sensor ****/
#include "MPU9250.h"
#include "IPS.h"
#include "Encoder.h"

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
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//================== printf의 출력 모드를 설정해주는 함수====================
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)  //for printf
PUTCHAR_PROTOTYPE 
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
//-==================================================================

/**** ESP-01 ****/
UART_HandleTypeDef *wifi_uart = &huart3;
UART_HandleTypeDef *pc_uart = &huart2;
char Link_ID;
char Server_Data;
volatile uint8_t flag_ISR;
volatile uint8_t state;

/**** MPU9250 ****/
float bias_accel[3];
uint8_t rawdata[14];

/**** PID control ****/
volatile int encoderL_Count_F;
volatile int encoderL_Count_B;
volatile int encoderR_Count_F;
volatile int encoderR_Count_B;
float kp, ki, kd;
float setpoint, sampletime;
bool result;
float output_L, output_R;

/**** main value ****/
float distance_dt;
float angle_dt;
float angle_enco;
float distance_enco;
volatile float angle_mpu;
float x_axis1;      
float y_axis1;			
float x_axis2;      
float y_axis2;			
int value1;
int value2;
int value3;
int value4;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	/**** MPU9250 ****/
	MPU_ConfigTypeDef   myMPUConfig;
	uint8_t Dev_ADDR;
	uint8_t ReadData[2];
	
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	
	/**** ESP-01 ****/
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // CH1 Output Pin : PA7
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // CH2 Output Pin : PB0

	ESP_Init(wifi_uart, "SK_WiFi815A","1501070647", MODE_STA);	 // MODE_STA : ESP Client mode, MODE_AP : ESP Host mode
	
	for (int i=0; i<6; i++)
	{
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
		
		HAL_Delay(500);
	}
	/****************/
	
	/**** MPU9250 ****/
	MPU9250_Init(&hi2c1);
	
	Dev_ADDR = MPU9250_ADDRESS_AD0 << 1;
	ReadData[0] = 0;
	HAL_I2C_Mem_Read(&hi2c1, Dev_ADDR, WHO_AM_I_MPU9250, I2C_MEMADD_SIZE_8BIT, &ReadData[0], 1, 100);
	
	if(ReadData[0] == 0x71)
	{	
		myMPUConfig.ClockSource = Auto_best_clk_1;
		myMPUConfig.CONFIG_DLPF = DLPF_41G_42T_Hz;
    myMPUConfig.ACCEL_DLPF = DLPF_44A_Hz;		
		myMPUConfig.Ascale = AFS_2G;
		myMPUConfig.Gscale = GFS_250DPS;
		MPU9250_Config(&myMPUConfig);
		
		accelCalibrate(bias_accel);
		
		for (int i=0; i<4; i++)
		{
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
			
			HAL_Delay(100);
		}
	}
	else
	{
		while(1)
		{
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
			HAL_Delay(100);
		}
	}
	/****************/
	
	/**** PID ****/
	kp = 2.0f;
	//ki = 0.1f;  // I제어를 현재의 프로세서에서 사용하는것은 부적절
	ki = 0.0f;
	kd = 0.6f;
	setpoint = 30;  // Velocity : 30[cm/s]
	sampletime = 0.02f;
	pidInit(kp, ki, kd, setpoint, sampletime); 
		
	// Variables for Algorithms
	float dt_mpu = 0;
	float dt_encoder = 0;
	float dt_angle = 0;
	float temp_mpu = 0;
	float sub = 0;
	float rate = 0;
	float angle = 0;
	int count = 0;
	
  /* USER CODE END 2 */
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		/**** PID control ****/
		result = pidValue(encoderL_Count_F, encoderL_Count_B, encoderR_Count_F, encoderR_Count_B, state);
    if (result)
		{
			kp = 1.2f;
			pidCompute(&output_L, &output_R, kp);
			pidSetPWM(output_L, output_R, state);
			encoderLocate(&angle_dt, &distance_dt);				
		}
	  
		/**** Angle Correction Algorithm (using gyro and encoder) ****/
		dt_encoder = angle_dt;
		if (temp_mpu != 0) dt_mpu = angle_mpu - temp_mpu;
		temp_mpu = angle_mpu;
		
		sub = fabs(dt_mpu - dt_encoder);
		if (sub >= 1) dt_angle = dt_mpu;
		else if (sub < 1)
		{
			if ( sub <= 0.3 ) 								 rate = 0;
			else if( sub > 0.3 && sub <= 0.7 ) rate = sub / (0.7 - 0.3); 
			else 															 rate = 1;
			dt_angle = dt_encoder * (1 - rate) + dt_mpu * rate;
		}
		angle += dt_angle;

		/**** Calculate Location ****/
		distance_enco += distance_dt;
		angle_enco += angle_dt;				
		if (angle_enco > 360) angle_enco -= 360;
		else if ( angle_enco < -360) angle_enco += 360;	
//		x_axis1 += distance_dt * cos(angle_enco*rad);
//		y_axis1 += distance_dt * sin(angle_enco*rad);
		x_axis1 += distance_dt * cos(angle*rad);      
		y_axis1 += distance_dt * sin(angle*rad);     
		x_axis2 += distance_dt * cos(angle_mpu*rad); 
		y_axis2 += distance_dt * sin(angle_mpu*rad); 
		angle_dt = 0;
		distance_dt = 0;
		
		/**** Location Transmit to PC ****/
		value1 = x_axis1; // X-axis of Algorithm
		value2 = y_axis1; // Y-axis of Algorithm
		value3 = x_axis2; // X-axis of Gyro
		value4 = y_axis2; // Y-axis of Gyro
		ESP_wifi_start((uint8_t *)&flag_ISR, value1, value2, value3, value4);  // DB에 송신시 실수형보다 정수형을 보내는 것이 좋다!	
		
		/**** MPU9250 DMA Recive ****/		
		HAL_I2C_Mem_Read_DMA(&hi2c1, Dev_ADDR, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, (uint8_t *)rawdata, 14);	
					
		HAL_Delay(20);
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  htim3.Init.Prescaler = 2-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1680-1;
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
  sConfigOC.Pulse = 1200;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE3 PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PE9 PE10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

void Uart_isr_RxCpltCallback(UART_HandleTypeDef *huart, char rxdata)
{
	uint8_t wifi_complet = 0;
	uint8_t ESP_error = 0;
	
	/** Print terminal Wifi all Recive data  **/ 
	/** Caution! Do not use during operation **/
//	Uart_sendstring(&rxdata, pc_uart);
	
	/** Check ESP transmit response data **/
	if ( Server_data_wait(rxdata) == 1 ) ESP_error = 1;
	if ( ESP_error )
	{
    __HAL_UART_DISABLE_IT(wifi_uart, UART_IT_RXNE);
		ESP_error = 0;
		flag_ISR = STATE_ERR;
		state    = STATE_ERR;
		return;
	}
	
	/** Check Wifi Receive Control data **/
	if ( Server_start_receive(&Link_ID, &Server_Data, rxdata) == 1 ) wifi_complet = 1;	 		
	if ( wifi_complet )
	{
		Server_control(Server_Data);
		Server_id_close(Link_ID);		
		wifi_complet = 0;				
	}
	
	/** Flag set **/  
	switch (Server_Data)
	{
		case 'F' : 
			Server_Data = 0;
			flag_ISR = STATE_FORWARD;
			state    = STATE_FORWARD;
		  break;
		case 'B' :
			Server_Data =0;
			flag_ISR = STATE_BACKWARD;
			state    = STATE_BACKWARD;
			break;
		case 'L' :
			Server_Data =0;
			flag_ISR = STATE_ROTATION;
			state    = STATE_LEFT;
			break;
		case 'R' :
			Server_Data =0;
			flag_ISR = STATE_ROTATION;
			state    = STATE_RIGHT;
			break;
		case 'S' :
			Server_Data =0;
			flag_ISR = STATE_STOP;
			state    = STATE_STOP;
			break;
	}
	return;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	float dt = 0;
	float accel[3] = {0,};
	float gyro[3]  = {0,};
	float angle_dt = 0;

	if (hi2c == &hi2c1) 
	{
		/************ Calculate the position of a robot ************/
		readDMAData(rawdata, accel, gyro, &dt, bias_accel);
		
		angle_dt = getPatternData(gyro, accel, dt);
		
		angle_mpu += angle_dt;
		if (angle_mpu > 360) angle_mpu -= 360;
		else if ( angle_mpu < -360) angle_mpu += 360;	
		/**********************************************************/
	}		
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// Gear ratio = 48 : 1 
	// 16 single pulse per 1 turn by motor shaft ( Rising edge + falling edge = 16)
	// 384 counts per 1 turn by wheel ( Rising edge : 48 * 8 = 384 / 390!!)
	if (GPIO_Pin == GPIO_PIN_2)
	{
		uint8_t encoderL = 0;
		char encoderL_Dir = 0;

		encoderL = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);
		if (encoderL == 0)
			encoderL_Dir = 'F';
		else if (encoderL == 1)
			encoderL_Dir = 'B';
		
		if(encoderL_Dir == 'F')
			encoderL_Count_F++; 
		else if (encoderL_Dir == 'B')
			encoderL_Count_B++;
	}
	
	else if (GPIO_Pin == GPIO_PIN_4)
	{
		uint8_t encoderR = 0;
		char encoderR_Dir = 0;

		encoderR = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5);
		if (encoderR == 0)
			encoderR_Dir = 'B';
		else if (encoderR == 1)
			encoderR_Dir = 'F';
		
		if(encoderR_Dir == 'F')
			encoderR_Count_F++; 
		else if (encoderR_Dir == 'B')
			encoderR_Count_B++;  
	}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
