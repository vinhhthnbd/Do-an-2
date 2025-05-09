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
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "Robot_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define CalibR 0
#define Test_UART_Receive 0
#define Test_UART_Transmit 0

//////
#define MAX_RPM 380
#define PULSE_PER_ROUND 200
#define PI 3.141592654f

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
//int8_t Dir_M1, Dir_M2, Dir_M3;                                               //Biến xác định hoạt động của động cơ và chi�?u quay Dir_Mx >0 quay thuận , Dir_Mx <0 quay ngược Dir_Mx =0 motor ngừng quay
//volatile int16_t Count_timer1, Count_timer2;                       //đếm các lần TIMER xuất hiện trong chu kỳ xung STEP
//volatile int32_t Step1, Step2, Step3;
//int16_t Count_TOP1, Count_BOT1, Count_TOP2, Count_BOT2;  //vị trí cuối của phần cao và cuối phần thấp trong 1 chu kỳ xung STEP

StepperHandlle_Typedef Stepper1;
StepperHandlle_Typedef Stepper2;

uint8_t tx_buffer2[64];
uint8_t rx_buffer2[64];
uint8_t final_rx_buffer2[64];
uint8_t bluetooth_rx_buffer[8];
uint8_t bluetooth_tx_buffer[64];
Robot_model_typedef R;
uint8_t timer2_int;
uint8_t timer3_int;
int8_t bluetooth_allow_tx;
int8_t bluetooth_rx_check;
int8_t uart2_allow_tx;
int8_t uart2_rx_check;
volatile uint8_t c_index;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM3)
	{
		timer3_int=1;
	}
	else if(htim->Instance==TIM2)
	{
		timer2_int=1;
	}
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART2)
	{
		memcpy(final_rx_buffer2,rx_buffer2,32);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{

	}
	else if(huart->Instance==USART2)
	{
		memcpy(final_rx_buffer2+32,rx_buffer2+32,32);
		uart2_rx_check=1;
	}
	else if(huart->Instance==USART3)
	{
		bluetooth_rx_check=1;
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void StepperSpeed(float v,StepperHandlle_Typedef *Stepper,controlDirection con)//(-200<y<200)
{
	float pps=v*200/60;
	float a=50000/pps;

    if (a < 0)
    {
    	Stepper->Dir_M = -1;
    }
    else if (a > 0)
    {
    	Stepper->Dir_M = 1;
    }
	else
		Stepper->Dir_M = 0;
    Stepper->Dir_M*=con;
	Stepper->Count_BOT = abs((int)a);
	Stepper->Count_TOP =  abs((int)a/2);
}

void ReadStepper(StepperHandlle_Typedef *stepper,readDirection read)
{
	stepper->delta_step=stepper->Step-stepper->previous_step;
	stepper->delta_step*=read;
	stepper->distance=((float)stepper->delta_step)/PULSE_PER_ROUND*2*PI*WHEEL_RADIUS;
	stepper->previous_step=stepper->Step;
}

void DealTimer3()
{
#if CalibR==1
		    	  if(Stepper1.Step==2000)
		    	  {
		    		  StepperSpeed(0,&Stepper1, STEPPER1_CONTROL);
		    		  StepperSpeed(0, &Stepper2, STEPPER2_CONTROL);
		    	  }
#endif
		if (Stepper1.Dir_M != 0)
		{                                                          //nếu MOTOR cho phép quay
			HAL_GPIO_WritePin(ENABLE1_GPIO_Port, ENABLE1_Pin, 0);
		    if (Stepper1.Dir_M > 0)
			{
		    	HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, 1);
			}
		    else if (Stepper1.Dir_M < 0)
		    {
		    	HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,0);
		    }
		    Stepper1.Count_timer++;
		    if (Stepper1.Count_timer <= Stepper1.Count_TOP)
		    	HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, 1);//nếu là nhịp nằm trong phần cao trong xung STEP
		    else
		    	HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, 0);                                                 //nếu là nhịp nằm trong phần thấp của xung STEP
		    if (Stepper1.Count_timer > Stepper1.Count_BOT)
		    {
		      Stepper1.Count_timer = 0;                             //nếu là nhịp cuối của 1 xung STEP
		      if (Stepper1.Dir_M > 0)
		      {
		    	  Stepper1.Step++;
		      }
		      else if (Stepper1.Dir_M < 0)
		      {
		    	  Stepper1.Step--;
		      }
		    }
		}
		else
		{
			HAL_GPIO_WritePin(ENABLE1_GPIO_Port,ENABLE1_Pin, 1);
		}
		if(Stepper2.Dir_M!=0)
		{
			HAL_GPIO_WritePin(ENABLE2_GPIO_Port, ENABLE2_Pin, 0);
			if (Stepper2.Dir_M > 0)
			{
				HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, 1);
			}
			else if (Stepper2.Dir_M < 0)
			{
				HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,0);
			}
			Stepper2.Count_timer++;
			if (Stepper2.Count_timer <= Stepper2.Count_TOP)
				HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, 1);//nếu là nhịp nằm trong phần cao trong xung STEP
			else
				HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, 0);                                                 //nếu là nhịp nằm trong phần thấp của xung STEP
			if (Stepper2.Count_timer > Stepper2.Count_BOT)
			{
			  Stepper2.Count_timer = 0;                             //nếu là nhịp cuối của 1 xung STEP
			  if (Stepper2.Dir_M > 0)
			  {
				  Stepper2.Step++;
			  }
			  else if (Stepper2.Dir_M < 0)
			  {
				  Stepper2.Step--;
			  }
			}
		}
		else
		{
			HAL_GPIO_WritePin(ENABLE2_GPIO_Port,ENABLE2_Pin, 1);
		}
}

void DealTimer2()
{
	ReadStepper(&Stepper1,STEPPER1_READ);
	ReadStepper(&Stepper2,STEPPER2_READ);
	calculatePredictionValue(&R, Stepper1.distance,Stepper2.distance);
#if Test_UART_Receive==0
#if Test_UART_Transmit==0
	sprintf((char*)tx_buffer2,"%f/%f/%f\n",(R.x_prediction-R.x_correction),(R.y_prediction-R.y_correction),(R.theta_prediction-R.theta_correction));
#endif
#if Test_UART_Transmit==1
	sprintf((char*)tx_buffer2,"%f/%f/%f\n",1.0f,1.0f,1.0f);
#endif

	HAL_UART_Transmit_DMA(&huart2, tx_buffer2, strlen((char*)tx_buffer2));
#endif
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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
#if CalibR ==1
  Stepper1.Step=0;
  Stepper1.previous_step=0;
  Stepper2.Step=0;
  Stepper2.previous_step=0;
#endif
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_DMA(&huart2, rx_buffer2, 64);
  HAL_UART_Receive_DMA(&huart3, bluetooth_rx_buffer, 1);
#if CalibR==1
  StepperSpeed(10, &Stepper1,STEPPER1_CONTROL);
  StepperSpeed(10, &Stepper2,STEPPER2_CONTROL);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	if(timer3_int==1)
	{
		DealTimer3();
		timer3_int=0;
	}
	if(timer2_int==1)
	{
		DealTimer2();
		timer2_int=0;
	}
	if(uart2_rx_check==1)
	{
		char *end;
		R.x_correction=strtof((char*)final_rx_buffer2,&end);
		R.y_correction=strtof(end,&end);
		R.theta_correction=strtof(end,NULL);
		R.x_prediction=R.x_correction;
		R.y_prediction=R.y_correction;
		R.theta_prediction=R.theta_correction;
#if Test_UART_Receive==1
		sprintf((char*)tx_buffer2,"%f/%f/%f\n",(R.x_correction),(R.y_correction),(R.theta_correction));
		HAL_UART_Transmit_DMA(&huart2, tx_buffer2, strlen((char*)tx_buffer2));
#endif
		uart2_rx_check=0;
	}
	if(bluetooth_rx_check==1)
	{
		BluetoothFrameHandler(bluetooth_rx_buffer,bluetooth_tx_buffer,&R);
		bluetooth_rx_check=0;
	}
	if(bluetooth_allow_tx==1)
	{
		sprintf((char*)bluetooth_tx_buffer,"%f/%f/%f\n",R.x_correction,R.y_correction,R.theta_correction);
		HAL_UART_Transmit_DMA(&huart3, bluetooth_tx_buffer,strlen((char*)bluetooth_tx_buffer));
		bluetooth_allow_tx=0;
	}
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 720-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 36-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20-1;
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
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ENABLE1_Pin|ENABLE2_Pin|STEP1_Pin|STEP2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR1_Pin|DIR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ENABLE1_Pin ENABLE2_Pin STEP1_Pin STEP2_Pin */
  GPIO_InitStruct.Pin = ENABLE1_Pin|ENABLE2_Pin|STEP1_Pin|STEP2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR1_Pin DIR2_Pin */
  GPIO_InitStruct.Pin = DIR1_Pin|DIR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
