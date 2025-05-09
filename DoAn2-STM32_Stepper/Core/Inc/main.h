/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PC_TO_MCU_TX_Pin GPIO_PIN_2
#define PC_TO_MCU_TX_GPIO_Port GPIOA
#define PC_TO_MCU_RX_Pin GPIO_PIN_3
#define PC_TO_MCU_RX_GPIO_Port GPIOA
#define ENABLE1_Pin GPIO_PIN_4
#define ENABLE1_GPIO_Port GPIOA
#define ENABLE2_Pin GPIO_PIN_5
#define ENABLE2_GPIO_Port GPIOA
#define STEP1_Pin GPIO_PIN_6
#define STEP1_GPIO_Port GPIOA
#define STEP2_Pin GPIO_PIN_7
#define STEP2_GPIO_Port GPIOA
#define DIR1_Pin GPIO_PIN_0
#define DIR1_GPIO_Port GPIOB
#define DIR2_Pin GPIO_PIN_1
#define DIR2_GPIO_Port GPIOB
#define BLUETOOTH_TX_Pin GPIO_PIN_10
#define BLUETOOTH_TX_GPIO_Port GPIOB
#define BLUETOOTH_RX_Pin GPIO_PIN_11
#define BLUETOOTH_RX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern int8_t bluetooth_allow_tx;
extern int8_t pc_allow_tx;

typedef enum
{
	READ_NORMAL=1,
	READ_INVERT=-1,
}readDirection;

typedef enum
{
	CONTROL_NORMAL=1,
	CONTROL_INVERT=-1,
}controlDirection;

typedef struct
{
	int8_t Dir_M;
	uint16_t Count_timer;
	int16_t Step;
	int16_t previous_step;
	uint16_t Count_TOP, Count_BOT;
	int16_t delta_step;
	float distance;
	int8_t flag;

}StepperHandlle_Typedef;

extern StepperHandlle_Typedef Stepper1;
extern StepperHandlle_Typedef Stepper2;

#define STEPPER1_READ READ_NORMAL
#define STEPPER2_READ READ_INVERT

#define STEPPER1_CONTROL CONTROL_NORMAL
#define STEPPER2_CONTROL CONTROL_INVERT

void StepperSpeed(float v,StepperHandlle_Typedef *Stepper,controlDirection con);
void ReadStepper(StepperHandlle_Typedef *stepper,readDirection read);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
