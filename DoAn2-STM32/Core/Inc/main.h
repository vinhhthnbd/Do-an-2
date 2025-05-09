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
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;
extern uint8_t recieve_index;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ECODER1_CHANNEL1_Pin GPIO_PIN_6
#define ECODER1_CHANNEL1_GPIO_Port GPIOA
#define ENCODER1_CHANNEL2_Pin GPIO_PIN_7
#define ENCODER1_CHANNEL2_GPIO_Port GPIOA
#define MOTOR1_IN1_Pin GPIO_PIN_0
#define MOTOR1_IN1_GPIO_Port GPIOB
#define MOTOR1_IN2_Pin GPIO_PIN_1
#define MOTOR1_IN2_GPIO_Port GPIOB
#define BLUETOOTH_TX_Pin GPIO_PIN_10
#define BLUETOOTH_TX_GPIO_Port GPIOB
#define BLUETOOTH_RX_Pin GPIO_PIN_11
#define BLUETOOTH_RX_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_8
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_9
#define PWM2_GPIO_Port GPIOA
#define MOTOR2_IN1_Pin GPIO_PIN_3
#define MOTOR2_IN1_GPIO_Port GPIOB
#define MOTOR2_IN2_Pin GPIO_PIN_4
#define MOTOR2_IN2_GPIO_Port GPIOB
#define ENCODER2_CHANNEL1_Pin GPIO_PIN_6
#define ENCODER2_CHANNEL1_GPIO_Port GPIOB
#define ENCODER2_CHANNEL2_Pin GPIO_PIN_7
#define ENCODER2_CHANNEL2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
