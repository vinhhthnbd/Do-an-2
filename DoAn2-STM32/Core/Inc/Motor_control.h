/*
 * Motor_control.h
 *
 *  Created on: Oct 14, 2024
 *      Author: PC
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#ifdef __cplusplus
 extern "C" {
#endif

#define PULSE_PER_ROUND 2970
#define MAX_RPM 37
#define MAX_CCR_VALUE 1000-1
#define SAMPLE_TIME 0.02

#include <stdio.h>
#include <stdint.h>
#include "PID.h"
#include "main.h"

/**
 * @note: INx: GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_2,GPIO_PIN_10
 **/
typedef enum
{
	READ_NORMAL=1,
	READ_MINUS=-1,
}Encoder_sign_enum_typedef;
typedef enum
{
	CONTROL_NORMAL,
	CONTROL_INVERT,
}Motor_direction_enum_typedef;
typedef enum
{
	MOTOR_1,
	MOTOR_2,
}Motor_pwm_pin_enum_typedef;
typedef struct{
	float rpm_measurement;
	float delta_round;
	float distance;
	int previous_cnt;
	int current_cnt;
	float current_delta;
	float previous_delta;
	float previous_command_rpm;
	float current_command_rpm;
	float expected_rpm;
	uint16_t IN1;
	uint16_t IN2;
}Motor_measurement_command_typedef;

void readEncoder(TIM_HandleTypeDef *TIM,Motor_measurement_command_typedef *M,Encoder_sign_enum_typedef s);
void resetDeltaRoundDistance(Motor_measurement_command_typedef *M);
void motorControl1(Motor_measurement_command_typedef *M,float rpm,Motor_direction_enum_typedef d);
void motorControl2(Motor_measurement_command_typedef *M,float rpm,Motor_direction_enum_typedef d);
void pidMotorControl(TIM_HandleTypeDef *TIM,Motor_measurement_command_typedef *Motor,
		PIDControllers_Typedef *pid,Motor_pwm_pin_enum_typedef Motor_num,
		Motor_direction_enum_typedef direction);



#ifdef __cplusplus
}
#endif
#endif /* INC_MOTOR_CONTROL_H_ */
