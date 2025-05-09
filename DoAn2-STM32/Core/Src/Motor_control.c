/*
 * Motor_control.c
 *
 *  Created on: Oct 14, 2024
 *      Author: PC
 */

#include <stdio.h>
#include <stdint.h>
#include "main.h"
#include "Motor_control.h"
#include "PID.h"
#include "Robot_control.h"

//TIM2,TIM3,TIM4,TIM1
void readEncoder(TIM_HandleTypeDef *TIM,Motor_measurement_command_typedef *M,Encoder_sign_enum_typedef s)
{
	M->current_cnt=__HAL_TIM_GET_COUNTER(TIM);

	M->current_delta=(float)(M->current_cnt-M->previous_cnt);

	if(M->current_delta<-60000)
	{
		M->current_delta+=65536;
	}
	else if(M->current_delta>60000)
	{
		M->current_delta-=65536;
	}

	M->current_delta=0.2222*M->current_delta+(1-0.2222)*M->previous_delta;
	M->previous_cnt=M->current_cnt;
	M->previous_delta=M->current_delta;

	M->rpm_measurement=s*(M->current_delta*60)/(PULSE_PER_ROUND*2*SAMPLE_TIME);
	M->delta_round=s*M->current_delta/(PULSE_PER_ROUND*2);
	M->distance=M->delta_round*WHEEL_RADIUS*2;
}

void resetDeltaRoundDistance(Motor_measurement_command_typedef *M)
{
	M->delta_round=0;
	M->distance=0;
}

/**
 * @note: excecute this function if we want to change rpm to our motor
 **/
void motorControl1(Motor_measurement_command_typedef *M,float rpm,Motor_direction_enum_typedef d)
{
	if(d==CONTROL_NORMAL)
		M->current_command_rpm=rpm;
	else if(d==CONTROL_INVERT)
		M->current_command_rpm=-rpm;
	if((M->current_command_rpm>0)&&(M->previous_command_rpm>=0))
	{
		htim1.Instance->CCR1=(uint16_t)(M->current_command_rpm/MAX_RPM*MAX_CCR_VALUE);
		HAL_GPIO_WritePin(GPIOB, M->IN1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, M->IN2, GPIO_PIN_RESET);
	}
	else if((M->current_command_rpm<0)&&(M->previous_command_rpm>0))
	{
		HAL_GPIO_WritePin(GPIOB, M->IN1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, M->IN2, GPIO_PIN_RESET);
		htim1.Instance->CCR1=(uint16_t)(-M->current_command_rpm/MAX_RPM*MAX_CCR_VALUE);
		HAL_GPIO_WritePin(GPIOB, M->IN1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, M->IN2, GPIO_PIN_SET);
	}
	else if((M->current_command_rpm>0)&&(M->previous_command_rpm<0))
	{
		HAL_GPIO_WritePin(GPIOB, M->IN1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, M->IN2, GPIO_PIN_RESET);
		htim1.Instance->CCR1=(uint16_t)(M->current_command_rpm/MAX_RPM*MAX_CCR_VALUE);
		HAL_GPIO_WritePin(GPIOB, M->IN1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, M->IN2, GPIO_PIN_RESET);
	}
	else if((M->current_command_rpm<0)&&(M->previous_command_rpm<=0))
	{
		htim1.Instance->CCR1=(uint16_t)(-M->current_command_rpm/MAX_RPM*MAX_CCR_VALUE);
		HAL_GPIO_WritePin(GPIOB, M->IN1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, M->IN2, GPIO_PIN_SET);
	}
	else if(M->current_command_rpm==0)
	{
		HAL_GPIO_WritePin(GPIOB, M->IN1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, M->IN2, GPIO_PIN_RESET);
	}
	M->previous_command_rpm=M->current_command_rpm;
}

void motorControl2(Motor_measurement_command_typedef *M,float rpm,Motor_direction_enum_typedef d)
{
	if(d==CONTROL_NORMAL)
			M->current_command_rpm=rpm;
	else if(d==CONTROL_INVERT)
			M->current_command_rpm=-rpm;
	if((M->current_command_rpm>0)&&(M->previous_command_rpm>=0))
	{
		htim1.Instance->CCR2=(uint16_t)(M->current_command_rpm/MAX_RPM*MAX_CCR_VALUE);
		HAL_GPIO_WritePin(GPIOB, M->IN1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, M->IN2, GPIO_PIN_RESET);
	}
	else if((M->current_command_rpm<0)&&(M->previous_command_rpm>0))
	{
		HAL_GPIO_WritePin(GPIOB, M->IN1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, M->IN2, GPIO_PIN_RESET);
		htim1.Instance->CCR2=(uint16_t)(-M->current_command_rpm/MAX_RPM*MAX_CCR_VALUE);
		HAL_GPIO_WritePin(GPIOB, M->IN1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, M->IN2, GPIO_PIN_SET);
	}
	else if((M->current_command_rpm>0)&&(M->previous_command_rpm<0))
	{
		HAL_GPIO_WritePin(GPIOB, M->IN1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, M->IN2, GPIO_PIN_RESET);
		htim1.Instance->CCR2=(uint16_t)(M->current_command_rpm/MAX_RPM*MAX_CCR_VALUE);
		HAL_GPIO_WritePin(GPIOB, M->IN1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, M->IN2, GPIO_PIN_RESET);
	}
	else if((M->current_command_rpm<0)&&(M->previous_command_rpm<=0))
	{
		htim1.Instance->CCR2=(uint16_t)(-M->current_command_rpm/MAX_RPM*MAX_CCR_VALUE);
		HAL_GPIO_WritePin(GPIOB, M->IN1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, M->IN2, GPIO_PIN_SET);
	}
	else if(M->current_command_rpm==0)
	{
		HAL_GPIO_WritePin(GPIOB, M->IN1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, M->IN2, GPIO_PIN_RESET);
	}
	M->previous_command_rpm=M->current_command_rpm;
}
/**
 * @note This function is used for controlling motor speed with pid controller
 * 		Before using this function, activate Timer1 with PWM mode,activate other Timers with encoder mode
 * 		Put this function into a sequental rountine with a defined sample time
 * 		We also need to declare typedef struct for storing measurement values and control values
 * @param TIM This is from STM32 API Layer Driver Library
 * @param Motor pointer to a typedef struct for Motor measurement and control
 * @param pid pointer to a typedef struct for PID controllers
 * @param Motor_num define the channel of Timer1 we want to use as pwm signal for our motor
 * @param direction define the direction for motor rotation and encoder reading
 * 
**/
void pidMotorControl(TIM_HandleTypeDef *TIM,Motor_measurement_command_typedef *Motor,
		PIDControllers_Typedef *pid,Motor_pwm_pin_enum_typedef Motor_num,
		Motor_direction_enum_typedef direction)
{
	if(direction==CONTROL_NORMAL)
	{
		readEncoder(TIM, Motor,READ_NORMAL);
		pidUpdate(pid, Motor->rpm_measurement, Motor->expected_rpm);
	}

	else if(direction==CONTROL_INVERT)
	{
		readEncoder(TIM, Motor, READ_MINUS);
		pidUpdate(pid, Motor->rpm_measurement,Motor->expected_rpm);
	}
	if(Motor_num==MOTOR_1)
	{
		motorControl1(Motor, pid->u, direction);
	}
	else if(Motor_num==MOTOR_2)
	{
		motorControl2(Motor, pid->u, direction);
	}

}

