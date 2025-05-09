/*
 * Robot_control.c
 *
 *  Created on: Oct 18, 2024
 *      Author: GIGABYTE
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "main.h"
#include "Robot_control.h"


/*
 *
 */
void calculatePredictionValue(Robot_model_typedef *R,float sl,float sr)
{
	R->sl=sl;
	R->sr=sr;
	R->delta_theta=(R->sr-R->sl)/(2*RW_DISTANCE_WHEEL_CENTER);
	R->s=(R->sr+R->sl)/2;
	R->delta_x=R->s*cos(R->theta_prediction+R->delta_theta/2);
	R->delta_y=R->s*sin(R->theta_prediction+R->delta_theta/2);
	R->x_prediction=R->x_prediction+R->delta_x;
	R->y_prediction=R->y_prediction+R->delta_y;
	R->theta_prediction=R->theta_prediction+R->delta_theta;
}

void resetDeltaValue(Robot_model_typedef *R)
{
	R->delta_theta=0;
	R->delta_x=0;
	R->delta_y=0;

}

/**
 * @note This is the function we use for dealing with control command that microcontroller gets from 
 * 		Bluetooth 
 * @note Put this function into void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){}
 * @param rx_buffer The pointers of data we receive from UART
 * @param M1 The pointer of typedef struct that store first motor information
 * @param M2 The pointer of typedef struct that store second motor information
**/
void BluetoothFrameHandler(uint8_t *rx_buffer,uint8_t *tx_buffer,Robot_model_typedef *R)
{
		if (*rx_buffer==FORWARD_COMMAND_CHARACTER)
		{
			StepperSpeed(10, &Stepper1,STEPPER1_CONTROL);
			StepperSpeed(10, &Stepper2,STEPPER2_CONTROL);
		}
		else if (*rx_buffer==BACKWARD_COMMAND_CHARACTER)
		{
			StepperSpeed(-10, &Stepper1,STEPPER1_CONTROL);
			StepperSpeed(-10, &Stepper2,STEPPER2_CONTROL);
		}
		else if (*rx_buffer==TURN_LEFT_COMMAND_CHARACTER)
		{
			StepperSpeed(3, &Stepper1,STEPPER1_CONTROL);
			StepperSpeed(-3, &Stepper2,STEPPER2_CONTROL);
		}
		else if (*rx_buffer==TURN_RIGHT_COMMAND_CHARACTER)
		{
			StepperSpeed(-3, &Stepper1,STEPPER1_CONTROL);
			StepperSpeed(3, &Stepper2,STEPPER2_CONTROL);
		}
		else if(*rx_buffer==STOP_COMMAND_CHARACTER)
		{
			StepperSpeed(0, &Stepper1,STEPPER1_CONTROL);
			StepperSpeed(0, &Stepper2,STEPPER2_CONTROL);
		}
		else if(*rx_buffer==SEND_CORRECTION_DATA)
		{
			 bluetooth_allow_tx=1;
		}
}



