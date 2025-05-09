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
#include "Motor_control.h"

/*
 *
 */
void calculatePredictionValue(Robot_model_typedef *R,float sl,float sr)
{
	R->sl=sl;
	R->sr=sr;
	R->delta_theta=(R->sr-R->sl)/(2*RW_DISTANCE_WHEEL_CENTER);
	R->s=(R->sr+R->sl)/2;
	R->delta_x=R->s*cosf(R->theta_prediction+R->delta_theta/2);
	R->delta_y=R->s*sinf(R->theta_prediction+R->delta_theta/2);
	R->x_prediction=R->x_prediction+R->delta_x;
	R->y_prediction=R->y_prediction+R->delta_y;
	R->theta_prediction=R->theta_prediction+R->delta_theta;
}

void resetDeltaValue(Robot_model_typedef *R,Motor_measurement_command_typedef *M1,Motor_measurement_command_typedef *M2)
{
	R->delta_theta=0;
	R->delta_x=0;
	R->delta_y=0;
	resetDeltaRoundDistance(M1);
	resetDeltaRoundDistance(M2);
}

/**
 * @note This is the function we use for dealing with commands we receive from PC, each time PC send 1 character
 * 		Put this function into void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){}
 * @note after robot has done the correction step, the PC send  UPDATE_PREDICTION command and a data string
 * 		which has a format ux/uy/utheta\n
 * @param rx_buffer The pointers of data we receive from UART
 * @param tx_buffer The pointers of data we want to transmit back to PC
 * @param R The pointer of typedef struct that store robot calculation data
 * @param M1 The pointer of typedef struct that store first motor information
 * @param M2 The pointer of typedef struct that store second motor information
**/
void PCFrameReceiveHandler(uint8_t *rx_buffer,uint8_t *tx_buffer,Robot_model_typedef *R,Motor_measurement_command_typedef *M1,Motor_measurement_command_typedef *M2)
{
	if(*rx_buffer==UPDATE_PREDICTION_CHARACTER)
	{
		if(rx_buffer[recieve_index]!=END_CORRECTION_DATA_CHARACTER)
		{
			recieve_index++;
			HAL_UART_Receive_IT(&huart2, rx_buffer+recieve_index, 1);
		}
		else
		{
			rx_buffer[recieve_index]=0;
			char *temp=strtok((char*)(rx_buffer+1),SPLIT_CORRECTION_DATA_CHARACTER);
			R->x_correction=atof(temp);
			temp=strtok(NULL,SPLIT_CORRECTION_DATA_CHARACTER);
			R->y_correction=atof(temp);
			temp=strtok(NULL,SPLIT_CORRECTION_DATA_CHARACTER);
			R->theta_correction=atof(temp);

			R->x_prediction=R->x_correction;
			R->y_prediction=R->y_correction;
			R->theta_prediction=R->theta_correction;

			for(int i=recieve_index;i>=1;i--)
			{
				rx_buffer[i]=0;
			}
			recieve_index=0;
		}
	}

	else if(*rx_buffer==SEND_ODOMETRY_DATA_COMMAND_CHARACTER)
	{
		sprintf((char*)tx_buffer,"%.3f_%.3f_%.3f\n",R->x_prediction,R->y_prediction,R->theta_prediction);
		HAL_UART_Transmit(&huart2, tx_buffer,30,1);
	}
}

/**
 * @note This is the function we use for dealing with control command that microcontroller gets from 
 * 		Bluetooth 
 * @note Put this function into void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){}
 * @param rx_buffer The pointers of data we receive from UART
 * @param M1 The pointer of typedef struct that store first motor information
 * @param M2 The pointer of typedef struct that store second motor information
**/
void BluetoothFrameHandler(uint8_t *rx_buffer,Motor_measurement_command_typedef *M1,Motor_measurement_command_typedef *M2)
{
		if (*rx_buffer==FORWARD_COMMAND_CHARACTER)
		{
			M1->expected_rpm=10;
			M2->expected_rpm=10;
		}
		else if (*rx_buffer==BACKWARD_COMMAND_CHARACTER)
		{
			M1->expected_rpm=-10;
			M2->expected_rpm=-10;
		}
		else if (*rx_buffer==TURN_LEFT_COMMAND_CHARACTER)
		{
			M1->expected_rpm=-10;
			M2->expected_rpm=10;
		}
		else if (*rx_buffer==TURN_RIGHT_COMMAND_CHARACTER)
		{
			M1->expected_rpm=10;
			M2->expected_rpm=-10;
		}
		else if(*rx_buffer==STOP_COMMAND_CHARACTER)
		{
			M1->expected_rpm=0;
			M2->expected_rpm=0;
		}
}
