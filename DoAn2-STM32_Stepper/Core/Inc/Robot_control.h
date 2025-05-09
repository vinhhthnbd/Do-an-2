/*
 * Robot_control.h
 *
 *  Created on: Oct 18, 2024
 *      Author: GIGABYTE
 */

#ifndef INC_ROBOT_CONTROL_H_
#define INC_ROBOT_CONTROL_H_
#ifdef __cplusplus
 extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>


#define WHEEL_RADIUS 31.8644
#define RW_DISTANCE_WHEEL_CENTER 22


#define STOP_COMMAND_CHARACTER 's'
#define FORWARD_COMMAND_CHARACTER 'f'
#define BACKWARD_COMMAND_CHARACTER 'b'
#define TURN_LEFT_COMMAND_CHARACTER 'l'
#define TURN_RIGHT_COMMAND_CHARACTER 'r'
#define SEND_CORRECTION_DATA 'c'

#define SEND_ODOMETRY_DATA_COMMAND_CHARACTER 'e'
#define UPDATE_PREDICTION_CHARACTER 'R'
#define END_CORRECTION_DATA_CHARACTER '\n'
#define SPLIT_CORRECTION_DATA_CHARACTER "/"
typedef struct
{

	float s;
	float sl;
	float sr;

	float x_prediction;
	float y_prediction;
	float theta_prediction;

	float x_correction;
	float y_correction;
	float theta_correction;

	float delta_x;
	float delta_y;
	float delta_theta;

}Robot_model_typedef;

void calculatePredictionValue(Robot_model_typedef *R,float sl,float sr);
void resetDeltaValue(Robot_model_typedef *R);
void observation(Robot_model_typedef *R,uint8_t *str);

void BluetoothFrameHandler(uint8_t *rx_buffer,uint8_t *tx_buffer,Robot_model_typedef *R);


#ifdef __cplusplus
}
#endif
#endif /* INC_ROBOT_CONTROL_H_ */
