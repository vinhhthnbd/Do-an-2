/*
 * PID.c
 *
 *  Created on: Oct 13, 2024
 *      Author: GIGABYTE
 */
#include <stdio.h>
#include <stdint.h>
#include "PID.h"

void pidControllersInit(PIDControllers_Typedef* pid,float Kp,float Ki,float Kd,float to,float T,float satuaration)
{
	pid->Kp=Kp;
	pid->Ki=Ki;
	pid->Kd=Kd;
	pid->to=to;
	pid->T=T;

	pid->et=0;
	pid->ek1=0;

	pid->eKit=0;
	pid->eKik1=0;

	pid->propotion=0;
	pid->integrator=0;
	pid->deviator=0;
	pid->integratork1=0;
	pid->deviatork1=0;

	pid->saturation=satuaration;

	pid->measurement=0;

	pid->u=0;

}

float pidUpdate(PIDControllers_Typedef* pid,float measurement,float input)
{
	//get pid input
	pid->expected=input;
	pid->measurement=measurement;
	pid->et=pid->expected-pid->measurement;
	// clamping section
	if(pid->u!=pid->v)
	{
		pid->uv=1;
	}
	else
	{
		pid->uv=0;
	}

	if((pid->et*pid->v)>=0)
	{
		pid->ev=1;
	}
	else
	{
		pid->ev=0;
	}

	if((pid->uv&pid->ev)==0)
	{
		pid->eKit=pid->et;
		pid->eKik1=pid->ek1;
	}
	else
	{
		pid->eKik1=0;
		pid->eKit=0;
	}
	//PID calculation
	pid->propotion=pid->Kp*pid->et;
	pid->integrator=((pid->Ki*pid->T)/2)*(pid->eKit+pid->eKik1)+pid->integratork1;

	pid->deviator=((2*pid->Kd)/(2*pid->to+pid->T))*(pid->et-pid->ek1)+((2*pid->to-pid->T)
			/(2*pid->to+pid->T))*pid->deviatork1;

	pid->ek1=pid->et;
	pid->integratork1=pid->integrator;
	pid->deviatork1=pid->deviator;
	pid->v=pid->propotion+pid->integrator+pid->deviator;

	//saturation
	if(pid->v>pid->saturation)
	{
		pid->u=pid->saturation;
	}
	else if(pid->v<(-pid->saturation))
	{
		pid->u=-pid->saturation;
	}
	else
	{
		pid->u=pid->v;
	}

	return pid->u;
}
