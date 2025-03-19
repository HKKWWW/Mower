#ifndef _PID_H
#define _PID_H

#include "main.h"
#include <stdio.h>
#include "usart.h"
#include "tim.h"
#include "car.h"

#define PULSE_COUNT 102000  //51 * 4 * 500 
#define WHEEL_RADIUS 56.52f

#define KP_VEL 2.56f
#define KI_VEL 36.66f
#define KD_VEL 0

//void PID_Velocity_3(float velocity);
void PID_Velocity_Left(float Target_Velocity);
void PID_Velocity_Right(float Target_Velocity);
void PID_allWheel(float Target_Velocity);
void sendVelocityData(void);
#endif
