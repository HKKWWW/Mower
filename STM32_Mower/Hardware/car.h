#ifndef _CAR_H
#define _CAR_H

#include "main.h"
#include "tim.h"

#define LEFT_WHEEL 1
#define RIGHT_WHEEL 2

#define LEFT_DIR 1
#define RIGHT_DIR 2

#define GO 1
#define BACK 2
#define LEFT 3
#define RIGHT 4
#define STOP 0

void car_oneWheel_go(int16_t PWM, uint8_t whatWheel);
void car_oneWheel_back(int16_t PWM, uint8_t whatWheel);
void car_go(int16_t PWM);
void car_back(int16_t PWM);
void car_stop(void);
void car_turn(int16_t PWM, uint8_t whatDiraction);

#endif
