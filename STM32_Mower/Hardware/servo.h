#ifndef SERVO_H
#define SERVO_H

#include "main.h"
#include "tim.h"

uint16_t setAngle(uint16_t angle);
void servoCtl(uint16_t angle);

#endif
