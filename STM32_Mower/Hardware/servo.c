#include "servo.h"

uint16_t setAngle(uint16_t angle)
{
	uint16_t PWM;
	
	PWM = (uint16_t)((2000 / 180.0f) * angle + 500);
	
	return PWM;
}

void servoCtl(uint16_t angle)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, setAngle(angle));
}

