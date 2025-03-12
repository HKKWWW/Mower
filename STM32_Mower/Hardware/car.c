#include "car.h"
/*========================记得改==========================*/
void car_oneWheel_go(int16_t PWM, uint8_t whatWheel)
{
	switch(whatWheel)
	{
		case LEFT_WHEEL:
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, PWM);
			HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
		break;
		
		case RIGHT_WHEEL: 
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 7199-PWM);
			HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
		break;		
	}
}
/*========================记得改==========================*/
void car_oneWheel_back(int16_t PWM, uint8_t whatWheel)
{
	switch(whatWheel)
	{
		case LEFT_WHEEL:
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 7199-PWM);
			HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
		break;
		
		case RIGHT_WHEEL: 
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, PWM);
			HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
		break;	
	}
}

void car_go(int16_t PWM)
{
	car_oneWheel_go(PWM, LEFT_WHEEL);
	car_oneWheel_go(PWM, RIGHT_WHEEL);
}

void car_back(int16_t PWM)
{
	car_oneWheel_back(PWM, LEFT_WHEEL);
	car_oneWheel_back(PWM, RIGHT_WHEEL);

}

void car_stop(void)
{
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 0);
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);


	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 0);
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
	
}

void car_turn(int16_t PWM, uint8_t whatDiraction)
{
	switch(whatDiraction)
	{
		case LEFT_DIR: 
			car_oneWheel_back(PWM, LEFT_WHEEL);
			car_oneWheel_go(PWM, RIGHT_WHEEL);
		break;
		
		case RIGHT_DIR: 
			car_oneWheel_back(PWM, RIGHT_WHEEL);
			car_oneWheel_go(PWM, LEFT_WHEEL);
		break;
	}
}

