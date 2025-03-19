#include "PID.h"

union float2Uint8_t{  
    uint8_t bytes[4];  
    float velocity;  
}sendVelocityL, sendVelocityR;

uint8_t sendDataL[8] = {0x56, 0x91, 0, 0, 0, 0, 0x55, 0x11};
uint8_t sendDataR[8] = {0x56, 0x91, 0, 0, 0, 0, 0x66, 0x11};

void PID_Velocity_Left(float Target_Velocity)  //PA6 PA7 TIM3(PC6 PC7) TIM_CHANNEL_2
{
	float Actual_Velocity;
	float KP = KP_VEL;
	float KI = KI_VEL;
	float Error ;
	static float LastError;
	static int16_t PWM;
	float  Add_PWM;
	uint8_t i;
	
	actualEncoderLNum = encoderLCnt * 65535 + (int32_t)__HAL_TIM_GetCounter(&htim2);
		
	Target_Velocity *= 1; 
	Actual_Velocity = (float)actualEncoderLNum * 50.0f * WHEEL_RADIUS / PULSE_COUNT;
	
//	Actual_Velocity = 1.25;
	sendVelocityL.velocity = Actual_Velocity;
		
	for(i = 0; i < 4; i++)
	{
		sendDataL[i+2] = sendVelocityL.bytes[i];
	}
	
//	printf("VL : %f \r\n", sendVelocityL.velocity);
//	printf("actualEncoderLNum :%d \r\n", actualEncoderLNum);
	
	Error = Target_Velocity - Actual_Velocity;
	
	Add_PWM = KP * (Error - LastError) + KI * (float)Error;
	
	LastError = Error;
	
	if(Add_PWM > 500) Add_PWM = 500;
	else if(Add_PWM < -500) Add_PWM = -500;
	
	PWM += Add_PWM;
	
	if(PWM > 3500) PWM = 3500;
	else if (PWM < -3500) PWM = -3500;
	
	if(PWM > 0)
	{
		car_oneWheel_go(PWM + 420, LEFT_WHEEL);
	}
	else if(PWM < 0)
	{
		car_oneWheel_back((-1*PWM) + 420, LEFT_WHEEL);
	}
	
	__HAL_TIM_SetCounter(&htim2, 0);
	encoderLCnt = 0;
	
//	if(PID_Clear == 1)
//	{
//		LastError = 0;
//		PWM = 0;
//	}
}

void PID_Velocity_Right(float Target_Velocity)
{
	float Actual_Velocity;
	float KP = KP_VEL;
	float KI = KI_VEL;
	float Error ;
	static float LastError;
	static int16_t PWM;
	float  Add_PWM;
	uint8_t i;
	
	actualEncoderRNum = encoderRCnt * 65535 + (int32_t)__HAL_TIM_GetCounter(&htim3);
	
	Target_Velocity *= 1; 
	Actual_Velocity = (float)actualEncoderRNum * 50.0f * WHEEL_RADIUS / PULSE_COUNT;
	
//	Actual_Velocity = -2.31;
	sendVelocityR.velocity = Actual_Velocity;
	
	for(i = 0; i < 4; i++)
	{
		sendDataR[i+2] = sendVelocityR.bytes[i];
	}
	
//	printf("VR : %f \r\n", sendVelocityR.velocity);
	
	Error = Target_Velocity - Actual_Velocity;
	
	Add_PWM = KP * (Error - LastError) + KI * (float)Error;
	
	LastError = Error;
	
	if(Add_PWM > 500) Add_PWM = 500;
	else if(Add_PWM < -500) Add_PWM = -500;
	
	PWM += Add_PWM;
	
	if(PWM > 3500) PWM = 3500;
	else if (PWM < -3500) PWM = -3500;
	
	if(PWM > 0)
	{
		car_oneWheel_go(PWM + 420, RIGHT_WHEEL);
	}
	else if(PWM < 0)
	{
		car_oneWheel_back((-1*PWM) + 420, RIGHT_WHEEL);
	}
	
	__HAL_TIM_SetCounter(&htim3, 0);
	encoderRCnt = 0;
	
//	if(PID_Clear == 1)
//	{
//		LastError = 0;
//		PWM = 0;
//	}
}

void sendVelocityData(void)
{
	HAL_UART_Transmit(&huart1, sendDataL, 8, 200);
	HAL_UART_Transmit(&huart1, sendDataR, 8, 200);
}


//void PID_allWheel(float Target_Velocity)
//{
//		__HAL_TIM_SetCounter(&htim2, 0);
//		__HAL_TIM_SetCounter(&htim3, 0);	
//		
//		HAL_Delay(20);
//	
//		PID_Velocity_1(Target_Velocity);
//		PID_Velocity_2(Target_Velocity);
//	
////		PID_Clear = 0;
//}

