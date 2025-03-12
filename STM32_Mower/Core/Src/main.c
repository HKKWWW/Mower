/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "car.h"
#include "PID.h"
#include "servo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
union FloatUnion{  
    uint8_t bytes[4];  
    float value;  
}floatUnion;

uint8_t commandData;
uint8_t receiveSomeData[5] = {0};
float velocityL, velocityR;
uint8_t receiveData;

uint8_t sendDataState = 0;

uint16_t receiveNum = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int8_t encoderLCnt = 0, encoderRCnt = 0;
int32_t actualEncoderLNum, actualEncoderRNum;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL); 
	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
	
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); //AIN1
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4); //BIN1
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //Servo

	HAL_TIM_Base_Start_IT(&htim4);
	
	HAL_TIM_Base_Start_IT(&htim5);
	
	encoderLCnt = 0;
	encoderRCnt = 0;
	
	servoCtl(90);
	
	HAL_UART_Receive_IT(&huart1, &receiveData, 1);
	
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
	
//	car_oneWheel_go(2000, LEFT_WHEEL);
//	car_oneWheel_go(2000, RIGHT_WHEEL);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(sendDataState == 1)
			{
				sendVelocityData();
				sendDataState = 0;
			}
			
		HAL_Delay(5);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == TIM2)
	{
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) == 0) encoderLCnt += 1;
		else encoderLCnt -= 1;		
	}
	
	if(htim->Instance == TIM3)
	{
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3) == 0) encoderRCnt += 1;
		else encoderRCnt -= 1;		
	}
	
	if(htim->Instance == TIM4)
	{
		PID_Velocity_Left(velocityL);
		PID_Velocity_Right(velocityR);	
	}
	
	if(htim->Instance == TIM5)
	{
		if(receiveNum == 0)
		{
			velocityL = 0;
			velocityR = 0;
		}
		else receiveNum = 0;
	}
}

void Clear_Data(uint8_t * dat)
{
	uint8_t i;
	
	for(i = 0; i < 5; i++) dat[i] = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		static uint8_t state, datanum;
//		uint8_t i;	
				
		if(huart->Instance == USART1)
		{
			if(state == 0 && receiveData == 0x56) //56
			{
				state = 1;
			}
			else if(state == 1 && receiveData == 0x91) //91
			{
				state = 2;
			}
			else if(state == 2)
			{
				receiveSomeData[datanum++] = receiveData;
				if(datanum == 5) state = 3;
			}
			else if(state == 3 && receiveData == 0x11) //11
			{
//				for(i = 0; i < 5; i++) printf(" %x ", receiveSomeData[i]);
				state = 0;
				datanum = 0;
						
				commandData = receiveSomeData[0];
				
				if(commandData == 0xaa)
				{
					floatUnion.bytes[0] = receiveSomeData[1];
					floatUnion.bytes[1] = receiveSomeData[2];
					floatUnion.bytes[2] = receiveSomeData[3];
					floatUnion.bytes[3] = receiveSomeData[4];
					
					velocityL = floatUnion.value;
					receiveNum++;
//					printf("\r\n velocityL: %.2f commandData: %x ", velocityL, commandData);
				}
				else if(commandData == 0xbb)
				{
					floatUnion.bytes[0] = receiveSomeData[1];
					floatUnion.bytes[1] = receiveSomeData[2];
					floatUnion.bytes[2] = receiveSomeData[3];
					floatUnion.bytes[3] = receiveSomeData[4];
					
					velocityR = floatUnion.value;
					receiveNum++;
//					printf("\r\n elocityR: %.2f commandData: %x ", velocityR, commandData);
				}
				
				else if(commandData == 0xcc)
				{
					sendDataState = 1;
				}
				
			}			
			else
			{
				state = 0;
				datanum = 0;
				Clear_Data(receiveSomeData);
			}
			HAL_UART_Receive_IT(&huart1, &receiveData, 1);
		}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

