#include "JY61P.h"

void set_yawZero(void)
{
	uint8_t unlock_command[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
	uint8_t z2Zero_command[5] = {0xFF, 0xAA, 0x01, 0x04, 0x00};
	uint8_t save_command[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};

		HAL_UART_Transmit(&huart2, unlock_command, 5, 1000);
		HAL_Delay(200);
		
		HAL_UART_Transmit(&huart2, z2Zero_command, 5, 1000);
		HAL_Delay(500);
		
		HAL_UART_Transmit(&huart2, save_command, 5, 1000);
}
