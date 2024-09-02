#include "keypad.h"




uint8_t keypad_scan(void)
{
	uint8_t keys[4][4] = {{'7', '8', '9', '/'},
			{'4', '5', '6', '*'},
			{'1', '2', '3', '-'},
			{'o', '0', '=', '+'}};

	for(int i = 0; i < 4; i++)
	{
		// Set current column as output and low
		switch(i)
		{
		case 0:
			HAL_GPIO_WritePin(GPIOA, COL_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, COL_2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, COL_3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, COL_4_Pin, GPIO_PIN_SET);
			break;

		case 1:
			HAL_GPIO_WritePin(GPIOA, COL_1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, COL_2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, COL_3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, COL_4_Pin, GPIO_PIN_SET);
			break;

		case 2:
			HAL_GPIO_WritePin(GPIOA, COL_1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, COL_2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, COL_3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, COL_4_Pin, GPIO_PIN_SET);
			break;

		case 3:
			HAL_GPIO_WritePin(GPIOA, COL_1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, COL_2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, COL_3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, COL_4_Pin, GPIO_PIN_RESET);
			break;
		}
		// Read current rows
		if(HAL_GPIO_ReadPin(GPIOA, ROW_1_Pin) == GPIO_PIN_RESET)
			return keys[0][i];
		if(HAL_GPIO_ReadPin(GPIOA, ROW_2_Pin) == GPIO_PIN_RESET)
			return keys[1][i];
		if(HAL_GPIO_ReadPin(GPIOA, ROW_3_Pin) == GPIO_PIN_RESET)
			return keys[2][i];
		if(HAL_GPIO_ReadPin(GPIOA, ROW_4_Pin) == GPIO_PIN_RESET)
			return keys[3][i];
	}
	return 0; // No key pressed
}

