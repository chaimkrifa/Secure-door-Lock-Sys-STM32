#ifndef INC_KEYPAD_H_
#define INC_KEYPAD_H_


#include "stm32f1xx_hal.h"


/*------------- Define For Connection -----------------*/

#define COL_1_Pin GPIO_PIN_0
#define COL_2_Pin GPIO_PIN_1
#define COL_3_Pin GPIO_PIN_2
#define COL_4_Pin GPIO_PIN_3
#define ROW_1_Pin GPIO_PIN_4
#define ROW_2_Pin GPIO_PIN_5
#define ROW_3_Pin GPIO_PIN_6
#define ROW_4_Pin GPIO_PIN_7


/*------------ Declaring Function Prototype -------------*/

uint8_t keypad_scan(void);












#endif
