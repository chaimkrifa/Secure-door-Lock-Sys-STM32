#include "lcd_txt.h"



/*--------------- Initialize LCD ------------------*/
void lcd_init(void)
{
	
	HAL_Delay(30);
	
	PIN_LOW(D4_PORT,D4_PIN);
	PIN_HIGH(D5_PORT,D5_PIN);
	PIN_LOW(D6_PORT,D6_PIN);
	PIN_LOW(D7_PORT,D7_PIN);
	PIN_LOW(RS_PORT,RS_PIN);
	
	PIN_HIGH(EN_PORT,EN_PIN);
	PIN_LOW(EN_PORT,EN_PIN);
	

	send_8_bits_to_lcd(0x28, 0);

	send_8_bits_to_lcd(0x0c, 0);

	send_8_bits_to_lcd(0x06, 0);

	send_8_bits_to_lcd(0x01, 0);

}

/*--------------- Write To LCD ---------------*/

void send_4_bits_to_lcd (uint8_t data, uint8_t rs_val)
{
	HAL_GPIO_WritePin(RS_PORT, RS_PIN, rs_val);

	HAL_GPIO_WritePin(D7_PORT, D7_PIN, (data>>3)&0x01);
	HAL_GPIO_WritePin(D6_PORT, D6_PIN, (data>>2)&0x01);
	HAL_GPIO_WritePin(D5_PORT, D5_PIN, (data>>1)&0x01);
	HAL_GPIO_WritePin(D4_PORT, D4_PIN, (data>>0)&0x01);

	HAL_GPIO_WritePin(EN_PORT, EN_PIN, 1);
	HAL_GPIO_WritePin(EN_PORT, EN_PIN, 0);

}

void send_8_bits_to_lcd (uint8_t data, uint8_t rs_val)
{
	send_4_bits_to_lcd((data>>4) & 0x0F, rs_val);

	send_4_bits_to_lcd(data & 0x0F, rs_val);


}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }
    send_8_bits_to_lcd (col,0);
}

void lcd_puts(int8_t *string , int n)
{
	//Set Cursor Position
	for (int i=0; i<n ;i++)
	{
		//lcd_write(1,*string);
		send_8_bits_to_lcd(*string, 1);
		string++;
	}
}
void lcd_clear(void)
{
	send_8_bits_to_lcd(0x01, 0);

}



