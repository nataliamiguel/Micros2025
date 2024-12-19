

#include "i2c-lcd.h"
#include "stdio.h"
extern I2C_HandleTypeDef hi2c1;

#define SLAVE_ADDRESS_LCD 0x4E

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_clear_row(int row)
{
	lcd_put_cur(row , 0);
	for (int i=0; i<35; i++)
	{
		lcd_send_data (' ');
	}
}

void lcd_clear (void)
{
	lcd_clear_row(1);
	lcd_clear_row(2);
}

void lcd_put_cur(int row, int col) //row [0,1], col[0,15]
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

    lcd_send_cmd (col);
}


void lcd_init (void) //La sucesión de inicialización se encuentra en el datasheet y en la memoria. Se usa el modo de 4 bits
{
	// 4 bit initialisation
	HAL_Delay(50);  // wait for >40ms
	lcd_send_cmd (0x30);
	HAL_Delay(5);  // wait for >4.1ms
	lcd_send_cmd (0x30);
	HAL_Delay(1);  // wait for >100us
	lcd_send_cmd (0x30);
	HAL_Delay(10);
	lcd_send_cmd (0x20);  // 4bit mode
	HAL_Delay(10);

  // dislay initialisation
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd (0x01);  // clear display
	HAL_Delay(1);
	HAL_Delay(1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

void Display_Rh(float Rh){  // Antes de mostrar el dato se limpia la fila

	char str[20] = {0};
	lcd_clear_row(1);
	lcd_put_cur(1,0);

	sprintf(str,"RH:%.2f",Rh/10);
	lcd_send_string(str);
	lcd_send_data('%');
}

void Display_Temp(float Temp, int row){  // Antes de mostrar el dato se limpia la fila

	char str[20] = {0};
	lcd_clear_row(row);
	lcd_put_cur(row,0);

	sprintf(str,"TEMP:%.2f",Temp/10);
	lcd_send_string(str);
	lcd_send_data('C');
}
