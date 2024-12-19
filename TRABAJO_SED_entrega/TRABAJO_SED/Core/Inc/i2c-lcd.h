#include "stm32f4xx_hal.h"

void lcd_init(void);   // Inicializacion del LCD

void lcd_send_cmd(char cmd);  // Envío de comandos

void lcd_send_data(char data);  // Envio de información

void lcd_send_string(char *str);  // Envío de una cadena de caracteres

void lcd_put_cur(int row, int col);  // Desplazamiento del cursor: row (0 or 1), col (0-15);

void lcd_clear(void);  // Limpieza de toda la pantalla

void lcd_clear_row (int row);  // Limpieza de una fila

void Display_Rh(float Rh);  // Muestra el valor Rh por pantalla, siempre en la fila 1

void Display_Temp(float Temp, int row);  // Muestra el valor Temp por pantalla, tanto en la fila 0 y 1
