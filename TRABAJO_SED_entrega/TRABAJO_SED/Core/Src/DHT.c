
#include "stm32f4xx_hal.h"
#include "DHT.h"
#include "i2c-lcd.h"

/* Se pueden escoger ambos sensores. Cambia la resolución de estos por lo que se hace distinción. */
//#define TYPE_DHT11	//8 bit
#define TYPE_DHT22		//16 bit

/* Usamos el PA1. Lo renombramos. */
#define DHT_PORT GPIOA
#define DHT_PIN GPIO_PIN_1

/* Variables globales */
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2; //Guardan los byte con información de la Temp y Rh
uint16_t SUM ; //Sirve para la comprobación de que se han enviado correctamente los datos
uint8_t Presence = 0 ;	//Indica si el sensor ha detectado la petición de recibir datos de la placa



uint32_t DWT_Delay_Init(void)
{
  /* Disable TRC */
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
  /* Enable TRC */
  CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;

  /* Disable clock cycle counter */
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
  /* Enable  clock cycle counter */
  DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

  /* Reset the clock cycle counter value */
  DWT->CYCCNT = 0;

     /* 3 NO OPERATION instructions */
     __ASM volatile ("NOP");
     __ASM volatile ("NOP");
     __ASM volatile ("NOP");

  /* Check if clock cycle counter has started */
     if(DWT->CYCCNT)
     {
       return 0; /*clock cycle counter started*/
     }
     else
  {
    return 1; /*clock cycle counter not started*/
  }
}

__STATIC_INLINE void delay(volatile uint32_t microseconds) //Se crea un delay usando la frecuencia del reloj del sistema
{
  uint32_t clk_cycle_start = DWT->CYCCNT;

  /* Go to number of cycles for system */
  microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);

  /* Delay till end */
  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) //Un pin dado se define como salida
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) //Un pin dado se define como entrada
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}


void DHT_Start (void){ //Petición de la placa para recibir datos.
					   //Se crea un pulso a nivel bajo de 1-10ms, seguido de uno a nivel alto de 20-40us. (DTH22)
					   //Tras mandar el pulso se pone el pin como entrada para recibir datos del sensor

	DWT_Delay_Init();
	Set_Pin_Output (DHT_PORT, DHT_PIN);  // set the pin as output
	HAL_GPIO_WritePin (DHT_PORT, DHT_PIN, 0);   // pull the pin low

#if defined(TYPE_DHT11)
	delay (18000);   // wait for 18ms
#endif

#if defined(TYPE_DHT22)
	delay (10000);  // >1ms delay
#endif

    HAL_GPIO_WritePin (DHT_PORT, DHT_PIN, 1);   // pull the pin high
    delay (30);   // wait for 43us
	Set_Pin_Input(DHT_PORT, DHT_PIN);    // set as input
}

uint8_t DHT_Check_Response (void){ //Comprobación de que el sensor ha detectado la petición de la placa
								   //Devuelve 0 si no lo ha detectado, -1 si el pulso de confirmación es errónea y 1 si es correcto

	uint8_t Response = 0;
	delay (40);
	if (!(HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)))
	{
		delay (80);
		if ((HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN))) Response = 1;
		else Response = -1;
	}
	while (HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN));   // wait for the pin to go low

	return Response;
}

uint8_t DHT_Read (void){ //Lee un byte de datos (1 bit cada 35us). Desplaza y concatena cada bit en una sola variable

	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)));   // wait for the pin to go high
		delay (35);   // wait for 35 us
		if (!(HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)));  // wait for the pin to go low
	}
	return i;
}



void DHT_GetData (DHT_DataTypedef *DHT_Data)
{
    DHT_Start ();
	Presence = DHT_Check_Response ();
	if (Presence == 1){
		Rh_byte1 = DHT_Read ();
		Rh_byte2 = DHT_Read ();
		Temp_byte1 = DHT_Read ();
		Temp_byte2 = DHT_Read ();
		SUM = DHT_Read();

		if (SUM == (Rh_byte1+Rh_byte2+Temp_byte1+Temp_byte2)) //Comprobación de envío correcto
		{
			#if defined(TYPE_DHT11)
				DHT_Data->Temperature = Temp_byte1;
				DHT_Data->Humidity = Rh_byte1;
			#endif

			#if defined(TYPE_DHT22)
				DHT_Data->Temperature = ((Temp_byte1<<8)|Temp_byte2);
				DHT_Data->Humidity = ((Rh_byte1<<8)|Rh_byte2);
			#endif
		}
	}
	else if (Presence == -1){ //Si se produce un error en la detección las señales se ponen a nivel alto
		#if defined(TYPE_DHT11)
			DHT_Data->Temperature = 0xFF;
			DHT_Data->Humidity = 0xFF;
		#endif

		#if defined(TYPE_DHT22)
			DHT_Data->Temperature = 0xFFFF;
			DHT_Data->Humidity = 0xFFFF;
		#endif
	}

}


