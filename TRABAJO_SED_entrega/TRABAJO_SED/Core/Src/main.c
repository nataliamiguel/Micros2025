/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * @brief          : Main program body
  **************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "i2c-lcd.h"
#include "DHT.h"
#include "math.h"

#define NUMERO_LED 10

typedef struct PINES{
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
}PINES;

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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

	/* Variables globales */
	PINES PINES_LED[] = {{GPIOB,GPIO_PIN_14},{GPIOB,GPIO_PIN_15},{GPIOD,GPIO_PIN_8},
						 {GPIOD,GPIO_PIN_9},{GPIOD,GPIO_PIN_10},{GPIOD,GPIO_PIN_11},
						 {GPIOD,GPIO_PIN_12},{GPIOD,GPIO_PIN_13},{GPIOD,GPIO_PIN_14},
						 {GPIOD,GPIO_PIN_15}};

	DHT_DataTypedef DHT22_Data; //Estructura que guarda la temeperatura y la humedad
	float Humidity; //Variable para guardar la humedad
	float Temperature; //Variable para guardar la temperatura
	volatile int Seleccion_estado = 0; //Se emplea para cambiar el estado y seleccionar la temperatura que queremos
									   //Se pone como volatile pues se usa como interrupción
	static float Temp_control = 25.0; //Temperatura que queremos mantener. Se inicializa en 25ºC
	uint16_t Lec_Temp_Control; //Guarda el valor de la lectura analógica del potenciómetro usado para establecer la Temperatura de control
							   //Se ha usado una resolución de 8 bits para el ADC
	static uint8_t T_min = 20 , T_max = 30 ; //Se definen unos márgenes de funcionamiento que se pueden cambiar
	uint8_t T_diff = 0; ; //Diferencia entre los márgenes de temperatura. Se emplea para la interpolación del potenciómetro



	/* Prototipo de funciones utilizadas */

	/* Devuelve TRUE cuando el boton ha sido presionado tras el periodo de dobounce */
	int debouncer(volatile int* button_int, GPIO_TypeDef* GPIO_port, uint16_t GPIO_number);

	/* Enciende los LED correspondientes a al temp seleccionada y apaga el resto */
	void encenderled(int i, PINES PINES_LED[]);

	/* INTERRUPCIONES */

	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) // Genera la interrupción cuando se pulsa el "user boton" de la placa (PA0)
	{
		//Interrupcion debida al PIN_0
		if(GPIO_Pin==GPIO_PIN_0){
			Seleccion_estado = 1;
		}
	}

	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // Genera una interrupción cada 5 segundos
	{
	  /* Prevent unused argument(s) compilation warning */
	  UNUSED(htim);
	  /* NOTE : This function Should not be modified, when the callback is needed,
	            the __HAL_TIM_PeriodElapsedCallback could be implemented in the user file */

	  /* Obtencion de datos del sensor */
	  /* Ver libreria "DHT.h" */
	  DHT_GetData(&DHT22_Data);

	  /* Asignación de variables globales de Temperatura y Humedad*/
	  Temperature = DHT22_Data.Temperature;
	  Humidity = DHT22_Data.Humidity;

	}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	/* Asignación de variables */
	T_diff = T_max - T_min ;
	float salto_temp = T_diff / NUMERO_LED ; //Se utiliza para el cambio en los LED iluminados
	int total_led = 0 ; //Será el número total de LED a encenderse
	int j = 1; //A usar dentro de un bucle

	/* Variables locales */
	uint32_t tick_start; //Pensado para guarda el valor de HAL_GetTick()
	int t_espera = 2000 ; //Tiempo de espera en ms

	// De momento estas variables no se usan, se pueden usar con la funcion HAL_GetTick() --> while((HAL_GetTick() - tick_pulso) < t_espera)

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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

	  /* Mensaje de inicio */
	  lcd_init();
	  lcd_clear();
	  lcd_put_cur(0,0);
	  lcd_send_string("INICIANDO>>>>");
	  HAL_Delay(2000);
	  lcd_clear();

	  /* Habilitacion de interrupciones */
	  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	  HAL_TIM_Base_Start_IT(&htim2);

	  /* Habilitamos el conversor analogico digital */
	  HAL_ADC_Start(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(debouncer(&Seleccion_estado, GPIOA, GPIO_PIN_0)){ //Estado para la eleccion de la temperatura de control. NO ESCRIBIR AQU�?

		  lcd_clear();
		  lcd_put_cur(0,0);
		  lcd_send_string("Choose new temp");
		  int i = 1;

		  while(!debouncer(&Seleccion_estado, GPIOA, GPIO_PIN_0)){

			  //Habilitacion del conversor
			  HAL_ADC_Start(&hadc1);

			  //Lectura del conversor
			  if(HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
				  Lec_Temp_Control = HAL_ADC_GetValue(&hadc1);

			  //Asignación de la temperatura de control
			  Temp_control = (float)Lec_Temp_Control / 255 * T_diff + 15; //La pendiente de la interpolación es 25/255

			  //Mostramos por pantalla la temperatura que escogemos
			  //Se ha añadido el bucle para reducir la frecuencia de la pantalla y evitar el parpadeo. Se ha tomado un valor empírico
			  if (!(i++ % 20000)){
				  Display_Temp(Temp_control*10 , 1);
				  i = 1;
			  }//End_IF

		  }//End_WHILE

		  //Deshabilitacion del conversor
		  HAL_ADC_Stop(&hadc1);

	  }//End_IF

	  else{ //Estado para la ejecución del programa con una temperatura de control dada

		  /* Mostramos los valores obtenidos en la pantalla LCD */
		  /* Ver libreria "i2c-lcd.h" */
		  if (!(j++ % 20000)){
			  Display_Rh(Humidity);
			  Display_Temp(Temperature , 0);
			  j = 1;
		  }
		  /* Control de la barra LED */

		  /* Se trunca el numero de led en función del salto térmico */
		  total_led = (int)((Temperature/10 - Temp_control)/ salto_temp);

		  /* Llamada a la función encenderled */

		  encenderled(total_led,PINES_LED);

		  /* Control del servomotor */


		}//End_ELSE

 }//End_WHILE(1)

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 50000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

	/* Definción de funciones utilizadas */
	int debouncer(volatile int* button_int, GPIO_TypeDef* GPIO_port, uint16_t GPIO_number){
		static uint8_t button_count=0;
		static int counter=0;

		if (*button_int==1){
			if (button_count==0) {
				counter=HAL_GetTick();
				button_count++;
			}
			if (HAL_GetTick()-counter>=20){
				counter=HAL_GetTick();
				if (HAL_GPIO_ReadPin(GPIO_port, GPIO_number)!=1){
					button_count=1;
				}
				else{
					button_count++;
				}
				if (button_count==4){ //Periodo antirebotes
					button_count=0;
					*button_int=0;
					return 1;
				}
			}
		}
		return 0;
	}

	void encenderled(int i, PINES PINES_LED[]){

		int j = 0;

		for(int n = 0 ; n < i ; n++){
			HAL_GPIO_WritePin(PINES_LED[n].GPIOx, PINES_LED[n].GPIO_Pin, GPIO_PIN_SET);
			j++;
		}//End_FOR

		while(j < NUMERO_LED){
			HAL_GPIO_WritePin(PINES_LED[j].GPIOx, PINES_LED[j].GPIO_Pin, GPIO_PIN_RESET);
			j++;
		}//End_WHILE
	};

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
