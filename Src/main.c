/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "ssd1306.h"
#include "fonts.h"
#include "BMP180.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void showPage(float, float, float, float, float, float);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DHT22_PORT GPIOB
#define DHT22_PIN GPIO_PIN_14

uint8_t hum1, hum2, tempC1, tempC2, SUM, CHECK;
uint32_t pMillis, cMillis;
float temp_Celsius = 0;
float temp_Fahrenheit = 0;
float Humidity = 0;
uint8_t hum_integral, hum_decimal, tempC_integral, tempC_decimal, tempF_integral, tempF_decimal;
char string[15];

void microDelay (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}

uint8_t DHT22_Start (void)
{
  uint8_t Response = 0;
  GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
  GPIO_InitStructPrivate.Pin = DHT22_PIN;
  GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStructPrivate);
  HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 0);
  microDelay (1300);
  HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 1);
  microDelay (30);
  GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStructPrivate);
  microDelay (40);
  if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))
  {
    microDelay (80);
    if ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN))) Response = 1;
  }
  pMillis = HAL_GetTick();
  cMillis = HAL_GetTick();
  while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
  {
    cMillis = HAL_GetTick();
  }
  return Response;
}

uint8_t DHT22_Read (void)
{
  uint8_t x,y;
  for (x=0;x<8;x++)
  {
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
    {
      cMillis = HAL_GetTick();
    }
    microDelay (40);
    if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))   // if the pin is low
      y&= ~(1<<(7-x));
    else
      y|= (1<<(7-x));
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go low
      cMillis = HAL_GetTick();
    }
  }
  return y;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
uint8_t message[35] = {'\0'};

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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  SSD1306_Init();
  BMP180_Start();

//  SSD1306_GotoXY (10,10); // goto 10, 10
//  SSD1306_Puts ("HELLO", &Font_11x18, 1); // print Hello
//  SSD1306_GotoXY (10, 30);
//  SSD1306_Puts ("WORLD !!", &Font_11x18, 1);
//  SSD1306_UpdateScreen(); // update screen

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t serial[35] = {'\0'};

  float Temperature = 0;
  float Pressure = 0;
  float Altitude = 0;

  char Temperature1[10];
  char Pressure1[10];
  char Altitude1[10];

  GPIO_PinState ButtonStateCurrent;
  GPIO_PinState ButtonStateLast = GPIO_PIN_SET;
  while (1)
    {
    /* USER CODE END WHILE */
	  if(DHT22_Start()) {
		  //Il DHT22 invia sul pin di output un totale 40 bit a blocchi di 8 bit la volta
		  //I primi due blocchi (1-16 bit) costituiscono l'umidita' relativa moltiplicato 10 (in percentuale)
		  //I successivi due blocchi (17-32 bit) costituiscono la temperatura in gradi celsius moltiplicato 10. Il primo bit (bit 17) costituisce il segno della temperatura
		  //I restanti bit costituiscono il checksum. Serve per verificare che sia stata effettuata una rilevazione consistente
		  hum1 = DHT22_Read();
		  hum2 = DHT22_Read();
		  tempC1 = DHT22_Read();
		  tempC2 = DHT22_Read();
		  SUM = DHT22_Read();
		  CHECK = hum1 + hum2 + tempC1 + tempC2;

		  if (CHECK == SUM) {
			 if (tempC1>127) // If TC1=10000000, negative temperature
			 {
			   temp_Celsius = (float)(((tempC1<<8)|tempC2)/10)*(-1);
			 }
			 else
			 {
			   temp_Celsius = (float)((tempC1<<8)|tempC2)/10;
			 }
		  }
		  Humidity = (float) ((hum1<<8)|hum2)/10; //Somma dei primi 8 bit e dei successivi 8 diviso 10

		  //Calcolo della temperatura percepita con indice Humidex
		  //float e = (6,112*10^(7,5*temp_Celsius/(237,7+temp_Celsius))*Humidity/100);
		  float e = (6.112*pow(10,7.5*temp_Celsius/(237.7+temp_Celsius))*Humidity/100);
		  float H = temp_Celsius + (double)5/9 * (e-10); //La divisione tra interi in c/c++ restituisce 0. Necessario cast a double

		  //Calcolo della temperatura percepita con heat index
		  float tF = temp_Celsius * ((double)9 / 5) + 32; //La divisione tra interi in c/c++ restituisce 0. Necessario cast a double
		  float t2 = tF * tF;
		  float u2 = Humidity * Humidity;
		  float HI = -42.379 + 2.04901523 * tF + 10.14333127 * Humidity - 0.22475541 * tF * Humidity - 6.83783e-3 * t2 - 5.481717e-2 * u2 + 1.22874e-3 * t2 * Humidity + 8.5282e-4 * tF * u2 - 1.99e-6 * t2 * u2;
		  HI = (double)5 / 9 * (HI - 32);
		  HI = floor(HI * 10) / 10;

		  if(HI<=25) {
			  if(HAL_GPIO_ReadPin(LED_VERDE_GPIO_Port, LED_VERDE_Pin) == 0) {
				  HAL_GPIO_WritePin(LED_ROSSO_GPIO_Port, LED_ROSSO_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LED_BLU_GPIO_Port, LED_BLU_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, GPIO_PIN_SET);
			  }
		  } else if(HI>25 && HI<=30){
			  if(HAL_GPIO_ReadPin(LED_BLU_GPIO_Port, LED_BLU_Pin) == 0) {
				  HAL_GPIO_WritePin(LED_ROSSO_GPIO_Port, LED_ROSSO_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LED_BLU_GPIO_Port, LED_BLU_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, GPIO_PIN_RESET);
			  }
		  } else {
			  if(HAL_GPIO_ReadPin(LED_ROSSO_GPIO_Port, LED_ROSSO_Pin) == 0) {
				  HAL_GPIO_WritePin(LED_ROSSO_GPIO_Port, LED_ROSSO_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(LED_BLU_GPIO_Port, LED_BLU_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, GPIO_PIN_RESET);
			  }
		  }
		  sprintf(serial, "T.perc.=%.2f\n", HI);
		  HAL_UART_Transmit(&huart2, serial, sizeof(serial), 100);

		  sprintf(serial, "T.misurata=%.2f\r\n", temp_Celsius);
		  HAL_UART_Transmit(&huart2, serial, sizeof(serial), 100);

		  sprintf(serial, "Umidita'=%.2f\r\n", Humidity);
		  HAL_UART_Transmit(&huart2, serial, sizeof(serial), 100);

		  Temperature = BMP180_GetTemp();
		  Pressure = BMP180_GetPress (0);
		  Altitude = BMP180_GetAlt(0);

		  showPage(HI, temp_Celsius, Humidity, Temperature, Pressure, Altitude);

		  HAL_Delay(2000);
	  }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void showPage(float HI, float temp_Celsius, float Humidity, float Temperature, float Pressure, float Altitude) {
	sprintf(message, "T.perc.=%.2fC", HI);
	SSD1306_GotoXY (0, 0);
	SSD1306_Puts (message, &Font_7x10, 1);

	sprintf(message, "T.misurata=%.2fC", temp_Celsius);
	SSD1306_GotoXY (0, 20);
	SSD1306_Puts (message, &Font_7x10, 1);

	sprintf(message, "Umidita'=%.2f %%", Humidity);
	SSD1306_GotoXY (0, 30);
	SSD1306_Puts (message, &Font_7x10, 1);

	sprintf(message, "Press.=%.2fhPa", Pressure);
	SSD1306_GotoXY (0, 40);
	SSD1306_Puts (message, &Font_7x10, 1);

	if(Pressure<980) {
		sprintf(message, "Piogge intense. Vento forte");
		SSD1306_GotoXY (0, 50);
		SSD1306_Puts (message, &Font_7x10, 1);
	} else if(Pressure>=980 && Pressure <990) {
		sprintf(message, "Molto nuvoloso.Piogge moderate");
		SSD1306_GotoXY (0, 50);
		SSD1306_Puts (message, &Font_7x10, 1);
	}
	else if(Pressure>=990 && Pressure <1000) {
		sprintf(message, "Nuvoloso. Piogge leggere");
		SSD1306_GotoXY (0, 50);
		SSD1306_Puts (message, &Font_7x10, 1);
	} else if(Pressure>=1000 && Pressure <1013) {
		sprintf(message, "Poco nuvoloso.Tempo stabile");
		SSD1306_GotoXY (0, 50);
		SSD1306_Puts (message, &Font_7x10, 1);
	} else {
		sprintf(message, "Cielo sereno. Bel tempo");
		SSD1306_GotoXY (0, 50);
		SSD1306_Puts (message, &Font_7x10, 1);
	}

	sprintf(message, "Altitudine=%.2fm", Altitude);
	SSD1306_GotoXY (0, 50);
	SSD1306_Puts (message, &Font_7x10, 1);

	SSD1306_UpdateScreen();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_ROSSO_Pin|LED_VERDE_Pin|DHT22_Pin|LED_BLU_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PUSH_BUTTON_Pin */
  GPIO_InitStruct.Pin = PUSH_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PUSH_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_ROSSO_Pin LED_VERDE_Pin DHT22_Pin LED_BLU_Pin */
  GPIO_InitStruct.Pin = LED_ROSSO_Pin|LED_VERDE_Pin|DHT22_Pin|LED_BLU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
