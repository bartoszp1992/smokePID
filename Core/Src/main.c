/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../Display/an_disp.h"
#include <math.h>
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
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim21;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM21_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float conversionToMultiplier(uint16_t conversion, uint16_t maxMultiplier);
float conversionToTemperature(uint16_t conversion);
uint16_t conversionToServo(uint16_t reading);
int32_t PID(float targetValue, float currentValue, float *integralSum,
		float *lastError, uint32_t *Dcounter, float *xD, float Pmultiplier,
		float Imultiplier, float Dmultiplier);

uint32_t adcReadings[5];

float temperature;

uint32_t setPoint;

float amplificationP;
float amplificationI;
float amplificationD;

float integralSum;
float lastError;
uint32_t Dcounter;
float xD;

uint32_t throttle;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_DMA_Init();
	MX_ADC_Init();
	MX_TIM2_Init();
	MX_TIM21_Init();
	/* USER CODE BEGIN 2 */

	//inicjalizacja wyświetlacza
	lcdInit();
	lcdLocate(4, 0);
	lcdStr("smokePID");
	lcdLocate(12, 1);
	lcdStr("v1.2");
	HAL_Delay(1000);

	//uruchomienie PWM
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	//test serwa
	lcdLocate(0, 1);
	lcdStr("MIN");
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
	HAL_Delay(1000);

	lcdLocate(0, 1);
	lcdStr("MAX");
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2000);
	HAL_Delay(1000);

	lcdLocate(0, 1);
	lcdStr("MIN");
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
	HAL_Delay(1000);

	lcdClear();

	//kalibracja ADC
	HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);

	//uruchomienie przetwornika
	HAL_ADC_Start_DMA(&hadc, adcReadings, 5);

	//uruchomienie niezależnego timera
	HAL_TIM_Base_Start_IT(&htim21);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		//pokaż położenie przepustnicy
//		lcdLocate(8, 0);
//		if (throttle < 2000)
//			lcdInt((throttle - 1000) / 10);
//		else
//			lcdInt(99);
//		lcdStr(" ");

		//pokaż regulator
//		lcdLocate(6, 0);
//		lcdInt(xD);
//		lcdStr("  ");

		lcdLocate(6, 0);
		lcdStr(" PID");

		//pokaż zadaną temperaturę
		lcdLocate(0, 0);
		lcdInt(setPoint);
		lcdStr("stC ");

		//pokaż rzeczywistą temperaturę
		lcdLocate(11, 0);
		lcdInt((uint32_t) temperature);
		lcdStr("stC");

		//pokaż wzmocnienia regulatorów
		lcdLocate(0, 1);
		lcdFloat(amplificationP, 10);
		lcdStr(" ");

		lcdLocate(6, 1);
		lcdFloat(amplificationI, 100);
		lcdStr(" ");

		lcdLocate(12, 1);
		lcdFloat(amplificationD, 10);
		lcdStr(" ");

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM21) {

		/*kanały ADC
		 * 0- ustawiona temperatura
		 * 1- wzmocnienie P
		 * 2- wzmocnienie I
		 * 3- wzmocnienie D
		 * 4- odczytana temperatura
		 */

		//test serwa i wejść adc
		//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,
		//conversionToServo(adcReadings[0]));

		//odczyt temperatury
		//różniczkowany
//		if (conversionToTemperature(adcReadings[4]) > temperature)
//			temperature = temperature + 0.01;
//		else if (conversionToTemperature(adcReadings[4]) < temperature)
//			temperature = temperature - 0.01;
		//bezpośredni
		temperature = conversionToTemperature(adcReadings[4]);

		//stała wartość
		//		temperature = 30.54;

		//odczyt zadanej temperatury
		setPoint = adcReadings[0] / 28;

		//odczyt wzmocnień
		amplificationP = conversionToMultiplier(adcReadings[1], 40);
		amplificationI = conversionToMultiplier(adcReadings[2], 10);
		amplificationD = conversionToMultiplier(adcReadings[3], 30);

		throttle = PID(setPoint, temperature, &integralSum, &lastError,
				&Dcounter, &xD, amplificationP, amplificationI, amplificationD)
				+ 1000;

		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, throttle);

	}

}

float conversionToMultiplier(uint16_t conversion, uint16_t maxMultiplier) {
	/*przelicz odczyt z potencjometru na mnożnik. Poniżej połowy- mnoznik ułamkowy.
	 Powyżej połowy- mnożnik całkowity o maksymalnej wartości maxMultiplier*/

	float reg;

	if (conversion < 2048) {
		reg = ((float) conversion * 1) / 2047;

	} else if (conversion >= 2048) {
		float conversion2 = (float) conversion - 2048;
		reg = ((maxMultiplier-1) * conversion2 / 2047) + 1;
	}
	return reg;
}

float conversionToTemperature(uint16_t conversion) {
	/*przelicz odczyt z termometru na temperaturę wg klucza 10mV/stC*/

	float temperature = (conversion * 3.3 / 4095) * 100;
	return temperature;
}

uint16_t conversionToServo(uint16_t reading) {
	return ((reading * 100) / 4095) + 100;
}

int32_t PID(float targetValue, float currentValue, float *integralSum,
		float *lastError, uint32_t *Dcounter, float *xD, float Pmultiplier,
		float Imultiplier, float Dmultiplier) {

	//obliczenie uchybu
	float currentError = targetValue - currentValue;

	//człon proporcjonalny
	float xP = currentError * Pmultiplier;

	//człon całkujący
	*integralSum = *integralSum + (currentError / 100);
	float xI = *integralSum * Imultiplier;

	//anty Wind-Up(blokada pętli całkowania)
	float antiWindUp = 1000;		//max windUp
	if (*integralSum >= antiWindUp && currentError > 0)
		*integralSum = antiWindUp;
	else if (*integralSum <= 0 - antiWindUp && currentError < 0)
		*integralSum = 0 - antiWindUp;
	//^ blokada ma zapobiec nieskończonemu wzrastaniu integralSum.

	//człon różniczkujący
	uint32_t Ddivider = 500;		//dzielnik opóźniający
	(*Dcounter)++;		//licznik wywołań funkcji PID
	if (*Dcounter >= Ddivider) {
		*xD = (currentError - *lastError) * Dmultiplier * 20; //-1
		*lastError = currentError;
		*Dcounter = 0;
	}
	//^Człon różniczkujący jest opóźniony, wykonuje się raz na divider wywołań funkcji.
	//potrzebuje do tego licznika(Dcounter), a xD jest zachowywane do następnego wywołania.

	//sumowanie korekt
	float sum = xP + xI + *xD;
	int32_t throttle = (int32_t) round(sum);

	int32_t maxThrottle = 1000;
	if (throttle > maxThrottle)
		throttle = maxThrottle;
	else if (throttle < 0)
		throttle = 0;

	return throttle;
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void) {

	/* USER CODE BEGIN ADC_Init 0 */

	/* USER CODE END ADC_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC_Init 1 */

	/* USER CODE END ADC_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC1;
	hadc.Init.OversamplingMode = DISABLE;
	hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ContinuousConvMode = ENABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.DMAContinuousRequests = ENABLE;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerFrequencyMode = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	if (HAL_ADC_Init(&hadc) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC_Init 2 */

	/* USER CODE END ADC_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 23;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 19999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM21 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM21_Init(void) {

	/* USER CODE BEGIN TIM21_Init 0 */

	/* USER CODE END TIM21_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM21_Init 1 */

	/* USER CODE END TIM21_Init 1 */
	htim21.Instance = TIM21;
	htim21.Init.Prescaler = 74;
	htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim21.Init.Period = 199;
	htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim21) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM21_Init 2 */

	/* USER CODE END TIM21_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, DISP_RS_Pin | DISP_E_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			LED_STATUS_Pin | DISP_D7_Pin | DISP_D5_Pin | DISP_D4_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(DISP_D6_GPIO_Port, DISP_D6_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : DISP_RS_Pin DISP_E_Pin */
	GPIO_InitStruct.Pin = DISP_RS_Pin | DISP_E_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LED_STATUS_Pin DISP_D7_Pin DISP_D5_Pin DISP_D4_Pin */
	GPIO_InitStruct.Pin = LED_STATUS_Pin | DISP_D7_Pin | DISP_D5_Pin
			| DISP_D4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : DISP_D6_Pin */
	GPIO_InitStruct.Pin = DISP_D6_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DISP_D6_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
