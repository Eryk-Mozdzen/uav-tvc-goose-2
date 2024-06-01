#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>

#include "stm32f4xx_hal.h"

#include "nmea.h"
#include "ublox_settings.h"
#include "estimator.h"

#define DEG2RAD 0.01745329251f

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
TIM_HandleTypeDef htim11;

uint8_t uart_data[32];
volatile uint8_t uart_ready = 0;
volatile bool ekf_predict = false;

void print(const char *format, ...) {
	va_list args;
    va_start(args, format);

	char buffer[128] = {0};
	const size_t len = vsnprintf(buffer, sizeof(buffer), format, args);

	HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, HAL_MAX_DELAY);
}

void clock() {
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
}

void periph() {
	__HAL_RCC_GPIOA_CLK_ENABLE();
  	__HAL_RCC_GPIOB_CLK_ENABLE();

	__HAL_RCC_DMA2_CLK_ENABLE();
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart1);

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart2);

	TIM_OC_InitTypeDef sConfigOC = {0};
	htim11.Instance = TIM11;
	htim11.Init.Prescaler = 999;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 1000-1;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim11);
	HAL_TIM_OC_Init(&htim11);
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_OC_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
	if(huart==&huart1) {
		uart_ready = 1;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart==&huart1) {
		uart_ready = 2;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim==&htim11) {
		ekf_predict = true;
	}
}

int main() {

	HAL_Init();

	clock();
	periph();

	HAL_TIM_Base_Start_IT(&htim11);

	const uint8_t config[] = {
		UBLOX_SETTINGS_OUTPUT_RATE_10HZ,
		UBLOX_SETTINGS_DISABLE_GPDTM,
		UBLOX_SETTINGS_DISABLE_GPDTM,
		UBLOX_SETTINGS_DISABLE_GPGBS,
		UBLOX_SETTINGS_DISABLE_GPGGA,
		UBLOX_SETTINGS_DISABLE_GPGLL,
		UBLOX_SETTINGS_DISABLE_GPGRS,
		UBLOX_SETTINGS_DISABLE_GPGSA,
		UBLOX_SETTINGS_DISABLE_GPGST,
		UBLOX_SETTINGS_DISABLE_GPGSV,
		UBLOX_SETTINGS_ENABLE_GPRMC,
		UBLOX_SETTINGS_DISABLE_GPVTG,
		UBLOX_SETTINGS_DISABLE_GPZDA,
		UBLOX_SETTINGS_OTHER_SAVE
	};

	HAL_Delay(500);
	HAL_UART_Transmit(&huart1, config, sizeof(config), HAL_MAX_DELAY);
	HAL_Delay(500);

	HAL_UART_Receive_DMA(&huart1, uart_data, sizeof(uart_data));

	NMEA_Message_t message;

	while(1) {
		if(uart_ready!=0) {
			for(size_t s=0; s<16; s++) {
				const char c = uart_data[(uart_ready==1) ? s : s+16];

				if(NMEA_Consume(&message, c)) {
					if(message.argv[2][0]=='A') {
						float latitude = 0;
						float longitude = 0;

						{
							const float minutes = atof(&message.argv[3][2]);
							message.argv[3][2] = '\0';
							const int degree = atoi(message.argv[3]);
							latitude = degree + minutes/60.f;
						}

						{
							const float minutes = atof(&message.argv[5][3]);
							message.argv[5][3] = '\0';
							const int degree = atoi(message.argv[5]);
							longitude = degree + minutes/60.f;
						}

						Estimator_FeedGPS(DEG2RAD*latitude, DEG2RAD*longitude);
					}

					Estimator_State_t state;
					Estimator_GetState(&state);
					print("position = [%+12.3f, %+12.3f] m      ", (double)state.position[0], (double)state.position[1]);
					print("velocity = [%+12.3f, %+12.3f] m/s    ", (double)state.velocity[0], (double)state.velocity[1]);
					print("%c\n\r", message.argv[2][0]);
				}
			}

			uart_ready = 0;
		}

		if(ekf_predict) {
			ekf_predict = false;
			Estimator_Predict();
		}
	}
}
