#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "stm32f4xx_hal.h"

#include "nmea.h"
#include "ublox_settings.h"

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

uint8_t data[32];
volatile uint8_t ready = 0;

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
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
	if(huart==&huart1) {
		ready = 1;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart==&huart1) {
		ready = 2;
	}
}

int main() {

	HAL_Init();

	clock();
	periph();

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

	HAL_UART_Receive_DMA(&huart1, data, sizeof(data));

	NMEA_Message_t message;

	while(1) {
		if(ready!=0) {
			for(size_t s=0; s<16; s++) {
				const char c = data[(ready==1) ? s : s+16];

				if(NMEA_Consume(&message, c)) {
					for(uint8_t i=0; i<message.argc; i++) {
						char buffer[32];
						sprintf(buffer, "%12s", message.argv[i]);
						HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
					}

					HAL_UART_Transmit(&huart2, (uint8_t *)"\n\r", 2, HAL_MAX_DELAY);
				}
			}

			ready = 0;
		}
	}
}
