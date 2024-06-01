#include <cstdio>
#include <cstring>

#include "stm32f4xx_hal.h"

#include "matrix.h"
#include <Eigen/Dense>

TIM_HandleTypeDef htim11;
UART_HandleTypeDef huart2;

volatile bool transmit = false;

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
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	htim11.Instance = TIM11;
	htim11.Init.Prescaler = 9999;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 9999;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim11);
	HAL_TIM_Base_Start_IT(&htim11);

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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	(void)htim;

	transmit = true;
}

int main() {

	HAL_Init();

	clock();
	periph();

	char buffer[16] = {0};
	uint32_t counter = 0;

	//Matrix<3, 3> mat = {1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f};
	//Matrix<3, 1> vec = {123.f, 234.f, 345.f};
	//Matrix<3, 1> result;
	//Eigen::Matrix<float, 3, 3> mat {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
	//Eigen::Vector<float, 3> vec {123, 234, 345};
	//Eigen::Vector<float, 3> result;

	//Matrix<12, 12> mat;
	//Matrix<12, 1> vec;
	//Matrix<12, 1> result;
	//Eigen::Matrix<float, 12, 12> mat;
	//Eigen::Vector<float, 12> vec;
	//Eigen::Vector<float, 12> result;
	//for(int i=0; i<12; i++) {
	//	for(int j=0; j<12; j++) {
	//		mat(i, j) = 12.f*i + j - ((float)i)/(j + 0.1f);
	//	}
	//	vec(i, 0) = -21.37f*i - 2/(i + 0.1f);
	//}

	//Matrix<24, 24> mat;
	//Matrix<24, 1> vec;
	//Matrix<24, 1> result;
	Eigen::Matrix<float, 24, 24> mat;
	Eigen::Vector<float, 24> vec;
	Eigen::Vector<float, 24> result;
	for(int i=0; i<24; i++) {
		for(int j=0; j<24; j++) {
			mat(i, j) = 12.f*i + j - ((float)i)/(j + 0.1f);
		}
		vec(i, 0) = -21.37f*i - 2/(i + 0.1f);
	}

	while(true) {
		if(transmit) {
			snprintf(buffer, sizeof(buffer), "%lu %d\n\r", counter, static_cast<int>(result(0, 0)));
			HAL_UART_Transmit(&huart2, reinterpret_cast<const uint8_t *>(buffer), strlen(buffer), HAL_MAX_DELAY);

			transmit = false;
			counter = 0;
		}

		mat(0, 2) +=0.00001f;
		vec(0, 0) +=0.00001f;

		//result = mat*vec;
		result = mat.transpose()*vec + (mat*mat).inverse()*vec;

		counter++;
	}
}
