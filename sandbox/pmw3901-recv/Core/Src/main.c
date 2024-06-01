/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//#include <string.h>

#include "pmw3901.h"

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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void pmw3901_write(const uint8_t address, const uint8_t value) {
	uint8_t tx[] = {address | 0x80, value};

	HAL_GPIO_WritePin(PMW3901_CS_GPIO_Port, PMW3901_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi1, tx, sizeof(tx), HAL_MAX_DELAY);
	HAL_Delay(1);
	HAL_GPIO_WritePin(PMW3901_CS_GPIO_Port, PMW3901_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
}

void pmw3901_read(const uint8_t address, uint8_t *buffer, const uint8_t len) {
	uint8_t tx[32] = {0};
	uint8_t rx[32] = {0};

	for(uint8_t i=0; i<len; i++) {
		tx[2*i] = (address + i) & ~0x80;
	}

	HAL_GPIO_WritePin(PMW3901_CS_GPIO_Port, PMW3901_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2*len, HAL_MAX_DELAY);
	HAL_Delay(1);
	HAL_GPIO_WritePin(PMW3901_CS_GPIO_Port, PMW3901_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(1);

	for(uint8_t i=0; i<len; i++) {
		buffer[i] = rx[2*i + 1];
	}
}

void pmw3901_init() {
	pmw3901_write(0x3A, 0x5A);

	HAL_Delay(5);

	pmw3901_write(0x7F, 0x00);
	pmw3901_write(0x61, 0xAD);
	pmw3901_write(0x7F, 0x03);
	pmw3901_write(0x40, 0x00);
	pmw3901_write(0x7F, 0x05);
	pmw3901_write(0x41, 0xB3);
	pmw3901_write(0x43, 0xF1);
	pmw3901_write(0x45, 0x14);
	pmw3901_write(0x5B, 0x32);
	pmw3901_write(0x5F, 0x34);
	pmw3901_write(0x7B, 0x08);
	pmw3901_write(0x7F, 0x06);
	pmw3901_write(0x44, 0x1B);
	pmw3901_write(0x40, 0xBF);
	pmw3901_write(0x4E, 0x3F);
	pmw3901_write(0x7F, 0x08);
	pmw3901_write(0x65, 0x20);
	pmw3901_write(0x6A, 0x18);
	pmw3901_write(0x7F, 0x09);
	pmw3901_write(0x4F, 0xAF);
	pmw3901_write(0x5F, 0x40);
	pmw3901_write(0x48, 0x80);
	pmw3901_write(0x49, 0x80);
	pmw3901_write(0x57, 0x77);
	pmw3901_write(0x60, 0x78);
	pmw3901_write(0x61, 0x78);
	pmw3901_write(0x62, 0x08);
	pmw3901_write(0x63, 0x50);
	pmw3901_write(0x7F, 0x0A);
	pmw3901_write(0x45, 0x60);
	pmw3901_write(0x7F, 0x00);
	pmw3901_write(0x4D, 0x11);
	pmw3901_write(0x55, 0x80);
	pmw3901_write(0x74, 0x1F);
	pmw3901_write(0x75, 0x1F);
	pmw3901_write(0x4A, 0x78);
	pmw3901_write(0x4B, 0x78);
	pmw3901_write(0x44, 0x08);
	pmw3901_write(0x45, 0x50);
	pmw3901_write(0x64, 0xFF);
	pmw3901_write(0x65, 0x1F);
	pmw3901_write(0x7F, 0x14);
	pmw3901_write(0x65, 0x60);
	pmw3901_write(0x66, 0x08);
	pmw3901_write(0x63, 0x78);
	pmw3901_write(0x7F, 0x15);
	pmw3901_write(0x48, 0x58);
	pmw3901_write(0x7F, 0x07);
	pmw3901_write(0x41, 0x0D);
	pmw3901_write(0x43, 0x14);
	pmw3901_write(0x4B, 0x0E);
	pmw3901_write(0x45, 0x0F);
	pmw3901_write(0x44, 0x42);
	pmw3901_write(0x4C, 0x80);
	pmw3901_write(0x7F, 0x10);
	pmw3901_write(0x5B, 0x02);
	pmw3901_write(0x7F, 0x07);
	pmw3901_write(0x40, 0x41);
	pmw3901_write(0x70, 0x00);

	HAL_Delay(100);

	pmw3901_write(0x32, 0x44);
	pmw3901_write(0x7F, 0x07);
	pmw3901_write(0x40, 0x40);
	pmw3901_write(0x7F, 0x06);
	pmw3901_write(0x62, 0xf0);
	pmw3901_write(0x63, 0x00);
	pmw3901_write(0x7F, 0x0D);
	pmw3901_write(0x48, 0xC0);
	pmw3901_write(0x6F, 0xd5);
	pmw3901_write(0x7F, 0x00);
	pmw3901_write(0x5B, 0xa0);
	pmw3901_write(0x4E, 0xA8);
	pmw3901_write(0x5A, 0x50);
	pmw3901_write(0x40, 0x80);
}

uint8_t pmw3901_check() {
	uint8_t id = 0;
	uint8_t inv = 0;

	pmw3901_read(PMW3901_REG_PRODUCT_ID, &id, 1);
	pmw3901_read(PMW3901_REG_INVERSE_PRODUCT_ID, &inv, 1);

	return ((id==PMW3901_PRODUCT_ID) && (inv==PMW3901_PRODUCT_ID_INVERSE));
}

float vel_x = 0;
float vel_y = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin==PMW3901_INT_Pin) {
		HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	pmw3901_init();

	uint32_t last_read = 0;
	uint32_t last_check = 0;

	while(1) {
		const uint32_t time = HAL_GetTick();

		if((time - last_read)>20) {
			const float h = 0.1f;
			const float dt = (time - last_read)*0.001f;

			last_read = time;

			uint8_t motion[5] = {0};
			pmw3901_read(PMW3901_REG_MOTION, motion, sizeof(motion));
			const int16_t delta_x = (((int16_t)motion[2])<<8) | motion[1];
			const int16_t delta_y = (((int16_t)motion[4])<<8) | motion[3];

			vel_x = h*delta_x/(dt*PMW3901_FOCAL_LENGTH);
			vel_y = h*delta_y/(dt*PMW3901_FOCAL_LENGTH);
		}

		if((time - last_check)>1000) {
			last_check = time;

			const uint8_t state = pmw3901_check();

			HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, state);
			HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, !state);
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PMW3901_CS_GPIO_Port, PMW3901_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PMW3901_CS_Pin */
  GPIO_InitStruct.Pin = PMW3901_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(PMW3901_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PMW3901_INT_Pin */
  GPIO_InitStruct.Pin = PMW3901_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PMW3901_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_ORANGE_Pin LED_RED_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
