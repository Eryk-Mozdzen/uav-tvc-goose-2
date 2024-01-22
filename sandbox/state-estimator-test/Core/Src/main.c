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

#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "register_map_l3GD20.h"

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

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile bool l3gd20_drdy = false;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin==L3GD20_INT2_Pin) {
		l3gd20_drdy = true;
	}
}

int __io_putchar(int ch) {
	ITM_SendChar(ch);
	return(ch);
}

void L3GD20_write(const uint8_t address, const uint8_t value) {
	uint8_t tx[2];
	tx[0] = address & 0x3F;
	tx[1] = value;

	HAL_GPIO_WritePin(L3GD20_CS_GPIO_Port, L3GD20_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi1, tx, sizeof(tx), HAL_MAX_DELAY);
	HAL_GPIO_WritePin(L3GD20_CS_GPIO_Port, L3GD20_CS_Pin, 1);
}

void L3GD20_read(const uint8_t address, void *rx, const uint8_t num) {
	const uint8_t tx = address | 0x80 | ((num>1) ? 0x40 : 0x00);

	HAL_GPIO_WritePin(L3GD20_CS_GPIO_Port, L3GD20_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&tx, sizeof(tx), HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, rx, num, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(L3GD20_CS_GPIO_Port, L3GD20_CS_Pin, 1);
}

#define L3GD20_SENSITIVITY 0.030518044f

float x = 0;
float y = 0;
float z = 0;

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_Delay(250);

  {
	  uint8_t who_i_am = 0;
	  L3GD20_read(L3GD20_REG_WHO_AM_I, &who_i_am, sizeof(who_i_am));

	  if(who_i_am==L3GD20_WHO_AM_I) {
		  printf("L3GD20 ok\n");
	  }
  }

  L3GD20_write(L3GD20_REG_CTRL_REG5,
      L3GD20_CTRL_REG5_BOOT
  );

  L3GD20_write(L3GD20_REG_CTRL_REG1,
      L3GD20_CTRL_REG1_DR_ODR_95HZ |
	  L3GD20_CTRL_REG1_BW_CUTOFF_LPF1_93HZ |
	  L3GD20_CTRL_REG1_PD_OPERATING_MODE_NORMAL |
	  L3GD20_CTRL_REG1_ZEN_ENABLE |
	  L3GD20_CTRL_REG1_YEN_ENABLE |
	  L3GD20_CTRL_REG1_XEN_ENABLE
  );

  L3GD20_write(L3GD20_REG_CTRL_REG3,
	  L3GD20_CTRL_REG3_I1_INT1_DISABLE |
	  L3GD20_CTRL_REG3_I1_BOOT_STATUS_AVAILABLE_DISABLE |
	  L3GD20_CTRL_REG3_H_LACTIVE_HIGH |
	  L3GD20_CTRL_REG3_PP_OD_PUSH_PULL |
	  L3GD20_CTRL_REG3_I2_DRDY_ENABLE |
	  L3GD20_CTRL_REG3_I2_WTM_FIFO_WATERMARK_DISABLE |
	  L3GD20_CTRL_REG3_I2_ORUN_FIFO_OVERRUN_DISABLE |
	  L3GD20_CTRL_REG3_I2_EMPTY_FIFO_EMPTY_DISABLE
  );

  L3GD20_write(L3GD20_REG_CTRL_REG4,
	  L3GD20_CTRL_REG4_BDU_ENABLE |
	  L3GD20_CTRL_REG4_BLE_LITTLE_ENDIAN |
	  L3GD20_CTRL_REG4_FS_FULL_SCALE_2000DPS |
	  L3GD20_CTRL_REG4_SIM_4_WIRE_INTERFACE
  );

  L3GD20_write(L3GD20_REG_CTRL_REG5,
      L3GD20_CTRL_REG5_FIFO_EN_FIFO_DISABLE |
	  L3GD20_CTRL_REG5_HPEN_HIGH_PASS_FILTER_DISABLE |
	  L3GD20_CTRL_REG5_INT1_SEL_00 |
	  L3GD20_CTRL_REG5_OUT_SEL_00
  );

  while(1) {

	  if(l3gd20_drdy) {
		  l3gd20_drdy = false;

		  int16_t raw[3] = {0};

		  L3GD20_read(L3GD20_REG_OUT_X_L, raw, sizeof(raw));

		  x = raw[0]*L3GD20_SENSITIVITY;
		  y = raw[1]*L3GD20_SENSITIVITY;
		  z = raw[2]*L3GD20_SENSITIVITY;
	  }

	  //printf("%+10.2f %+10.2f %+10.2f\n", x, y, z);

	  HAL_Delay(10);

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(L3GD20_CS_GPIO_Port, L3GD20_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LSM303DLHC_DRDY_Pin L3GD20_INT1_Pin L3GD20_INT2_Pin */
  GPIO_InitStruct.Pin = LSM303DLHC_DRDY_Pin|L3GD20_INT1_Pin|L3GD20_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : L3GD20_CS_Pin */
  GPIO_InitStruct.Pin = L3GD20_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L3GD20_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

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
