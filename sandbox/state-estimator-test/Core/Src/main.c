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
#include "register_map_lsm303dlhc.h"

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
volatile bool lsm303dlhc_drdy = false;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin==L3GD20_INT2_Pin) {
		l3gd20_drdy = true;
	} else if(GPIO_Pin==LSM303DLHC_DRDY_Pin) {
		lsm303dlhc_drdy = true;
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

void LSM303DLHC_A_write(const uint8_t address, const uint8_t value) {
	HAL_I2C_Mem_Write(&hi2c1, LSM303DLHC_A_I2C_ADDRESS, address, 1, (uint8_t *)&value, 1, HAL_MAX_DELAY);
}

void LSM303DLHC_A_read(const uint8_t address, void *data, const uint8_t num) {
	HAL_I2C_Mem_Read(&hi2c1, LSM303DLHC_A_I2C_ADDRESS, address | (1<<7), 1, data, num, HAL_MAX_DELAY);
}

void LSM303DLHC_M_write(const uint8_t address, const uint8_t value) {
	HAL_I2C_Mem_Write(&hi2c1, LSM303DLHC_M_I2C_ADDRESS, address, 1, (uint8_t *)&value, 1, HAL_MAX_DELAY);
}

void LSM303DLHC_M_read(const uint8_t address, void *data, const uint8_t num) {
	HAL_I2C_Mem_Read(&hi2c1, LSM303DLHC_M_I2C_ADDRESS, address, 1, data, num, HAL_MAX_DELAY);
}

#define SENSITIVITY_GYROSCOPE		0.030518044f
#define SENSITIVITY_ACCELEROMETER	1.f/1340.f

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
	  L3GD20_read(L3GD20_WHO_AM_I, &who_i_am, sizeof(who_i_am));

	  if(who_i_am==L3GD20_WHO_AM_I_VALUE) {
		  printf("L3GD20 ok\n");
	  }
  }

  L3GD20_write(L3GD20_CTRL_REG5,
      L3GD20_CTRL_REG5_BOOT
  );

  L3GD20_write(L3GD20_CTRL_REG1,
      L3GD20_CTRL_REG1_DR_ODR_95HZ |
	  L3GD20_CTRL_REG1_BW_CUTOFF_LPF1_93HZ |
	  L3GD20_CTRL_REG1_PD_OPERATING_MODE_NORMAL |
	  L3GD20_CTRL_REG1_ZEN_ENABLE |
	  L3GD20_CTRL_REG1_YEN_ENABLE |
	  L3GD20_CTRL_REG1_XEN_ENABLE
  );

  L3GD20_write(L3GD20_CTRL_REG3,
	  L3GD20_CTRL_REG3_I1_INT1_DISABLE |
	  L3GD20_CTRL_REG3_I1_BOOT_STATUS_AVAILABLE_DISABLE |
	  L3GD20_CTRL_REG3_H_LACTIVE_HIGH |
	  L3GD20_CTRL_REG3_PP_OD_PUSH_PULL |
	  L3GD20_CTRL_REG3_I2_DRDY_ENABLE |
	  L3GD20_CTRL_REG3_I2_WTM_FIFO_WATERMARK_DISABLE |
	  L3GD20_CTRL_REG3_I2_ORUN_FIFO_OVERRUN_DISABLE |
	  L3GD20_CTRL_REG3_I2_EMPTY_FIFO_EMPTY_DISABLE
  );

  L3GD20_write(L3GD20_CTRL_REG4,
	  L3GD20_CTRL_REG4_BDU_ENABLE |
	  L3GD20_CTRL_REG4_BLE_LITTLE_ENDIAN |
	  L3GD20_CTRL_REG4_FS_FULL_SCALE_2000DPS |
	  L3GD20_CTRL_REG4_SIM_4_WIRE_INTERFACE
  );

  L3GD20_write(L3GD20_CTRL_REG5,
      L3GD20_CTRL_REG5_FIFO_EN_FIFO_DISABLE |
	  L3GD20_CTRL_REG5_HPEN_HIGH_PASS_FILTER_DISABLE |
	  L3GD20_CTRL_REG5_INT1_SEL_00 |
	  L3GD20_CTRL_REG5_OUT_SEL_00
  );

  {
	  uint8_t irx[3] = {0};
	  LSM303DLHC_M_read(LSM303DLHC_IRA_REG_M, irx, sizeof(irx));

	  if(irx[0]==LSM303DLHC_IRA_REG_M_VALUE && irx[1]==LSM303DLHC_IRB_REG_M_VALUE && irx[2]==LSM303DLHC_IRC_REG_M_VALUE) {
		  printf("LSM303DLHC ok\n");
	  }
  }

  LSM303DLHC_A_write(LSM303DLHC_CTRL_REG5_A, LSM303DLHC_CTRL_REG5_A_BOOT);

  LSM303DLHC_A_write(LSM303DLHC_CTRL_REG1_A,
	  LSM303DLHC_CTRL_REG1_A_ODR_MODE_NORMAL_LOW_POWER_100HZ |
	  LSM303DLHC_CTRL_REG1_A_LPEN_NORMAL_MODE |
	  LSM303DLHC_CTRL_REG1_A_ZEN_ENABLE |
	  LSM303DLHC_CTRL_REG1_A_YEN_ENABLE |
	  LSM303DLHC_CTRL_REG1_A_XEN_ENABLE
  );

  LSM303DLHC_A_write(LSM303DLHC_CTRL_REG2_A,
  	  LSM303DLHC_CTRL_REG2_A_HPM_NORMAL_MODE |
	  //LSM303DLHC_CTRL_REG2_A_HPCF |
	  LSM303DLHC_CTRL_REG2_A_FDS_INTERNAL_FILTER_BAYPASSED |
	  LSM303DLHC_CTRL_REG2_A_HPCLICK_FILTER_BAPASSED |
	  LSM303DLHC_CTRL_REG2_A_HPIS2_FILTER_BAPASSED |
	  LSM303DLHC_CTRL_REG2_A_HPIS1_FILTER_BAPASSED
  );

  LSM303DLHC_A_write(LSM303DLHC_CTRL_REG3_A,
      LSM303DLHC_CTRL_REG3_A_I1_CLICK_DISABLE |
	  LSM303DLHC_CTRL_REG3_A_I1_AOI1_DISABLE |
	  LSM303DLHC_CTRL_REG3_A_I1_AOI2_DISABLE |
	  LSM303DLHC_CTRL_REG3_A_I1_DRDY1_ENABLE |
	  LSM303DLHC_CTRL_REG3_A_I1_DRDY2_DISABLE |
	  LSM303DLHC_CTRL_REG3_A_I1_WTM_FIFO_WATERMARK_DISABLE |
	  LSM303DLHC_CTRL_REG3_A_I1_OVERRUN_FIFO_OVERRUN_DISABLE
  );

  LSM303DLHC_A_write(LSM303DLHC_CTRL_REG4_A,
      LSM303DLHC_CTRL_REG4_A_BDU_ENABLE |
	  LSM303DLHC_CTRL_REG4_A_BLE_LITTLE_ENDIAN |
	  LSM303DLHC_CTRL_REG4_A_FS_FULL_SCALE_16G |
	  LSM303DLHC_CTRL_REG4_A_HR_HIGH_RESOLUTION_ENABLE |
	  LSM303DLHC_CTRL_REG4_A_SIM_4_WIRE_INTERFACE
  );

  LSM303DLHC_A_write(LSM303DLHC_CTRL_REG5_A,
      LSM303DLHC_CTRL_REG5_A_FIFO_EN_FIFO_DISABLE |
	  LSM303DLHC_CTRL_REG5_A_LIR_INT1_NOT_LATCHED |
	  LSM303DLHC_CTRL_REG5_A_D4D_INT1_DISABLE |
	  LSM303DLHC_CTRL_REG5_A_LIR_INT2_NOT_LATCHED |
	  LSM303DLHC_CTRL_REG5_A_D4D_INT2_DISABLE
  );

  LSM303DLHC_A_write(LSM303DLHC_CTRL_REG6_A,
      LSM303DLHC_CTRL_REG6_A_I2_CLICKEN_DISABLE |
	  LSM303DLHC_CTRL_REG6_A_I2_INT1_INTERRUPT_1_ON_PAD2_DISABLE |
	  LSM303DLHC_CTRL_REG6_A_I2_INT2_INTERRUPT_2_ON_PAD2_DISABLE |
	  LSM303DLHC_CTRL_REG6_A_P2_ACT_ACTIVE_FUNCTION_STATUS_ON_PAD2_DISABLE |
	  LSM303DLHC_CTRL_REG6_A_H_LACTIVE_ACTIVE_HIGH
  );

  LSM303DLHC_M_write(LSM303DLHC_CRA_REG_M,
	  LSM303DLHC_CRA_REG_M_TEMP_EN_TEMPERATURE_SENSOR_DISABLE |
	  LSM303DLHC_CRA_REG_M_DO_MINIMUM_DATA_OUTPUT_RATE_75HZ
  );

  LSM303DLHC_M_write(LSM303DLHC_CRB_REG_M,
      LSM303DLHC_CRB_REG_M_GN_SENSOR_INPUT_RAGE_8_1GAUSS
  );

  LSM303DLHC_M_write(LSM303DLHC_MR_REG_M,
      LSM303DLHC_MR_REG_M_MD_OPERATING_MODE_CONTINUOUS_CONVERSION
  );

  int16_t raw[3] = {0};
  L3GD20_read(L3GD20_OUT_X_L, raw, sizeof(raw));

  while(1) {

	  if(l3gd20_drdy) {
		  l3gd20_drdy = false;

		  L3GD20_read(L3GD20_OUT_X_L, raw, sizeof(raw));

		  //x = raw[0]*SENSITIVITY_GYROSCOPE;
		  //y = raw[1]*SENSITIVITY_GYROSCOPE;
		  //z = raw[2]*SENSITIVITY_GYROSCOPE;
	  }

	  if(lsm303dlhc_drdy) {
		  lsm303dlhc_drdy = false;

		  LSM303DLHC_A_read(LSM303DLHC_OUT_X_L_A, raw, sizeof(raw));
		  //LSM303DLHC_M_read(LSM303DLHC_OUT_X_H_M, raw, sizeof(raw));

		  x = raw[0]*SENSITIVITY_ACCELEROMETER;
		  y = raw[1]*SENSITIVITY_ACCELEROMETER;
		  z = raw[2]*SENSITIVITY_ACCELEROMETER;
	  }

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

  /*Configure GPIO pins : LSM303DLHC_DRDY_Pin LSM303DLHC_INT1_Pin LSM303DLHC_INT2_Pin L3GD20_INT1_Pin
                           L3GD20_INT2_Pin */
  GPIO_InitStruct.Pin = LSM303DLHC_DRDY_Pin|LSM303DLHC_INT1_Pin|LSM303DLHC_INT2_Pin|L3GD20_INT1_Pin
                          |L3GD20_INT2_Pin;
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

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
