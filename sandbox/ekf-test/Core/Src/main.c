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
#include <stdlib.h>
#include <string.h>

#include "protocol.h"
#include "protocol_data.h"

#include "mpu6050_regs.h"
//#include "hmc5883l_regs.h"
#include "qmc5883l_regs.h"
#include "bmp280_regs.h"
#include "bmp280_compensate.h"

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
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*void i2c2_slave_recovery() {
	// config I2C SDA and SCL pin as IO pins
	// manualy toggle SCL line to generate clock pulses until 10 consecutive 1 on SDA occure

	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

	int ones = 0;
	while(ones<=10) {
		if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)) {
			ones = 0;
		}

		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
		HAL_Delay(10);

		ones++;
	}
}*/

void bmp280_write(uint8_t address, uint8_t value) {
	HAL_I2C_Mem_Write(&hi2c2, BMP280_ADDR<<1, address, 1, &value, 1, 100);
}

void bmp280_init() {
	bmp280_write(BMP280_REG_RESET,
		BMP280_RESET_VALUE
	);

	bmp280_write(BMP280_REG_CTRL_MEAS,
		BMP280_CTRL_TEMP_OVERSAMPLING_2 |
		BMP280_CTRL_PRESS_OVERSAMPLING_16 |
		BMP280_CTRL_MODE_NORMAL
	);

	bmp280_write(BMP280_REG_CONFIG,
		BMP280_CONFIG_STANDBY_0_5MS |
		BMP280_CONFIG_FILTER_X16 |
		BMP280_CONFIG_SPI_3WIRE_DISABLE
	);

	uint8_t buffer[24] = {0};

	HAL_I2C_Mem_Read(&hi2c2, BMP280_ADDR<<1, BMP280_REG_CALIB00, 1, buffer, sizeof(buffer), 100);

	dig_T1 = (((uint16_t)buffer[1])<<8) | buffer[0];
	dig_T2 = (((int16_t)buffer[3])<<8) | buffer[2];
	dig_T3 = (((int16_t)buffer[5])<<8) | buffer[4];
	dig_P1 = (((uint16_t)buffer[7])<<8) | buffer[6];
	dig_P2 = (((int16_t)buffer[9])<<8) | buffer[8];
	dig_P3 = (((int16_t)buffer[11])<<8) | buffer[10];
	dig_P4 = (((int16_t)buffer[13])<<8) | buffer[12];
	dig_P5 = (((int16_t)buffer[15])<<8) | buffer[14];
	dig_P6 = (((int16_t)buffer[17])<<8) | buffer[16];
	dig_P7 = (((int16_t)buffer[19])<<8) | buffer[18];
	dig_P8 = (((int16_t)buffer[21])<<8) | buffer[20];
	dig_P9 = (((int16_t)buffer[23])<<8) | buffer[22];
}

void bmp280_read(float *temperature, float *pressure) {
	uint8_t buffer[6] = {0};

	HAL_I2C_Mem_Read(&hi2c2, BMP280_ADDR<<1, BMP280_REG_PRESS_MSB, 1, buffer, sizeof(buffer), 100);

	const int32_t raw_temperature  = (((int32_t)buffer[3])<<12) | (((int32_t)buffer[4])<<4) | (((int32_t)buffer[5])>>4);
	const int32_t raw_pressure     = (((int32_t)buffer[0])<<12) | (((int32_t)buffer[1])<<4) | (((int32_t)buffer[2])>>4);

	*temperature = bmp280_compensate_T_int32(raw_temperature)/100.f;	// *C
	*pressure    = bmp280_compensate_P_int64(raw_pressure)/256.f;		// Pa
}

/*void hmc5883l_write(uint8_t address, uint8_t value) {
	HAL_I2C_Mem_Write(&hi2c2, HMC5883L_ADDR<<1, address, 1, &value, 1, 100);
}

void hmc5883l_init() {
	hmc5883l_write(HMC5883L_REG_CONFIG_A,
		HMC5883L_CONFIG_A_MEAS_NORMAL |
		HMC5883L_CONFIG_A_RATE_75 |
		HMC5883L_CONFIG_A_SAMPLES_8
	);

	hmc5883l_write(HMC5883L_REG_CONFIG_B,
		HMC5883L_CONFIG_B_RANGE_1_3GA
	);

	hmc5883l_write(HMC5883L_REG_MODE,
		HMC5883L_MODE_CONTINOUS
	);
}

void hmc5883l_read(float *mag) {
	uint8_t buffer[6] = {0};

	HAL_I2C_Mem_Read(&hi2c2, HMC5883L_ADDR<<1, HMC5883L_REG_DATA_X_MSB, 1, buffer, sizeof(buffer), 100);

	const int16_t raw_x = (((int16_t)buffer[0])<<8) | buffer[1];
	const int16_t raw_y = (((int16_t)buffer[4])<<8) | buffer[5];
	const int16_t raw_z = (((int16_t)buffer[2])<<8) | buffer[3];

	const float gain = 1090.f;

	mag[0] = raw_x/gain;
	mag[1] = raw_y/gain;
	mag[2] = raw_z/gain;
}*/

void qmc5883l_write(uint8_t address, uint8_t value) {
	HAL_I2C_Mem_Write(&hi2c2, QMC5883L_ADDR<<1, address, 1, &value, 1, 100);
}

void qmc5883l_init() {
	qmc5883l_write(QMC5883L_REG_CONTROL_2,
		QMC5883L_CONFIG_2_SOFT_RST
	);

	qmc5883l_write(QMC5883L_REG_CONTROL_1,
		QMC5883L_CONFIG_1_MODE_CONTINOUS |
		QMC5883L_CONFIG_1_ODR_200HZ |
		QMC5883L_CONFIG_1_OSR_512 |
		QMC5883L_CONFIG_1_RNG_2G
	);

	qmc5883l_write(QMC5883L_REG_CONTROL_2,
		QMC5883L_CONFIG_2_INT_ENB_ENABLE
	);

	qmc5883l_write(QMC5883L_REG_SET_RESET,
		QMC5883L_SET_RESET_RECOMMENDED
	);
}

void qmc5883l_read(float *mag) {
	uint8_t buffer[6] = {0};

	HAL_I2C_Mem_Read(&hi2c2, QMC5883L_ADDR<<1, QMC5883L_REG_DATA_OUTPUT_X_LSB, 1, buffer, sizeof(buffer), 100);

	const int16_t raw_x = (((int16_t)buffer[1])<<8) | buffer[0];
	const int16_t raw_y = (((int16_t)buffer[3])<<8) | buffer[2];
	const int16_t raw_z = (((int16_t)buffer[5])<<8) | buffer[4];

	const float gain = 2.f/(1<<15);

	mag[0] = raw_x*gain;
	mag[1] = raw_y*gain;
	mag[2] = raw_z*gain;
}

protocol_readings_t readings = {0};

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
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 10);

	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);

	/*for(uint8_t i=0; i<127; i++) {
		uint8_t reg;
		if(HAL_I2C_Mem_Read(&hi2c2, i<<1, 0, 1, &reg, 1, 100)==HAL_OK) {
			reg++;
		}
	}*/

	bmp280_init();
	//hmc5883l_init();
	qmc5883l_init();

	uint8_t buffer[1024];

	protocol_message_t message;

    while(1) {
		//uint8_t byte;
		//HAL_UART_Receive(&huart6, &byte, 1, HAL_MAX_DELAY);

    	//message.size = sprintf(message.payload, "witam: %lu", HAL_GetTick());

    	float temperature;
    	bmp280_read(&temperature, &readings.barometer);
    	readings.valid.barometer = 1;

    	//hmc5883l_read(readings.magnetometer);
    	qmc5883l_read(readings.magnetometer);
    	readings.valid.magnetometer = 1;

    	readings.rangefinder = 0.00017015f*HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
    	readings.valid.rangefinder = 1;

    	message.id = PROTOCOL_ID_READINGS;
    	message.payload = &readings;
    	message.size = sizeof(readings);
	  	const uint16_t size = protocol_encode(buffer, &message);

	  	/*char xd[256];
    	for(uint16_t i=0; i<size; i++) {
    		sprintf(xd + 3*i, "%02X ", buffer[i]);
    	}
    	sprintf(xd+3*size, "\n\r");
    	HAL_UART_Transmit(&huart2, xd, strlen(xd), HAL_MAX_DELAY);*/

		HAL_UART_Transmit(&huart2, buffer, size, HAL_MAX_DELAY);

		HAL_Delay(20);

		//const uint32_t time = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
		//distance = 0.00017015f*time;

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 100-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
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
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 100-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 20000-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
