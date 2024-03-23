#include "stm32f4xx_hal.h"

extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;
extern DMA_HandleTypeDef hdma_i2c3_rx;
extern DMA_HandleTypeDef hdma_i2c3_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

void i2c_recovery(GPIO_TypeDef *sda_port, uint16_t sda_pin, GPIO_TypeDef *scl_port, uint16_t scl_pin) {
	// config I2C SDA and SCL pin as IO pins
	// manualy toggle SCL line to generate clock pulses until 10 consecutive 1 on SDA occure

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = scl_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	HAL_GPIO_Init(scl_port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = sda_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(sda_port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_RESET);

	int ones = 0;
    int max = 100;
	while(ones<=10 && max--) {
		if(HAL_GPIO_ReadPin(sda_port, sda_pin)) {
			ones++;
		} else {
            ones = 0;
        }

		HAL_GPIO_TogglePin(scl_port, scl_pin);
		HAL_Delay(10);
	}
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void HAL_MspInit() {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(hi2c->Instance==I2C1) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        i2c_recovery(GPIOB, GPIO_PIN_7, GPIOB, GPIO_PIN_6);

        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        __HAL_RCC_I2C1_CLK_ENABLE();

        hdma_i2c1_rx.Instance = DMA1_Stream0;
        hdma_i2c1_rx.Init.Channel = DMA_CHANNEL_1;
        hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
        hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_i2c1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_i2c1_rx);

        __HAL_LINKDMA(hi2c,hdmarx,hdma_i2c1_rx);

        hdma_i2c1_tx.Instance = DMA1_Stream1;
        hdma_i2c1_tx.Init.Channel = DMA_CHANNEL_0;
        hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_i2c1_tx.Init.Mode = DMA_NORMAL;
        hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_i2c1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_i2c1_tx);

        __HAL_LINKDMA(hi2c,hdmatx,hdma_i2c1_tx);
    } else if(hi2c->Instance==I2C2) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        i2c_recovery(GPIOB, GPIO_PIN_3, GPIOB, GPIO_PIN_10);

        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_I2C2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        __HAL_RCC_I2C2_CLK_ENABLE();

        hdma_i2c2_rx.Instance = DMA1_Stream3;
        hdma_i2c2_rx.Init.Channel = DMA_CHANNEL_7;
        hdma_i2c2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_i2c2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_i2c2_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_i2c2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_i2c2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_i2c2_rx.Init.Mode = DMA_NORMAL;
        hdma_i2c2_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_i2c2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_i2c2_rx);

        __HAL_LINKDMA(hi2c,hdmarx,hdma_i2c2_rx);

        hdma_i2c2_tx.Instance = DMA1_Stream7;
        hdma_i2c2_tx.Init.Channel = DMA_CHANNEL_7;
        hdma_i2c2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_i2c2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_i2c2_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_i2c2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_i2c2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_i2c2_tx.Init.Mode = DMA_NORMAL;
        hdma_i2c2_tx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_i2c2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_i2c2_tx);

        __HAL_LINKDMA(hi2c,hdmatx,hdma_i2c2_tx);
    } else if(hi2c->Instance==I2C3) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        i2c_recovery(GPIOC, GPIO_PIN_9, GPIOA, GPIO_PIN_8);

        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_8;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        __HAL_RCC_I2C3_CLK_ENABLE();

        hdma_i2c3_rx.Instance = DMA1_Stream2;
        hdma_i2c3_rx.Init.Channel = DMA_CHANNEL_3;
        hdma_i2c3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_i2c3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_i2c3_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_i2c3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_i2c3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_i2c3_rx.Init.Mode = DMA_NORMAL;
        hdma_i2c3_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_i2c3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_i2c3_rx);

        __HAL_LINKDMA(hi2c,hdmarx,hdma_i2c3_rx);

        hdma_i2c3_tx.Instance = DMA1_Stream4;
        hdma_i2c3_tx.Init.Channel = DMA_CHANNEL_3;
        hdma_i2c3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_i2c3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_i2c3_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_i2c3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_i2c3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_i2c3_tx.Init.Mode = DMA_NORMAL;
        hdma_i2c3_tx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_i2c3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_i2c3_tx);

        __HAL_LINKDMA(hi2c,hdmatx,hdma_i2c3_tx);
    }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c) {
    if(hi2c->Instance==I2C1) {
        __HAL_RCC_I2C1_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

        HAL_DMA_DeInit(hi2c->hdmarx);
        HAL_DMA_DeInit(hi2c->hdmatx);
    } else if(hi2c->Instance==I2C2) {
        __HAL_RCC_I2C2_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3);

        HAL_DMA_DeInit(hi2c->hdmarx);
        HAL_DMA_DeInit(hi2c->hdmatx);
    } else if(hi2c->Instance==I2C3) {
        __HAL_RCC_I2C3_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

        HAL_DMA_DeInit(hi2c->hdmarx);
        HAL_DMA_DeInit(hi2c->hdmatx);
    }

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(htim_base->Instance==TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_0;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    } else if(htim_base->Instance==TIM10) {
        __HAL_RCC_TIM10_CLK_ENABLE();

        HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    } else if(htim_base->Instance==TIM11) {
        __HAL_RCC_TIM11_CLK_ENABLE();
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(htim->Instance==TIM11) {
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM11;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base) {
    if(htim_base->Instance==TIM2) {
        __HAL_RCC_TIM2_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);
    } else if(htim_base->Instance==TIM10) {
        __HAL_RCC_TIM10_CLK_DISABLE();

        HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
    } else if(htim_base->Instance==TIM11) {
        __HAL_RCC_TIM11_CLK_DISABLE();
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(huart->Instance==USART2) {
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        hdma_usart2_tx.Instance = DMA1_Stream6;
        hdma_usart2_tx.Init.Channel = DMA_CHANNEL_4;
        hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart2_tx.Init.Mode = DMA_NORMAL;
        hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_usart2_tx);

        __HAL_LINKDMA(huart,hdmatx,hdma_usart2_tx);

        hdma_usart2_rx.Instance = DMA1_Stream5;
        hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
        hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
        hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_usart2_rx);

        __HAL_LINKDMA(huart,hdmarx,hdma_usart2_rx);

        HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    } else if(huart->Instance==USART6) {
        __HAL_RCC_USART6_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        hdma_usart6_rx.Instance = DMA2_Stream1;
        hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
        hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart6_rx.Init.Mode = DMA_CIRCULAR;
        hdma_usart6_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_usart6_rx);

        __HAL_LINKDMA(huart,hdmarx,hdma_usart6_rx);

        hdma_usart6_tx.Instance = DMA2_Stream6;
        hdma_usart6_tx.Init.Channel = DMA_CHANNEL_5;
        hdma_usart6_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart6_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart6_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart6_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart6_tx.Init.Mode = DMA_NORMAL;
        hdma_usart6_tx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_usart6_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_usart6_tx);

        __HAL_LINKDMA(huart,hdmatx,hdma_usart6_tx);

        HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART6_IRQn);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart) {
    if(huart->Instance==USART2) {
        __HAL_RCC_USART2_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

        HAL_DMA_DeInit(huart->hdmatx);
        HAL_DMA_DeInit(huart->hdmarx);

        HAL_NVIC_DisableIRQ(USART2_IRQn);
    } else if(huart->Instance==USART6) {
        __HAL_RCC_USART6_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);

        HAL_DMA_DeInit(huart->hdmarx);
        HAL_DMA_DeInit(huart->hdmatx);

        HAL_NVIC_DisableIRQ(USART6_IRQn);
    }
}
