#include "stm32f4xx_hal.h"

extern DMA_HandleTypeDef hdma_usart1_rx;

void HAL_MspInit() {
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if(huart->Instance==USART1) {
		__HAL_RCC_USART1_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();

		GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		hdma_usart1_rx.Instance = DMA2_Stream2;
		hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
		hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
		hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_usart1_rx);

		__HAL_LINKDMA(huart,hdmarx,hdma_usart1_rx);

		HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
	} else if(huart->Instance==USART2) {
		__HAL_RCC_USART2_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart) {
	if(huart->Instance==USART1) {
		__HAL_RCC_USART1_CLK_DISABLE();

		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3|GPIO_PIN_6);
		HAL_DMA_DeInit(huart->hdmarx);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
	} else if(huart->Instance==USART2) {
		__HAL_RCC_USART2_CLK_DISABLE();

		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);
	}
}
