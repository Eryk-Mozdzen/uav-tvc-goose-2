#include "stm32f4xx_hal.h"

extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;

void USART1_IRQHandler() {
    HAL_UART_IRQHandler(&huart1);
}

void DMA2_Stream2_IRQHandler() {
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}
