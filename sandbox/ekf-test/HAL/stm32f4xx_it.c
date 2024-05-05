#include "stm32f4xx_hal.h"

extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;
extern DMA_HandleTypeDef hdma_i2c3_rx;
extern DMA_HandleTypeDef hdma_i2c3_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim10;

void NMI_Handler() {
    while(1) {

    }
}

void HardFault_Handler() {
    while(1) {

    }
}

void MemManage_Handler() {
    while(1) {

    }
}

void BusFault_Handler() {
    while(1) {

    }
}

void UsageFault_Handler() {
    while(1) {

    }
}

void SVC_Handler() {

}

void DebugMon_Handler() {

}

void PendSV_Handler() {

}

void SysTick_Handler() {
    HAL_IncTick();
}

void EXTI0_IRQHandler() {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void EXTI1_IRQHandler() {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

void DMA1_Stream0_IRQHandler() {
    HAL_DMA_IRQHandler(&hdma_i2c1_rx);
}

void DMA1_Stream1_IRQHandler() {
    HAL_DMA_IRQHandler(&hdma_i2c1_tx);
}

void DMA1_Stream2_IRQHandler() {
    HAL_DMA_IRQHandler(&hdma_i2c3_rx);
}

void DMA1_Stream3_IRQHandler() {
    HAL_DMA_IRQHandler(&hdma_i2c2_rx);
}

void DMA1_Stream4_IRQHandler() {
    HAL_DMA_IRQHandler(&hdma_i2c3_tx);
}

void DMA1_Stream5_IRQHandler() {
    HAL_DMA_IRQHandler(&hdma_usart2_rx);
}

void DMA1_Stream6_IRQHandler() {
    HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

void DMA1_Stream7_IRQHandler() {
    HAL_DMA_IRQHandler(&hdma_i2c2_tx);
}

void DMA2_Stream1_IRQHandler() {
    HAL_DMA_IRQHandler(&hdma_usart6_rx);
}

void DMA2_Stream2_IRQHandler() {
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

void DMA2_Stream6_IRQHandler() {
    HAL_DMA_IRQHandler(&hdma_usart6_tx);
}

void DMA2_Stream7_IRQHandler() {
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

void USART1_IRQHandler() {
    HAL_UART_IRQHandler(&huart1);
}

void USART2_IRQHandler() {
    HAL_UART_IRQHandler(&huart2);
}

void USART6_IRQHandler() {
    HAL_UART_IRQHandler(&huart6);
}

void TIM2_IRQHandler() {
    HAL_TIM_IRQHandler(&htim2);
}

void TIM1_UP_TIM10_IRQHandler() {
    HAL_TIM_IRQHandler(&htim10);
}
