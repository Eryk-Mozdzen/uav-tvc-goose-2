#include "stm32f4xx_hal.h"

void NMI_Handler() {

	while(1);
}

void HardFault_Handler() {

	while(1);
}

void MemManage_Handler() {

	while(1);
}

void BusFault_Handler() {

	while(1);
}

void UsageFault_Handler() {

	while(1);
}

void SysTick_Handler() {
	HAL_IncTick();
}
