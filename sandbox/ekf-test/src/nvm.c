#include <stdint.h>
#include <string.h>

#include "stm32f4xx_hal.h"

#define SECTOR_BEGIN 0x08060000

void nvm_read(const uint32_t address, void *dest, const uint32_t len) {
	memcpy(dest, (void *)(SECTOR_BEGIN + address), len);
}

void nvm_write(const uint32_t address, const void *src, const uint32_t len) {
	HAL_FLASH_Unlock();

	uint32_t error;

	FLASH_EraseInitTypeDef erase;
	erase.TypeErase = FLASH_TYPEERASE_SECTORS;
	erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	erase.Sector = FLASH_SECTOR_7;
	erase.NbSectors = 1;
	HAL_FLASHEx_Erase(&erase, &error);

	for(uint32_t i=0; i<len; i++) {
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, SECTOR_BEGIN + address + i, ((uint8_t *)src)[i]);
	}

	HAL_FLASH_Lock();
}
