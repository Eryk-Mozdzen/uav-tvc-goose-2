#ifndef NVM_H
#define NVM_H

#include <stdint.h>

void nvm_read(const uint32_t address, void *dest, const uint32_t len);
void nvm_write(const uint32_t address, const void *src, const uint32_t len);

#endif
