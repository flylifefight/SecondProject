#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include <stdint.h>

uint8_t fmc_read(uint32_t address, uint8_t* data, uint32_t length);
uint8_t fmc_write(uint32_t address, uint8_t* data, uint32_t length);

#endif /* BSP_FLASH_H */