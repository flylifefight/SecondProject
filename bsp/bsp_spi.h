#ifndef BSP_SPI_H
#define BSP_SPI_H

#include <stdint.h>

void spi1_config(void);
uint8_t spi1_transmit_and_receive(uint8_t* transmit_data, int8_t* receive_data, uint16_t num);
uint8_t spi1_receive(uint8_t* receive_data, uint16_t num);
uint8_t spi1_transmit(uint8_t* transmit_data, uint16_t num);
uint8_t is_spi1_transmit_complete();

extern volatile uint8_t SPI_Send_Complete;
extern volatile uint8_t SPI_Receive_Complete;
extern volatile uint8_t spi_int_cnt;

#endif /* BSP_SPI_H */