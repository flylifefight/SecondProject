#ifndef BSP_USART_H
#define BSP_USART_H

#include <stdint.h>

uint32_t usart0_receive(uint32_t* array_address);
uint8_t usart0_transmit(uint8_t* data, uint16_t num);
void usart0_config(void);

#endif /* BSP_USART_H */