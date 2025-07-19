#ifndef BSP_EXTI_H
#define BSP_EXTI_H

#include <stdint.h>


void VI4302_GPIO0_INT_Init();
void VI4302_SPI_INT_Init();


extern volatile uint8_t gpio0_int_cnt;
extern volatile uint8_t EXTI_FLAG;

#endif /* BSP_EXTI_H */