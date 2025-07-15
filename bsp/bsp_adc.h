#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>

void adc_config(void);
uint16_t get_avg_temp_adc(void);

#endif /* BSP_ADC_H */