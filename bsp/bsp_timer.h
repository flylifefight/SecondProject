#ifndef BSP_TIMER_H
#define BSP_TIMER_H

#include <stdint.h>

void timer2_config(void);
void timer2_frequency_set(uint32_t frequency);
void timer2_duty_set(uint32_t duty);

#endif /* BSP_TIMER_H */