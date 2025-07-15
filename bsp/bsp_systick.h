#ifndef BSP_SYSTICK_H
#define BSP_SYSTICK_H

#include <stdint.h>

/* configure systick */
void systick_config(void);
/* delay a time in milliseconds */
void delay_1ms(uint32_t count);
/* delay decrement */
void delay_decrement(void);

#endif /* BSP_SYSTICK_H */
