#include "main.h"
#include "bsp_systick.h"
#include "bsp_adc.h"
#include "bsp_timer.h"
#include "bsp_usart.h"
#include "bsp_rcc.h"


/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    /* system clocks configuration */
    rcc_config();
    /* NVIC configuration */
    //nvic_config();
    /* systick configuration */
    systick_config(); 
    
    adc_config();
    usart0_config();
    timer2_config();
 
    while(1){
    }
}

