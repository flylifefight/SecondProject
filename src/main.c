#include "main.h"
#include "bsp_systick.h"
#include "bsp_adc.h"
#include "bsp_timer.h"
#include "bsp_usart.h"
#include "bsp_rcc.h"
#include "bsp_misc.h"
#include "mid_vi4302.h"


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
    misc_config();
    /* systick configuration */
    systick_config(); 
    
    adc_config();
    usart0_config();
    timer2_config();
    vi4302_all_init();
    while(1){
    }
}

