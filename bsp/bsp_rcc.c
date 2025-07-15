#include "bsp_rcc.h"
#include "n32g430_rcc.h"

/*!
    \brief      configure the different system clocks
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcc_config(void)
{
    /* Enable peripheral clocks ------------------------------------------------*/
    RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_AFIO|RCC_APB2_PERIPH_SPI1|RCC_APB2_PERIPH_USART1);
    RCC_APB1_Peripheral_Clock_Enable(RCC_APB1_PERIPH_TIM2);
    /* Enable DMA GPIO ADC clocks */
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_DMA|RCC_AHB_PERIPH_GPIOA|RCC_AHB_PERIPH_GPIOB|RCC_AHB_PERIPH_ADC);
    /* RCC_ADCHCLK_DIV16*/
    ADC_Clock_Mode_Config(ADC_CKMOD_AHB, RCC_ADCHCLK_DIV16);
    RCC_ADC_1M_Clock_Config(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8);  //selsect HSE as RCC ADC1M CLK Source
    
    /* MCO */
    GPIO_InitType GPIO_InitStructure;
    GPIO_Structure_Initialize(&GPIO_InitStructure);
    GPIO_InitStructure.Pin             = GPIO_PIN_8;
    GPIO_InitStructure.GPIO_Mode       = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate  = GPIO_AF9_MCO;
    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);
    RCC_MCO_Source_Config(RCC_MCO_HSE);
}

