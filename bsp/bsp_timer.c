#include "bsp_timer.h"
#include "n32g430_tim.h"

#define INIT_TIM2_FREQUENCY 10000
#define INIT_TIM2_DUTY      500


uint32_t Tim2Frequency = INIT_TIM2_FREQUENCY;
uint16_t Tim2Duty = INIT_TIM2_DUTY;


/*!
    \brief      configure the TIMER2 peripheral: A0 PWMOUT
    \param[in]  none
    \param[out] none
    \retval     none
*/
void timer2_config(void)
{
    //GPIO config A0
    GPIO_InitType GPIO_InitStructure;
    GPIO_Structure_Initialize(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Current = GPIO_DS_4MA;
    GPIO_InitStructure.Pin        = GPIO_PIN_0;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF6_TIM2;
    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);
    
    //timer config TIMER2
	TIM_TimeBaseInitType TIM_TimeBaseStructure;
    TIM_Base_Struct_Initialize(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.Period    = (1000000/INIT_TIM2_FREQUENCY)-1;
    TIM_TimeBaseStructure.Prescaler = (SystemClockFrequency/1000000)-1;
    TIM_TimeBaseStructure.ClkDiv    = 0;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;
    TIM_Base_Initialize(TIM2, &TIM_TimeBaseStructure);   
    
    //oc ch1 config
    OCInitType TIM_OCInitStructure;
    TIM_Output_Channel_Struct_Initialize(&TIM_OCInitStructure);
    TIM_OCInitStructure.OcMode       = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OutputState  = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.OutputNState = TIM_OUTPUT_NSTATE_DISABLE;
    TIM_OCInitStructure.Pulse        = ((uint32_t)1000000*INIT_TIM2_DUTY/1000/INIT_TIM2_FREQUENCY)-1;
    TIM_OCInitStructure.OcPolarity   = TIM_OC_POLARITY_HIGH;
    TIM_OCInitStructure.OcNPolarity  = TIM_OCN_POLARITY_HIGH;
    TIM_OCInitStructure.OcIdleState  = TIM_OC_IDLE_STATE_RESET;
    TIM_OCInitStructure.OcNIdleState = TIM_OCN_IDLE_STATE_RESET;
    TIM_Output_Channel1_Initialize(TIM2, &TIM_OCInitStructure);
    
    TIM_On(TIM2);
    TIM_PWM_Output_Enable(TIM2);
}

/*!
    \brief      configure the TIMER2 frequency
    \param[in]  frequency
    \param[out] none
    \retval     none
*/
void timer2_frequency_set(uint32_t frequency)
{
    Tim2Frequency = frequency;
    TIM_Base_Auto_Reload_Set(TIM2,(1000000/Tim2Frequency)-1);
}

/*!
    \brief      configure the TIMER2 duty
    \param[in]  frequency
    \param[out] none
    \retval     none
*/
void timer2_duty_set(uint32_t duty)
{
    Tim2Duty = duty;
    TIM_Compare1_Set(TIM2,((uint32_t)1000000*Tim2Duty/1000/Tim2Frequency)-1);
}


