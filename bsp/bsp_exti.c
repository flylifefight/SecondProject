#include "bsp_exti.h"
#include "n32g430_exti.h"

volatile uint8_t gpio0_int_cnt = 0;
volatile uint8_t EXTI_FLAG = 0;

//SPI_INT PB0 EXTI_TRIG_RISING
void VI4302_SPI_INT_Init()
{   
    GPIO_InitType GPIO_InitStructure;
    GPIO_Structure_Initialize(&GPIO_InitStructure);
    GPIO_InitStructure.Pin        = GPIO_PIN_0;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_INPUT;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_DOWN;
    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);
    
    GPIO_EXTI_Line_Set(EXTI_LINE_SOURCE0, AFIO_EXTI_PB0);
    
    NVIC_InitType NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                    = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = NVIC_PER_PRIORITY_1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority         = NVIC_SUB_PRIORITY_0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);
    
    EXTI_Flag_Status_Clear(EXTI_LINE0);
    EXTI_InitType EXTI_InitStructure;
    EXTI_Structure_Initializes(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line    = EXTI_LINE0;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Peripheral_Initializes(&EXTI_InitStructure);
}
//GPIO_0 init	PB6   EXTI_TRIG_RISING
void VI4302_GPIO0_INT_Init()
{
    GPIO_InitType GPIO_InitStructure;
    GPIO_Structure_Initialize(&GPIO_InitStructure);
    GPIO_InitStructure.Pin        = GPIO_PIN_6;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_INPUT;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_DOWN;
    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);
    
    GPIO_EXTI_Line_Set(EXTI_LINE_SOURCE1, AFIO_EXTI_PB6);
    
    NVIC_InitType NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                    = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = NVIC_PER_PRIORITY_1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority         = NVIC_SUB_PRIORITY_0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);
    
    EXTI_Flag_Status_Clear(EXTI_LINE1);
    EXTI_InitType EXTI_InitStructure;
    EXTI_Structure_Initializes(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line    = EXTI_LINE1;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Peripheral_Initializes(&EXTI_InitStructure);
}


