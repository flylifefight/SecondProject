#include "bsp_spi.h"
#include "n32g430_spi.h"
#include "n32g430_exti.h"
#include "n32g430_dma.h"
#include "string.h"

#define BufferSize 500
uint8_t SPI_Master_Buffer_Tx[BufferSize] = {0};
uint8_t SPI_Master_Buffer_Rx[BufferSize] = {0};

volatile uint8_t SPI_Send_Complete = 0;
volatile uint8_t SPI_Receive_Complete = 0;
volatile uint8_t spi_int_cnt = 0;

/*!
    \brief      configure SPI 1
    \param[in]  none
    \param[out] none
    \retval     none
*/
void spi1_config(void)
{
    GPIO_InitType GPIO_InitStructure;
    GPIO_Structure_Initialize(&GPIO_InitStructure);
    /* Configure master pins: NSS, SCK, MISO and MOSI */
    /* Confugure SPI pins as Alternate Function Push Pull */
    GPIO_InitStructure.Pin        = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF1_SPI1;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_DOWN;
    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);
    
    
    DMA_InitType DMA_InitStructure;
    DMA_Reset(DMA_CH3);
    DMA_Reset(DMA_CH4);
    /* SPI_MASTER TX DMA config */
	DMA_Structure_Initializes(&DMA_InitStructure);
    DMA_InitStructure.MemAddr = (uint32_t)&SPI_Master_Buffer_Tx[0];
    DMA_InitStructure.MemDataSize = DMA_MEM_DATA_WIDTH_HALFWORD;
    DMA_InitStructure.MemoryInc = DMA_MEM_INC_MODE_ENABLE;
    DMA_InitStructure.Direction = DMA_DIR_PERIPH_DST;
    DMA_InitStructure.PeriphAddr = (uint32_t)&SPI1->DAT;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_WIDTH_BYTE;
    DMA_InitStructure.PeriphInc = DMA_PERIPH_INC_MODE_DISABLE;
    DMA_InitStructure.BufSize = 0;
    DMA_InitStructure.CircularMode = DMA_CIRCULAR_MODE_DISABLE;
    DMA_InitStructure.Mem2Mem = DMA_MEM2MEM_DISABLE;
    DMA_InitStructure.Priority = DMA_CH_PRIORITY_MEDIUM;
    DMA_Initializes(DMA_CH3, &DMA_InitStructure);
    DMA_Channel_Request_Remap(DMA_CH3, DMA_REMAP_SPI1_TX);
    /* SPI_MASTER RX DMA config */
    DMA_InitStructure.MemAddr = (uint32_t)&SPI_Master_Buffer_Rx[0];
    DMA_InitStructure.Direction = DMA_DIR_PERIPH_SRC;
    DMA_InitStructure.Priority = DMA_CH_PRIORITY_HIGHEST;
    DMA_Initializes(DMA_CH4, &DMA_InitStructure);
    DMA_Channel_Request_Remap(DMA_CH4, DMA_REMAP_SPI1_RX);
    DMA_Channel_Enable(DMA_CH3);
    DMA_Channel_Enable(DMA_CH4);
    
    DMA_Interrupts_Enable(DMA_CH3, DMA_INT_TXC);
    DMA_Interrupts_Enable(DMA_CH4, DMA_INT_TXC);
    
    DMA_Interrupt_Status_Clear(DMA, DMA_CH3_INT_GLB|DMA_CH3_INT_TXC|DMA_CH3_INT_HTX|DMA_CH3_INT_ERR);
    DMA_Interrupt_Status_Clear(DMA, DMA_CH4_INT_GLB|DMA_CH4_INT_TXC|DMA_CH4_INT_HTX|DMA_CH4_INT_ERR);
    
    NVIC_InitType NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                    = DMA_Channel3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = NVIC_PER_PRIORITY_0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority         = NVIC_SUB_PRIORITY_0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel                    = DMA_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = NVIC_PER_PRIORITY_0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority         = NVIC_SUB_PRIORITY_0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);
    
    SPI_InitType SPI_InitStructure;
    SPI_I2S_Reset(SPI1);
    /* SPI_MASTER configuration ------------------------------------------------------*/
    SPI_Initializes_Structure(&SPI_InitStructure);
    SPI_InitStructure.DataDirection = SPI_DIR_DOUBLELINE_FULLDUPLEX;
    SPI_InitStructure.SpiMode       = SPI_MODE_MASTER;
    SPI_InitStructure.DataLen       = SPI_DATA_SIZE_8BITS;
    SPI_InitStructure.CLKPOL        = SPI_CLKPOL_LOW;
    SPI_InitStructure.CLKPHA        = SPI_CLKPHA_FIRST_EDGE;
    SPI_InitStructure.NSS           = SPI_NSS_SOFT;
    /* APB2 64MHz SPI < 20MHz  SPI = 16MHz*/
    SPI_InitStructure.BaudRatePres  = SPI_BR_PRESCALER_4;
    SPI_InitStructure.FirstBit      = SPI_FB_MSB;
    SPI_InitStructure.CRCPoly       = 7;
    SPI_Initializes(SPI1, &SPI_InitStructure);
    
    SPI_I2S_DMA_Transfer_Enable(SPI1, SPI_I2S_DMA_TX);
    SPI_I2S_DMA_Transfer_Enable(SPI1, SPI_I2S_DMA_RX);
    
    SPI_ON(SPI1);
}

uint8_t spi1_transmit(uint8_t* transmit_data, uint16_t num)
{
    if(num > BufferSize || DMA_Current_Data_Transfer_Number_Get(DMA_CH3) != 0 || DMA_Current_Data_Transfer_Number_Get(DMA_CH4) != 0)
    {
        return 0;
    }
    
    DMA_Channel_Disable(DMA_CH3);
    DMA_Channel_Disable(DMA_CH4);
    memcpy(SPI_Master_Buffer_Tx, transmit_data, num);
    DMA_Memory_Address_Config(DMA_CH3, (uint32_t)SPI_Master_Buffer_Tx);
    DMA_Buffer_Size_Config(DMA_CH3, num);
    DMA_Memory_Address_Config(DMA_CH4, (uint32_t)SPI_Master_Buffer_Rx);
    DMA_Buffer_Size_Config(DMA_CH4, num);
    DMA_Channel_Enable(DMA_CH3);
    DMA_Channel_Enable(DMA_CH4);
    return 1;
}

uint8_t spi1_receive(uint8_t* receive_data, uint16_t num)
{
    if(num > BufferSize || DMA_Current_Data_Transfer_Number_Get(DMA_CH3) != 0 || DMA_Current_Data_Transfer_Number_Get(DMA_CH4) != 0)
    {
        return 0;
    }
    
    DMA_Channel_Disable(DMA_CH3);
    DMA_Channel_Disable(DMA_CH4);
    memset(SPI_Master_Buffer_Tx, 0, num);
    DMA_Memory_Address_Config(DMA_CH3, (uint32_t)SPI_Master_Buffer_Tx);
    DMA_Buffer_Size_Config(DMA_CH3, num);
    DMA_Memory_Address_Config(DMA_CH4, (uint32_t)receive_data);
    DMA_Buffer_Size_Config(DMA_CH4, num);
    DMA_Channel_Enable(DMA_CH3);
    DMA_Channel_Enable(DMA_CH4);
    return 1;
}

uint8_t spi1_transmit_and_receive(uint8_t* transmit_data, int8_t* receive_data, uint16_t num)
{
    if(num > BufferSize || DMA_Current_Data_Transfer_Number_Get(DMA_CH3) != 0 || DMA_Current_Data_Transfer_Number_Get(DMA_CH4) != 0)
    {
        return 0;
    }
    
    DMA_Channel_Disable(DMA_CH3);
    DMA_Channel_Disable(DMA_CH4);
    memcpy(SPI_Master_Buffer_Tx, transmit_data, num);
    DMA_Memory_Address_Config(DMA_CH3, (uint32_t)SPI_Master_Buffer_Tx);
    DMA_Buffer_Size_Config(DMA_CH3, num);
    DMA_Memory_Address_Config(DMA_CH4, (uint32_t)receive_data);
    DMA_Buffer_Size_Config(DMA_CH4, num);
    DMA_Channel_Enable(DMA_CH3);
    DMA_Channel_Enable(DMA_CH4);
    return 1;
}

uint8_t is_spi1_transmit_complete()
{
    if(DMA_Current_Data_Transfer_Number_Get(DMA_CH3) != 0 || DMA_Current_Data_Transfer_Number_Get(DMA_CH4) != 0)
    {
        return 0;
    }
    
    return 1;
}

