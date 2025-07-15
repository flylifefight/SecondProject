#include "bsp_usart.h"
#include "n32g430_usart.h"
#include "n32g430_dma.h"
#include "string.h"

#define MAX_USART_BUFFER_NUM 1024
uint8_t receiveBuffer[MAX_USART_BUFFER_NUM] = {0};
uint8_t transmitBuffer[MAX_USART_BUFFER_NUM] = {0};


/*!
    \brief      configure USART
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usart0_config(void)
{
    GPIO_InitType GPIO_InitStructure;
    /* Initialize GPIO_InitStructure */
    GPIO_Structure_Initialize(&GPIO_InitStructure);
    /* Configure USARTy RX Tx as alternate function push-pull and pull-up */
    GPIO_InitStructure.Pin            = GPIO_PIN_9;    
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF5_USART1;
    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.Pin            = GPIO_PIN_10;    
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF5_USART1;
    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);
    
    
    DMA_InitType DMA_InitStructure;
    /* USART1_Tx_DMA_Channel (triggered by USART1 Tx event) Config */
    DMA_Reset(DMA_CH1);
    DMA_Structure_Initializes(&DMA_InitStructure);
    DMA_InitStructure.PeriphAddr     = (USART1_BASE + 0x04);
    DMA_InitStructure.MemAddr        = (uint32_t)transmitBuffer;
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_DST;
    DMA_InitStructure.BufSize        = 0;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_MODE_DISABLE;
    DMA_InitStructure.MemoryInc      = DMA_MEM_INC_MODE_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_WIDTH_BYTE;
    DMA_InitStructure.MemDataSize    = DMA_MEM_DATA_WIDTH_BYTE;
    DMA_InitStructure.CircularMode   = DMA_CIRCULAR_MODE_DISABLE;
    DMA_InitStructure.Priority       = DMA_CH_PRIORITY_HIGHEST;
    DMA_InitStructure.Mem2Mem        = DMA_MEM2MEM_DISABLE;
    DMA_Initializes(DMA_CH1, &DMA_InitStructure);
    DMA_Channel_Request_Remap(DMA_CH1,DMA_REMAP_USART1_TX);

    /* USART1_Rx_DMA_Channel (triggered by USART1 Rx event) Config */
    DMA_Reset(DMA_CH2);
    DMA_InitStructure.PeriphAddr = (USART1_BASE + 0x04);
    DMA_InitStructure.MemAddr    = (uint32_t)receiveBuffer;
    DMA_InitStructure.Direction  = DMA_DIR_PERIPH_SRC;
    DMA_InitStructure.BufSize    = MAX_USART_BUFFER_NUM;
    DMA_InitStructure.CircularMode   = DMA_CIRCULAR_MODE_ENABLE;
    DMA_Initializes(DMA_CH2, &DMA_InitStructure);
    DMA_Channel_Request_Remap(DMA_CH2,DMA_REMAP_USART1_RX);
    
    
    /* USARTy and USARTz configuration */
    USART_InitType USART_InitStructure;
    USART_Structure_Initializes(&USART_InitStructure);
    USART_InitStructure.BaudRate            = 921600;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;
    /* Configure USART1 */
    USART_Initializes(USART1, &USART_InitStructure);

    /* Enable USART1 DMA Rx and TX request */
    USART_DMA_Transfer_Enable(USART1, USART_DMAREQ_RX | USART_DMAREQ_TX);

    /* Enable USART1 TX DMA Channel */
    DMA_Channel_Enable(DMA_CH1);
    /* Enable USART1 RX DMA Channel */
    DMA_Channel_Enable(DMA_CH2);

    /* Enable the USART1 */
    USART_Enable(USART1);
}

/*!
    \brief      configure the USART0 transmit
    \param[in]  data:array_ptr num:length
    \param[out] success or failure
    \retval     none
*/
uint8_t usart0_transmit(uint8_t* data, uint16_t num)
{
    if(num > MAX_USART_BUFFER_NUM || DMA_Current_Data_Transfer_Number_Get(DMA_CH1) != 0)
    {
        return 0;
    }
    
    DMA_Channel_Disable(DMA_CH1);
    memcpy(transmitBuffer, data, num);
    DMA_Memory_Address_Config(DMA_CH1, (uint32_t)transmitBuffer);
    DMA_Buffer_Size_Config(DMA_CH1, num);
    DMA_Channel_Enable(DMA_CH1);
    return 1;
}    

/*!
    \brief      configure the USART0 receive
    \param[in]  data:array_ptr(get)
    \param[out] receive_num
    \retval     none
*/
uint32_t usart0_receive(uint32_t* array_address)
{
    *array_address = (uint32_t)receiveBuffer;
    return MAX_USART_BUFFER_NUM-DMA_Current_Data_Transfer_Number_Get(DMA_CH2);
}
