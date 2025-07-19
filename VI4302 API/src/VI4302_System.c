/********************************** (C) COPYRIGHT *******************************
 * File Name          : VI4302_System.c
 * Author             : visionICs
 * Version            : V0.3.1
 * Date               : 2023/11/23
 * Description        : Vi4302 function
 *******************************************************************************/
#include "VI4302_System.h"
#include "User_Driver.h"
#include "VI4302_Config.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
extern volatile uint8_t gpio0_int_cnt;
extern volatile uint8_t EXTI_FLAG ;
extern uint8_t Work_Mode;
extern uint16_t     run_event;
VSM_E               VSM_e   = VSM_RANGING_STOP;
BVD_CAL             Bvd_Cal = { 0 };
Algorithm_Parameter Al_Pa   = { 0 };

/**
 * @brief  Get the status from firmware.
 * @param  repeat_num - loop times
 * @retval status - last valid firmware status
 */
uint8_t vi4302_read_status_from_firmware ( const uint32_t repeat_num )
{
    uint32_t retry_cnt = repeat_num;
    uint8_t  status    = 0;

    while ( retry_cnt != 0 )
    {
        retry_cnt--;

        DelayMs( 1 );
        status = Spi_Read_Reg ( 0x30 );
        if ( ( status == FIRMWARE_STATUS_ACK ) || ( status == FIRMWARE_STATUS_STALL ) ||
             ( status == FIRMWARE_STATUS_DONE ) || ( status == FIRMWARE_STATUS_FAIL ) )
        {
            break;
        }
    }

    return status;
}

/*******************************************************************************
 * Function Name  : VI4302_Reg_Init.
 * Description    : ��ʼ���Ĵ����б�
 * Input          : None.
 * Output         : None.
 * Return         : 0 success !0 error
 *******************************************************************************/
uint8_t VI4302_Reg_Init ()
{
    uint8_t status = 0;

    if ( Work_Mode == _SINGLE_PIXEL_MODE )
    {
        for ( uint8_t i = 0; i < S_VI4302_REG_NUM; i++ )
        {
            status += Spi_Write_Reg ( S_regist_buffer_default[i][0], S_regist_buffer_default[i][1] );
            DelayMs( 1 );
        }
    }
    else if ( Work_Mode == _RANG_MODE )
    {
        for ( uint8_t i = 0; i < R_VI4302_REG_NUM; i++ )
        {
            uint8_t firmware_status = FIRMWARE_STATUS_NONE;
            firmware_status         = VI4302_Write_Reg ( R_regist_buffer_default[i][0], R_regist_buffer_default[i][1] );
            if ( firmware_status != FIRMWARE_STATUS_ACK )
            {
                status++;
            }
            DelayMs( 1 );
        }
    }

    return status;
}

/*******************************************************************************
 * Function Name  : VI4302_Pin_Init.
 * Description    : ��ʼ��.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void VI4302_Pin_Init ()
{
    Chip_En ();
    Spi_Init_S ();
}

/*******************************************************************************
 * Function Name  : VI4302_Init.
 * Description    : VI4302 initialization.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void VI4302_Init ()
{
    VI4302_Pin_Init ();
    DelayMs( 10 );
    VI4302_Reg_Init ();
    VI4302_Enable_DcDc ();
}

/*******************************************************************************
 * Function Name  : VI4302_Enable_DcDc.
 * Description    : enable dcdc.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void VI4302_Enable_DcDc ( void )
{
    uint8_t chip_rev_id   = Spi_Read_Reg ( 0x14 );
    uint8_t dcdc_ctrl_reg = Spi_Read_Reg ( 0x250 );

    if ( chip_rev_id == 0xA0 )
    {
        dcdc_ctrl_reg &= ~0x02;
    }
    else
    {
        dcdc_ctrl_reg |= 0x02;
    }

    Spi_Write_Reg ( 0x250, dcdc_ctrl_reg );
}

/*******************************************************************************
* Function Name  : VI4302_Stream_On.
* Description    : ��ʼ��ࡡ����SINGLE_PIXEL_MODE ��Ч
* Input          : mode
                                     [STREAM_SINGLE]
                                     [STREAM_MULT]
                 : en
                                     [ENABLE]
                                     [Disbale]
* Output         : None
* Return         : 0 success  !0 error
*******************************************************************************/
uint8_t VI4302_Stream_On ( uint8_t mode, STATUS_E en )
{
    uint8_t reg_val = 0;

    Spi_Write_Reg ( 0xd6, ( 1 << 0 ) );

    reg_val = Spi_Read_Reg ( 0x8c );
    if ( mode == STREAM_SINGLE )
    {
        reg_val &= ~( ( 1 << 4 ) | ( 1 << 1 ) );
        reg_val |= ( en << 1 );
    }
    else if ( mode == STREAM_MULT )
    {
        reg_val |= ( 1 << 4 );
        reg_val &= ~( 1 << 0 );
        reg_val |= ( en << 0 );
    }
    else
        return 1;
    Spi_Write_Reg ( 0x8c, reg_val );

    return 0;
}

/*******************************************************************************
 * Function Name  : VI4302_Get_Frame_Cnt.
 * Description    : ��ȡ��ǰ����֡
 * Input          : None.
 * Output         : None.
 * Return         : frame count
 *******************************************************************************/
uint8_t VI4302_Get_Frame_Cnt ( void )
{
    return Spi_Read_Reg ( 0xdd );
}

/*******************************************************************************
* Function Name  : VI4302_read_frame.
* Description    : ��ȡ����֡������SINGLE_PIXEL_MODE ��Ч
* Input          : num
                 : frame_data_start
                 : frame_data_end
* Output         : recv_frame_buf
* Return         : 0 success  !0 error
*******************************************************************************/
uint8_t VI4302_read_frame ( uint8_t num, uint16_t frame_data_start, uint16_t frame_data_end, uint8_t *recv_frame_buf )
{
    uint8_t frame_idx  = 0;

    uint8_t tx_data[4] = { 0, 0, 0, 0 };

    if ( !recv_frame_buf || frame_data_start > 0x12a || frame_data_start < 0xfd || frame_data_end > 0x12a ||
         frame_data_end < frame_data_start )
        return 1;

    if ( num > MAX_FRAME_NUM_READ_PER_TIME )
        num = MAX_FRAME_NUM_READ_PER_TIME;

    for ( ; frame_idx < num; frame_idx++ )
    {
        Spi_Write_Reg ( 0xd7, Enable );
        Spi_Read_Mul_Reg ( frame_data_start,
                           &recv_frame_buf[frame_idx * ( frame_data_end - frame_data_start + 1 )],
                           frame_data_end - frame_data_start + 1,
                           tx_data );
    }
    return 0;
}

/*******************************************************************************
* Function Name  : VI4302_Set_Frame_Data_Format.
* Description    : ���������֡���ݸ�ʽ ���� SINGLE_PIXEL_MODE ��Ч
* Input          : mp_type
                        [MP_TYPE_NORMAL]
                        [MP_TYPE_ATTENUATION]
                        [MP_TYPE_REFERENCE]
                 : tof_mask
                        [BIT[3:0] normal&attenuation tof output mask]
                        [BIT[0]   reference tof output mask]
                 : noise_multshot_mask
                        [BIT[0] multi_shot output mask]
                        [BIT[1] noise output mask]
                        note: only the normal&attenuation has this field
* Output         : recv_frame_buf
* Return         : 0 success  !0 error
*******************************************************************************/
uint8_t
VI4302_Set_Frame_Data_Format ( uint8_t mp_type, uint8_t tof_mask, uint8_t peak_mask, uint8_t noise_multshot_mask )
{
    uint8_t reg_val = 0;

    if ( mp_type == MP_TYPE_NORMAL )
    {
        reg_val = Spi_Read_Reg ( 0xd8 );
        reg_val &= ~0x0F;
        reg_val |= ( tof_mask & 0x0F );
        Spi_Write_Reg ( 0xd8, reg_val );

        reg_val = Spi_Read_Reg ( 0xda );
        reg_val &= ~0x0F;
        reg_val |= ( peak_mask & 0x0F );
        Spi_Write_Reg ( 0xda, reg_val );

        reg_val = Spi_Read_Reg ( 0xdc );
        reg_val &= ~0x01;
        reg_val |= ( noise_multshot_mask & 0x01 );
        reg_val &= ~( 1 << 2 );
        reg_val |= ( ( noise_multshot_mask & 0x02 ) << 1 );
        Spi_Write_Reg ( 0xdc, reg_val );
    }
    else if ( mp_type == MP_TYPE_ATTENUATION )
    {
        reg_val = Spi_Read_Reg ( 0xd8 );
        reg_val &= ~0xF0;
        reg_val |= ( 0xF0 & ( tof_mask << 4 ) );
        Spi_Write_Reg ( 0xd8, reg_val );

        reg_val = Spi_Read_Reg ( 0xda );
        reg_val &= ~0xF0;
        reg_val |= ( ( peak_mask << 4 ) & 0xF0 );
        Spi_Write_Reg ( 0xda, reg_val );

        reg_val = Spi_Read_Reg ( 0xdc );
        reg_val &= ~( 1 << 1 );
        reg_val |= ( ( noise_multshot_mask & 0x01 ) << 1 );
        reg_val &= ~( 1 << 3 );
        reg_val |= ( ( noise_multshot_mask & 0x02 ) << 2 );
        Spi_Write_Reg ( 0xdc, reg_val );
    }
    else if ( mp_type == MP_TYPE_REFERENCE )
    {
        reg_val = Spi_Read_Reg ( 0xd9 );
        reg_val &= ~0x01;
        reg_val |= ( tof_mask & 0x01 );
        Spi_Write_Reg ( 0xd9, reg_val );

        reg_val = Spi_Read_Reg ( 0xdb );
        reg_val &= ~0x01;
        reg_val |= ( peak_mask & 0x01 );
        Spi_Write_Reg ( 0xdb, reg_val );
    }
    else
        return 1;

    return 0;
}

/*******************************************************************************
 * Function Name  : VI4302_Set_Mp_Openonly.
 * Description    : ����ֻ����ĳ��MP������SINGLE_PIXEL_MODE ��Ч
 * Input          : mp_index
 * Output         : None.
 * Return         : 0 success !0 error
 *******************************************************************************/
uint8_t VI4302_Set_Mp_Openonly ( uint8_t mp_index )
{
    uint16_t start    = 0x231;
    uint16_t reg_addr = 0;
    uint8_t  reg_val  = 0;

    if ( mp_index > MP_INDEX_MAX )
        return 1;

    reg_addr = 0x231 + mp_index / 8;

    for ( ; start < 0x235; start++ )
    {
        if ( reg_addr == start )
        {
            reg_val |= ( Enable << ( mp_index % 8 ) );
        }

        Spi_Write_Reg ( start, reg_val );
        reg_val = 0;
    }
    return 0;
}

/*******************************************************************************
* Function Name  : vi4302_set_mp_all
* Description    : �������е�MP ����SINGLE_PIXEL_MODE ��Ч
* Input          : en
                    [enable],[disable]
* Output         : None.
* Return         : 0 success !0 error
*******************************************************************************/
uint8_t VI4302_Set_Mp_All ( STATUS_E en )
{
    uint16_t start   = 0x231;
    uint8_t  reg_val = 0;

    if ( en )
        reg_val = 0xff;

    for ( ; start < 0x235; start++ )
    {
        if ( start == 0x234 )
            Spi_Write_Reg ( start, reg_val & 0x01 );
        else
            Spi_Write_Reg ( start, reg_val );
    }

    return 0;
}

/*******************************************************************************
 * Function Name  : VI4302_SinglePixel_Output.
 * Description    : ���25��pixel��Ӧ��peakֵ  ����SING_PIXEL_MODE ��Ч
 * Input          : peak_data:��ȡpeak���ݵ�ָ��
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void VI4302_SinglePixel_Output ( uint16_t *peak_data )
{
    uint8_t avg_frame_cnt                           = 0;
    uint8_t frame_sum                               = 0;
    uint8_t frame_cnt                               = 0;
    uint8_t index                                   = 0;
    uint8_t rd_len                                  = 0;

    uint8_t   frame_rcv_buf[FRAME_SIZE_IN_BYTE_DEF] = { 0 };
    uint16_t *p_peak                                = NULL;
    uint16_t *p_output                              = NULL;
    uint16_t  start_addr                            = 0x12b;
    uint16_t  peak_out                              = 0;

    p_output                                        = peak_data;

    // ranging off
    Spi_Write_Reg ( 0xd6, ( 1 << 0 ) );
    VI4302_Stream_On ( STREAM_MULT, Disable );

    // frame format: normal_peak0 + attenuation_peak0
    VI4302_Set_Frame_Data_Format ( MP_TYPE_NORMAL, 0x00, 0x01, 0x00 ); // peak0+multshot
    rd_len += 2;
    VI4302_Set_Frame_Data_Format ( MP_TYPE_ATTENUATION, 0x00, 0x01, 0x00 ); // peak0+multshot
    rd_len += 2;
    VI4302_Set_Frame_Data_Format ( MP_TYPE_REFERENCE, 0x00, 0x00, 0x00 );
    rd_len += 0 + 0;

    start_addr -= rd_len;
    if ( rd_len == 0 )
    {
        // read from data0 address
        start_addr = 0xfd;
        rd_len     = FRAME_SIZE_IN_BYTE_DEF;
    }
    // set mp selection
    for ( index = MP_INDEX_0; index <= MP_INDEX_MAX; index++ )
    {
        // ranging off
        Spi_Write_Reg ( 0xd6, ( 1 << 0 ) );
        VI4302_Stream_On ( STREAM_MULT, Disable );

        // select mp to enable
        VI4302_Set_Mp_Openonly ( index );

        // ranging on
        Spi_Write_Reg ( 0xd6, ( 1 << 0 ) );
        VI4302_Stream_On ( STREAM_MULT, Enable );

        while ( 1 )
        {
            if ( EXTI_FLAG )
            {
                EXTI_FLAG = 0;
                frame_cnt = VI4302_Get_Frame_Cnt ();
                if ( !frame_cnt )
                    continue;

                VI4302_read_frame ( frame_cnt, start_addr, start_addr + rd_len - 1, frame_rcv_buf );

                if ( avg_frame_cnt < MP_5X5_AVG_FRAME_CNT )
                {
                    for ( frame_sum = 0; frame_sum < frame_cnt; frame_sum++ )
                    {
                        if ( index == MP_INDEX_0 || index == MP_INDEX_4 || index == MP_INDEX_20 ||
                             index == MP_INDEX_24 )
                        { // attenuation
                            p_peak = (uint16_t *)&frame_rcv_buf[4 * frame_sum + 2];
                            peak_out += *p_peak;
                        }
                        else // normal
                        {
                            p_peak = (uint16_t *)&frame_rcv_buf[4 * frame_sum];
                            peak_out += *p_peak;
                        }
                        avg_frame_cnt++;
                        if ( avg_frame_cnt == MP_5X5_AVG_FRAME_CNT )
                            break;
                    }
                }

                if ( avg_frame_cnt == MP_5X5_AVG_FRAME_CNT )
                {
                    peak_out /= avg_frame_cnt;
                    *p_output++   = (uint16_t)peak_out;

                    avg_frame_cnt = 0;
                    peak_out      = 0;
                    break;
                }
            }
        }
    }
    VI4302_Set_Mp_All ( Enable ); // enable all mp
}

/*******************************************************************************
 * Function Name  : VI4302_Set_BVD.
 * Description    : ����BVD
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void VI4302_Set_Bvd ( uint8_t base, uint8_t step )
{
    uint8_t bvd_val = ( ( base & 0x3 ) << 6 ) | ( step & 0x3F );
    Spi_Write_Reg ( 0x24f, bvd_val );
}

/*******************************************************************************
 * Function Name  : VI4302_BVD_Calculate.
 * Description    : Vsapd��λ����
 * Input          : None.
 * Output         : None.
 * Return         : uint8_t�����У����λ,0x0A ����ֵ��
 *******************************************************************************/
uint8_t VI4302_BVD_Calculate ()
{
    uint8_t  Cal_BVD = 0;
    uint8_t  step    = 0;
    uint8_t  val     = 0;
    uint16_t dcr_cnt = 0;

    Spi_Write_Reg ( 0x24f, 0x80 );
    Spi_Write_Reg ( 0x24e, 0x81 );

    Spi_Write_Reg ( 0x144, ( SPAD_COUNT_TIME >> 0 ) & 0xFF );
    Spi_Write_Reg ( 0x143, ( SPAD_COUNT_TIME >> 8 ) & 0xFF );
    Spi_Write_Reg ( 0x142, ( SPAD_COUNT_TIME >> 16 ) & 0xFF );
    Spi_Write_Reg ( 0x141, ( SPAD_COUNT_TIME >> 24 ) & 0xFF );

    for ( step = 0; step <= 0x3F; step++ )
    {
        VI4302_Set_Bvd ( 0x3, step );
        val = Spi_Read_Reg ( 0x140 );
        Spi_Write_Reg ( 0x140, val | 0x01 ); // sw trigger to enable DCR
        val = Spi_Read_Reg ( 0x140 );

        while ( val & 0x10 )
        {
            val = Spi_Read_Reg ( 0x140 );
        }

        dcr_cnt = ( Spi_Read_Reg ( 0x177 ) << 8 ) | Spi_Read_Reg ( 0x178 );

        if ( dcr_cnt > 1 )
        {
            Cal_BVD = ( 0x3 << 6 ) | ( step & 0x3f );
            break;
        }
    }

    Cal_BVD += 15;
    if ( Cal_BVD >= 0xFF )
        Cal_BVD = 0xFF;
    Spi_Write_Reg ( 0x24f, Cal_BVD );

    return Cal_BVD;
}

/*******************************************************************************
 * Function Name  : VI4302_Stop_Ranging.
 * Description    : ��ʼRanging
 * Input          : None.
 * Output         : None.
 * Return         : uint8_t 0 timeout  0x11 success  0x12 fail��
 *******************************************************************************/
uint8_t VI4302_Stop_Ranging ()
{
    uint8_t status = 0;

    Spi_Write_Reg ( 0x31, 0x00 ); // stop ranging
    Spi_Write_Reg ( 0x32, 0x00 ); // ����ģʽ
    Spi_Write_Reg ( 0x33, 0x01 ); //
    VSM_e = VSM_RANGING_GOING_TO_STOP;
    Spi_Send_Cmd ( 0x19 ); // ����0x19����

    status = vi4302_read_status_from_firmware ( FIRMWARE_MAX_REPLY_TIME );

    if ( status == FIRMWARE_STATUS_ACK )
    {
        VSM_e = VSM_RANGING_STOP;
    }

    return status;
}

/*******************************************************************************
 * Function Name  : VI4302_Start_Ranging.
 * Description    : ��ʼRanging
 * Input          : None.
 * Output         : None.
 * Return         : uint8_t 0 timeout  0x11 success  0x12 fail
 *******************************************************************************/
uint8_t VI4302_Start_Ranging ()
{
    uint8_t status = 0;

    VI4302_Stop_Ranging ();

    Spi_Write_Reg ( 0x31, 0x01 ); 
    Spi_Write_Reg ( 0x32, 0x02 ); // 02 ��������� 01 �����������
    Spi_Write_Reg ( 0x33, 0x02 );

    Spi_Send_Cmd ( 0x19 ); // ����0x19����

    status = vi4302_read_status_from_firmware ( FIRMWARE_MAX_REPLY_TIME );
    if ( status == FIRMWARE_STATUS_ACK )
    {
        VSM_e = VSM_RANGING_START;
    }

    return status;
}

/*******************************************************************************
 * Function Name  : VI4302_Update_Config.
 * Description    : ���¹̼�������
 * Input          : uint8_t��1 ��ʼ���¹̼�ǰ���� 0 ���¹̼�������
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void VI4302_Update_Config ( uint8_t Start_End )
{
    volatile uint8_t reg_val = 0;
    if ( Start_End == 1 ) // ���¹̼�ǰ����
    {
        Spi_Write_Reg ( 0x30, 0 );
        reg_val = Spi_Read_Reg ( 0x01 );
        reg_val &= ~( 0x02 );
        Spi_Write_Reg ( 0x01, reg_val );
        Spi_Write_Reg ( 0x02, 0 );
        reg_val = 0x06;
        Spi_Send_Cmd ( reg_val );
    }
    else // ���¹̼�������
    {
        reg_val = 0x07;
        Spi_Send_Cmd ( reg_val );

        reg_val = Spi_Read_Reg ( 0x01 );
        reg_val |= 0x02;
        Spi_Write_Reg ( 0x01, reg_val );
    }
}

/*******************************************************************************
 * Function Name  : VI4302_Update_Succcess_Fail.
 * Description    : �̼�����ʧ�ܻ��߳ɹ�
 * Input          : None.
 * Output         : None.
 * Return         : uint8_t 0x11 success  0x12 fail  0x00 timeout.
 *******************************************************************************/
uint8_t VI4302_Update_Succcess_Fail ()
{
    uint8_t val    = 0;
    uint8_t status = 0;
    for ( uint16_t idx = 0; idx < 1000; idx++ )
    {
        val = Spi_Read_Reg ( 0x30 );
        if ( val == FW_UP_STATUS )
            break;
    }
    if ( val == FW_UP_STATUS )
        status = 0x11;
    else
        status = 0x12;

    return status;
}

/*******************************************************************************
* Function Name  : VI4302_Update_firmware.  ���һ֡1024�ֽ�
* Description    : ���¹̼�
* Input          : uint8_t*:���ݿ�
                   uint16_t:���ݳ���
* Output         : None.
* Return         : None.
*******************************************************************************/

void VI4302_Update_firmware ( uint8_t *p, uint16_t len )
{
    uint32_t addr             = 0;
    uint16_t firm_len_integer = 0;
    uint16_t firm_len_decimal = 0;
    uint8_t *Updata           = (uint8_t *)malloc ( sizeof ( uint8_t ) * ( UPDATE_BUFF_SIZE + 3 ) );
    firm_len_integer          = len / UPDATE_BUFF_SIZE;
    firm_len_decimal          = len % UPDATE_BUFF_SIZE;

    for ( uint16_t i = 0; i < firm_len_integer; i++ )
    {
        memcpy ( &Updata[3], &p[i * UPDATE_BUFF_SIZE], UPDATE_BUFF_SIZE );
        Updata[0] = LOAD_FIRMWARE;
        Updata[1] = ( addr >> 8 );
        Updata[2] = ( addr );
        Spi_Write_Mul_Reg ( Updata, UPDATE_BUFF_SIZE + 3 );
        addr += UPDATE_BUFF_SIZE;
        memset ( Updata, 0, UPDATE_BUFF_SIZE + 3 );
    }
    if ( firm_len_decimal != 0 )
    {
        memcpy ( &Updata[3], &p[firm_len_integer * UPDATE_BUFF_SIZE], firm_len_decimal );
        Updata[0] = LOAD_FIRMWARE;
        Updata[1] = ( addr >> 8 );
        Updata[2] = ( addr );
        Spi_Write_Mul_Reg ( Updata, firm_len_decimal + 3 );
        addr += firm_len_decimal;
        memset ( Updata, 0, UPDATE_BUFF_SIZE + 3 );
    }
    free ( Updata );
}

/*******************************************************************************
 * Function Name  : VI4302_Read_His_Config.
 * Description    : ֱ��ͼ������
 * Input          : uint8_t��1 ��ʼ���¹̼�ǰ���� 0 ���¹̼�������
 * Output         : None.
 * Return         : uint8_t 0 timeout  0x11 success  0x12 fail
 *******************************************************************************/
uint8_t VI4302_Read_His_Config ( HIS_MODE HIS_mode )
{
    uint8_t status = 0;

    switch ( HIS_mode )
    {
    case NORM_HIS :
        Spi_Write_Reg ( 0x31, NORM_HIS );
        break;
    case ATTEN_HIS :
        Spi_Write_Reg ( 0x31, ATTEN_HIS );
        break;
    case REF_HIS :
        Spi_Write_Reg ( 0x31, REF_HIS );
        break;
    default :
        break;
    }
    Spi_Send_Cmd ( 0x18 ); // ����0x18����

    status = vi4302_read_status_from_firmware ( FIRMWARE_MAX_REPLY_TIME );
    if ( status == FIRMWARE_STATUS_ACK )
    {
        VSM_e = VSM_RANGING_START;
    }

    return status;
}

/*******************************************************************************
 * Function Name  : VI4302_Read_Histogram.
 * Description    : ��ȡֱ��ͼ��
 * Input          : HIS_MODE ��ȡ�ĸ�ֱ��ͼ
 * Output         : uint8_t* ֱ��ͼ���ݿ�
 * Return         : uint8_t 1 fail 0 success
 *******************************************************************************/
uint8_t VI4302_Read_Histogram ( HIS_MODE HIS_mode, uint8_t *HisData )
{
    uint8_t  hist_idx   = 0;
    uint8_t  val        = 0;
    uint16_t hist_addr  = 0;
    uint8_t  tx_data[4] = { 0 };
    switch ( HIS_mode )
    {
    case NORM_HIS :
        hist_idx = 0;
        break;
    case ATTEN_HIS :
        hist_idx = 5;
        break;
    case REF_HIS :
        hist_idx = 6;
        break;
    default :
        break;
    }

    Spi_Write_Reg ( 0x50, 0x01 );
    val = Spi_Read_Reg ( 0x50 );
    if ( val )
    {
        val = 0;
    }
    else
    {
        return 1;
    }

    hist_addr  = hist_idx * 2048;

    tx_data[0] = READ_HIST_DATA;
    tx_data[1] = ( hist_addr >> 8 );
    tx_data[2] = hist_addr;

    // Spi_Read_Mul_Reg(hist_addr,HisData,2048,tx_data);

    READ_HIS_REG ( hist_addr, HisData, 2048, tx_data );
    Spi_Write_Reg ( 0x50, 0x00 );

    return 0;
}

/*******************************************************************************
 * Function Name  : VI4302_Frame_Rate_Config.
 * Description    : frame rate
 * Input          : uint16_t us.
 * Output         : None.
 * Return         : uint8_t 0 timeout  0x11 success  0x12 fail
 *******************************************************************************/
uint8_t VI4302_Frame_Rate_Config ( uint16_t us )
{
    uint8_t status = 0;

    Spi_Write_Reg ( 0x30, 0x00 );
    Spi_Write_Reg ( 0x31, us );
    Spi_Write_Reg ( 0x32, us >> 8 );

    Spi_Send_Cmd ( 0x28 ); // ����0x28����

    status = vi4302_read_status_from_firmware ( FIRMWARE_MAX_REPLY_TIME );
    if ( status == FIRMWARE_STATUS_ACK )
    {
        VSM_e = VSM_RANGING_START;
    }

    return status;
}

/*******************************************************************************
 * Function Name  : VI4302_Temp_Bvd.
 * Description    : bvd�¶�У׼
 * Input          : BVD_CAL *BVD_CAL *Temp_Bvd[Cur_Temp:��ǰ���¶�,OTP_Temp ���bvd��λ���¶� OTP_BVD��BVDֵ]
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void VI4302_Temp_Bvd ( BVD_CAL *Temp_Bvd )
{
    uint8_t Check_Sum = 0;

    Check_Sum         = ~( Temp_Bvd->Cur_Temp + Temp_Bvd->OTP_Temp + Temp_Bvd->OTP_BVD + 0x29 );
    Spi_Write_Reg ( 0x31, Temp_Bvd->Cur_Temp );
    Spi_Write_Reg ( 0x32, Temp_Bvd->OTP_Temp );
    Spi_Write_Reg ( 0x33, Temp_Bvd->OTP_BVD );
    Spi_Write_Reg ( 0x34, 0x00 );
    Spi_Write_Reg ( 0x35, Check_Sum );
    Spi_Send_Cmd ( 0x29 ); // ����0x29����
}

/*******************************************************************************
 * Function Name  : VI4302_Bvd_Cal.
 * Description    : bvd�궨����
 * Input          : uint8_t *Bvd_Val �궨ֵ��ַ
 * Output         : None.
 * Return         : uint8_t 0 timeout  0x11 success  0x12 fail
 *******************************************************************************/
uint8_t VI4302_Bvd_Cal ( uint8_t *Bvd_Val )
{
    uint8_t status = 0;

    Spi_Write_Reg ( 0x30, 0x00 );
    Spi_Write_Reg ( 0x31, ( SPAD_COUNT_TIME >> 0 ) & 0xFF );
    Spi_Write_Reg ( 0x32, ( SPAD_COUNT_TIME >> 8 ) & 0xFF );
    Spi_Write_Reg ( 0x33, ( SPAD_COUNT_TIME >> 16 ) & 0xFF );
    Spi_Write_Reg ( 0x34, ( SPAD_COUNT_TIME >> 24 ) & 0xFF );

    Spi_Send_Cmd ( 0x12 ); // VBDУ׼��Ҫʱ��

    status = vi4302_read_status_from_firmware ( FIRMWARE_MAX_REPLY_TIME );
    if ( status == FIRMWARE_STATUS_ACK )
    {
        *Bvd_Val = Spi_Read_Reg ( 0x31 );
    }
    else
    {
        *Bvd_Val = 0;
    }

    return status;
}

/*******************************************************************************
 * Function Name  : VI4302_TDC_Cal.
 * Description    : TDC�궨����
 * Input          : uint8_t *Bvd_Val �궨ֵ��ַ
 * Output         : None.
 * Return         : uint8_t 0 timeout  0x11 success  0x12 fail
 *******************************************************************************/
uint8_t VI4302_TDC_Cal ( uint8_t *TDC_Val )
{
    uint8_t status = 0;

    Spi_Write_Reg ( 0x30, 0x00 );
    Spi_Write_Reg ( 0x31, 0x0A );
    Spi_Write_Reg ( 0x32, 0x0B );
    Spi_Send_Cmd ( 0x24 );

    status = vi4302_read_status_from_firmware ( FIRMWARE_MAX_REPLY_TIME );
    if ( status == FIRMWARE_STATUS_ACK )
    {
        *TDC_Val = Spi_Read_Reg ( 0x31 );
    }
    else
    {
        *TDC_Val = 0;
    }

    return status;
}

/*******************************************************************************
 * Function Name  : VI4302_Frame_Rate_AuteCtrl
 * Description    : ����VI4302���֡��
 * Input          : uint16_t Fps ���õ�֡��
 * Output         : None.
 * Return         : uint8_t  0x11 success  0x12 fail
 *******************************************************************************/
uint8_t VI4302_Frame_Rate_AutoCtrl ( uint16_t Set_Fps, uint16_t *Real_Fps )
{
    uint8_t status = 0;

    Spi_Write_Reg ( 0x30, 0x00 );
    Spi_Write_Reg ( 0x31, Set_Fps );
    Spi_Write_Reg ( 0x32, Set_Fps >> 8 );
    Spi_Send_Cmd ( 0x1B );

    status = vi4302_read_status_from_firmware ( FIRMWARE_MAX_REPLY_TIME );
    if ( status == FIRMWARE_STATUS_ACK )
    {
        *Real_Fps = Spi_Read_Reg ( 0x31 ) | ( Spi_Read_Reg ( 0x32 ) << 8 );
    }
    else
    {
        *Real_Fps = 0;
    }

    return status;
}

/*******************************************************************************
* Function Name  : Adaptation_REG_and_ALGO
* Description    : VI4302 �㷨������ҪԤ֪�Ĵ���
* Input          : uint16_t MA_coefficient  MA ϵ��
                   uint16_t PEAK_coefficient peak ����ϵ��
                   uint16_t LSB_coefficient  �ֱ���
* Output         : None.
* Return         : uint8_t  0x11 success  0x12 fail
*******************************************************************************/
void Adaptation_REG_and_ALGO ( Algorithm_Parameter *Coefficient )
{
    Coefficient->MA_Coefficient   = ( ( Spi_Read_Reg ( 0x00b4 ) & 0x0f ) + ( ( Spi_Read_Reg ( 0x00b4 ) & 0xf0 ) >> 4 ) +
                                    ( Spi_Read_Reg ( 0x00b3 ) & 0x0f ) + ( ( Spi_Read_Reg ( 0x00b3 ) & 0xf0 ) >> 4 ) +
                                    ( Spi_Read_Reg ( 0x00b2 ) & 0x0f ) + ( ( Spi_Read_Reg ( 0x00b2 ) & 0xf0 ) >> 4 ) +
                                    ( Spi_Read_Reg ( 0x00b1 ) & 0x0f ) + ( ( Spi_Read_Reg ( 0x00b1 ) & 0xf0 ) >> 4 ) +
                                    ( Spi_Read_Reg ( 0x00b0 ) & 0x0f ) + ( ( Spi_Read_Reg ( 0x00b0 ) & 0xf0 ) >> 4 ) +
                                    ( Spi_Read_Reg ( 0x00AF ) & 0x0f ) + ( ( Spi_Read_Reg ( 0x00AF ) & 0xf0 ) >> 4 ) +
                                    ( Spi_Read_Reg ( 0x00AE ) & 0x0f ) + ( ( Spi_Read_Reg ( 0x00AE ) & 0xf0 ) >> 4 ) +
                                    ( Spi_Read_Reg ( 0x00AD ) & 0x0f ) + ( ( Spi_Read_Reg ( 0x00AD ) & 0xf0 ) >> 4 ) );
    Coefficient->PEAK_Coefficient = pow ( 2, Spi_Read_Reg ( 0x00A2 ) );
    Coefficient->LSB_Coefficient  = 498 + 498 * ( Spi_Read_Reg ( 0x0042 ) & 0x03 );
    Coefficient->Intg_Num         = ( Spi_Read_Reg ( 0x0086 ) << 8 ) + Spi_Read_Reg ( 0x0087 );
}

/*******************************************************************************
 * Function Name  : VI4302_Read_Reg
 * Description    : VI4302���ⲿ�������Ĵ���
 * Input          : uint16_t Reg_Addr д�Ĵ�����ַ
 * Output         : uint8_t *Val      �洢����ָ��
 * Return         : uint8_t  0x11 success  0x12 fail
 *******************************************************************************/
uint8_t VI4302_Read_Reg ( uint16_t Reg_Addr, uint8_t *Val )
{
    uint8_t status = 0;

    Spi_Write_Reg ( 0x30, 0x00 );
    Spi_Write_Reg ( 0x31, Reg_Addr );
    Spi_Write_Reg ( 0x32, Reg_Addr >> 8 );
    Spi_Send_Cmd ( 0x10 );

    status = vi4302_read_status_from_firmware ( FIRMWARE_MAX_REPLY_TIME );
    if ( status == FIRMWARE_STATUS_ACK )
    {
        *Val = Spi_Read_Reg ( 0x31 );
    }
    else
    {
        *Val = 0;
    }

    return status;
}

/*******************************************************************************
 * Function Name  : VI4302_Read_Reg
 * Description    : VI4302���ⲿ�������Ĵ���
 * Input          : uint16_t Reg_Addr д�Ĵ�����ַ
 * Output         : uint8_t *Val      �洢����ָ��
 * Return         : uint8_t  0x11 success  0x12 fail
 *******************************************************************************/
uint8_t VI4302_Write_Reg ( uint16_t Reg_addr, uint8_t Val )
{
    uint8_t status = 0;

    Spi_Write_Reg ( 0x30, 0x00 );
    Spi_Write_Reg ( 0x31, Reg_addr );
    Spi_Write_Reg ( 0x32, Reg_addr >> 8 );
    Spi_Write_Reg ( 0x33, Val );
    Spi_Send_Cmd ( 0x11 );

    status = vi4302_read_status_from_firmware ( FIRMWARE_MAX_REPLY_TIME );

    return status;
}

/**
 * @brief read sensor otp data
 * @param[in] addr otp addr
 * @return otp value
 */
uint8_t Vi4302_Read_Otp ( uint8_t Addr )
{
    const uint16_t otp_pa_addr   = 0x09;
    const uint16_t otp_prd_addr  = 0x0c;
    const uint16_t otp_pdob_addr = 0x0f;
    uint8_t        otp_val       = 0;

    Spi_Write_Reg ( otp_pa_addr, Addr );
    Spi_Write_Reg ( otp_prd_addr, 1 );
    Spi_Write_Reg ( otp_prd_addr, 0 );

    otp_val = Spi_Read_Reg ( otp_pdob_addr );

    return otp_val;
}
/*******************************************************************************
 * Function Name  : Xtalk_Cal
 * Description    : XtalУ׼
 * Input          : 
										uint8_t bin_num ѡ��Xtal ��0-bin_num�ķ�Χ�ڱ궨��Ҫ��ֱ��ͼ��ʵ��׶ο�ֱ��ͼȷ��Xtalk�ķ�Χ��
										uint8_t *X_bin
										uint16_t *X_peak
										uint32_t retry_cnt
 * Output         : None.
 * Return         : uint8_t  0x11 success  0x12 fail
 *******************************************************************************/
uint8_t Xtalk_Cal(uint8_t bin_num,uint16_t *X_bin,uint16_t *X_peak,uint32_t retry_cnt)
{
	uint8_t status = 0U;
	gpio0_int_cnt = 0;
	Spi_Write_Reg ( 0x31, bin_num&0xff);
    Spi_Write_Reg ( 0x32, (bin_num>>8)&0xff);
	Spi_Send_Cmd (XTALK_CAL_CMD);	
	DelayMs(100);
	while ( retry_cnt != 0 )
    {
        retry_cnt--;
				if(gpio0_int_cnt > 0)
					{
					  status = Spi_Read_Reg ( 0x30 );
						if(status == FIRMWARE_STATUS_ACK)
						{
						  	*X_bin = Spi_Read_Reg ( 0x31 )+( Spi_Read_Reg ( 0x32 ) <<8);
								*X_peak = Spi_Read_Reg ( 0x33 )+( Spi_Read_Reg ( 0x34 ) <<8);
							   break;
						}
						else
						{
							status = 0x88;//�ж��е���Ӧ��ֵ����
							break;
						}
					}	
    }
	return status;
} 

/*******************************************************************************
 * Function Name  : S_parameter_init
 * Description    : S����ʹ��ʧ��
 * Input          : 
										uint8_t S_Para ʹ��ʧ�ܲ���
 * Output         : None.
 * Return         : uint8_t  0x11 success  0x12 fail
 *******************************************************************************/
uint8_t S_Parameter_Init(uint8_t S_Para)
{ 
	uint8_t status = 0U;
	uint8_t s_para_write=0;
	if(S_Para!=0)
	{
	  s_para_write=1;
	}
	uint16_t parameter_types = S_PARAMETER;
	Spi_Write_Reg ( 0x31, parameter_types&0xff);
	Spi_Write_Reg ( 0x32, (parameter_types>>8)&0xff); 
	Spi_Write_Reg ( 0x33, s_para_write);
	Spi_Send_Cmd ( 0X43);
	status = vi4302_read_status_from_firmware ( FIRMWARE_MAX_REPLY_TIME );
	return status;
}
/*******************************************************************************
 * Function Name  : D_parameter_init
 * Description    : D����ʹ��ʧ��
 * Input          : 
										uint8_t D_para ʹ��ʧ�ܲ���
 * Output         : None.
 * Return         : uint8_t  0x11 success  0x12 fail
 *******************************************************************************/ 
uint8_t D_parameter_init(uint8_t D_Para)
{ 
	uint8_t status = 0U;
	uint8_t D_para_write=0;
	uint16_t parameter_types = D_PARAMETER;
	Spi_Write_Reg ( 0x31, parameter_types&0xff);
	Spi_Write_Reg ( 0x32, (parameter_types>>8)&0xff); 
	Spi_Write_Reg ( 0x34, D_Para);
	Spi_Send_Cmd ( 0X43);
	status = vi4302_read_status_from_firmware ( FIRMWARE_MAX_REPLY_TIME );
	return status;
}
/*******************************************************************************
 * Function Name  : MP_Cnt_Init
 * Description    : Xtalk ��������
 * Input          : 
										uint8_t D_para ʹ��ʧ�ܲ���
 * Output         : None.
 * Return         : uint8_t  0x11 success  0x12 fail
 *******************************************************************************/ 
uint8_t MP_Cnt_Init(uint8_t MP_cnt)
{ 
	uint8_t status = 0U;
	uint16_t parameter_types = MP_PARAMETER;
	Spi_Write_Reg ( 0x31, parameter_types&0xff);
	Spi_Write_Reg ( 0x32, (parameter_types>>8)&0xff); 
	Spi_Write_Reg ( 0x35, MP_cnt);
	Spi_Send_Cmd ( 0X43);
	status = vi4302_read_status_from_firmware ( FIRMWARE_MAX_REPLY_TIME );
	return status;
}


/*******************************************************************************
 * Function Name  : Xtalk_init
 * Description    : Xtal��������
 * Input          : Xtalk_parameter_struct *xtalk_struc xtalk ����
 * Output         : None.
 * Return         : uint8_t  0x11 success  0x12 fail
 *******************************************************************************/
uint8_t Xtalk_Init(Xtalk_parameter_struct *xtalk_struct)
{
	uint8_t status = 0U;
	Spi_Write_Reg ( 0x31, xtalk_struct->parameter_types&0XFF);
	Spi_Write_Reg ( 0x32, (xtalk_struct->parameter_types>>8)&0XFF);
	Spi_Write_Reg ( 0x33, xtalk_struct->para_S);
	Spi_Write_Reg ( 0x34, xtalk_struct->para_D);
	Spi_Write_Reg ( 0x35, xtalk_struct->para_MP);
	Spi_Write_Reg ( 0x36, xtalk_struct->xtalk_bin&0xff);
	Spi_Write_Reg ( 0x37, (xtalk_struct->xtalk_bin>>8)&0xff);
	Spi_Write_Reg ( 0x38, xtalk_struct->xtalk_peak&0xff);
	Spi_Write_Reg ( 0x39, (xtalk_struct->xtalk_peak>>8)&0xff);
	Spi_Write_Reg ( 0x3A, xtalk_struct->flag_1_gap);
	Spi_Write_Reg ( 0x3B, xtalk_struct->flag_1_peak_ratio);
	Spi_Write_Reg ( 0x3C, xtalk_struct->xtalk_rang_cnt);
	Spi_Write_Reg ( 0x3D, xtalk_struct->confidence_k);
	Spi_Write_Reg ( 0x3E, xtalk_struct->flag_2_gap);
	Spi_Write_Reg ( 0x3F, xtalk_struct->confidence_mode);
	Spi_Send_Cmd ( XTALK_CONFIG_CMD );
	status = vi4302_read_status_from_firmware ( FIRMWARE_MAX_REPLY_TIME );
	return status;
}
/*******************************************************************************
 * Function Name  : Xtalk_Para_Init.
 * Description    : Xtalk������
 * Input 
										s_mode��	Enable S����������Ч����ʱ�����multshot��Ĭ�ϼ�ȥS����
										d_mode��	Enable D������Ч��ԭ�������r-tof��λ�û����D������XtalkԽ��D����ֵԽ��
										xtalk_bin :�۲� normal tofֱ��ͼ����¼���ӳɷ��Ӧ��bin1���л�Ϊreference tofֱ��ͼ����¼bin2����ֵΪbin1 - bin2�Ĳ�
										xtalk_peak��ģ��Կտ������ģʽ����ʱ���������peakֵ�����������Ӻ��ڻ����ۣ���ֵ����Ĭ�ϲ���2500
										flag_1_gap:xtalkλ�������
										flag_1_peak_ratio:�ҳ����ٷֱ� 5����0.5
										Output  	: None.
	*Output					: None.
  *Return         : uint8_t 0 timeout  0x11 success  0x12 fail
 *******************************************************************************/
uint8_t Xtalk_Para_Init(uint16_t xtalk_bin,uint16_t xtalk_peak,uint8_t xtalk_gap1)
{
	uint8_t status = 0U;
	Xtalk_parameter_struct  xtalk_init_struct;

	xtalk_init_struct.parameter_types = S_PARAMETER|D_PARAMETER|XTALK_BIN|XTALK_PEAK|GAP_1|RAITO;
	xtalk_init_struct.para_S = Enable;
	xtalk_init_struct.para_D = Disable;
	xtalk_init_struct.xtalk_bin = xtalk_bin;			//xtalk��ֵ��Ӧ��bin��ȥr-tof��ֵ��Ӧ��bin 
	xtalk_init_struct.xtalk_peak = xtalk_peak;		//xtalk��ֵ��peakֵ
	xtalk_init_struct.flag_1_gap = xtalk_gap1;		//xtalk�����
	xtalk_init_struct.flag_1_peak_ratio = 5;			//�ҳ����ٷֱ� 5����0.5
	status = Xtalk_Init(&xtalk_init_struct);
	return status;
}
/*******************************************************************************
 * Function Name  : Xtalk_rang_cnt_init
 * Description    : Xtalk�궨����
 * Input          : Rang_Cnt ������
 * Output         : None.
 * Return         : uint8_t  0x11 success  0x12 fail
 *******************************************************************************/
uint8_t Xtalk_rang_cnt_init(uint8_t Rang_Cnt)
{
	uint8_t status = 0U;
	uint16_t parameter_types = XTALK_RANG_CNT;
	Spi_Write_Reg ( 0x31, parameter_types&0xff);
	Spi_Write_Reg ( 0x32, (parameter_types>>8)&0xff); 
	Spi_Write_Reg ( 0x3C, Rang_Cnt);
	Spi_Send_Cmd ( 0X43);
	status = vi4302_read_status_from_firmware ( FIRMWARE_MAX_REPLY_TIME );
	return status;
}

/*******************************************************************************
 * Function Name  : Dust_detection_init
 * Description    : �ҳ��������
 * Input          : 
										gap1: �ó����������������ǰ��flag = 1�����ж�������binλ��xtalk�帽��ʱ������ռ� 15 >= gap1 >=5 
										gap2���ó����Ӽ������ۻ�������Ӵ�����ʵĿ�����������ǰ��flag = 2�����ж�������binλ��xtalk�帽��ʱ������ռ� 12 >= gap2 >=2 
										raito���ж���flag = 1����peak���������� peak > raito*xtalk_peak (0<raito<=10)
 * Output         : None.
 * Return         : uint8_t  0x11 success  0x12 fail
 *******************************************************************************/
uint8_t Dust_detection_init(uint8_t Gap1,uint8_t Gap2,uint8_t Raito)
{
	uint8_t status = 0U;
	uint16_t parameter_types = GAP_1|GAP_2|RAITO;
	Spi_Write_Reg ( 0x31, parameter_types&0xff);
	Spi_Write_Reg ( 0x32, (parameter_types>>8)&0xff); 
	Spi_Write_Reg ( 0x3A, Gap1);
	Spi_Write_Reg ( 0x3B, Gap2);
	Spi_Write_Reg ( 0x3E, Raito);
	Spi_Send_Cmd ( 0X43);
 status = vi4302_read_status_from_firmware ( FIRMWARE_MAX_REPLY_TIME );
 return status;
}

/*******************************************************************************
 * Function Name  : Confidence_K_Init
 * Description    : �ҳ��������
 * Input          :
										Conf_k: �����������ֱ�� �� ǿ�����ʰ��ǶȻز��źŵ����Ŷ�[�������ڳ����߼�Ϊģʽ0�������ʹ��]
										0-64   ֵԽС�⵽�������ʰ�Ŀ�����Խ����̫С���Ժ�����Ҳ��ܸߣ��Ӷ������쳣 
 * Output         : None.
 * Return         : uint8_t  0x11 success  0x12 fail
 *******************************************************************************/
uint8_t Confidence_K_Init(uint8_t Conf_K)
{
	uint8_t status = 0U;
	uint16_t parameter_types = CONFIDENCE_K;
	Spi_Write_Reg ( 0x31, parameter_types&0xff);
	Spi_Write_Reg ( 0x32, (parameter_types>>8)&0xff); 
	Spi_Write_Reg ( 0x3D, Conf_K);
	Spi_Send_Cmd ( 0X43);
	status = vi4302_read_status_from_firmware ( FIRMWARE_MAX_REPLY_TIME );
	return status;
}
/*******************************************************************************
 * Function Name  : Search_Logic_Init
 * Description    : �ҳ��������
 * Input          :
									  logic:0 �������TOFС�ķ� 1������������Ŷȴ�ķ壨���Ŷ���ͬ���peak��ķ壩
 * Output         : None.
 * Return         : uint8_t  0x11 success  0x12 fail
 *******************************************************************************/
uint8_t Search_Logic_Init(uint8_t Logic)
{
	uint8_t status = 0U;
	uint16_t parameter_types = CONFIDENCE_MODE;
	Spi_Write_Reg ( 0x31, parameter_types&0xff);
	Spi_Write_Reg ( 0x32, (parameter_types>>8)&0xff); 
	Spi_Write_Reg ( 0x3F, Logic);
	Spi_Send_Cmd ( 0X43);
	status = vi4302_read_status_from_firmware ( FIRMWARE_MAX_REPLY_TIME );
	return status;
}

