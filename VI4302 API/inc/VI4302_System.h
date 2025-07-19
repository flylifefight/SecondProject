
#ifndef SYS_VI430X_H
#define SYS_VI430X_H


#include <stdint.h>


#define _SINGLE_PIXEL_MODE 0x55
#define _RANG_MODE 0xAA
extern uint8_t Work_Mode;

/* macro definition */
#define STREAM_SINGLE    ( 0 )
#define STREAM_MULT      ( 1 )
#define UPDATE_BUFF_SIZE 128
/* fifo depth definition */
#define MAX_FIFO_DEPTH ( 8 )

/* user operation definition */
#define MAX_FRAME_NUM_READ_PER_TIME ( MAX_FIFO_DEPTH )

#define SPI_INT_EVENT ( 1 << 0 )

#define MP_TYPE_NORMAL      ( 0 )
#define MP_TYPE_ATTENUATION ( 1 )
#define MP_TYPE_REFERENCE   ( 2 )

#define MP_5X5_AVG_FRAME_CNT ( 10 )

#define FRAME_SIZE_IN_BYTE_DEF ( 46 )

#define FW_UP_STATUS ( 0xaa ) // flag indicate that fw is running
/* sensor mp definition */
#define MP_INDEX_0   ( 0 )
#define MP_INDEX_1   ( 1 )
#define MP_INDEX_2   ( 2 )
#define MP_INDEX_3   ( 3 )
#define MP_INDEX_4   ( 4 )
#define MP_INDEX_5   ( 5 )
#define MP_INDEX_6   ( 6 )
#define MP_INDEX_7   ( 7 )
#define MP_INDEX_8   ( 8 )
#define MP_INDEX_9   ( 9 )
#define MP_INDEX_10  ( 10 )
#define MP_INDEX_11  ( 11 )
#define MP_INDEX_12  ( 12 )
#define MP_INDEX_13  ( 13 )
#define MP_INDEX_14  ( 14 )
#define MP_INDEX_15  ( 15 )
#define MP_INDEX_16  ( 16 )
#define MP_INDEX_17  ( 17 )
#define MP_INDEX_18  ( 18 )
#define MP_INDEX_19  ( 19 )
#define MP_INDEX_20  ( 20 )
#define MP_INDEX_21  ( 21 )
#define MP_INDEX_22  ( 22 )
#define MP_INDEX_23  ( 23 )
#define MP_INDEX_24  ( 24 )
#define MP_INDEX_MAX ( MP_INDEX_24 )
typedef enum
{
    Disable = 0,
    Enable  = 1
} STATUS_E;

typedef enum
{
    FF_FIRST_4_TOF_PEAK         = 0x0, // 输出最先找到的4个峰
    FF_FIRST_AND_3_MAX_TOF_PEAK = 0x2, // 输出1个最先找到的峰 + 3个最大峰
    FF_MAX_4_TOF_PEAK           = 0x3, // 输出4个最大峰
} DSP_FF_E;

typedef enum
{
    VSM_RANGING_STOP = 0x0,    // 测距停止        -->RANGING_STOP->RANGING_START-|
    VSM_RANGING_START,         // 正在测距       |                               |
    VSM_RANGING_GOING_TO_STOP, // 即将停止测距   |<--------------<---------------V
} VSM_E;

typedef enum
{
    NORM_HIS = 0x11,
    ATTEN_HIS,
    REF_HIS,
} HIS_MODE;

typedef struct
{
    int8_t  Cur_Temp;
    int8_t  OTP_Temp;
    uint8_t OTP_BVD;
} BVD_CAL;
extern BVD_CAL Bvd_Cal;

typedef struct
{
    uint16_t MA_Coefficient;
    uint16_t PEAK_Coefficient;
    uint16_t LSB_Coefficient;
    uint16_t Intg_Num;
} Algorithm_Parameter;

extern Algorithm_Parameter Al_Pa;

/* operation command definition */
#define READ_REG_CMD    ( 0x00 )
#define WRITE_REG_CMD   ( 0x01 )
#define READ_HIST_DATA  ( 0x03 )
#define LOAD_FIRMWARE   ( 0x04 )
#define READ_FIRMWARE   ( 0x05 )
#define UNLOCK_FIRMWARE ( 0x06 )
#define LOCK_FIRMWARE   ( 0x07 )
#define WRITE_DRAM      ( 0x08 )
#define READ_DRAM       ( 0x09 )

/* Xtalk Command definnition */

#define XTALK_CONFIG_CMD  0X43
#define XTALK_CAL_CMD     0X44
#define S_PARAMETER       0X01
#define D_PARAMETER       0X02
#define MP_PARAMETER      0X04
#define XTALK_BIN         0X08
#define XTALK_PEAK        0X10
#define GAP_1      	        0X20
#define RAITO      	        0X40
#define XTALK_RANG_CNT      0X80
#define CONFIDENCE_K        0X100         
#define GAP_2               0X200 
#define CONFIDENCE_MODE     0X400 
#define CONF_PARA_CONFIG_CMD  0X43 //参数配置命令
#define CONF_PARA_READ_CMD    0X45

/* interface definition */
#define HISTOGRAM_DATA_SIZE  ( 2048 )
#define SPAD_COUNT_TIME_10MS ( 0x00171e8f )
#define SPAD_COUNT_TIME      ( SPAD_COUNT_TIME_10MS )

/* max reply time in ms from firmware */
#define FIRMWARE_MAX_REPLY_TIME ( 1000)

/* firmware reply status */
#define FIRMWARE_STATUS_NONE  ( 0x00 )
#define FIRMWARE_STATUS_ACK   ( 0x11 ) /* command execute successful  */
#define FIRMWARE_STATUS_STALL ( 0x12 ) /* sensor is busy */
#define FIRMWARE_STATUS_DONE  ( 0x13 )
#define FIRMWARE_STATUS_FAIL  ( 0x14 ) /* command execute failed  */

/* firmware version */
#define VI4302_FIRMWARE_VERSION 4200000107

void    VI4302_Init ( void );
void    VI4302_enable_dcdc ( void );
uint8_t VI4302_Reg_Init ( void );
void    VI4302_Pin_Init ( void );
uint8_t VI4302_Stream_On ( uint8_t mode, STATUS_E en );
uint8_t VI4302_Get_Frame_Cnt ( void );
uint8_t VI4302_read_frame ( uint8_t num, uint16_t frame_data_start, uint16_t frame_data_end, uint8_t *recv_frame_buf );
uint8_t
VI4302_Set_Frame_Data_Format ( uint8_t mp_type, uint8_t tof_mask, uint8_t peak_mask, uint8_t noise_multshot_mask );
uint8_t VI4302_Set_Mp_Openonly ( uint8_t mp_index );
uint8_t VI4302_Set_Mp_All ( STATUS_E en );
void    VI4302_SinglePixel_Output ( uint16_t *peak_data );
void    VI4302_Set_Bvd ( uint8_t base, uint8_t step );
uint8_t VI4302_BVD_Calculate ( void );
uint8_t VI4302_Stop_Ranging ( void );
uint8_t VI4302_Start_Ranging ( void );
void    VI4302_Update_Config ( uint8_t Start_End );
uint8_t VI4302_Update_Succcess_Fail ( void );
void    VI4302_Update_firmware ( uint8_t *p, uint16_t len );
uint8_t VI4302_Read_His_Config ( HIS_MODE HIS_mode );
uint8_t VI4302_Read_Histogram ( HIS_MODE HIS_mode, uint8_t *HisData );
uint8_t VI4302_Frame_Rate_Config ( uint16_t us );
void    VI4302_Temp_Bvd ( BVD_CAL *Temp_Bvd );
uint8_t VI4302_Bvd_Cal ( uint8_t *Bvd_Val );
uint8_t VI4302_TDC_Cal ( uint8_t *TDC_Val );
uint8_t VI4302_Frame_Rate_AutoCtrl ( uint16_t Set_Fps, uint16_t *Real_Fps );
void    Adaptation_REG_and_ALGO ( Algorithm_Parameter *Coefficient );
void    VI4302_Read_firmware ( uint8_t *p, uint16_t len );
uint8_t VI4302_Write_Reg ( uint16_t Reg_addr, uint8_t Val );
uint8_t VI4302_Read_Reg ( uint16_t Reg_Addr, uint8_t *Val );
uint8_t vi4302_read_otp ( uint8_t addr );
void VI4302_Enable_DcDc ( void );
uint8_t vi4302_read_status_from_firmware ( const uint32_t repeat_num );

typedef struct {
    uint16_t parameter_types;
	  uint8_t  para_S;
	  uint8_t  para_D;
	  uint8_t  para_MP;
		uint16_t  xtalk_bin;
		uint16_t xtalk_peak;
	  uint8_t  xtalk_rang_cnt;
	  uint8_t  flag_1_gap;
	  uint8_t  flag_1_peak_ratio; 
		uint8_t	 flag_2_gap; 
		uint8_t	 confidence_k;
	  uint8_t  confidence_mode; 		
} Xtalk_parameter_struct;

uint8_t S_Parameter_Init(uint8_t S_Para);
uint8_t D_parameter_init(uint8_t D_Para);
uint8_t MP_Cnt_Init(uint8_t MP_cnt);
uint8_t Xtalk_Init(Xtalk_parameter_struct *xtalk_struct);
uint8_t Xtalk_Cal(uint8_t bin_num,uint16_t *X_bin,uint16_t *X_peak,uint32_t retry_cnt);
uint8_t Xtalk_Para_Init(uint16_t xtalk_bin,uint16_t xtalk_peak,uint8_t xtalk_gap1);
uint8_t Search_Logic_Init(uint8_t Logic);
uint8_t Confidence_K_Init(uint8_t Conf_K);
uint8_t Dust_detection_init(uint8_t Gap1,uint8_t Gap2,uint8_t Raito);
uint8_t Xtalk_rang_cnt_init(uint8_t Rang_Cnt);
#endif
