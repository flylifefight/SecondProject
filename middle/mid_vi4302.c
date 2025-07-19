#include "mid_vi4302.h"
#include "bsp_spi.h"
#include "bsp_adc.h"
#include "bsp_systick.h"
#include "bsp_flash.h"
#include "bsp_exti.h"
#include "n32g430.h"
#include "VI4302_System.h"
#include "User_Driver.h"
#include "bin.h"

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
} A2_Configurable_Parameters;

typedef struct
{
    // sensor work mode, could be _SINGLE_PIXEL_MODE or _RANG_MODE
    uint8_t work_mode;
    // whether sensor is calibrated
    uint8_t sensor_is_calibrated;
    // sensor calibrated tdc value
    uint8_t sensor_cal_tdc;
    // sensor calibrated bvd value
    uint8_t sensor_cal_bvd;
    // temperature when sensor is calibrating bvd
    int8_t   sensor_cal_ts;
    uint8_t  xtalk_is_calibrated;
    uint8_t  xtalk_is_write;
    A2_Configurable_Parameters  sys_A2_Conf_Para;
		
} system_config_t;

volatile uint8_t Spi_busy = 0;
uint8_t Work_Mode =0;		//工作模式、【测距模式（连续测距&直方图）】【单pixel模式】
uint8_t BVD_Save_flash = 0; //上电进行BVD矫正 写入flash
int8_t TEMP_Save_flash = 0; //有符号单字节温度，不含小数【10℃ 调整一个档】
uint8_t TDC_Save_flash = 0;	//获取TDC标定的值。
uint8_t Para_Save_Flash = 0;//获取内部参数的标志位，只获取一次。
uint16_t Fw_size = 0;		//芯片内部固件的大小
uint16_t VI4302_Frame = 0;  //芯片返回的实际帧率
uint16_t VI4302_FTDC = 0;	//芯片FTDC 0x20A
uint16_t R_Status = 0;		//设置帧率的状态
system_config_t g_sys_cfg;
uint8_t x_tof=0;
uint16_t x_peak=0;

#define FLASH_SYSTEM_CONFIG_ADDRESS 0x0800FC00
#define _SINGLE_PIXEL_MODE 0x55
#define _RANG_MODE 0xAA
#define SPI_CS_ENABLE GPIO_Pins_Reset(GPIOA, GPIO_PIN_4)
#define SPI_CS_DISABLE GPIO_Pins_Set(GPIOA, GPIO_PIN_4)

/**
  * @brief  SPI CS SET.
  * @param  None
  * @retval None
  */
void spi_cs_init(void)
{
    GPIO_InitType GPIO_InitStructure;
    GPIO_Structure_Initialize(&GPIO_InitStructure);
    GPIO_InitStructure.Pin        = GPIO_PIN_4;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_OUT_PP;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);
}


/**
  * @brief  Set VI4302 entry hardware standby.
  * @param  None
  * @retval None
  */
void VI4302_HW_Standby(void)
{   
    GPIO_InitType GPIO_InitStructure;
    GPIO_Structure_Initialize(&GPIO_InitStructure);
    GPIO_InitStructure.Pin        = GPIO_PIN_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_OUT_PP;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_DOWN;
    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);
	GPIO_Pins_Reset(GPIOA, GPIO_PIN_3);
	delay_1ms(10);
	GPIO_Pins_Set(GPIOA, GPIO_PIN_3);
	delay_1ms(10);
}

/**
  * @brief  Read register from VI4302.
  * @param  reg_addr
  * @retval reg_value
*/
uint8_t vi4302_read_register(uint16_t reg_addr)
{
    while (Spi_busy) ;
    Spi_busy = 1;
	uint8_t Txbuf[4] = { 0 };
	uint8_t Rxbuf[4] = { 0 };
	SPI_CS_ENABLE;
	Txbuf[0] = 0x00;
	Txbuf[1] = (reg_addr & 0xff00) >> 8;
	Txbuf[2] = (reg_addr & 0x00ff);
	Txbuf[3] = 0xff;
	
    spi1_transmit_and_receive(Txbuf, Rxbuf, 4);
	
	while (is_spi1_transmit_complete() == 0);  
	SPI_CS_DISABLE; 
	Spi_busy = 0;
	return Rxbuf[3];  
}

/**
  * @brief  Write VI4302 register.
  * @param  reg_addr - address
  * @param  val - value to be write
  * @retval None
*/
uint8_t vi4302_write_register(uint16_t reg_addr, uint8_t val)
{ 
	while (Spi_busy);
    Spi_busy = 1;
	uint8_t Txbuf[4] = { 1 };
	SPI_CS_ENABLE;
	
	Txbuf[0] = 0x01;
	Txbuf[1] = (reg_addr & 0xff00) >> 8;
	Txbuf[2] = (reg_addr & 0x00ff);
	Txbuf[3] = val;
    spi1_transmit(Txbuf, 4);
	while (is_spi1_transmit_complete() == 0);  
	SPI_CS_DISABLE; 
	Spi_busy = 0;
	return 0;
}


/**
  * @brief  Read mul register.
  * @param  reg_addr - address
  * @param  Rx_Data - pointer to the read buffer
  * @param  Rx_Lenth - read size
  * @param  Tx_Data - Tx cmd
  * @retval None
*/
void vi4302_read_mul_reg(uint16_t Reg_Addr,uint8_t *Rx_Data,uint16_t Rx_Lenth,uint8_t *Tx_Data)
{
	while (Spi_busy);
    Spi_busy = 1;
	SPI_CS_ENABLE;
	
	Tx_Data[0] = 0x00;
	Tx_Data[1] = (Reg_Addr & 0xff00) >> 8;
	Tx_Data[2] = (Reg_Addr & 0x00ff);

    spi1_transmit(Tx_Data, 3);
	while (is_spi1_transmit_complete() == 0);  

	
    spi1_receive(Rx_Data, Rx_Lenth);
	SPI_Receive_Complete=1; 
	while (SPI_Receive_Complete); 
    Spi_busy = 0; 
}

/**
  * @brief  Write CMD to VI4302.
  * @param  cmd
  * 		READ_REG_CMD, WRITE_REG_CMD, READ_HIST_DATA, LOAD_FIRMWARE     etc..
  * @retval None
*/
void vi4302_write_cmd(uint8_t cmd)
{	
	while (Spi_busy);
    Spi_busy = 1;
	SPI_CS_ENABLE;
	spi1_transmit(&cmd, 1);
	while (is_spi1_transmit_complete() == 0);  
	SPI_CS_DISABLE; 
	Spi_busy = 0;
}

/**
  * @brief  SPI write bytes.
  * @param  data - pointer to the write buffer
  * @param  size - size of bytes to be read
  * @retval None
*/
void spi_write_data(uint8_t *data, uint16_t size)
{
    while (Spi_busy);
    Spi_busy = 1;
	SPI_CS_ENABLE;
	
	spi1_transmit(data, size);
	while (is_spi1_transmit_complete() == 0);  
	SPI_CS_DISABLE; 
	Spi_busy = 0;
}

void vi4302_read_his_reg(uint16_t Reg_Addr,uint8_t *Rx_Data,uint16_t Rx_Lenth,uint8_t *Tx_Data)
{
	 while (Spi_busy);
    Spi_busy = 1;
	SPI_CS_ENABLE;
	
    spi1_transmit(Tx_Data, 3);
	while (is_spi1_transmit_complete() == 0);  

	
    spi1_receive(Rx_Data, Rx_Lenth);
	SPI_Receive_Complete=1; 
	while (SPI_Receive_Complete); 
    Spi_busy = 0;
}

/**
  * @brief  user driver function point init.
  * @param  None
  * @retval None
*/
void user_driver_init(void)
{
    READ_HIS_REG = vi4302_read_his_reg;
    DelayMs = delay_1ms;
	Spi_Init_S = spi_cs_init;
	Chip_En = VI4302_HW_Standby;
	Spi_Read_Reg = vi4302_read_register;
	Spi_Write_Reg = vi4302_write_register;
	Spi_Read_Mul_Reg = vi4302_read_mul_reg;
	Spi_Send_Cmd = vi4302_write_cmd;
	Spi_Write_Mul_Reg = spi_write_data;
}

void vi4302_all_init(void)
{
    fmc_read(FLASH_SYSTEM_CONFIG_ADDRESS, (uint8_t*)&g_sys_cfg, sizeof(g_sys_cfg));
    Work_Mode = g_sys_cfg.work_mode;
    VI4302_SPI_INT_Init();
    spi1_config();
    user_driver_init();
    VI4302_Pin_Init();
    
    if(Work_Mode != _SINGLE_PIXEL_MODE && Work_Mode != _RANG_MODE)
	{
        Work_Mode = _SINGLE_PIXEL_MODE;//配置为单pixel模式
        g_sys_cfg.work_mode = Work_Mode;
        fmc_write(FLASH_SYSTEM_CONFIG_ADDRESS, (uint8_t*)&g_sys_cfg, sizeof(g_sys_cfg));
	}
	if(Work_Mode == _SINGLE_PIXEL_MODE)//单pixel模式
	{
		VI4302_Reg_Init();
		VI4302_BVD_Calculate();//外部MCU进行BVD校准
	}
	else if(Work_Mode == _RANG_MODE)//测距模式
	{
		uint8_t  VI4302_Update_flg=0;//内部固件下载 成功|失败 标志位
		VI4302_GPIO0_INT_Init();	 //GPIO_0外部中断
		Fw_size = Get_Fw_bytes();	 //获取内部固件大小【Byte】 
		VI4302_Update_Config(1);     //更新内部固件前 对4302进行配置，使其准备好 
		VI4302_Update_firmware((uint8_t*)vis_sensor_fw,Fw_size);//下载
		VI4302_Update_Config(0);		 //更新完、再次配置4302
		VI4302_Update_flg =  VI4302_Update_Succcess_Fail();//#* Return         : uint8_t 0x11 success  0x12 fail  0x00 timeout.
		if(VI4302_Update_flg != 0x11)//升级失败
		{ 
            while(1);//后续做别的处理
		}	
		delay_1ms(100);

        if( g_sys_cfg.sensor_is_calibrated != 0XA5 )
		{
			VI4302_TDC_Cal ( &TDC_Save_flash );         // 获取TDC标定值
			VI4302_Bvd_Cal ( &BVD_Save_flash );         // 获取BVD标定值 内部默认加12档 r03
			TEMP_Save_flash = get_avg_temp_adc() / 100; // 获取BVD标定时的温度【有符号单字节温度，不含小数【bvd会随温度调整：10℃// 调整一个档，内部固件来做】】
			g_sys_cfg.sensor_cal_bvd       = BVD_Save_flash;
			g_sys_cfg.sensor_cal_tdc       = TDC_Save_flash;
			g_sys_cfg.sensor_cal_ts        = TEMP_Save_flash;
			g_sys_cfg.sensor_is_calibrated = 0XA5;
			
			fmc_write(FLASH_SYSTEM_CONFIG_ADDRESS, (uint8_t*)&g_sys_cfg, sizeof(g_sys_cfg));
			Bvd_Cal.OTP_BVD  = BVD_Save_flash;
			Bvd_Cal.OTP_Temp = TEMP_Save_flash;
		}
		else
		{
            // load data from global config to local variable
            BVD_Save_flash  = g_sys_cfg.sensor_cal_bvd;
            TDC_Save_flash  = g_sys_cfg.sensor_cal_tdc;
            TEMP_Save_flash = g_sys_cfg.sensor_cal_ts;

            // set previous calibrated value to sensor
            VI4302_Write_Reg ( 0x24f, BVD_Save_flash );
            VI4302_Write_Reg ( 0x242, TDC_Save_flash );

            // save data for later ts bvd
            Bvd_Cal.OTP_BVD  = BVD_Save_flash;
            Bvd_Cal.OTP_Temp = TEMP_Save_flash;
		}
		VI4302_Reg_Init();//寄存器配置
		Adaptation_REG_and_ALGO(&Al_Pa);   //算法适配，MA  PEAK缩放、LSB
		R_Status = VI4302_Frame_Rate_AutoCtrl( 4000, &VI4302_Frame);	
    }
        
}



