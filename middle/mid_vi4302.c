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
uint8_t Work_Mode =0;		//����ģʽ�������ģʽ���������&ֱ��ͼ��������pixelģʽ��
uint8_t BVD_Save_flash = 0; //�ϵ����BVD���� д��flash
int8_t TEMP_Save_flash = 0; //�з��ŵ��ֽ��¶ȣ�����С����10�� ����һ������
uint8_t TDC_Save_flash = 0;	//��ȡTDC�궨��ֵ��
uint8_t Para_Save_Flash = 0;//��ȡ�ڲ������ı�־λ��ֻ��ȡһ�Ρ�
uint16_t Fw_size = 0;		//оƬ�ڲ��̼��Ĵ�С
uint16_t VI4302_Frame = 0;  //оƬ���ص�ʵ��֡��
uint16_t VI4302_FTDC = 0;	//оƬFTDC 0x20A
uint16_t R_Status = 0;		//����֡�ʵ�״̬
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
        Work_Mode = _SINGLE_PIXEL_MODE;//����Ϊ��pixelģʽ
        g_sys_cfg.work_mode = Work_Mode;
        fmc_write(FLASH_SYSTEM_CONFIG_ADDRESS, (uint8_t*)&g_sys_cfg, sizeof(g_sys_cfg));
	}
	if(Work_Mode == _SINGLE_PIXEL_MODE)//��pixelģʽ
	{
		VI4302_Reg_Init();
		VI4302_BVD_Calculate();//�ⲿMCU����BVDУ׼
	}
	else if(Work_Mode == _RANG_MODE)//���ģʽ
	{
		uint8_t  VI4302_Update_flg=0;//�ڲ��̼����� �ɹ�|ʧ�� ��־λ
		VI4302_GPIO0_INT_Init();	 //GPIO_0�ⲿ�ж�
		Fw_size = Get_Fw_bytes();	 //��ȡ�ڲ��̼���С��Byte�� 
		VI4302_Update_Config(1);     //�����ڲ��̼�ǰ ��4302�������ã�ʹ��׼���� 
		VI4302_Update_firmware((uint8_t*)vis_sensor_fw,Fw_size);//����
		VI4302_Update_Config(0);		 //�����ꡢ�ٴ�����4302
		VI4302_Update_flg =  VI4302_Update_Succcess_Fail();//#* Return         : uint8_t 0x11 success  0x12 fail  0x00 timeout.
		if(VI4302_Update_flg != 0x11)//����ʧ��
		{ 
            while(1);//��������Ĵ���
		}	
		delay_1ms(100);

        if( g_sys_cfg.sensor_is_calibrated != 0XA5 )
		{
			VI4302_TDC_Cal ( &TDC_Save_flash );         // ��ȡTDC�궨ֵ
			VI4302_Bvd_Cal ( &BVD_Save_flash );         // ��ȡBVD�궨ֵ �ڲ�Ĭ�ϼ�12�� r03
			TEMP_Save_flash = get_avg_temp_adc() / 100; // ��ȡBVD�궨ʱ���¶ȡ��з��ŵ��ֽ��¶ȣ�����С����bvd�����¶ȵ�����10��// ����һ�������ڲ��̼���������
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
		VI4302_Reg_Init();//�Ĵ�������
		Adaptation_REG_and_ALGO(&Al_Pa);   //�㷨���䣬MA  PEAK���š�LSB
		R_Status = VI4302_Frame_Rate_AutoCtrl( 4000, &VI4302_Frame);	
    }
        
}



