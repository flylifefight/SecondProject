/********************************** (C) COPYRIGHT *******************************
 * File Name          : VI4302_Config.c
 * Author             : visionICs
 * Version            : V0.3.1
 * Date               : 2023/11/23
 * Description        : Vi4302 register initialization
 *******************************************************************************/
#include "VI4302_Config.h"

const uint16_t S_regist_buffer_default[S_VI4302_REG_NUM][2] = {
    /* clang-format off */

    0x0209,0x20,//tx trigger 粗调
    0x0243,0x07,//tx trigger 精调
    0x0242,0x00,
    0x024E,0x81,
    0x0250,0x02,
    0x0231,0xFF,
    0x0232,0xFF,
    0x0233,0xFF,
    0x0234,0x01,//MP [0-24] enable
    0x00d3,0x01,//使能SPI 外部中断
    0x00d4,0x04,//设置FIFO的深度
    0x00d5,0x01,//当FIFO已经满时候，丢弃新的数据
    0x00ad,0x23,
    0x00ae,0x45,
    0x00af,0x78,
    0x00b0,0xab,
    0x00b1,0xcd,
    0x00b2,0xef,
    0x00b3,0xed,
    0x00b4,0xa5,//MA参数
    0x00e5,0x01,//输出最大峰依次递减(OP2)
    0x0080,0x00,
    0x0081,0x96,//配置激光重频为1M
    0x0086,0x00,
    0x0087,0x96,//配置积分为150

    /* clang-format on */
};
const uint16_t R_regist_buffer_default[R_VI4302_REG_NUM][2] = {
    /* clang-format off */

    0x0209,0x20,//tx trigger 粗调
    0x0243,0x07,//tx trigger 精调
    0x0230,0xF0,//max cd  min QCH
    0x0235,0x42,//dead time + cd window
    0x0080,0x00,
    0x0081,0x4B,//配置激光重频为2M
    0x0086,0x00,
    0x0087,0xC8,//配置积分为200
    0x00A2,0x02,//peak 缩放

    /* clang-format on */
};
