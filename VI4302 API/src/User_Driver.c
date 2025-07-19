/********************************** (C) COPYRIGHT *******************************
 * File Name          : User_Driver.c
 * Author             : visionICs
 * Version            : V0.3.1
 * Date               : 2023/11/23
 * Description        : Define user function pointer
 *******************************************************************************/
#include "User_Driver.h"

pREAD_HIS_REG      READ_HIS_REG;
pDelayMs           DelayMs;
pSPI_INIT_S        Spi_Init_S;
pCHIP_EN           Chip_En;
pSPI_READ_REG      Spi_Read_Reg;
pSPI_Write_REG     Spi_Write_Reg;
pSPI_READ_Mul_REG  Spi_Read_Mul_Reg;
pSPI_SEND_CMD      Spi_Send_Cmd;
pSPI_WRITE_Mul_REG Spi_Write_Mul_Reg;
