#ifndef _USER_DRIVER_H
#define _USER_DRIVER_H

#include <stdint.h>

typedef void ( *pREAD_HIS_REG ) ( uint16_t Reg_Addr, uint8_t *Rx_Data, uint16_t Rx_Lenth, uint8_t *Tx_Data );
typedef void ( *pDelayMs ) ();                                           // delay_ms
typedef void ( *pSPI_INIT_S ) ();                                        // Vi4302 Pin 9
typedef void ( *pCHIP_EN ) ();                                           // Vi4302 Pin 27
typedef uint8_t ( *pSPI_READ_REG ) ( uint16_t Reg_Addr );                // SPI read Data
typedef void ( *pSPI_SEND_CMD ) ( uint8_t Cmd );                         // SPI read Data
typedef uint8_t ( *pSPI_Write_REG ) ( uint16_t Reg_Addr, uint8_t Data ); // SPI write Datta;
typedef void ( *pSPI_READ_Mul_REG ) ( uint16_t Reg_Addr,
                                      uint8_t *Rx_Data,
                                      uint16_t Rx_Lenth,
                                      uint8_t *Tx_Data ); // SPI burst read Data

typedef void ( *pSPI_WRITE_Mul_REG ) ( uint8_t *pData, uint16_t Lenth ); // SPI burst read Data

extern pREAD_HIS_REG      READ_HIS_REG;
extern pDelayMs           DelayMs;
extern pSPI_INIT_S        Spi_Init_S;
extern pCHIP_EN           Chip_En;
extern pSPI_READ_REG      Spi_Read_Reg;
extern pSPI_Write_REG     Spi_Write_Reg;
extern pSPI_READ_Mul_REG  Spi_Read_Mul_Reg;
extern pSPI_SEND_CMD      Spi_Send_Cmd;
extern pSPI_WRITE_Mul_REG Spi_Write_Mul_Reg;
#endif
