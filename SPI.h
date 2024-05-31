#ifndef __SPI_H
#define __SPI_H
#include <stdint.h>
void SPI_Config(void);
void SPI_NSS_ON(void);
void SPI_NSS_OFF(void);
uint8_t SPI1SendByte(uint8_t);
void SPI1_WriteReg(uint8_t, uint8_t);
uint8_t SPI1_ReadReg(uint8_t);
void SPI1_MulReadReg(uint8_t,uint8_t*,uint8_t);
#endif
