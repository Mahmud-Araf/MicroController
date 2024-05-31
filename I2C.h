#ifndef __I2C_H
#define __I2C_H
#include <stm32f446xx.h>
#include <stdint.h>
#define MASTER 1
#define SLAVE 0
/******************************************
* I2C communication
******************************************/
typedef struct i2cs{
	I2C_TypeDef *i2c;
	uint8_t SDA;
	uint8_t SCL;
}I2CDev;
 void I2C_Config(uint8_t,uint8_t);
 void I2C_Start(void);
 void I2C_AddressW(uint8_t);
 void I2C_AddressR(uint8_t);
 void I2C_Write(uint8_t data);
 void I2C_MultiWrite(uint8_t*, uint8_t);
 void I2C_Stop(void);
 void I2C_write(uint8_t data);
 uint8_t* I2C_SRead(uint8_t*);
 uint8_t I2C_MRead();
 uint8_t I2C_Busy(I2C_TypeDef*);
 //reset i2c bus
void I2C_Bus_Reset(I2C_TypeDef *I2C,GPIO_TypeDef *GPIO, uint8_t SCL, uint8_t SDA);
void I2C_Clear_Busy_Flag(I2C_TypeDef *I2C,GPIO_TypeDef *GPIO, uint8_t SCL, uint8_t SDA);
void I2C_Reset(GPIO_TypeDef *GPIO, uint8_t SCL);
void I2C_MasterTransEn(I2C_TypeDef *I2C);
void I2C_MasterRcvEn(I2C_TypeDef *I2C);
void I2C_SlaveTransEn(I2C_TypeDef *I2C);
void I2C_SlaveRcvEn(I2C_TypeDef *I2C);
 
#endif
