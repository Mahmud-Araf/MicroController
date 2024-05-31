#include "I2C.h"
#include "TIMER.h"
/**************************************
* I2C communication
**************************************/
void I2C_MasterTransEn(I2C_TypeDef *I2C){
	I2C->CR1 &= ~(1<<10);
}
void I2C_MasterRcvEn(I2C_TypeDef *I2C){
	I2C->CR1 |= (1<<10);
}
void I2C_SlaveTransEn(I2C_TypeDef *I2C){
	I2C->CR1 &= ~(1<<10);
}
void I2C_SlaveRcvEn(I2C_TypeDef *I2C){
	I2C->CR1 |= (1<<10);
}

void I2C_Config(uint8_t address,uint8_t master){
/**
	1. Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings
	2. Configure the clock control registers
	3. Configure the rise time register
	4. Program the I2C_CR1 register to enable the peripheral
	5. Set the START bit in the I2C_CR1 register to generate a Start condition
**/
	// 1. Enable I2C-1 and GPIO clock
	RCC->APB1ENR |= (1<<21); //enable I2C1 clock
	RCC->AHB1ENR |= (1<<1); //Enable GPIOB clock
	//RCC->AHB1ENR |= (1<<2); //Enable GPIOC clock
	//I2C_Bus_Reset(I2C1,GPIOB,8,9);
	//I2C_Clear_Busy_Flag(I2C1,GPIOB,8,9);
	// Configure the GPIO pin for I2C
	I2C_Reset(GPIOB,8);
	GPIOB->MODER |= (2<<16); //pin 8  of GPIOB for Alternate function
	GPIOB->MODER |= (2<<18); //pin 9 of GPIOB for Alternate function
	GPIOB->OTYPER |= (1<<8) ; //Set pins output open drain
	GPIOB->OTYPER |= (1<<9) ;
	// Pin speed configuration
	GPIOB->OSPEEDR |= (3<<16); // set high speed for GPIOB pin 8 
	GPIOB->OSPEEDR |= (3<<18); // set high speed for GPIOB pin 9 
	// Configure pull up register internal for GPIOB pin 8 & 9
	GPIOB->PUPDR |= (1<<16); //Configure pullup register
	GPIOB->PUPDR |= (1<<18);
	// Congfigure the alternate function
	GPIOB->AFR[1] |= (4<<0); //Alternate fuction for PIN 8 of GPIOB
	GPIOB->AFR[1] |= (4<<4); //Alternate fuction for PIN 9 of GPIOB
	
	
	//while(I2C1->SR2 & 1<<1){
	//	I2C1->CR1|= (0<<0);
	// Enable I2C1 reset 
	I2C1->CR1 |= (1<<15);
  //delay_ms(1000);	
	I2C1->CR1 &= ~(1<<15);
	//delay_ms(1000);
	
	//I2C Configure the input clock CR2 register 
	I2C1->CR2 |= (8<<0); // Clock frequency 8 MHz APB1 bus limit is 45MHz
	// Configure clock control register
	I2C1->CCR |= (40<<0); //CCR=[(Tw(sclh)+Tr(scl))/Tpclk1]
	//Configure the rise time register
	I2C1->TRISE |= (9<<0); //5:0 of TRISE register set value [Tr(scl)/Th(pclk1)]+1
	
	//Only 7-bit address allowed
	I2C1->OAR2 &= ~(1<<0);
	// set slave address
	if(master == SLAVE)
	{
		// Assign I2C Address
		I2C1->OAR1 |= (address<<1);
		//enable addressing mode
		I2C1->OAR1 |= (0<<15);
	}
	//Program the I2C_CR1 register to enable the peripheral
	I2C1->CR1|= (1<<0);
	
	if(master == SLAVE) //slave
	{
		I2C1->CR1 |= (1<<10); //set ack for slave receiving                                
	}
//}
}

void I2C_Start(void)
{
	/********
	1. Send Start Condition
	2. Wait for SB to set in SR1 register
	**/
	//ackowledgement
	I2C1->CR1 |= (1<<10);
	// Send Start Condition
	I2C1->CR1 |= (1<<8);
	while(!(I2C1->SR1 & (1<<0)));// wait for start condition generated
	//uint8_t tmp = I2C1->SR1 | I2C1->SR2;  
}

void I2C_Write(uint8_t data)
{
	/****************************
	1. Wait for TxE bit in SR1 to set. It indicate DR is empty
	2. Send data to the DR register
	3. Wait for BTF bit to set for end of the data transmission
	*****************************/
	// 1. Wait for TxE bit in SR1 to set. It indicate DR is empty
	while(!(I2C1->SR1 & (1<<7)));
	I2C1->DR = data;
	while(!(I2C1->SR1 & (1<<2)));
}

void I2C_AddressW(uint8_t address)
{
	I2C1->DR = (address<<1); //master writing enable LSB is zero '0'; master is setting for sending data to the slave
	//uint16_t SR1REG=I2C1->SR1;
	while(!(I2C1->SR1 & (1<<1)));
	uint8_t tmp = I2C1->SR1;
	tmp	|= I2C1->SR2; //read SR1 and SR2 to clear addr bit
}
void I2C_AddressR(uint8_t address)
{
	I2C1->DR = (address<<1)+1; //master reading enable LSB is one '1'; master is setting for sending address to the slave for reading
	while(!(I2C1->SR1 & (1<<1)));
	uint8_t tmp = I2C1->SR1;
	tmp |= I2C1->SR2; //read SR1 and SR2 to clear addr bit
}
void I2C_Stop(void)
{
	I2C1->CR1 |= (1<<9);
}
 void I2C_MultiWrite(uint8_t *data, uint8_t size)
 {
	 while(!(I2C1->SR1 & (1<<7)));
	 for(uint8_t i=0;i<size;i++)
	 {
		 while(!(I2C1->SR1 & (1<<7)));
		 I2C1->DR = data[i];
	 }
	 while(!(I2C1->SR1 & (1<<2)));
 }
 
 uint8_t I2C_MRead()
 {
	 while(!(I2C1->SR1 & (1<<6)));
	 uint8_t tmp = I2C1->DR;
	 return tmp;
 }
 
 uint8_t* I2C_SRead(uint8_t *data)
 {
	 /*******************************
	 1. Check Address match flag 
	 2. Clear if address matched
	 3. Send ack
	 4. Receive data byte
	 5. Send ack
	 6. Receive another data byte
	 7. Recognize stop signal after receive of last byte 
	 8. Release SDA and SCL line
	 *********************************/
	 while(!(I2C1->SR1 & (1<<1)));
	 //clear address flag
	 uint8_t tmp = I2C1->SR1 | I2C1->SR2;
	 while(!(I2C1->SR1 & (1<<6))); // wait for the data bytes to receive RxNE
	 uint8_t _size = I2C1->DR; //size of the data in byte exclusing size byte
	 for(uint8_t i=0;i<_size;i++)
	 {
		 while(!(I2C1->SR1 & (1<<6))); // wait for the data bytes to receive RxNE
		 data[i]=I2C1->DR;
	 }
	 //end of data receive
	 while(!(I2C1->SR1 & (1<<4))); //while not receiving STOPF
	 //clear STOPF and release the SDA and scl line
		I2C1->CR1 |= (1<<9);
	 return data;
 }
 
 void I2C_SWrite(uint8_t data)
 {
	 while(!(I2C1->SR1 & (1<<7)));
	 I2C1->DR = data;
	 while(!(I2C1->SR1 & (1<<2)));
 }
uint8_t I2C_Busy(I2C_TypeDef *I2C){
		if(I2C->SR2 & (1<<1)){
			I2C_Stop();
		}
		if(!(I2C->SR2 & (1<<1))) return 0;
		else return 1;
 }
void I2C_Master_Bus_Release(){
}
void I2C_Bus_Reset(I2C_TypeDef *I2C,GPIO_TypeDef *GPIO, uint8_t SCL, uint8_t SDA){
	//disable I2C1 
	//RCC->AHB1ENR |= (1<<1); //Enable GPIOB clock
	I2C->CR1|= (0<<0); //PE
	GPIO->BSRR |= (1<<SCL);
	GPIO->BSRR |= (1<<SDA);
	GPIO->MODER |= (1<<(SCL<<1)); //general purpose output
	GPIO->OTYPER |= (1<<(SCL)); //output drain
	GPIO->MODER |= (1<<(SDA<<1)); //general purpose output
	GPIO->OTYPER |= (1<<(SDA)); //output drain
	GPIO->OSPEEDR |= (3<<(SCL<<1)); // set high speed 
	GPIO->OSPEEDR |= (3<<(SDA<<1)); // set high speed
	// Read GPIO PIN
	while(!(GPIO->IDR & (1<SDA))){
		while(!(GPIO->IDR & (1<SCL)));
		delay_us(10);
		
		/*Pull Low */
		GPIO->BSRR |= 1<<SCL<<16;
		delay_us(10);
		
		/* release high again */
		GPIO->BSRR |= (1<<SCL);
		delay_us(10);
	}
		/* generate start stop condition */
	for(uint8_t i=0;i<30;i++){
	//GPIO->BSRR |= (1<<SDA<<16);
	//delay_us(10);
	GPIO->BSRR |= 1<<SCL<<16;
	delay_us(10);	
	GPIO->BSRR |= (1<<SCL);
	delay_us(10);
//	GPIO->BSRR |= (1<<SDA);
//	delay_us(10);
	}
	GPIO->BSRR |= (1<<SDA<<16);
	delay_us(10);
	GPIO->BSRR |= 1<<SCL<<16;
	delay_us(10);	
	I2C->CR1=0;
}
void I2C_Clear_Busy_Flag(I2C_TypeDef *I2C,GPIO_TypeDef *GPIO, uint8_t SCL, uint8_t SDA){
	I2C->CR1 &= ~(1<<0);
	GPIO->BSRR |= (1<<SCL);
	GPIO->BSRR |= (1<<SDA);
	GPIO->MODER |= (1<<(SCL<<1)); //general purpose output
	GPIO->OTYPER |= (1<<(SCL)); //output open drain
	GPIO->MODER |= (1<<(SDA<<1)); //general purpose output
	GPIO->OTYPER |= (1<<(SDA)); //output open drain
	GPIO->OSPEEDR |= (3<<(SCL<<1)); // set high speed 
	GPIO->OSPEEDR |= (3<<(SDA<<1)); // set high speed
	while(!(GPIO->IDR & (1<SCL)));
	while(!(GPIO->IDR & (1<SDA)));
	
	GPIO->BSRR |= (1<<SDA<<16);
	while(GPIO->IDR & (1<SDA));
	
	GPIO->BSRR |= (1<<SCL<<16);
	while(GPIO->IDR & (1<SCL));
	
	GPIO->BSRR |= (1<<SCL);
	while(!(GPIO->IDR & (1<SCL)));
	
	GPIO->BSRR |= (1<<SDA);
	while(!(GPIO->IDR & (1<SDA)));
	
	GPIO->MODER |= (2<<(SCL<<1)); //general purpose output
	GPIO->MODER |= (2<<(SDA<<1));
	
	if(SCL>7)
	GPIO->AFR[1] |= (4<<(SCL*4));
	else GPIO->AFR[0] |= (4<<(SCL*4));
  if(SDA>7)	
	GPIO->AFR[1] |= (4<<(SDA*4)); 
	else GPIO->AFR[0] |= (4<<(SDA*4));
	
	I2C->CR1 |= (1<<15);
	delay_us(10);
	
	I2C->CR1 &= ~(1<<15);
	delay_us(10);
}

void I2C_Reset(GPIO_TypeDef *GPIO, uint8_t SCL){
	GPIO->MODER |= (1<<SCL<<1);
	GPIO->OTYPER |= (1<<SCL);
	GPIO->OSPEEDR |= (3<<SCL<<1);	
	for(uint8_t i=0;i<10;i++){  //changed by me previously it was 10
		GPIO->BSRR |= (1<<SCL);
		delay_ms(100);
		GPIO->BSRR |= ((1<<SCL)<<16);
		delay_ms(100);
	}
}