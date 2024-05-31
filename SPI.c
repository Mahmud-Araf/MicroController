#include "SPI.h"
#include "TIMER.h"
#include <stm32f446xx.h>
/**
Write to the SPI_CR1 register:
a) Configure the serial clock baud rate using the BR[2:0] bits (Note: 3).
b) Configure the CPOL and CPHA bits combination to define one of the four
relationships between the data transfer and the serial clock. (Note: 2 - except the
case when CRC is enabled at TI mode).
c) Select simplex or half-duplex mode by configuring RXONLY or BIDIMODE and BIDIOE (RXONLY and BIDIMODE can't be set at the same time).
d) Configure the LSBFIRST bit to define the frame format (Note: 2).
e) Configure the CRCEN and CRCEN bits if CRC is needed (while SCK clock signal
is at idle state).
f) Configure SSM and SSI (Note: 2).
g) Configure the MSTR bit (in multimaster NSS configuration, avoid conflict state on NSS if master is configured to prevent MODF error).
h) Set the DFF bit to configure the data frame format (8 or 16 bits).
3. Write to SPI_CR2 register:
a) Configure SSOE (Note: 1 & 2).
b) Set the FRF bit if the TI protocol is required.
4. Write to SPI_CRCPR register: Configure the CRC polynomial if needed.
5. Write proper DMA registers: Configure DMA
**/
void SPI_Config(void) {
    // Enable clock for SPI1
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Enable clock for GPIOA (NSS pin is on GPIOA)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Configure GPIOA pin 5 (SCK) as alternate function
    GPIOA->MODER &= ~(3U << (5 * 2)); // Clear mode bits for PA5
    GPIOA->MODER |= (2U << (5 * 2));  // Set PA5 as alternate function mode


    //Configure GPIOA pin 6 (MISO) as alternate function
    GPIOA->MODER &= ~(3U << (6 * 2)); // Clear mode bits for PA6
    GPIOA->MODER |= (2U << (6 * 2));  // Set PA6 as alternate function mode

    // Configure GPIOA pin 7 (MOSI) as alternate function
    GPIOA->MODER &= ~(3U << (7 * 2)); // Clear mode bits for PA7
    GPIOA->MODER |= (2U << (7 * 2));  // Set PA7 as alternate function mode


    // Set PA5, PA6, and PA7 to high speed
    GPIOA->OSPEEDR |= (3U << (5 * 2)); // Set high speed for PA5
    GPIOA->OSPEEDR |= (3U << (6 * 2)); // Set high speed for PA6
    GPIOA->OSPEEDR |= (3U << (7 * 2)); // Set high speed for PA7





    // Configure GPIOA pin 4 (NSS) as alternate function
    GPIOA->MODER &= ~(3U << (4 * 2)); // Clear mode bits for PA4
    GPIOA->MODER |= (1U << (4 * 2));  // Set PA4 as general purpose output mode

    // Set PA4 to push-pull mode
    GPIOA->OTYPER &= ~(1U << 4); // Clear OTYPER bit for PA4

    // Set PA4 to high speed
    GPIOA->OSPEEDR |= (3U << (4 * 2)); // Set high speed for PA4

    // Set PA4 to no pull-up/pull-down
    GPIOA->PUPDR &= ~(3U << (4 * 2)); // Clear PUPDR bits for PA4

    // Set Alert function for PA5, PA6, and PA7
    GPIOA->AFR[0] &= ~(0xFU << (5 * 4)); // Clear alternate function bits for PA5
    GPIOA->AFR[0] |= (5U << (5 * 4));    // Set alternate function to AF5 for PA5

    GPIOA->AFR[0] &= ~(0xFU << (6 * 4)); // Clear alternate function bits for PA6
    GPIOA->AFR[0] |= (5U << (6 * 4));    // Set alternate function to AF5 for PA6

    GPIOA->AFR[0] &= ~(0xFU << (7 * 4)); // Clear alternate function bits for PA7
    GPIOA->AFR[0] |= (5U << (7 * 4));    // Set alternate function to AF5 for PA7


    /**
     * Write to the SPI_CR1 register:
     * a) Configure the serial clock baud rate using the BR[2:0] bits (Note: 3).
     * b) Configure the CPOL and CPHA bits combination to define one of the four
     * relationships between the data transfer and the serial clock. (Note: 2 - except the
     * case when CRC is enabled at TI mode).
     **/

    // Configure the baud rate (BR[2:0] bits) to 5MHz
    SPI1->CR1 |= SPI_CR1_BR_2;

    // Configure CPHA and CPOL (Phase and Polarity) to 0
    SPI1->CR1 &= ~SPI_CR1_CPHA; // CPHA = 0
    SPI1->CR1 &= ~SPI_CR1_CPOL; // CPOL = 0

    // Configure as Master
    SPI1->CR1 |= SPI_CR1_MSTR;

    // Configure to send MSB first
    SPI1->CR1 &= ~SPI_CR1_LSBFIRST;

    // Enable Software Slave Management (SSM) and set SSI
    // SPI1->CR1 |= SPI_CR1_SSM; // SSM = 1
    // SPI1->CR1 |= SPI_CR1_SSI; // SSI = 1

    // Optional configurations: disable bidirectional mode and RX only mode
    // SPI1->CR1 &= ~SPI_CR1_BIDIMODE;
    // SPI1->CR1 &= ~SPI_CR1_RXONLY;

    // Set data frame format to 8 bits (optional, if needed)
    // SPI1->CR1 &= ~(1U << 11);

    // Configure SPI_CR2 register: default to 0 (all optional settings disabled)
    SPI1->CR2 = 0;

    // Enable SPI1 peripheral
    SPI1->CR1 |= SPI_CR1_SPE;
}



void SPI_NSS_ON(void)
{
	GPIOA->BSRR|=(1<<4)<<16 ; //changed by me 
}
void SPI_NSS_OFF(void)
{
	GPIOA->BSRR |= (1<<4); //changed by me
}


uint8_t SPI1SendByte(uint8_t data) 
{
	while (!(SPI1->SR & SPI_SR_TXE));    // Verify TXE
	SPI1->DR=data;										   // Send data to SPI1
	while (!(SPI1->SR & SPI_SR_RXNE));   // Is receive data (normally garbage) 
	return (uint8_t)SPI1->DR;		         	// Garbage
}

void SPI1_WriteReg(uint8_t address, uint8_t value) 
{
	SPI_NSS_ON();											// CS_Low
	SPI1SendByte(address&0x7F);						// Send Address
	SPI1SendByte(value);							// Send value to the address for write
	SPI_NSS_OFF();										// CS_HIGH
}

uint8_t SPI1_ReadReg(uint8_t address) 
{
	uint8_t	val;

	SPI_NSS_ON();										// CS_Low
	SPI1SendByte(address|0x80);					// Send address
	val = SPI1SendByte(address);				// send for receive data 	
	SPI_NSS_OFF();									// CS_HIGH
	return val;
}
void SPI1_MulReadReg(uint8_t address,uint8_t *buff,uint8_t size) 
{
	uint8_t tmp;
	SPI_NSS_ON();										// CS_Low
	while (!(SPI1->SR & SPI_SR_TXE));    // Verify TXE
	SPI1->DR=(address|0x80);										   // Send data to SPI1
	while (!(SPI1->SR & SPI_SR_RXNE));   // Is receive data (normally garbage) 
	tmp=(uint8_t)SPI1->DR;		         	// Garbage
	delay_ms(5);
	while(size--)
	{
		while (!(SPI1->SR & SPI_SR_TXE));
		SPI1->DR=(0x00);
		while (!(SPI1->SR & SPI_SR_RXNE));
		tmp = (uint8_t)SPI1->DR;
		*buff++=tmp;
	}	
	SPI_NSS_OFF();									// CS_HIGH
}
