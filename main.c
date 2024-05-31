#include "TIMER.h"
#include "CLOCK.h"
#include "GPIO.h"
#include "USART.h"
#include "SYS_INIT.h"
#include "stm32f446xx.h"
#include "stm32f4xx.h"

int main (void)
{
	initClock ();
	TIM6Config ();
	UART2_Config ();
	
	
	while (1)
	{
		UART_SendString (USART2,"hello\n");
		Delay_ms (1000);
		
//		uint8_t data = UART2_GetChar ();
//		UART2_SendChar (data);
	}
	
}
