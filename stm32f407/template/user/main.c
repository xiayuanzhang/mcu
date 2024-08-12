#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "beep.h"
#include "key.h"




int main(void)
{ 
	dwt_init();		
	uart_init(115200);	
	led_init();	
	while(1)
	{
		LED0 = 1;
	  delay(0.5);
		LED0 = 0;
		delay(0.5);	
	}
}

