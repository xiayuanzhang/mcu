#include "led.h"
#include "delay.h"

volatile float dwt  = 0;
volatile float dwt2  = 0;
volatile float dwt3  = 0;



int main(void)
{
	led_init();
	dwt_init();
	
	while(1)
	{
		time_test_run();
		delay_us(1);
		dwt = time_test_get();
		
	}
}

