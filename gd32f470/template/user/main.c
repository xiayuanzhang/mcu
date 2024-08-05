#include "led.h"
#include "delay.h"

volatile float dwt  = 0;
volatile float dwt2  = 0;
volatile float dwt3  = 0;

const uint32_t ys = 100;
uint8_t test[ys];


int main(void)
{
	led_init();
	dwt_init();
	
	while(1)
	{
		time_test_run();
		delay_us(1);
		dwt = time_test_gets();
		
		time_test_run();
		delay_ms(1000);
		dwt2 = time_test_gets();
		
		time_test_run();
		delay(1);
		dwt3 = time_test_gets();
	}
}

