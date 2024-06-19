#include "led.h"
#include "delay.h"
#include "w25qxx.h"
#include "lfs_port.h"
#include "lfs_wrtest.h"


int main(void)
{
	led_init();
	dwt_init();

	
    mlfs_init();
	
	mlfs_wrtest();
	while(1)
	{
		LED_MCU_TOG;
		delay_ms(500);
	}
}

