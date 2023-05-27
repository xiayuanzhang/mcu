#include "gd32f4xx.h"
#include "systick.h"

uint32_t locatime_us;


static volatile uint32_t delayus; 
static volatile uint32_t delayms; 


void systick_config(void)
{
	/* setup systick timer for 1000Hz interrupts *///1ms  //1us延时
	if (SysTick_Config(SystemCoreClock/1000000)){
			/* capture error */
			while (1){
			}
	}
	/* configure the systick handler priority */
	NVIC_SetPriority(SysTick_IRQn, 0x00U);
}

void delay_us(uint32_t us)
{
    delayus = us;

    while(0U != delayus){
    }
}

void delay_ms(uint32_t ms)
{
	delay_us(1000*ms);
}




void SysTick_Handler(void)
{
	if(delayus > 0)
		delayus--;
	locatime_us++;
}

uint32_t systick_getms(void)
{
	return locatime_us/1000;
}



int systick_delay(int *timeBuff, int delayms)
{
	//如果时间有所改变,就进行初始话时间
	int time = 0;
	if(*timeBuff == 0)
	{
		*timeBuff = systick_getms();
	}
	time = delayms - (systick_getms() - *timeBuff);
	if(time <= 0)
	{
		*timeBuff = 0; //清空计时器
		return 0; //返回值
	}
	else
	{
		return time;
	}
}
