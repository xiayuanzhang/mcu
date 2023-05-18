#include "bsp_led.hpp"




void LEDClass::init(void)
{
  /* 使能时钟 */
	rcu_periph_clock_enable(RCU_LED2);
	/* 配置GPIO的模式 */
	gpio_mode_set(PORT_LED2,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,PIN_LED2);
	/* 配置GPIO的输出 */
	gpio_output_options_set(PORT_LED2,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,PIN_LED2);
}


