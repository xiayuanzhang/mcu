#ifndef _BSP_LED_H
#define _BSP_LED_H



#include "gd32f4xx.h"
#include "systick.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define RCU_LED2  RCU_GPIOC
#define PORT_LED2 GPIOC
#define PIN_LED2 GPIO_PIN_15

void led_gpio_config(void);


#ifdef __cplusplus
}
#endif

#endif

