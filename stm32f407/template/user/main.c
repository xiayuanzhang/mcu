#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "beep.h"
#include "yplot.h"
#include <math.h>

uint8_t rxbuffer[1024];
uint8_t txbuffer[256];

yplot_config_t yplot_config = {
    .write = usart2_write_bytes,
    .rxbuffer = rxbuffer,
    .rxbuffer_len = 1024,
    .txbuffer = txbuffer,
    .txbuffer_len = 256,
};

float plot[8];
float x = 0;
int main(void)
{
    dwt_init();
    usart2_init(115200);
    led_init();

    yplot_init(&yplot_config);

    while (1)
    {
        //		LED0 = 1;
        //	    delay(0.5);
        //		LED0 = 0;
        //		delay(0.5);
        x += 0.1f;
        for (int i = 0; i < 8; i++)
        {
            plot[i] = sin(x + i);
        }
        yplot_plot(plot, 8);
    }
}
