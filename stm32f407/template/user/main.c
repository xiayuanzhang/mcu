#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "yplot.h"
#include <math.h>


float plot[8];
float x = 0;
uint8_t rx_temp_test[512];
int main(void)
{
    dwt_init();
    usart2_init(115200);
    led_init();


    while (1)
    {
        //		LED0 = 1;
        //	  delay(0.5);
        //		LED0 = 0;
        //		delay(0.5);
        x += 0.01f;
        for (int i = 0; i < 8; i++)
        {
            plot[i] = sin(x + i);
        }
        yplot_plot(plot, 8);

        yplot_rxframe_t rx;
        while (!yplot_analyse(&rx))
        {
            switch (rx.id)
            {
            case YPLOT_ID_SENDCMD:
                memcpy(rx_temp_test, rx.data, rx.len);
                break;
            }
        }
    }
}
