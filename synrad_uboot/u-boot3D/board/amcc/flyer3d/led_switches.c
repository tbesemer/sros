





#include <common.h>
#include <ppc4xx.h>
#include <asm/processor.h>
#include <asm/io.h>
#include <status_led.h>
#include <asm/gpio.h>

void __led_init (led_id_t mask, int state)
{
    // Intitalize the LED and set to requested state
    //int	val = GET_LEDS;
    int out_val;
    out_val = (state == STATUS_LED_ON)?GPIO_OUT_1:GPIO_OUT_0;
    gpio_config(mask, GPIO_OUT, 0,out_val)
}

void __led_set (led_id_t mask, int state)
{
    int out_val;
    out_val = (state == STATUS_LED_ON)?GPIO_OUT_1:GPIO_OUT_0;
    gpio_write_bit(mask, int val)
}
/*
void __led_toggle (led_id_t mask)
{
    int	val = GET_LEDS;

    val ^= mask;
    set_leds (val);
}
*/
