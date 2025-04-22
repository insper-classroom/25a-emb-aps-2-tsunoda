#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

const uint SEL_A_4051 = 13;
const uint SEL_B_4051 = 12;
const uint SEL_C_4051 = 11;
const uint INH_4051   = 10;

void polling_adc_init(void) {
    gpio_init(SEL_A_4051);
    gpio_set_dir(SEL_A_4051, GPIO_OUT);

    gpio_init(SEL_B_4051);
    gpio_set_dir(SEL_B_4051, GPIO_OUT);

    gpio_init(SEL_C_4051);
    gpio_set_dir(SEL_C_4051, GPIO_OUT);

    gpio_init(INH_4051);
    gpio_set_dir(INH_4051, GPIO_OUT);
    gpio_put(INH_4051, 1); // desabilita por padrão
}

void select_4051_channel(uint channel) {
    gpio_put(SEL_A_4051, channel & 0x01);
    gpio_put(SEL_B_4051, (channel >> 1) & 0x01);
    gpio_put(SEL_C_4051, (channel >> 2) & 0x01);
}

int main() {
    stdio_init_all();

    polling_adc_init();

    adc_init();
    adc_gpio_init(27);    // GPIO27 = ADC1
    adc_select_input(1);  // Canal 1 do ADC

    while (1) {
        uint16_t acelerador, freio;

        // Canal 0 = Acelerador
        gpio_put(INH_4051, 1);
        select_4051_channel(0);
        sleep_ms(2);
        gpio_put(INH_4051, 0);
        sleep_ms(2);
        acelerador = adc_read();

        // Canal 1 = Freio
        gpio_put(INH_4051, 1);
        select_4051_channel(1);
        sleep_ms(2);
        gpio_put(INH_4051, 0);
        sleep_ms(2);
        freio = adc_read();

        printf("Acelerador: %d\tFreio: %d\n", acelerador, freio);

        sleep_ms(500);
    }
}