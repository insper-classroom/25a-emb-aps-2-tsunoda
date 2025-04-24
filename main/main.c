#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

#define ADC_MAX 4095
#define ADC_MIN 0
#define ACELERADOR_MIN 11
#define ACELERADOR_MAX 3560
#define FREIO_MIN 700
#define FREIO_MAX 2700

// Função para mapear valores do ADC para a escala de 0 a 100
uint8_t map_adc_to_percentage(uint16_t adc_value, uint16_t min, uint16_t max)
{
    if (adc_value < min)
    {
        return 0;
    }
    else if (adc_value > max)
    {
        return 100;
    }
    return ((adc_value - min) * 100) / (max - min);
}

int main()
{
    stdio_init_all();

    adc_init();
    adc_gpio_init(26); // GPIO26 = ADC0
    adc_gpio_init(27); // GPIO27 = ADC1

    while (1)
    {
        uint16_t acelerador_raw, freio_raw;
        uint8_t acelerador, freio;

        // Leitura do freio no ADC0
        adc_select_input(0);
        freio_raw = adc_read();
        freio = map_adc_to_percentage(freio_raw, FREIO_MIN, FREIO_MAX);

        // Leitura do acelerador no ADC1
        adc_select_input(1);
        acelerador_raw = adc_read();
        acelerador = map_adc_to_percentage(acelerador_raw, ACELERADOR_MIN, ACELERADOR_MAX);

        printf("Acelerador: %d%% (Raw: %d)\tFreio: %d%% (Raw: %d)\n", acelerador, acelerador_raw, freio, freio_raw);

        sleep_ms(500);
    }
}