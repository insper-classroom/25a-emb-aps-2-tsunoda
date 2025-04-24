#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"
#include "Fusion.h"

#define SAMPLE_PERIOD (0.01f)

typedef struct
{
    int eixo;
    int valor;
} adc_t;

QueueHandle_t xQueueADC;
SemaphoreHandle_t xSemaphore_up;
SemaphoreHandle_t xSemaphore_down;
SemaphoreHandle_t xSemaphore_ovtk;
SemaphoreHandle_t xSemaphore_drs;

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;
const int UART_TX_PIN = 0;
const int UART_RX_PIN = 1;
const int UART_BAUD = 115200;
const int UPSHIFT_PIN = 20;
const int DOWNSHIFT_PIN = 22;
const int OVTK_PIN = 7;
const int DRS_PIN = 6;
const int LED_OVTK_PIN = 16;
const int LED_DRS_PIN = 2;

void btn_callback(uint gpio, uint32_t events)
{
    if (events == 0x4 && gpio == UPSHIFT_PIN)
    { // fall edge
        xSemaphoreGiveFromISR(xSemaphore_up, 0);
    }
    if (events == 0x4 && gpio == DOWNSHIFT_PIN)
    { // fall edge
        xSemaphoreGiveFromISR(xSemaphore_down, 0);
    }
    if (events == 0x4 && gpio == OVTK_PIN)
    { // fall edge
        xSemaphoreGiveFromISR(xSemaphore_ovtk, 0);
    }
    if (events == 0x4 && gpio == DRS_PIN)
    { // fall edge
        xSemaphoreGiveFromISR(xSemaphore_drs, 0);
    }
}

static void mpu6050_reset()
{
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp)
{
    uint8_t buffer[6];
    uint8_t reg;

    // Accel
    reg = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++)
    {
        accel[i] = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
    }

    // Gyro
    reg = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++)
    {
        gyro[i] = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
    }

    // Temp
    reg = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);
    *temp = (buffer[0] << 8) | buffer[1];
}

void upshift_task(void *p)
{
    gpio_init(UPSHIFT_PIN);
    gpio_set_dir(UPSHIFT_PIN, GPIO_IN);
    gpio_pull_up(UPSHIFT_PIN);
    gpio_set_irq_enabled_with_callback(UPSHIFT_PIN, GPIO_IRQ_EDGE_FALL, true,
                                       &btn_callback);

    while (true)
    {
        if (xSemaphoreTake(xSemaphore_up, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            // printf("Upshift\n");
            adc_t adc;
            adc.eixo = 1;
            adc.valor = 1;
            xQueueSend(xQueueADC, &adc, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void downshift_task(void *p)
{
    gpio_init(DOWNSHIFT_PIN);
    gpio_set_dir(DOWNSHIFT_PIN, GPIO_IN);
    gpio_pull_up(DOWNSHIFT_PIN);
    gpio_set_irq_enabled_with_callback(DOWNSHIFT_PIN, GPIO_IRQ_EDGE_FALL, true,
                                       &btn_callback);

    while (true)
    {
        if (xSemaphoreTake(xSemaphore_down, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            // printf("Downshift\n");
            adc_t adc;
            adc.eixo = 2;
            adc.valor = 1;
            xQueueSend(xQueueADC, &adc, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void ovtk_task(void *p)
{
    gpio_init(OVTK_PIN);
    gpio_set_dir(OVTK_PIN, GPIO_IN);
    gpio_pull_up(OVTK_PIN);
    gpio_set_irq_enabled_with_callback(OVTK_PIN, GPIO_IRQ_EDGE_FALL, true,
                                       &btn_callback);

    while (true)
    {
        if (xSemaphoreTake(xSemaphore_ovtk, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            printf("Overtake\n");
            adc_t adc;
            adc.eixo = 3;
            adc.valor = 1;
            xQueueSend(xQueueADC, &adc, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void drs_task(void *p)
{
    gpio_init(DRS_PIN);
    gpio_set_dir(DRS_PIN, GPIO_IN);
    gpio_pull_up(DRS_PIN);
    gpio_set_irq_enabled_with_callback(DRS_PIN, GPIO_IRQ_EDGE_FALL, true,
                                       &btn_callback);

    while (true)
    {
        if (xSemaphoreTake(xSemaphore_drs, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            printf("DRS\n");
            adc_t adc;
            adc.eixo = 4;
            adc.valor = 1;
            xQueueSend(xQueueADC, &adc, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void led_drs_task(void *p)
{
    gpio_init(LED_DRS_PIN);
    gpio_set_dir(LED_DRS_PIN, GPIO_OUT);
    gpio_put(LED_DRS_PIN, 1);
    while (1)
    {
        if (xSemaphoreTake(xSemaphore_drs, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            gpio_put(LED_DRS_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_put(LED_DRS_PIN, 0);
        }
    }
}

void led_ovtk_task(void *p)
{
    gpio_init(LED_OVTK_PIN);
    gpio_set_dir(LED_OVTK_PIN, GPIO_OUT);
    gpio_put(LED_OVTK_PIN, 1);
    while (1)
    {
        if (xSemaphoreTake(xSemaphore_ovtk, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            gpio_put(LED_OVTK_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_put(LED_OVTK_PIN, 0);
        }
    }
}

void mpu6050_task(void *p)
{
    // Inicializa I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    // Reset do MPU
    mpu6050_reset();

    int16_t accel[3], gyro_raw[3], temp;
    FusionAhrs ahrs;

    // Calibração de offset do giroscópio
    float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;
    const int calib_samples = 500;
    for (int i = 0; i < calib_samples; i++)
    {
        mpu6050_read_raw(accel, gyro_raw, &temp);
        gyro_offset_x += gyro_raw[0] / 131.0f;
        gyro_offset_y += gyro_raw[1] / 131.0f;
        gyro_offset_z += gyro_raw[2] / 131.0f;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    gyro_offset_x /= calib_samples;
    gyro_offset_y /= calib_samples;
    gyro_offset_z /= calib_samples;

    FusionAhrsInitialise(&ahrs);

    while (1)
    {
        // Lê dados brutos
        mpu6050_read_raw(accel, gyro_raw, &temp);

        // Giroscópio (°/s) menos offset
        FusionVector gyroscope = {
            .axis.x = gyro_raw[0] / 131.0f - gyro_offset_x,
            .axis.y = gyro_raw[1] / 131.0f - gyro_offset_y,
            .axis.z = gyro_raw[2] / 131.0f - gyro_offset_z,
        };
        // Deadzone
        const float deadzone = 0.1f;
        if (fabsf(gyroscope.axis.x) < deadzone)
            gyroscope.axis.x = 0;
        if (fabsf(gyroscope.axis.y) < deadzone)
            gyroscope.axis.y = 0;
        if (fabsf(gyroscope.axis.z) < deadzone)
            gyroscope.axis.z = 0;

        // Acelerômetro (g)
        FusionVector accelerometer = {
            .axis.x = accel[0] / 16384.0f,
            .axis.y = accel[1] / 16384.0f,
            .axis.z = accel[2] / 16384.0f,
        };

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        adc_t adc;
        adc.eixo = 0; // eixo X

        // Captura o yaw em graus (invertido porque depende da direção de rotação)
        float yaw = -euler.angle.roll;

        // Limita yaw entre -135 e 135 graus (saturação)
        if (yaw > 135.0f)
            yaw = 135.0f;
        if (yaw < -135.0f)
            yaw = -135.0f;

        // Mapeia yaw para faixa de -127 a 127
        int yaw_mapped = (int)((yaw / 135.0f) * 127.0f);

        // Garante que não ultrapasse os limites (por segurança)
        if (yaw_mapped > 127)
            yaw_mapped = 127;
        if (yaw_mapped < -127)
            yaw_mapped = -127;

        adc.valor = yaw_mapped;

        xQueueSend(xQueueADC, &adc, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void uart_task(void *p)
{
    // Inicializa UART
    uart_init(uart0, UART_BAUD);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    adc_t data;
    while (1)
    {
        if (xQueueReceive(xQueueADC, &data, portMAX_DELAY))
        {
            uint8_t bytes[4] = {
                (uint8_t)data.eixo,
                (uint8_t)((data.valor >> 8) & 0xFF),
                (uint8_t)(data.valor & 0xFF),
                0xFF};
            uart_write_blocking(uart0, bytes, 4);
        }
    }
}

void pedal_task(void *p)
{
    const int ACELERADOR_MIN = 11;
    const int ACELERADOR_MAX = 3560;
    const int FREIO_MIN = 700;
    const int FREIO_MAX = 2700;

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
        freio = ((freio_raw < FREIO_MIN) ? 0 : (freio_raw > FREIO_MAX) ? 100
                                                                       : ((freio_raw - FREIO_MIN) * 100) / (FREIO_MAX - FREIO_MIN));

        // Leitura do acelerador no ADC1
        adc_select_input(1);
        acelerador_raw = adc_read();
        acelerador = ((acelerador_raw < ACELERADOR_MIN) ? 0 : (acelerador_raw > ACELERADOR_MAX) ? 100
                                                                                                : ((acelerador_raw - ACELERADOR_MIN) * 100) / (ACELERADOR_MAX - ACELERADOR_MIN));

        // Envia os valores via UART
        uint8_t bytes[4] = {
            0x01, // Identificador para R2 (acelerador)
            acelerador,
            0x02, // Identificador para L2 (freio)
            freio};
        uart_write_blocking(uart0, bytes, 4);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int main()
{
    stdio_init_all();
    xSemaphore_up = xSemaphoreCreateBinary();
    xSemaphore_down = xSemaphoreCreateBinary();
    xSemaphore_ovtk = xSemaphoreCreateBinary();
    xSemaphore_drs = xSemaphoreCreateBinary();
    xQueueADC = xQueueCreate(8, sizeof(adc_t));
    xTaskCreate(mpu6050_task, "MPU6050_Task", 8192, NULL, 1, NULL);
    xTaskCreate(upshift_task, "Upshift_Task", 4096, NULL, 1, NULL);
    xTaskCreate(downshift_task, "Downshift_Task", 4096, NULL, 1, NULL);
    xTaskCreate(ovtk_task, "Overtake_Task", 4096, NULL, 1, NULL);
    xTaskCreate(led_ovtk_task, "LED_Overtake_Task", 4096, NULL, 1, NULL);
    xTaskCreate(led_drs_task, "LED_DRS_Task", 4096, NULL, 1, NULL);
    xTaskCreate(drs_task, "DRS_Task", 4096, NULL, 1, NULL);
    xTaskCreate(uart_task, "UART_Task", 4096, NULL, 1, NULL);
    xTaskCreate(pedal_task, "Pedal_Task", 4096, NULL, 1, NULL);
    vTaskStartScheduler();
    while (1)
        ;
    return 0;
}
