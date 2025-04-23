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

typedef struct {
    int eixo;
    int valor;
} adc_t;

QueueHandle_t xQueueADC;

const int MPU_ADDRESS   = 0x68;
const int I2C_SDA_GPIO  = 4;
const int I2C_SCL_GPIO  = 5;
const int UART_TX_PIN   = 0;
const int UART_RX_PIN   = 1;
const int UART_BAUD     = 115200;

static void mpu6050_reset() {
    uint8_t buf[] = { 0x6B, 0x00 };
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t reg;

    // Accel
    reg = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i*2] << 8) | buffer[i*2 + 1];
    }

    // Gyro
    reg = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i*2] << 8) | buffer[i*2 + 1];
    }

    // Temp
    reg = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);
    *temp = (buffer[0] << 8) | buffer[1];
}

void mpu6050_task(void *p) {
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
    for (int i = 0; i < calib_samples; i++) {
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

    while (1) {
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
        if (fabsf(gyroscope.axis.x) < deadzone) gyroscope.axis.x = 0;
        if (fabsf(gyroscope.axis.y) < deadzone) gyroscope.axis.y = 0;
        if (fabsf(gyroscope.axis.z) < deadzone) gyroscope.axis.z = 0;

        // Acelerômetro (g)
        FusionVector accelerometer = {
            .axis.x = accel[0] / 16384.0f,
            .axis.y = accel[1] / 16384.0f,
            .axis.z = accel[2] / 16384.0f,
        };

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        adc_t adc;
        adc.eixo = 0;  // eixo X

    // Captura o yaw em graus (invertido porque depende da direção de rotação)
    float yaw = -euler.angle.roll;

    // Limita yaw entre -135 e 135 graus (saturação)
    if (yaw > 135.0f) yaw = 135.0f;
    if (yaw < -135.0f) yaw = -135.0f;

    // Mapeia yaw para faixa de -127 a 127
    int yaw_mapped = (int)((yaw / 135.0f) * 127.0f);

    // Garante que não ultrapasse os limites (por segurança)
    if (yaw_mapped > 127) yaw_mapped = 127;
    if (yaw_mapped < -127) yaw_mapped = -127;

    adc.valor = yaw_mapped;

        xQueueSend(xQueueADC, &adc, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void uart_task(void *p) {
    // Inicializa UART
    uart_init(uart0, UART_BAUD);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    adc_t data;
    while (1) {
        if (xQueueReceive(xQueueADC, &data, portMAX_DELAY)) {
            uint8_t bytes[4] = {
                (uint8_t)data.eixo,
                (uint8_t)((data.valor >> 8) & 0xFF),
                (uint8_t)(data.valor & 0xFF),
                0xFF
            };
            uart_write_blocking(uart0, bytes, 4);
        }
    }
}

int main() {
    stdio_init_all();
    xQueueADC = xQueueCreate(8, sizeof(adc_t));
    xTaskCreate(mpu6050_task, "MPU6050_Task", 8192, NULL, 1, NULL);
    xTaskCreate(uart_task,    "UART_Task",    4096, NULL, 1, NULL);
    vTaskStartScheduler();
    while (1);
    return 0;
}
