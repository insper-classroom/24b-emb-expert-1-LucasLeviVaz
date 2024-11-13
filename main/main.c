#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "servo.h"



#define SERVO_PIN_ONE 2
#define SERVO_PIN_TWO 3
#define VRX_PIN 26 // Joystick X no ADC0 (GPIO 26)
#define VRY_PIN 27 // Joystick Y no ADC1 (GPIO 27)
#define LDR_PIN 15

//#define UART_TX_PIN 0
//#define UART_RX_PIN 1

QueueHandle_t xQueueAdc;
QueueHandle_t yQueueAdc;

typedef struct adc {
    int axis;
    int val;
} adc_t;

void sensor_task(void *p) {
    int currentMillisOne = 400;
    int currentMillisTwo = 1600;

    while (1) {
        adc_t data;

        // Ler o valor da fila X (Eixo X)
        if (xQueueReceive(xQueueAdc, &data, portMAX_DELAY)) {
            currentMillisOne = data.val; // Atualiza o valor do servo X
            setMillis(SERVO_PIN_ONE, currentMillisOne);
        }

        // Ler o valor da fila Y (Eixo Y)
        if (xQueueReceive(yQueueAdc, &data, portMAX_DELAY)) {
            currentMillisTwo = data.val; // Atualiza o valor do servo Y
            setMillis(SERVO_PIN_TWO, currentMillisTwo);
        }

        vTaskDelay(pdMS_TO_TICKS(5)); // Delay para evitar uso excessivo de CPU
    }
}

void joystick_task(void *p) {
    adc_t data;

    while (1) {
        // Leitura do eixo Y
        adc_select_input(1);  // Seleciona o canal 1 para o eixo Y
        int raw_value_y = adc_read();
        
        // Mapeando o valor do joystick para o intervalo de controle do servo
        data.val = (raw_value_y * (2400 - 400)) / 4095 + 400;  // Mapeamento de 0-4095 para 400-2400
        data.axis = 1;


        if (data.val < -900 && data.val > 900){
            data.val = 0;
        }

        printf("Y Axis: %d\n", data.val);  // Para depuração

        xQueueSend(yQueueAdc, &data, portMAX_DELAY);

        // Leitura do eixo X
        adc_select_input(0);  // Seleciona o canal 0 para o eixo X
        int raw_value_x = adc_read();

        // Mapeando o valor do joystick para o intervalo de controle do servo
        data.val = (raw_value_x * (2400 - 400)) / 4095 + 400;  // Mapeamento de 0-4095 para 400-2400
        data.axis = 0;

        if (data.val < -900 && data.val > 900){
            data.val = 0;
        }
        printf("X Axis: %d\n", data.val);  // Para depuração

        xQueueSend(xQueueAdc, &data, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(5));  // Delay para evitar sobrecarga
    }
}


void ldr_task(void *pvParameters) {
    // Inicializa o pino ADC

    while (true) {
        // Leitura do valor do LDR
        uint16_t ldr_value = adc_read();

        // Imprime o valor lido no console
        printf("Valor do LDR: %d\n", ldr_value);

        // Aguarda um tempo antes de fazer uma nova leitura (1000ms = 1 segundo)
        vTaskDelay(pdMS_TO_TICKS(5)); 
    }
}

int main() {
    stdio_init_all();
    adc_init();
    
    xQueueAdc = xQueueCreate(32, sizeof(adc_t));
    yQueueAdc = xQueueCreate(32, sizeof(adc_t));

    setServo(SERVO_PIN_ONE, 400);
    setServo(SERVO_PIN_TWO, 1600);

    adc_gpio_init(VRX_PIN);
    adc_gpio_init(VRY_PIN);

    adc_gpio_init(LDR_PIN);

    xTaskCreate(joystick_task, "joystick_task", 4096, NULL, 1, NULL);
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 1, NULL);
    xTaskCreate(ldr_task, "ldr_ask", 256, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1);  // Esse loop nunca deve ser alcançado, pois o FreeRTOS toma controle
}
