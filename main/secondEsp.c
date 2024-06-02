#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "sdkconfig.h"
#include "esp_timer.h"

#define TRIG_PIN 2
#define ECHO_PIN 3
#define BUZZER_PIN 9

#define GREEN_LED_GPIO 4
#define YELLOW_LED_GPIO 7
#define RED_LED_GPIO 5

void setup(void) {
    gpio_reset_pin(GREEN_LED_GPIO);
    gpio_reset_pin(YELLOW_LED_GPIO);
    gpio_reset_pin(RED_LED_GPIO);

    // Set TRIG_PIN as output
    esp_rom_gpio_pad_select_gpio(TRIG_PIN);
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(TRIG_PIN, 0);

    // Set ECHO_PIN as input
    esp_rom_gpio_pad_select_gpio(ECHO_PIN);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);

    /* Set the GPIOs as a push/pull output */
    gpio_set_direction(GREEN_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(YELLOW_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(RED_LED_GPIO, GPIO_MODE_OUTPUT);
}

void app_main(void) {
    // Measure distance from ultrasonic sensor HC-SR04 using ESP32c3
    // Ultrasonic sensor HC-SR04 has 4 pins: VCC, GND, TRIG, ECHO --> 5V, GND, GPIO 2, GPIO 3

    setup();

    while (1) {
        gpio_set_level(TRIG_PIN, 1);
        esp_rom_delay_us(10);
        gpio_set_level(TRIG_PIN, 0);

        int timeout = 0;
        while (gpio_get_level(ECHO_PIN) == 0 && timeout < 10000) {
            timeout++;
            esp_rom_delay_us(1); // microsecond delay
        }
        if (timeout >= 10000) {
            printf("Timeout: No echo received\n");
            continue; 
        }

        int start = esp_timer_get_time();

        timeout = 0;
        while (gpio_get_level(ECHO_PIN) == 1 && timeout < 10000) {
            timeout++;
            esp_rom_delay_us(1); // microsecond delay
        }
        if (timeout >= 10000) {
            printf("Timeout: Echo not received after pulse\n");
            continue;
        }

        int end = esp_timer_get_time();

        float distance = (end - start) / 58.0;
        // if distance is 10 make the buzzer ring
        if (distance <= 10) {
            gpio_set_level(BUZZER_PIN, 1);
        } else {
            gpio_set_level(BUZZER_PIN, 0);
        }
        printf("Distance: %.2f cm\n", distance);

        // the closer the object, the better
        if (distance <= 25) {
            printf("ACCURATE!\n");
            gpio_set_level(GREEN_LED_GPIO, 1);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            gpio_set_level(GREEN_LED_GPIO, 0);
        } else if (distance <= 35) {
            printf("NOT QUITE ACCURATE, COME CLOSER FOR BETTER READINGS!\n");
            gpio_set_level(YELLOW_LED_GPIO, 1);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            gpio_set_level(YELLOW_LED_GPIO, 0);
        } else {
            printf("DEFINETLY NOT ACCURATE, COME CLOSER FOR BETTER READINGS!\n");
            gpio_set_level(RED_LED_GPIO, 1);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            gpio_set_level(RED_LED_GPIO, 0);
            
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
