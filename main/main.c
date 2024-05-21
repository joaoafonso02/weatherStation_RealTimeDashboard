#include "BME280.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

#define SDA_PIN 2
#define SCL_PIN 6
#define CLK_SPEED 100000 // 100 kHz

static const char *TAG = "main";

void read_sensor_task(void *pvParameters) {
    i2c_master_bus_handle_t busHandle = NULL;
    i2c_master_dev_handle_t sensorHandle = NULL;

    if (bme280_init(&busHandle, &sensorHandle, BME280_I2C_ADDRESS, SDA_PIN, SCL_PIN, CLK_SPEED) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor");
        return;
    }

    uint8_t status = 0;
    if (bme280_read_status(sensorHandle, &status) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor status");
        return;
    }

    ESP_LOGI(TAG, "Sensor status: 0x%02X", status);

    int32_t raw_temp = 0;
    int32_t raw_humidity = 0;
    int32_t raw_pressure = 0;

    while (1) {
        if (bme280_read_raw_temp(sensorHandle, &raw_temp) == ESP_OK) {
            int32_t temperature = BME280_compensate_T_int32(raw_temp);
            float temperature_celsius = temperature / 100.0;
            ESP_LOGI(TAG, "Temperature: %.2f°C", temperature_celsius);
        } else {
            ESP_LOGE(TAG, "Failed to read temperature");
        }

        if (bme280_read_raw_humidity(sensorHandle, &raw_humidity) == ESP_OK) {
            int32_t humidity = BME280_compensate_H_int32(raw_humidity);
            float humidity_percent = humidity / 1024.0;
            ESP_LOGI(TAG, "Humidity: %.2f%%", humidity_percent);
        } else {
            ESP_LOGE(TAG, "Failed to read humidity");
        }

        if (bme280_read_raw_pressure(sensorHandle, &raw_pressure) == ESP_OK) {
            int32_t pressure = BME280_compensate_P_int32(raw_pressure);
            float pressure_hPa = pressure / 256.0;
            ESP_LOGI(TAG, "Pressure: %.2fhPa", pressure_hPa/100);
        } else {
            ESP_LOGE(TAG, "Failed to read pressure");
        }
       
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main() {
    xTaskCreate(read_sensor_task, "read_sensor_task", 2048, NULL, 5, NULL);
}
