// new main
#include "driver/i2c_master.h"
#include "esp_http_server.h"
#include "esp_http_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "web_server.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_err.h"
#include "BME280.h"
#include "cJSON.h"
#include "wifi.h"
#include "nvs.h"
#include "driver/adc.h"
// #include "driver/spi_master.h"
#include "driver/gpio.h"
// #include "esp_ota_ops.h"
// #include "esp_https_ota.h"
// #include "esp_pm.h"

/**
 * @brief Declaration of global variables (SDA and SCL pins and CLK_SPEED)
 */
#define SDA_PIN 2
#define SCL_PIN 6
#define CLK_SPEED 100000 // 100 kHz

/**
 * @brief Define the SPI configuration for the SD card
 */
#define SD_HOST SPI2_HOST
#define SD_CS_PIN 3
#define SD_MOSI_PIN 0
#define SD_CLK_PIN 7
#define SD_MISO_PIN 5

/**
 * @brief Define light sensor GPIO  
*/
#define LIGHT_SENSOR_GPIO 1

static const char *TAG = "main";

/**
 * @brief Read sensor task - Read Temperature, Humidity, and Pressure from the BME280 sensor
 * 
 * @param pvParameters 
 */
void read_sensor_task(void *pvParameters) {
    i2c_master_bus_handle_t busHandle = NULL;
    i2c_master_dev_handle_t sensorHandle = NULL;

    if (bme280_init(&busHandle, &sensorHandle, BME280_I2C_ADDRESS, SDA_PIN, SCL_PIN, CLK_SPEED) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor");
        vTaskDelete(NULL); 
    }

    while (1) {
        int32_t raw_temp = 0;
        int32_t raw_humidity = 0;
        int32_t raw_pressure = 0;

        if (bme280_read_raw_temp(sensorHandle, &raw_temp) == ESP_OK &&
            bme280_read_raw_humidity(sensorHandle, &raw_humidity) == ESP_OK &&
            bme280_read_raw_pressure(sensorHandle, &raw_pressure) == ESP_OK) {
            
            float temperature = BME280_compensate_T_int32(raw_temp) / 100.0;
            float humidity = BME280_compensate_H_int32(raw_humidity) / 1024.0;
            float pressure = BME280_compensate_P_int32(raw_pressure) / 25600.0;

            if (xSemaphoreTake(g_sensor_data_semaphore, portMAX_DELAY)) {
                g_sensor_data.temperature = temperature;
                g_sensor_data.humidity = humidity;
                g_sensor_data.pressure = pressure;
                xSemaphoreGive(g_sensor_data_semaphore);
            }

            ESP_LOGI(TAG, "Temperature: %.2f°C", temperature);
            ESP_LOGI(TAG, "Humidity: %.2f%%", humidity);
            ESP_LOGI(TAG, "Pressure: %.2fhPa", pressure);

            // Read the value from the light sensor
            int light_sensor_value = adc1_get_raw(LIGHT_SENSOR_GPIO);
            ESP_LOGI(TAG, "Light sensor value: %d", light_sensor_value);

            if(light_sensor_value < 200) {
                ESP_LOGI(TAG, "The environment is dark\n");
            } else if(light_sensor_value < 1000) {
                ESP_LOGI(TAG, "The environment is mid\n");
            } else if(light_sensor_value < 2000) {
                ESP_LOGI(TAG, "The environment is bright\n");
            } else {
                ESP_LOGI(TAG, "The environment is very bright\n");
            }

        } else {
            ESP_LOGE(TAG, "Failed to read sensor data");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "Sensor task completed");
    vTaskDelete(NULL);
}


void app_main() {
    // Initialize NVS (Non-Volatile Storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Semaphore to protect the global variable g_sensor_data from concurrent access by different tasks.
    g_sensor_data_semaphore = xSemaphoreCreateMutex();
    if (g_sensor_data_semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor data semaphore");
        return;
    }

    // setup light sensor
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LIGHT_SENSOR_GPIO, ADC_ATTEN_DB_11);

    // Initialize Wi-Fi connection
    wifi_init_station();

    // Start web server
    start_webserver();

    // Start sensor task
    xTaskCreate(read_sensor_task, "read_sensor_task", 2048, NULL, 5, NULL);
}
