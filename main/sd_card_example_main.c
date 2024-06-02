/* SD card and FAT filesystem example.
   This example uses SPI peripheral to communicate with SD card.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
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
#include "sd_card.h"
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
 * @brief Define light sensor GPIO  
*/
#define LIGHT_SENSOR_GPIO 1

#define EXAMPLE_MAX_CHAR_SIZE  1024

static const char *TAG = "example";

#define MOUNT_POINT "/sdcard"

// Pin assignments can be set in menuconfig, see "SD SPI Example Configuration" menu.
// You can also change the pin assignments here by changing the following 4 lines.
#define PIN_NUM_MISO  5
#define PIN_NUM_MOSI  0
#define PIN_NUM_CLK   7
#define PIN_NUM_CS    3

static esp_err_t s_example_write_file(const char *path, char *data)
{
    ESP_LOGI(TAG, "Opening file %s", path);
    FILE *f = fopen(path, "w"); // Open in write mode
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, "%s", data);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    return ESP_OK;
}

static esp_err_t s_example_read_file(const char *path)
{
    ESP_LOGI(TAG, "Reading file %s", path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    char line[EXAMPLE_MAX_CHAR_SIZE];
    fgets(line, sizeof(line), f);
    fclose(f);

    // strip newline
    char *pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);

    return ESP_OK;
}

void read_sensor_task(void *pvParameters, esp_err_t ret) {
    i2c_master_bus_handle_t busHandle = NULL;
    i2c_master_dev_handle_t sensorHandle = NULL;

    if (bme280_init(&busHandle, &sensorHandle, BME280_I2C_ADDRESS, SDA_PIN, SCL_PIN, CLK_SPEED) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor");
        vTaskDelete(NULL); 
    }

    int iteration = 0;
    const int max_iterations = 60; // Run for 60 seconds

    while (iteration < max_iterations) {
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

            const char *file_sensor = MOUNT_POINT"/test.txt";
            char data[2048];
            FILE *f = fopen(file_sensor, "a"); 
            if (f == NULL) {
                ESP_LOGE(TAG, "Failed to open file for writing");
                return;
            }
            snprintf(data, 2048, "Temperature: %.2f°C\nHumidity: %.2f%%\nPressure: %.2fhPa\nLight Value: %d\n\n", temperature, humidity, pressure, light_sensor_value);
            fprintf(f, "%s", data);
            fclose(f);

            // Open file for reading
            ret = s_example_read_file(file_sensor);
            if (ret != ESP_OK) {
                return;
            }
            
        } else {
            ESP_LOGE(TAG, "Failed to read sensor data");
        }

        iteration++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "Sensor task completed");
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_err_t ret;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    ESP_LOGI(TAG, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

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
    // wifi_init_station();

    // // Start web server
    // start_webserver();

    // get sensor data through a freeRTOS task
    xTaskCreate(read_sensor_task, "read_sensor_task", 16384, NULL, 5, NULL); 
}
