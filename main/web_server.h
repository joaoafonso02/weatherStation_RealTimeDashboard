#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_http_server.h"
#include "freertos/semphr.h"

/**
 * @brief BME280 sensor data structure 
 * 
 */
typedef struct {
    float temperature;
    float humidity;
    float pressure;
} sensor_data_t;

extern sensor_data_t g_sensor_data;
extern SemaphoreHandle_t g_sensor_data_semaphore;

esp_err_t sensor_handler(httpd_req_t *req);

esp_err_t index_handler(httpd_req_t *req);

httpd_handle_t start_webserver(void);

#endif // WEB_SERVER_H