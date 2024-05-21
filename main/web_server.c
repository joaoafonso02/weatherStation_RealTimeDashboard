#include "web_server.h"
#include "cJSON.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_http_server.h"

sensor_data_t g_sensor_data;
SemaphoreHandle_t g_sensor_data_semaphore;


esp_err_t sensor_handler(httpd_req_t *req) {
    char response[128];
    if (xSemaphoreTake(g_sensor_data_semaphore, portMAX_DELAY)) {
        cJSON *root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "temperature", g_sensor_data.temperature);
        cJSON_AddNumberToObject(root, "humidity", g_sensor_data.humidity);
        cJSON_AddNumberToObject(root, "pressure", g_sensor_data.pressure);

        const char *json_response = cJSON_Print(root);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, json_response, strlen(json_response));
        cJSON_Delete(root);
        free((void *)json_response);

        xSemaphoreGive(g_sensor_data_semaphore);
    } else {
        snprintf(response, sizeof(response), "{\"error\": \"Failed to acquire sensor data\"}");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, response, strlen(response));
    }

    return ESP_OK;
}

esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");

    const char* resp_str =
        "<!DOCTYPE html>"
        "<html>"
            "<head>"
                "<style>"
                    "h1 { text-align: center; }"
                    "h2 { text-align: center; }"
                    ".chart-container { width: 50%; margin: auto; }"
                "</style>"
            "</head>"
            "<body>"
                "<h1>Real Time Sensor Data Charts</h1>"
                "<div class='chart-container'>"
                    "<h2>Temperature (&deg;C)</h2>"
                    "<canvas id='temperatureChart' width='400' height='250'></canvas>"
                "</div>"
                "<div class='chart-container'>"
                    "<h2>Humidity (%)</h2>"
                    "<canvas id='humidityChart' width='400' height='250'></canvas>"
                "</div>"
                "<div class='chart-container'>"
                    "<h2>Pressure (hPa)</h2>"
                    "<canvas id='pressureChart' width='400' height='250'></canvas>"
                "</div>"
                "<script src='https://cdn.jsdelivr.net/npm/chart.js'></script>"
                "<script>"
                "function createChart(ctx, label, color) {"
                "    return new Chart(ctx, {"
                "        type: 'line',"
                "        data: {"
                "            labels: [],"
                "            datasets: [{"
                "                label: label,"
                "                data: [],"
                "                borderColor: color,"
                "                backgroundColor: color + '33',"
                "            }]"
                "        },"
                "        options: {"
                "            scales: {"
                "                y: {"
                "                    beginAtZero: true"
                "                }"
                "            }"
                "        }"
                "    });"
                "}"
                "var temperatureChart = createChart(document.getElementById('temperatureChart').getContext('2d'), 'Temperature', 'rgba(255, 99, 132, 1)');"
                "var humidityChart = createChart(document.getElementById('humidityChart').getContext('2d'), 'Humidity', 'rgba(75, 192, 192, 1)');"
                "var pressureChart = createChart(document.getElementById('pressureChart').getContext('2d'), 'Pressure', 'rgba(153, 102, 255, 1)');"
                "setInterval(function() {"
                "    fetch('/sensor')"
                "    .then(response => response.json())"
                "    .then(data => {"
                "        var now = new Date().toLocaleTimeString();"
                "        temperatureChart.data.labels.push(now);"
                "        temperatureChart.data.datasets[0].data.push(data.temperature);"
                "        temperatureChart.update();"
                "        humidityChart.data.labels.push(now);"
                "        humidityChart.data.datasets[0].data.push(data.humidity);"
                "        humidityChart.update();"
                "        pressureChart.data.labels.push(now);"
                "        pressureChart.data.datasets[0].data.push(data.pressure);"
                "        pressureChart.update();"
                "    });"
                "}, 1000);"
                "</script>"
            "</body>"
        "</html>";

    httpd_resp_send(req, resp_str, strlen(resp_str));

    return ESP_OK;
}

httpd_uri_t sensor_uri = {
    .uri       = "/sensor",
    .method    = HTTP_GET,
    .handler   = sensor_handler,
    .user_ctx  = NULL
};

httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_handler,
    .user_ctx  = NULL
};


httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &sensor_uri);
        //ESP_LOGI(TAG, "HTTP server started");
        return server;
    }

    ESP_LOGI("WEB SERVER", "Error starting HTTP server");
    return NULL;
}