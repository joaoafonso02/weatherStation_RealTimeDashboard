#include "BME280.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "BME280";

static int32_t t_fine;
static uint16_t dig_T1;
static int16_t dig_T2, dig_T3;

esp_err_t bme280_read_calibration_data(i2c_master_dev_handle_t sensorHandle) {
    uint8_t calib[6];
    uint8_t reg = 0x88; // Starting address for temperature calibration data
    esp_err_t ret = i2c_master_transmit_receive(sensorHandle, &reg, 1, calib, 6, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data");
        return ret;
    }

    dig_T1 = (calib[1] << 8) | calib[0];
    dig_T2 = (calib[3] << 8) | calib[2];
    dig_T3 = (calib[5] << 8) | calib[4];

    ESP_LOGI(TAG, "Calibration data: T1=%u, T2=%d, T3=%d", dig_T1, dig_T2, dig_T3);
    return ESP_OK;
}

esp_err_t bme280_init(i2c_master_bus_handle_t* pBusHandle,
                      i2c_master_dev_handle_t* pSensorHandle,
                      uint8_t sensorAddr, int sdaPin, int sclPin, uint32_t clkSpeedHz) {

    i2c_master_bus_config_t i2cMasterConfig = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = sclPin,
        .sda_io_num = sdaPin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_LOGI(TAG, "Initializing I2C master\nSCL: %d\nSDA: %d\n", sclPin, sdaPin);

    esp_err_t ret = i2c_new_master_bus(&i2cMasterConfig, pBusHandle);
    if (ret != ESP_OK) return ret;

    i2c_device_config_t i2cDevCfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = sensorAddr,
        .scl_speed_hz = clkSpeedHz,
    };

    ret = i2c_master_bus_add_device(*pBusHandle, &i2cDevCfg, pSensorHandle);
    if (ret != ESP_OK) return ret;

    uint8_t buffer[2] = {BME280_REG_CTRL_MEAS, BME280_MODE_NORMAL};
    ret = i2c_master_transmit(*pSensorHandle, buffer, sizeof(buffer), -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set control measurement register");
        return ret;
    }

    return bme280_read_calibration_data(*pSensorHandle);
}

esp_err_t bme280_read_status(i2c_master_dev_handle_t sensorHandle, uint8_t* pStatus) {
    uint8_t reg = BME280_REG_STATUS;
    esp_err_t ret = i2c_master_transmit_receive(sensorHandle, &reg, 1, pStatus, 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read status register");
    }
    return ret;
}

esp_err_t bme280_read_raw_temp(i2c_master_dev_handle_t sensorHandle, int32_t* pRawTemp) {
    uint8_t data[3] = {0};
    uint8_t reg = BME280_REG_TEMP_MSB;
    
    esp_err_t ret = i2c_master_transmit_receive(sensorHandle, &reg, 1, data, 3, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temperature data");
        return ret;
    }
    // ESP_LOGI(TAG, "Raw temperature data: %02X %02X %02X", data[0], data[1], data[2]);

    *pRawTemp = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);
    return ESP_OK;
}

int32_t BME280_compensate_T_int32(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T; 
}
