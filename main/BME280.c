#include "BME280.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "BME280";

static int32_t t_fine;
static uint16_t dig_T1;
static int16_t dig_T2, dig_T3;

static uint16_t dig_P1;
static int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

static uint8_t dig_H1;
static int16_t dig_H2;
static uint8_t dig_H3;
static int16_t dig_H4, dig_H5;
static int8_t dig_H6;

/**
 * @brief Read the calibration data from the BME280 sensor
 * 
 * @param sensorHandle  I2C device handle
 * @return esp_err_t ESP OK if successful, ESP FAIL otherwise
 */
esp_err_t bme280_read_calibration_data(i2c_master_dev_handle_t sensorHandle) {
    uint8_t calib[26];
    uint8_t reg = 0x88; // Starting address for calibration data
    esp_err_t ret = i2c_master_transmit_receive(sensorHandle, &reg, 1, calib, 26, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data");
        return ret;
    }

    dig_T1 = (calib[1] << 8) | calib[0];
    dig_T2 = (calib[3] << 8) | calib[2];
    dig_T3 = (calib[5] << 8) | calib[4];

    dig_P1 = (calib[7] << 8) | calib[6];
    dig_P2 = (calib[9] << 8) | calib[8];
    dig_P3 = (calib[11] << 8) | calib[10];
    dig_P4 = (calib[13] << 8) | calib[12];
    dig_P5 = (calib[15] << 8) | calib[14];
    dig_P6 = (calib[17] << 8) | calib[16];
    dig_P7 = (calib[19] << 8) | calib[18];
    dig_P8 = (calib[21] << 8) | calib[20];
    dig_P9 = (calib[23] << 8) | calib[22];

    dig_H1 = calib[25];

    reg = 0xE1;
    ret = i2c_master_transmit_receive(sensorHandle, &reg, 1, calib, 7, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read humidity calibration data");
        return ret;
    }

    dig_H2 = (calib[1] << 8) | calib[0];
    dig_H3 = calib[2];
    dig_H4 = (calib[3] << 4) | (calib[4] & 0x0F);
    dig_H5 = (calib[5] << 4) | ((calib[4] >> 4) & 0x0F);
    dig_H6 = (int8_t)calib[6];

    ESP_LOGI(TAG, "Calibration data: T1=%u, T2=%d, T3=%d, P1=%u, P2=%d, P3=%d, P4=%d, P5=%d, P6=%d, P7=%d, P8=%d, P9=%d, H1=%u, H2=%d, H3=%u, H4=%d, H5=%d, H6=%d",
             dig_T1, dig_T2, dig_T3, dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9,
             dig_H1, dig_H2, dig_H3, dig_H4, dig_H5, dig_H6);
    return ESP_OK;
}

/**
 * @brief I2C initialization for the BME280 sensor 
 * 
 * @param pBusHandle  Pointer to I2C bus handle
 * @param pSensorHandle  Pointer to I2C device handle
 * @param sensorAddr  I2C address of the sensor (0x76 --> BME280 SDO pin connected to GND)
 * @param sdaPin  GPIO pin for SDA (2)
 * @param sclPin  GPIO pin for SCL (6)
*/
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

/**
 * @brief Read the status register of the BME280 sensor
 * 
 * @param sensorHandle  I2C device handle
 * @param pStatus  Pointer to store the status register value
 * @return esp_err_t ESP OK if successful, ESP FAIL otherwise
 */
esp_err_t bme280_read_status(i2c_master_dev_handle_t sensorHandle, uint8_t* pStatus) {
    uint8_t reg = BME280_REG_STATUS;
    esp_err_t ret = i2c_master_transmit_receive(sensorHandle, &reg, 1, pStatus, 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read status register");
    }
    return ret;
}

/**
 * @brief Read the raw temperature data from the BME280 sensor
 * 
 * @param sensorHandle I2C device handle
 * @param pRawTemp Pointer to store the raw temperature data
 * @return esp_err_t ESP OK if successful, ESP_FAIL otherwise
 */
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

/**
 * @brief Read the raw humidity data from the BME280 sensor
 * 
 * @param sensorHandle I2C device handle
 * @param pRawHumidity Pointer to store the raw humidity data
 * @return esp_err_t ESP OK if successful, ESP_FAIL otherwise
 */
esp_err_t bme280_read_raw_humidity(i2c_master_dev_handle_t sensorHandle, int32_t* pRawHumidity) {
    uint8_t data[2] = {0};
    uint8_t reg = 0xFD;
    
    esp_err_t ret = i2c_master_transmit_receive(sensorHandle, &reg, 1, data, 2, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read humidity data");
        return ret;
    }
    // ESP_LOGI(TAG, "Raw humidity data: %02X %02X", data[0], data[1]);

    *pRawHumidity = ((int32_t)data[0] << 8) | data[1];
    return ESP_OK;
}

/**
 * @brief Read the raw pressure data from the BME280 sensor
 * 
 * @param sensorHandle I2C device handle
 * @param pRawPressure Pointer to store the raw pressure data
 * @return esp_err_t ESP OK if successful, ESP_FAIL otherwise
 */
esp_err_t bme280_read_raw_pressure(i2c_master_dev_handle_t sensorHandle, int32_t* pRawPressure) {
    uint8_t data[3] = {0};
    uint8_t reg = 0xF7;
    
    esp_err_t ret = i2c_master_transmit_receive(sensorHandle, &reg, 1, data, 3, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read pressure data");
        return ret;
    }
    // ESP_LOGI(TAG, "Raw pressure data: %02X %02X %02X", data[0], data[1], data[2]);

    *pRawPressure = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);
    return ESP_OK;
}

// BME280 DATASHEET COVERSION FORMULAS - PAGE 25,26
/**
 * @brief Compensate the temperature data
 * 
 * @param adc_T Raw temperature data
 * @return int32_t Temperature in degrees Celsius
 */
int32_t BME280_compensate_T_int32(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T; 
}

/**
 * @brief Compensate the pressure data
 * 
 * @param adc_P Raw pressure data
 * @return int32_t Pressure in Pascals
 */
int32_t BME280_compensate_P_int32(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

    if (var1 == 0) {
        return 0; 
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (int32_t)p;
}

/**
 * @brief Compensate the humidity data
 * 
 * @param adc_H Raw humidity data
 * @return int32_t Humidity in %RH
 */
int32_t BME280_compensate_H_int32(int32_t adc_H) {
    int32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) +
              ((int32_t)16384)) >> 15) *
            (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) *
                 (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >>
                10) +
               ((int32_t)2097152)) *
                  ((int32_t)dig_H2) +
              8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (int32_t)(v_x1_u32r >> 12);
}