#ifndef BME280_H
#define BME280_H

#include "esp_err.h"
#include "driver/i2c_master.h"

// BME280 I2C address
#define BME280_I2C_ADDRESS 0x76

// BME280 registers
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_MODE_NORMAL 0x27
#define BME280_REG_TEMP_MSB 0xFA
#define BME280_REG_PRESS_MSB 0xF7
#define BME280_REG_HUM_MSB 0xFD
#define BME280_REG_STATUS 0xF3

esp_err_t bme280_init(i2c_master_bus_handle_t* pBusHandle,
                      i2c_master_dev_handle_t* pSensorHandle,
                      uint8_t sensorAddr, int sdaPin, int sclPin, uint32_t clkSpeedHz);

esp_err_t bme280_read_status(i2c_master_dev_handle_t sensorHandle, uint8_t* pStatus);

esp_err_t bme280_read_raw_temp(i2c_master_dev_handle_t sensorHandle, int32_t* pRawTemp);

esp_err_t bme280_read_raw_humidity(i2c_master_dev_handle_t sensorHandle, int32_t* pRawHumidity);

esp_err_t bme280_read_raw_pressure(i2c_master_dev_handle_t sensorHandle, int32_t* pRawPressure);

int32_t BME280_compensate_T_int32(int32_t adc_T);

int32_t BME280_compensate_P_int32(int32_t adc_P);

int32_t BME280_compensate_H_int32(int32_t adc_H);

#endif // BME280_H