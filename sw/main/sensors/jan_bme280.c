// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "jan_bme280.h"
#include <stdint.h>
#include <esp_log.h>
#include <esp_err.h>

#define BME280_I2C_TIMEOUT 50

// Register addresses
#define BME280_REG_ID 0xD0
#define BME280_REG_RESET 0xE0
#define BME280_REG_CTRL_HUM 0xF2
#define BME280_REG_STATUS 0xF3
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG 0xF5
#define BME280_REG_PRESS_MSB 0xF7
#define BME280_REG_PRESS_LSB 0xF8
#define BME280_REG_PRESS_XLSB 0xF9
#define BME280_REG_TEMP_MSB 0xFA
#define BME280_REG_TEMP_LSB 0xFB
#define BME280_REG_TEMP_XLSB 0xFC
#define BME280_REG_HUM_MSB 0xFD
#define BME280_REG_HUM_LSB 0xFE

static const char *TAG = "JAN_BME280";

// static uint8_t BME280_I2C_ADDR;
// static i2c_port_t BME280_I2C_PORT;

esp_err_t bme280_read_register(bme280_dev_t *bme, uint8_t reg, uint8_t *data);
esp_err_t bme280_write_register(bme280_dev_t *bme, uint8_t reg, uint8_t data);
esp_err_t bme280_read_registers(bme280_dev_t *bme, uint8_t reg, void *data, size_t size);
void bme280_read_cal_dig(bme280_dev_t *bme, uint8_t reg, uint16_t *dig);
uint32_t bme280_compensate_temp(bme280_dev_t *bme, int32_t adc_temp, int32_t *t_fine);
uint32_t bme280_compensate_pres(bme280_dev_t *bme, int32_t adc_p, int32_t t_fine);
uint32_t bme280_compensate_humi(bme280_dev_t *bme, int32_t adc_h, int32_t t_fine);

bme280_dev_t bme280_begin(uint8_t i2c_addr, i2c_port_t i2c_port)
{
    bme280_calib_data_t calib_data = {};
    bme280_dev_t bme280 = {
        .i2c_addr = i2c_addr,
        .i2c_port = i2c_port,
        .calib_data = calib_data,
    };
    ESP_LOGI(TAG, "Reading BME280 chip ID...");
    uint8_t chip_id;
    bme280_read_register(&bme280, BME280_REG_ID, &chip_id);
    if (chip_id == 0x60)
        ESP_LOGI(TAG, "Chip ID: 0x%02x", chip_id);
    else
        ESP_LOGE(TAG, "Wrong chip ID: 0x%02x", chip_id);
    bme280_soft_reset(&bme280);
    return bme280;
};

void bme280_soft_reset(bme280_dev_t *bme)
{
    ESP_LOGI(TAG, "Soft reseting BME280...");
    bme280_write_register(bme, BME280_REG_RESET, 0xB6);
}

void bme280_default_params(bme280_params_t *params)
{
    params->mode = BME280_MODE_NORMAL;
    params->standby_time = BME280_STANDBY_MS_250;
    params->iir_filter = BME280_FILTER_OFF;
    params->oversampling_temp = BME280_SAMPLING_X4;
    params->oversampling_pres = BME280_SAMPLING_X4;
    params->oversampling_humi = BME280_SAMPLING_X4;
}

void bme280_set_params(bme280_dev_t *bme, bme280_params_t *params)
{

    // Config register
    uint8_t reg_config = 0;
    reg_config |= ((params->standby_time & 0x07) << 5); // Set standby time (bits 7,6,5)
    reg_config |= ((params->iir_filter & 0x07) << 2);   // Set IIR filter (bits 4,3,2)
    reg_config &= 0b11111100;                           // Disable SPI
    ESP_LOGD(TAG, "Writing value 0x%02x to config register", reg_config);
    bme280_write_register(bme, BME280_REG_CONFIG, reg_config);

    // Ctrl_hum register
    uint8_t reg_ctrl_hum = (params->oversampling_humi & 0x07);
    ESP_LOGD(TAG, "Writing value 0x%02x to ctrl_hum register", reg_ctrl_hum);
    bme280_write_register(bme, BME280_REG_CTRL_HUM, reg_ctrl_hum);

    // Ctrl_meas register
    uint8_t reg_ctrl_meas = 0;
    reg_ctrl_meas |= ((params->oversampling_temp & 0x07) << 5);
    reg_ctrl_meas |= ((params->oversampling_pres & 0x07) << 2);
    reg_ctrl_meas |= ((params->mode & 0x03));
    ESP_LOGD(TAG, "Writing value 0x%02x to ctrl_meas register", reg_ctrl_meas);
    bme280_write_register(bme, BME280_REG_CTRL_MEAS, reg_ctrl_meas);
}

void bme280_measurement(bme280_dev_t *bme)
{
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int32_t adc_temp, adc_pres, adc_humi, t_fine;

    // Check the current sensor mode
    uint8_t ctrl_meas;
    bme280_read_register(bme, BME280_REG_CTRL_MEAS, &ctrl_meas);
    if ((ctrl_meas & 0x03) == 0)
    {
        // If in sleep mode, do forced measurement
        uint8_t new_ctrl_meas = (ctrl_meas & 0b11111100) | (BME280_MODE_FORCED & 0x03);
        ESP_LOGD(TAG, "In sleep mode. Writing 0x%02x to ctrl_meas to enter forced mode", new_ctrl_meas);
        bme280_write_register(bme, BME280_REG_CTRL_MEAS, new_ctrl_meas);
    }

    // Check the status registers. Wait until measurement is complete
    uint8_t status;
    bme280_read_register(bme, BME280_REG_STATUS, &status);
    while (status & 0x09)
    {
        ESP_LOGD(TAG, "Waiting for status register to clear. Current value: 0x%02x", status & 0x09);
        vTaskDelay(1 / portTICK_PERIOD_MS);
        bme280_read_register(bme, BME280_REG_STATUS, &status);
    }

    // Burst read all the measurement registers
    bme280_read_registers(bme, BME280_REG_PRESS_MSB, data, 8);

    adc_pres = data[0] << 12 | data[1] << 4 | data[2] >> 4;
    ESP_LOGD(TAG, "Read ADC pressure: %d", adc_pres);

    adc_temp = data[3] << 12 | data[4] << 4 | data[5] >> 4;
    ESP_LOGD(TAG, "Read ADC temperature: %d", adc_temp);

    adc_humi = data[6] << 8 | data[7];
    ESP_LOGD(TAG, "Read ADC humidity: %d", adc_humi);

    uint32_t temp_fixed = bme280_compensate_temp(bme, adc_temp, &t_fine);
    ESP_LOGD(TAG, "Read fixed temperature: %d ", temp_fixed);
    ESP_LOGD(TAG, "Read t_fine=%d", t_fine);
    bme->temperature = (float)temp_fixed / 100.00;

    uint32_t pres_fixed = bme280_compensate_pres(bme, adc_pres, t_fine);
    ESP_LOGD(TAG, "Read fixed pressure: %d", pres_fixed);
    bme->pressure = (float)pres_fixed / 256.00;

    uint32_t humi_fixed = bme280_compensate_humi(bme, adc_humi, t_fine);
    ESP_LOGD(TAG, "Read fixed humidity: %d", humi_fixed);
    bme->humidity = (float)humi_fixed / 1024.00;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "******** Begin Bosch BME280 measurement ********");
    ESP_LOGI(TAG, "Temperature: %f °C", bme->temperature);
    ESP_LOGI(TAG, "Humidity: %f %%RH", bme->humidity);
    ESP_LOGI(TAG, "Pressure: %f Pa", bme->pressure);
    ESP_LOGI(TAG, "******** End Bosch BME280 measurement ********");
    ESP_LOGI(TAG, "");
}

uint32_t bme280_compensate_temp(bme280_dev_t *bme, int32_t adc_temp, int32_t *t_fine)
{
    int32_t var1, var2;

    var1 = ((((adc_temp >> 3) - ((int32_t)bme->calib_data.dig_T1 << 1))) * (int32_t)bme->calib_data.dig_T2) >> 11;
    var2 = (((((adc_temp >> 4) - (int32_t)bme->calib_data.dig_T1) * ((adc_temp >> 4) - (int32_t)bme->calib_data.dig_T1)) >> 12) * (int32_t)bme->calib_data.dig_T3) >> 14;

    *t_fine = var1 + var2;
    return (*t_fine * 5 + 128) >> 8;
}

uint32_t bme280_compensate_pres(bme280_dev_t *bme, int32_t adc_press, int32_t t_fine)
{
    int64_t var1, var2, p;

    var1 = (int64_t)t_fine - 128000;
    var2 = var1 * var1 * (int64_t)bme->calib_data.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bme->calib_data.dig_P5) << 17);
    var2 = var2 + (((int64_t)bme->calib_data.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bme->calib_data.dig_P3) >> 8) + ((var1 * (int64_t)bme->calib_data.dig_P2) << 12);
    var1 = (((int64_t)1 << 47) + var1) * ((int64_t)bme->calib_data.dig_P1) >> 33;

    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }

    p = 1048576 - adc_press;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)bme->calib_data.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)bme->calib_data.dig_P8 * p) >> 19;

    p = ((p + var1 + var2) >> 8) + ((int64_t)bme->calib_data.dig_P7 << 4);
    return (uint32_t)p;
}

uint32_t bme280_compensate_humi(bme280_dev_t *bme, int32_t adc_hum, int32_t t_fine)
{
    int32_t v_x1_u32r;

    v_x1_u32r = t_fine - (int32_t)76800;
    v_x1_u32r = ((((adc_hum << 14) - ((int32_t)bme->calib_data.dig_H4 << 20) - ((int32_t)bme->calib_data.dig_H5 * v_x1_u32r)) + (int32_t)16384) >> 15) * (((((((v_x1_u32r * (int32_t)bme->calib_data.dig_H6) >> 10) * (((v_x1_u32r * (int32_t)bme->calib_data.dig_H3) >> 11) + (int32_t)32768)) >> 10) + (int32_t)2097152) * (int32_t)bme->calib_data.dig_H2 + 8192) >> 14);
    v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * (int32_t)bme->calib_data.dig_H1) >> 4);
    v_x1_u32r = v_x1_u32r < 0 ? 0 : v_x1_u32r;
    v_x1_u32r = v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r;
    return (uint32_t)v_x1_u32r >> 12;
}

void bme280_load_calib_data(bme280_dev_t *bme)
{
    ESP_LOGD(TAG, "Reading calibration data...");

    bme280_read_cal_dig(bme, 0x88, &bme->calib_data.dig_T1);
    ESP_LOGD(TAG, "DIG_T1: %d", bme->calib_data.dig_T1);

    bme280_read_cal_dig(bme, 0x8A, (uint16_t *)&bme->calib_data.dig_T2);
    ESP_LOGD(TAG, "DIG_T2: %d", bme->calib_data.dig_T2);

    bme280_read_cal_dig(bme, 0x8C, (uint16_t *)&bme->calib_data.dig_T3);
    ESP_LOGD(TAG, "DIG_T3: %d", bme->calib_data.dig_T3);

    bme280_read_cal_dig(bme, 0x8E, &bme->calib_data.dig_P1);
    ESP_LOGD(TAG, "DIG_P1: %d", bme->calib_data.dig_P1);

    bme280_read_cal_dig(bme, 0x90, (uint16_t *)&bme->calib_data.dig_P2);
    ESP_LOGD(TAG, "DIG_P2: %d", bme->calib_data.dig_P2);

    bme280_read_cal_dig(bme, 0x92, (uint16_t *)&bme->calib_data.dig_P3);
    ESP_LOGD(TAG, "DIG_P3: %d", bme->calib_data.dig_P3);

    bme280_read_cal_dig(bme, 0x94, (uint16_t *)&bme->calib_data.dig_P4);
    ESP_LOGD(TAG, "DIG_P4: %d", bme->calib_data.dig_P4);

    bme280_read_cal_dig(bme, 0x96, (uint16_t *)&bme->calib_data.dig_P5);
    ESP_LOGD(TAG, "DIG_P5: %d", bme->calib_data.dig_P5);

    bme280_read_cal_dig(bme, 0x98, (uint16_t *)&bme->calib_data.dig_P6);
    ESP_LOGD(TAG, "DIG_P6: %d", bme->calib_data.dig_P6);

    bme280_read_cal_dig(bme, 0x9A, (uint16_t *)&bme->calib_data.dig_P7);
    ESP_LOGD(TAG, "DIG_P7: %d", bme->calib_data.dig_P7);

    bme280_read_cal_dig(bme, 0x9C, (uint16_t *)&bme->calib_data.dig_P8);
    ESP_LOGD(TAG, "DIG_P8: %d", bme->calib_data.dig_P8);

    bme280_read_cal_dig(bme, 0x9E, (uint16_t *)&bme->calib_data.dig_P9);
    ESP_LOGD(TAG, "DIG_P9: %d", bme->calib_data.dig_P9);

    bme280_read_register(bme, 0xA1, &bme->calib_data.dig_H1);
    ESP_LOGD(TAG, "DIG_H1: %d", bme->calib_data.dig_H1);

    bme280_read_cal_dig(bme, 0xE1, (uint16_t *)&bme->calib_data.dig_H2);
    ESP_LOGD(TAG, "DIG_H2: %d", bme->calib_data.dig_H2);

    bme280_read_register(bme, 0xE3, &bme->calib_data.dig_H3);
    ESP_LOGD(TAG, "DIG_H3: %d", bme->calib_data.dig_H3);

    uint8_t dig_h45[3] = {0, 0, 0};
    bme280_read_registers(bme, 0xE4, &dig_h45, 3);

    bme->calib_data.dig_H4 = (((uint16_t)dig_h45[0]) << 4) | (dig_h45[1] & 0x0F);
    ESP_LOGD(TAG, "DIG_H4: %d", bme->calib_data.dig_H4);

    bme->calib_data.dig_H5 = (((uint16_t)dig_h45[2]) << 4) | ((dig_h45[1] & 0xF0) >> 4);
    ESP_LOGD(TAG, "DIG_H5: %d", bme->calib_data.dig_H5);

    bme280_read_register(bme, 0xE7, (uint8_t *)&bme->calib_data.dig_H6);
    ESP_LOGD(TAG, "DIG_H6: %d", bme->calib_data.dig_H6);
}

esp_err_t bme280_read_register(bme280_dev_t *bme, uint8_t reg, uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    ESP_ERROR_CHECK(i2c_master_start(cmd));                                                               // Produce start conditon
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (bme->i2c_addr << 1) | I2C_MASTER_WRITE, true)); // Select I2C device
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, true));                                     // Write the register address to be read later

    ESP_ERROR_CHECK(i2c_master_start(cmd));                                                              // Repeated start condition
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (bme->i2c_addr << 1) | I2C_MASTER_READ, true)); // Select I2C device
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data, I2C_MASTER_NACK));                                   // Read the data from device
    ESP_ERROR_CHECK(i2c_master_stop(cmd));                                                               // Stop conditon
    esp_err_t ret = i2c_master_cmd_begin(bme->i2c_port, cmd, BME280_I2C_TIMEOUT / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading register from BME280: %d %s", ret, esp_err_to_name(ret));
    }
    i2c_cmd_link_delete(cmd);
    return ret;
};

esp_err_t bme280_write_register(bme280_dev_t *bme, uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (bme->i2c_addr << 1) | I2C_MASTER_WRITE, true)); // Select I2C device
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, true));                                     // Write the register address to be written later
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data, true));                                    // Write the data
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t ret = i2c_master_cmd_begin(bme->i2c_port, cmd, BME280_I2C_TIMEOUT / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading register from BME280: %d %s", ret, esp_err_to_name(ret));
    }
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t bme280_read_registers(bme280_dev_t *bme, uint8_t reg, void *data, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    ESP_ERROR_CHECK(i2c_master_start(cmd));                                                               // Produce start conditon
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (bme->i2c_addr << 1) | I2C_MASTER_WRITE, true)); // Select I2C device
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, true));                                     // Write the register address to be read later

    ESP_ERROR_CHECK(i2c_master_start(cmd));                                                              // Repeated start condition
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (bme->i2c_addr << 1) | I2C_MASTER_READ, true)); // Select I2C device
    ESP_ERROR_CHECK(i2c_master_read(cmd, data, size, I2C_MASTER_LAST_NACK));                             // Read the data from device
    ESP_ERROR_CHECK(i2c_master_stop(cmd));                                                               // Stop conditon
    esp_err_t ret = i2c_master_cmd_begin(bme->i2c_port, cmd, BME280_I2C_TIMEOUT / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading register from BME280: %d %s", ret, esp_err_to_name(ret));
    }
    i2c_cmd_link_delete(cmd);
    return ret;
}

void bme280_read_cal_dig(bme280_dev_t *bme, uint8_t reg, uint16_t *dig)
{
    uint8_t tmp[2] = {0, 0};
    bme280_read_registers(bme, reg, tmp, 2);
    *dig = tmp[0] | (tmp[1] << 8);
}