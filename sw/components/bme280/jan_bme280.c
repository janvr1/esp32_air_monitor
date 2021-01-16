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

esp_err_t bme280_read_register(bme280_dev_t *bme, uint8_t reg, uint8_t *data);
esp_err_t bme280_write_register(bme280_dev_t *bme, uint8_t reg, uint8_t data);
esp_err_t bme280_read_registers(bme280_dev_t *bme, uint8_t reg, void *data, size_t size);
esp_err_t bme280_read_cal_dig(bme280_dev_t *bme, uint8_t reg, uint16_t *dig);
esp_err_t bme280_check_chip_id(bme280_dev_t *bme);
esp_err_t bme280_soft_reset(bme280_dev_t *bme);
esp_err_t bme280_load_calib_data(bme280_dev_t *bme);
esp_err_t bme280_set_ctrl_hum_register(bme280_dev_t *bme, bme280_oversampling_t osrs_hum);
esp_err_t bme280_get_ctrl_hum_register(bme280_dev_t *bme, bme280_oversampling_t *osrs_hum);
esp_err_t bme280_get_status_register(bme280_dev_t *bme, bme280_status_register_t *status);
esp_err_t bme280_set_ctrl_meas_register(bme280_dev_t *bme, bme280_ctrl_meas_register_t *ctrl_meas);
esp_err_t bme280_get_ctrl_meas_register(bme280_dev_t *bme, bme280_ctrl_meas_register_t *ctrl_meas);
esp_err_t bme280_set_config_register(bme280_dev_t *bme, bme280_config_register_t *config);
esp_err_t bme280_get_config_register(bme280_dev_t *bme, bme280_config_register_t *config);

uint32_t bme280_compensate_temp(bme280_dev_t *bme, int32_t adc_temp, int32_t *t_fine);
uint32_t bme280_compensate_pres(bme280_dev_t *bme, int32_t adc_p, int32_t t_fine);
uint32_t bme280_compensate_humi(bme280_dev_t *bme, int32_t adc_h, int32_t t_fine);

esp_err_t bme280_begin(bme280_dev_t *bme, bme280_params_t *params, uint8_t i2c_addr, i2c_port_t i2c_port)
{
    esp_err_t ret;

    bme->i2c_addr = i2c_addr;
    bme->i2c_port = i2c_port;
    bme->temperature = 0;
    bme->humidity = 0;
    bme->pressure = 0;

    // Check chip ID
    ret = bme280_check_chip_id(bme);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
        return ret;

    // Perform soft reset
    ret = bme280_soft_reset(bme);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    vTaskDelay(5 / portTICK_PERIOD_MS); // Wait for BME to power on

    // Load calibration data
    ret = bme280_load_calib_data(bme);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
        return ret;

    // Set the parameters
    ret = bme280_set_params(bme, params);
    if (ret != ESP_OK)
        return ret;

    return ret;
};

esp_err_t bme280_check_chip_id(bme280_dev_t *bme)
{
    uint8_t chip_id;
    esp_err_t ret = bme280_read_register(bme, BME280_REG_ID, &chip_id);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading chip ID");
        return ret;
    }
    if (chip_id == 0x60)
    {
        ESP_LOGD(TAG, "Correct chip ID: 0x%02x", chip_id);
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG, "Wrong chip ID: 0x%02x", chip_id);
        return ESP_FAIL;
    }
}

esp_err_t bme280_soft_reset(bme280_dev_t *bme)
{
    ESP_LOGD(TAG, "Soft reseting BME280...");
    esp_err_t ret = bme280_write_register(bme, BME280_REG_RESET, 0xB6);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reseting BME280");
    }
    return ret;
}

esp_err_t bme280_set_ctrl_hum_register(bme280_dev_t *bme, bme280_oversampling_t osrs_hum)
{
    uint8_t reg = (osrs_hum & 0x07);
    esp_err_t ret = bme280_write_register(bme, BME280_REG_CTRL_HUM, reg);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Error setting ctrl_hum register");
    else
        ESP_LOGD(TAG, "Successfully set ctrl_hum register to 0x%02x", reg);
    return ret;
}

esp_err_t bme280_get_ctrl_hum_register(bme280_dev_t *bme, bme280_oversampling_t *osrs_hum)
{
    uint8_t reg = 0;
    esp_err_t ret = bme280_read_register(bme, BME280_REG_CTRL_HUM, &reg);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading ctrl_hum register");
        return ret;
    }
    else
    {
        ESP_LOGD(TAG, "Successfully read ctrl_hum register: 0x%02x", reg);
        *osrs_hum = reg & 0x07;
        return ret;
    }
}

esp_err_t bme280_get_status_register(bme280_dev_t *bme, bme280_status_register_t *status)
{
    uint8_t reg = 0;
    esp_err_t ret = bme280_read_register(bme, BME280_REG_STATUS, &reg);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading status register");
        return ret;
    }
    else
    {
        ESP_LOGD(TAG, "Successfully read status register: 0x%02x", reg);
        status->measuring = (reg >> 3) & 0x01;
        status->im_update = reg & 0x01;
        return ret;
    }
}

esp_err_t bme280_set_ctrl_meas_register(bme280_dev_t *bme, bme280_ctrl_meas_register_t *ctrl_meas)
{
    uint8_t reg = 0;
    reg |= ((ctrl_meas->osrs_temp & 0x07) << 5);
    reg |= ((ctrl_meas->osrs_pres & 0x07) << 2);
    reg |= ((ctrl_meas->mode & 0x03));

    esp_err_t ret = bme280_write_register(bme, BME280_REG_CTRL_MEAS, reg);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Error setting ctrl_meas register");
    else
        ESP_LOGD(TAG, "Successfully set ctrl_meas register to 0x%02x", reg);
    return ret;
}

esp_err_t bme280_get_ctrl_meas_register(bme280_dev_t *bme, bme280_ctrl_meas_register_t *ctrl_meas)
{
    uint8_t reg = 0;
    esp_err_t ret = bme280_read_register(bme, BME280_REG_CTRL_MEAS, &reg);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading ctrl_meas register");
        return ret;
    }
    else
    {
        ESP_LOGD(TAG, "Successfully read ctrl_meas register: 0x%02x", reg);
        ctrl_meas->osrs_temp = (reg >> 5) & 0x07;
        ctrl_meas->osrs_pres = (reg >> 2) & 0x07;
        ctrl_meas->mode = reg & 0x03;
        return ret;
    }
}

esp_err_t bme280_set_config_register(bme280_dev_t *bme, bme280_config_register_t *config)
{
    uint8_t reg = 0;
    reg |= ((config->t_sb & 0x07) << 5);       // Set standby time (bits 7,6,5)
    reg |= ((config->iir_filter & 0x07) << 2); // Set IIR filter (bits 4,3,2)
    reg &= 0b11111100;                         // Disable SPI

    esp_err_t ret = bme280_write_register(bme, BME280_REG_CONFIG, reg);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Error setting config register");
    else
        ESP_LOGD(TAG, "Successfully set config register to 0x%02x", reg);
    return ret;
}

esp_err_t bme280_get_config_register(bme280_dev_t *bme, bme280_config_register_t *config)
{
    uint8_t reg = 0;
    esp_err_t ret = bme280_read_register(bme, BME280_REG_CONFIG, &reg);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading config register");
        return ret;
    }

    ESP_LOGD(TAG, "Successfully read config register: 0x%02x", reg);
    config->t_sb = (reg >> 5) & 0x07;
    config->iir_filter = (reg >> 2) & 0x07;
    config->spi3w_en = reg & 0x03;
    return ret;
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

esp_err_t bme280_set_params(bme280_dev_t *bme, bme280_params_t *params)
{
    esp_err_t ret;

    // Config register
    bme280_config_register_t config;
    config.t_sb = params->standby_time;
    config.iir_filter = params->iir_filter;
    config.spi3w_en = 0;
    ret = bme280_set_config_register(bme, &config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error setting parameters");
        return ret;
    }

    // Ctrl_hum register
    ret = bme280_set_ctrl_hum_register(bme, params->oversampling_humi);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error setting parameters");
        return ret;
    }

    // Ctrl_meas register
    bme280_ctrl_meas_register_t ctrl_meas;
    ctrl_meas.osrs_temp = params->oversampling_temp;
    ctrl_meas.osrs_pres = params->oversampling_pres;
    ctrl_meas.mode = params->mode;
    ret = bme280_set_ctrl_meas_register(bme, &ctrl_meas);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error setting parameters");
        return ret;
    }
    ESP_LOGI(TAG, "Successfully set BME280 params");
    return ret;
}

esp_err_t bme280_measurement(bme280_dev_t *bme)
{
    esp_err_t ret;
    uint8_t data[8] = {};
    int32_t adc_temp, adc_pres, adc_humi, t_fine;

    // Check the current sensor mode
    bme280_ctrl_meas_register_t ctrl_meas;
    ret = bme280_get_ctrl_meas_register(bme, &ctrl_meas);
    if (ret != ESP_OK)
        return ret;
    if (ctrl_meas.mode == BME280_MODE_SLEEP)
    {
        // If in sleep mode, do forced measurement
        ctrl_meas.mode = BME280_MODE_FORCED;
        ESP_LOGI(TAG, "In sleep mode. Performing forced measurement");
        bme280_set_ctrl_meas_register(bme, &ctrl_meas);
    }

    // Check the status registers. Wait until measurement is complete
    bme280_status_register_t status;
    ESP_ERROR_CHECK(bme280_get_status_register(bme, &status));

    while (status.measuring || status.im_update)
    {
        ESP_LOGD(TAG, "Waiting for status register to clear...");
        vTaskDelay(1 / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK_WITHOUT_ABORT(bme280_get_status_register(bme, &status));
    }

    // Burst read all the measurement registers
    ret = bme280_read_registers(bme, BME280_REG_PRESS_MSB, data, 8);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
        return ret;

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

    return ret;
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

esp_err_t bme280_read_register(bme280_dev_t *bme, uint8_t reg, uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    ESP_ERROR_CHECK(i2c_master_start(cmd));                                                     // Produce start conditon
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (bme->i2c_addr << 1) | I2C_MASTER_WRITE, true)); // Select I2C device
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, true));                                     // Write the register address to be read later

    ESP_ERROR_CHECK(i2c_master_start(cmd));                                                    // Repeated start condition
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (bme->i2c_addr << 1) | I2C_MASTER_READ, true)); // Select I2C device
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data, I2C_MASTER_NACK));                         // Read the data from device
    ESP_ERROR_CHECK(i2c_master_stop(cmd));                                                     // Stop conditon
    esp_err_t ret = i2c_master_cmd_begin(bme->i2c_port, cmd, BME280_I2C_TIMEOUT / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading register 0x%02x", reg);
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
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error writing register 0x%02x with data 0x%02x", reg, data);
    }
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t bme280_read_registers(bme280_dev_t *bme, uint8_t reg, void *data, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    ESP_ERROR_CHECK(i2c_master_start(cmd));                                                     // Produce start conditon
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (bme->i2c_addr << 1) | I2C_MASTER_WRITE, true)); // Select I2C device
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, true));                                     // Write the register address to be read later

    ESP_ERROR_CHECK(i2c_master_start(cmd));                                                    // Repeated start condition
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (bme->i2c_addr << 1) | I2C_MASTER_READ, true)); // Select I2C device
    ESP_ERROR_CHECK(i2c_master_read(cmd, data, size, I2C_MASTER_LAST_NACK));                   // Read the data from device
    ESP_ERROR_CHECK(i2c_master_stop(cmd));                                                     // Stop conditon
    esp_err_t ret = i2c_master_cmd_begin(bme->i2c_port, cmd, BME280_I2C_TIMEOUT / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading registers starting from register: 0x%02x, read size: %d", reg, size);
    }
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t bme280_read_cal_dig(bme280_dev_t *bme, uint8_t reg, uint16_t *dig)
{
    uint8_t tmp[2] = {0, 0};
    esp_err_t ret = bme280_read_registers(bme, reg, tmp, 2);
    *dig = tmp[0] | (tmp[1] << 8);
    return ret;
}

esp_err_t bme280_load_calib_data(bme280_dev_t *bme)
{
    ESP_LOGI(TAG, "Reading BME280 calibration data...");
    esp_err_t ret;

    ret = bme280_read_cal_dig(bme, 0x88, &bme->calib_data.dig_T1);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading calibration digit");
        return ret;
    }
    ESP_LOGD(TAG, "DIG_T1: %d", bme->calib_data.dig_T1);

    ret = bme280_read_cal_dig(bme, 0x8A, (uint16_t *)&bme->calib_data.dig_T2);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading calibration digit");
        return ret;
    }
    ESP_LOGD(TAG, "DIG_T2: %d", bme->calib_data.dig_T2);

    ret = bme280_read_cal_dig(bme, 0x8C, (uint16_t *)&bme->calib_data.dig_T3);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading calibration digit");
        return ret;
    }
    ESP_LOGD(TAG, "DIG_T3: %d", bme->calib_data.dig_T3);

    ret = bme280_read_cal_dig(bme, 0x8E, &bme->calib_data.dig_P1);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading calibration digit");
        return ret;
    }
    ESP_LOGD(TAG, "DIG_P1: %d", bme->calib_data.dig_P1);

    ret = bme280_read_cal_dig(bme, 0x90, (uint16_t *)&bme->calib_data.dig_P2);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading calibration digit");
        return ret;
    }
    ESP_LOGD(TAG, "DIG_P2: %d", bme->calib_data.dig_P2);

    ret = bme280_read_cal_dig(bme, 0x92, (uint16_t *)&bme->calib_data.dig_P3);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading calibration digit");
        return ret;
    }
    ESP_LOGD(TAG, "DIG_P3: %d", bme->calib_data.dig_P3);

    ret = bme280_read_cal_dig(bme, 0x94, (uint16_t *)&bme->calib_data.dig_P4);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading calibration digit");
        return ret;
    }
    ESP_LOGD(TAG, "DIG_P4: %d", bme->calib_data.dig_P4);

    ret = bme280_read_cal_dig(bme, 0x96, (uint16_t *)&bme->calib_data.dig_P5);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading calibration digit");
        return ret;
    }
    ESP_LOGD(TAG, "DIG_P5: %d", bme->calib_data.dig_P5);

    ret = bme280_read_cal_dig(bme, 0x98, (uint16_t *)&bme->calib_data.dig_P6);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading calibration digit");
        return ret;
    }
    ESP_LOGD(TAG, "DIG_P6: %d", bme->calib_data.dig_P6);

    ret = bme280_read_cal_dig(bme, 0x9A, (uint16_t *)&bme->calib_data.dig_P7);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading calibration digit");
        return ret;
    }
    ESP_LOGD(TAG, "DIG_P7: %d", bme->calib_data.dig_P7);

    ret = bme280_read_cal_dig(bme, 0x9C, (uint16_t *)&bme->calib_data.dig_P8);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading calibration digit");
        return ret;
    }
    ESP_LOGD(TAG, "DIG_P8: %d", bme->calib_data.dig_P8);

    ret = bme280_read_cal_dig(bme, 0x9E, (uint16_t *)&bme->calib_data.dig_P9);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading calibration digit");
        return ret;
    }
    ESP_LOGD(TAG, "DIG_P9: %d", bme->calib_data.dig_P9);

    ret = bme280_read_register(bme, 0xA1, &bme->calib_data.dig_H1);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading calibration digit");
        return ret;
    }
    ESP_LOGD(TAG, "DIG_H1: %d", bme->calib_data.dig_H1);

    ret = bme280_read_cal_dig(bme, 0xE1, (uint16_t *)&bme->calib_data.dig_H2);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading calibration digit");
        return ret;
    }
    ESP_LOGD(TAG, "DIG_H2: %d", bme->calib_data.dig_H2);

    ret = bme280_read_register(bme, 0xE3, &bme->calib_data.dig_H3);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading calibration digit");
        return ret;
    }
    ESP_LOGD(TAG, "DIG_H3: %d", bme->calib_data.dig_H3);

    uint8_t dig_h45[3] = {0, 0, 0};
    ret = bme280_read_registers(bme, 0xE4, &dig_h45, 3);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading calibration digit");
        return ret;
    }

    bme->calib_data.dig_H4 = (((uint16_t)dig_h45[0]) << 4) | (dig_h45[1] & 0x0F);
    ESP_LOGD(TAG, "DIG_H4: %d", bme->calib_data.dig_H4);

    bme->calib_data.dig_H5 = (((uint16_t)dig_h45[2]) << 4) | ((dig_h45[1] & 0xF0) >> 4);
    ESP_LOGD(TAG, "DIG_H5: %d", bme->calib_data.dig_H5);

    ret = bme280_read_register(bme, 0xE7, (uint8_t *)&bme->calib_data.dig_H6);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading calibration digit");
        return ret;
    }
    ESP_LOGD(TAG, "DIG_H6: %d", bme->calib_data.dig_H6);

    return ESP_OK;
}