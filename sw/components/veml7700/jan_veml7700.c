// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "jan_veml7700.h"

#include <esp_log.h>
#include <esp_err.h>
#include <math.h>

#define VEML_I2C_TIMEOUT 50
#define VEML_CMD_CONFIG 0x00
#define VEML_CMD_ALS_WH 0x01
#define VEML_CMD_ALS_WL 0x02
#define VEML_CMD_PS 0x03
#define VEML_CMD_ALS 0x04
#define VEML_CMD_WHITE 0x05
#define VEML_CMD_ALS_INT 0x06

static const char *TAG = "JAN_VEML7700";

esp_err_t veml7700_write(veml7700_dev_t *veml, uint8_t veml_cmd, uint16_t data);
esp_err_t veml7700_read(veml7700_dev_t *veml, uint8_t veml_cmd, uint16_t *data);
esp_err_t veml7700_get_als_conf_reg(veml7700_dev_t *veml, veml7700_als_conf_reg_t *als_conf);
esp_err_t veml7700_set_als_conf_reg(veml7700_dev_t *veml, veml7700_als_conf_reg_t *als_conf);
esp_err_t veml7700_auto_parameters(veml7700_dev_t *veml, uint16_t raw_value);
float veml7700_calculate_lux(veml7700_dev_t *veml, uint16_t data);
float veml7700_gain_to_value(veml7700_gain_t gain);
uint16_t veml7700_integration_to_value(veml7700_integration_t integration);
float veml7700_lux_correction(float lux);
veml7700_gain_t veml7700_gain_inc_dec(veml7700_gain_t g, int8_t inc_dec);
veml7700_integration_t veml7700_integration_inc_dec(veml7700_integration_t it, int8_t inc_dec);

esp_err_t veml7700_begin(veml7700_dev_t *veml, i2c_port_t i2c_port, bool auto_parameters)
{
    esp_err_t ret;
    veml->i2c_port = i2c_port;
    veml->auto_parameters = auto_parameters;
    veml->als = 0;
    veml->white = 0;

    // Power off the sensor
    ret = veml7700_enable(veml, false);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error initializing VEML7700");
        return ret;
    }
    // Write fresh configuration
    ret = veml7700_write(veml, VEML_CMD_CONFIG, 0x1001); // Start with gain x1/8, integration 100 ms
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error initializing VEML7700");
        return ret;
    }
    // Power on the sensor
    ret = veml7700_enable(veml, true);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error initializing VEML7700");
        return ret;
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
    return ESP_OK;
}

esp_err_t veml7700_set_params(veml7700_dev_t *veml, veml7700_gain_t gain, veml7700_integration_t integration)
{
    esp_err_t ret;
    // Power off the sensor
    ret = veml7700_enable(veml, false);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error setting parameters");
        return ret;
    }
    vTaskDelay(3 / portTICK_PERIOD_MS);

    // Write the new configuration
    veml->als_conf.als_gain = gain;
    veml->als_conf.als_it = integration;
    ret = veml7700_set_als_conf_reg(veml, &veml->als_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error setting parameters");
        return ret;
    }
    vTaskDelay(3 / portTICK_PERIOD_MS);

    // Power on the sensor
    ret = veml7700_enable(veml, true);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error setting parameters");
        return ret;
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Successfully set gain: %f, integration: %d ms", veml7700_gain_to_value(veml->als_conf.als_gain), veml7700_integration_to_value(veml->als_conf.als_it));
    return ret;
}

esp_err_t veml7700_read_als(veml7700_dev_t *veml)
{
    esp_err_t ret;
    uint16_t data = 0;
    ret = veml7700_read(veml, VEML_CMD_ALS, &data);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading ALS data");
        return ret;
    }
    ESP_LOGD(TAG, "Read ALS raw data: %d", data);
    veml->als = veml7700_calculate_lux(veml, data);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "******** Begin Vishay VEML7700 measurement ********");
    ESP_LOGI(TAG, "ALS: %f lux", veml->als);
    ESP_LOGI(TAG, "Gain: %f", veml7700_gain_to_value(veml->als_conf.als_gain));
    ESP_LOGI(TAG, "Integration time: %d ms", veml7700_integration_to_value(veml->als_conf.als_it));
    ESP_LOGI(TAG, "******** End Vishay VEML7700 measurement ********");
    ESP_LOGI(TAG, "");
    if (veml->auto_parameters)
    {
        ret = veml7700_auto_parameters(veml, data);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Error setting auto paramters");
        }
    }
    return ret;
}

esp_err_t veml7700_enable(veml7700_dev_t *veml, bool enable)
{
    esp_err_t ret;
    ret = veml7700_get_als_conf_reg(veml, &veml->als_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error enabling VEML7700");
        return ret;
    }
    if (enable) // Enable the sensors
    {
        if (veml->als_conf.als_sd == VEML_ALS_SD_OFF)
        {
            veml->als_conf.als_sd = VEML_ALS_SD_ON;
            ret = veml7700_set_als_conf_reg(veml, &veml->als_conf);
            if (ret != ESP_OK)
                ESP_LOGE(TAG, "Error enabling VEML7700");
            else
                vTaskDelay(5 / portTICK_PERIOD_MS);
        }
        else
            ESP_LOGD(TAG, "VEML7700 was already enabled");
    }
    else // Disable the sensor
    {
        veml->als_conf.als_sd = VEML_ALS_SD_OFF;
        ret = veml7700_set_als_conf_reg(veml, &veml->als_conf);
        if (ret != ESP_OK)
            ESP_LOGE(TAG, "Error disabling VEML7700");
    }
    return ret;
}

float veml7700_calculate_lux(veml7700_dev_t *veml, uint16_t data)
{
    uint16_t it = veml7700_integration_to_value(veml->als_conf.als_it);
    float g = veml7700_gain_to_value(veml->als_conf.als_gain);
    float r = 5.76 / (g * it);
    ESP_LOGD(TAG, "Calculating lux value with g=%f (%d), it=%d (%d), r=%f, raw_data=%d", g, veml->als_conf.als_gain, it, veml->als_conf.als_it, r, data);
    float lux = r * (float)data;
    if (((veml->als_conf.als_gain == VEML_GAIN_x1_8) || (veml->als_conf.als_gain == VEML_GAIN_x1_4)) && lux > 1000)
    {
        lux = veml7700_lux_correction(lux);
    }
    return lux;
}

esp_err_t veml7700_get_als_conf_reg(veml7700_dev_t *veml, veml7700_als_conf_reg_t *als_conf)
{
    uint16_t reg = 0;
    esp_err_t ret;
    ret = veml7700_read(veml, VEML_CMD_CONFIG, &reg);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading als_conf register");
        return ret;
    }

    ESP_LOGD(TAG, "Successfully read als_conf register: 0x%04x", reg);
    als_conf->als_gain = (reg >> 11) & 0x03;
    als_conf->als_it = (reg >> 6) & 0x0F;
    als_conf->als_pers = (reg >> 4) & 0x03;
    als_conf->als_int_en = (reg >> 1) & 0x01;
    als_conf->als_sd = reg & 0x01;

    return ret;
}

esp_err_t veml7700_set_als_conf_reg(veml7700_dev_t *veml, veml7700_als_conf_reg_t *als_conf)
{
    uint16_t reg = 0;
    reg |= ((uint16_t)als_conf->als_gain & 0x03) << 11;
    reg |= ((uint16_t)als_conf->als_it & 0x0F) << 6;
    reg |= ((uint16_t)als_conf->als_pers & 0x03) << 4;
    reg |= ((uint16_t)als_conf->als_int_en & 0x01) << 1;
    reg |= ((uint16_t)als_conf->als_sd) & 0x01;

    esp_err_t ret = veml7700_write(veml, VEML_CMD_CONFIG, reg);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Error setting als_conf register");
    else
        ESP_LOGD(TAG, "Successfully set als_conf register to 0x%04x", reg);
    return ret;
}

esp_err_t veml7700_write(veml7700_dev_t *veml, uint8_t veml_cmd, uint16_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (VEML_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true)); // Select I2C device
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, veml_cmd, true));                                   // Command MSB
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (uint8_t)(data & 0xFF), true));                     // Data LSB
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (uint8_t)(data >> 8), true));                       // Data MSB
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t ret = i2c_master_cmd_begin(veml->i2c_port, cmd, VEML_I2C_TIMEOUT / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret == ESP_OK)
        ESP_LOGD(TAG, "Wrote data 0x%04x to VEML7700 address 0x%02x", data, veml_cmd);
    else
        ESP_LOGE(TAG, "Error reading from VEML7700 address 0x%02x", veml_cmd);

    i2c_cmd_link_delete(cmd);
    return ret;
}
esp_err_t veml7700_read(veml7700_dev_t *veml, uint8_t veml_cmd, uint16_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (VEML_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true)); // Select I2C device
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, veml_cmd, true));                                   // Command MSB
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (VEML_I2C_ADDRESS << 1) | I2C_MASTER_READ, true));
    ESP_ERROR_CHECK(i2c_master_read(cmd, (void *)data, 2, I2C_MASTER_LAST_NACK));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    *data = *data << 8 | *data >> 8; // Swap MSB and LSB
    esp_err_t ret = i2c_master_cmd_begin(veml->i2c_port, cmd, VEML_I2C_TIMEOUT / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret == ESP_OK)
        ESP_LOGD(TAG, "Read data 0x%04x from VEML7700 address 0x%02x", *data, veml_cmd);
    else
        ESP_LOGE(TAG, "Error reading from VEML7700 address 0x%02x", veml_cmd);

    i2c_cmd_link_delete(cmd);
    return ret;
}

float veml7700_lux_correction(float lux)
{
    return 6.0135e-13 * pow(lux, 4) - 9.3924E-09 * pow(lux, 3) + 8.1488E-05 * pow(lux, 2) + 1.0023e+00 * lux;
}

esp_err_t veml7700_auto_parameters(veml7700_dev_t *veml, uint16_t raw_data)
{
    esp_err_t ret;
    if (raw_data < 101)
    {
        if (veml->als_conf.als_gain == VEML_GAIN_x2)
        {
            veml7700_integration_t new_it = veml7700_integration_inc_dec(veml->als_conf.als_it, 1);
            if (new_it != veml->als_conf.als_it)
            {
                ret = veml7700_set_params(veml, veml->als_conf.als_gain, new_it);
                if (ret != ESP_OK)
                {
                    ESP_LOGE(TAG, "Error during auto parameters");
                    return ret;
                }
                else
                    ESP_LOGD(TAG, "AutoParameters incremented integration time to %d", veml7700_integration_to_value(veml->als_conf.als_it));
            }
            else
            {
                ESP_LOGD(TAG, "AutoParameters reached max integration time");
            }
        }
        else
        {
            veml7700_gain_t new_g = veml7700_gain_inc_dec(veml->als_conf.als_gain, 1);
            if (new_g != veml->als_conf.als_gain)
            {
                ret = veml7700_set_params(veml, new_g, veml->als_conf.als_it);
                if (ret != ESP_OK)
                {
                    ESP_LOGE(TAG, "Error during auto parameters");
                    return ret;
                }
                else
                    ESP_LOGD(TAG, "AutoParameters incremented gain to %f", veml7700_gain_to_value(veml->als_conf.als_gain));
            }
            else
            {
                ESP_LOGD(TAG, "AutoParameters reached max gain");
            }
        }
    }
    if (raw_data > 10000)
    {
        if (veml->als_conf.als_it == VEML_IT_25_MS)
        {
            veml7700_gain_t new_g = veml7700_gain_inc_dec(veml->als_conf.als_gain, -1);
            if (new_g != veml->als_conf.als_gain)
            {
                ret = veml7700_set_params(veml, new_g, veml->als_conf.als_it);
                if (ret != ESP_OK)
                {
                    ESP_LOGE(TAG, "Error during auto parameters");
                    return ret;
                }
                else
                    ESP_LOGD(TAG, "AutoParameters decremented gain to %f", veml7700_gain_to_value(veml->als_conf.als_gain));
            }
            else
            {
                ESP_LOGD(TAG, "AutoParameters reached min gain");
            }
        }
        else
        {
            veml7700_integration_t new_it = veml7700_integration_inc_dec(veml->als_conf.als_it, -1);
            if (new_it != veml->als_conf.als_it)
            {
                ret = veml7700_set_params(veml, veml->als_conf.als_gain, new_it);
                if (ret != ESP_OK)
                {
                    ESP_LOGE(TAG, "Error during auto parameters");
                    return ret;
                }
                else
                    ESP_LOGD(TAG, "AutoParameters decremented integration time to %d", veml7700_integration_to_value(veml->als_conf.als_it));
            }
            else
            {
                ESP_LOGD(TAG, "AutoParameters reached min integration time");
            }
        }
    }
    return ESP_OK;
}

veml7700_gain_t veml7700_gain_inc_dec(veml7700_gain_t g, int8_t inc_dec)
{
    if (inc_dec > 0)
    {
        switch (g)
        {
        case VEML_GAIN_x1_8:
            return VEML_GAIN_x1_4;
            break;
        case VEML_GAIN_x1_4:
            return VEML_GAIN_x1;
            break;
        case VEML_GAIN_x1:
            return VEML_GAIN_x2;
            break;
        case VEML_GAIN_x2:
            return VEML_GAIN_x2;
            break;
        default:
            ESP_LOGW(TAG, "Unknown gain value %d", g);
            return g;
            break;
        };
    }

    if (inc_dec < 0)
    {
        switch (g)
        {
        case VEML_GAIN_x1_8:
            return VEML_GAIN_x1_8;
            break;
        case VEML_GAIN_x1_4:
            return VEML_GAIN_x1_8;
            break;
        case VEML_GAIN_x1:
            return VEML_GAIN_x1_4;
            break;
        case VEML_GAIN_x2:
            return VEML_GAIN_x1;
            break;
        default:
            ESP_LOGW(TAG, "Unknown gain value %d", g);
            return g;
            break;
        }
    }
    ESP_LOGW(TAG, "Gain increment: invalid parameter");
    return g;
}

veml7700_integration_t veml7700_integration_inc_dec(veml7700_integration_t it, int8_t inc_dec)
{
    if (inc_dec > 0)
    {
        switch (it)
        {
        case VEML_IT_25_MS:
            return VEML_IT_50_MS;
            break;
        case VEML_IT_50_MS:
            return VEML_IT_100_MS;
            break;
        case VEML_IT_100_MS:
            return VEML_IT_200_MS;
            break;
        case VEML_IT_200_MS:
            return VEML_IT_400_MS;
            break;
        case VEML_IT_400_MS:
            return VEML_IT_800_MS;
            break;
        case VEML_IT_800_MS:
            return VEML_IT_800_MS;
            break;
        default:
            ESP_LOGW(TAG, "Unknown integration value %d", it);
            return it;
            break;
        }
    }
    if (inc_dec < 0)
    {
        switch (it)
        {
        case VEML_IT_25_MS:
            return VEML_IT_25_MS;
            break;
        case VEML_IT_50_MS:
            return VEML_IT_25_MS;
            break;
        case VEML_IT_100_MS:
            return VEML_IT_50_MS;
            break;
        case VEML_IT_200_MS:
            return VEML_IT_100_MS;
            break;
        case VEML_IT_400_MS:
            return VEML_IT_200_MS;
            break;
        case VEML_IT_800_MS:
            return VEML_IT_400_MS;
            break;
        default:
            ESP_LOGW(TAG, "Unknown integration value %d", it);
            return it;
            break;
        }
    }
    ESP_LOGW(TAG, "Integration increment: invalid parameter");

    return it;
}

float veml7700_gain_to_value(veml7700_gain_t gain)
{
    switch (gain)
    {
    case VEML_GAIN_x1_8:
        return 0.125;
        break;
    case VEML_GAIN_x1_4:
        return 0.25;
        break;
    case VEML_GAIN_x1:
        return 1;
        break;
    case VEML_GAIN_x2:
        return 2;
        break;
    default:
        ESP_LOGW(TAG, "Unknown gain value %d", gain);
        return 0.0;
        break;
    };
}
uint16_t veml7700_integration_to_value(veml7700_integration_t integration)
{
    switch (integration)
    {
    case VEML_IT_25_MS:
        return 25;
        break;
    case VEML_IT_50_MS:
        return 50;
        break;
    case VEML_IT_100_MS:
        return 100;
        break;
    case VEML_IT_200_MS:
        return 200;
        break;
    case VEML_IT_400_MS:
        return 400;
        break;
    case VEML_IT_800_MS:
        return 800;
        break;
    default:
        ESP_LOGW(TAG, "Unknown integration value %d", integration);
        return 0.0;
        break;
    };
}