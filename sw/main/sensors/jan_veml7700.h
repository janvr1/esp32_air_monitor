#ifndef jan_veml7700_h
#define jan_veml7700_h

#include <driver/i2c.h>

#define VEML_I2C_ADDRESS 0x10

typedef enum veml7700_integration
{
    VEML_IT_25_MS = 0b1100,
    VEML_IT_50_MS = 0b1000,
    VEML_IT_100_MS = 0b0000,
    VEML_IT_200_MS = 0b0001,
    VEML_IT_400_MS = 0b0010,
    VEML_IT_800_MS = 0b0011
} veml7700_integration_t;

typedef enum veml7700_gain
{
    VEML_GAIN_x1 = 0b00,
    VEML_GAIN_x2 = 0b01,
    VEML_GAIN_x1_8 = 0b10,
    VEML_GAIN_x1_4 = 0b11,
} veml7700_gain_t;

typedef enum veml7700_persistence
{
    VEML_PERS_1 = 0x00,
    VEML_PERS_2 = 0x01,
    VEML_PERS_4 = 0x10,
    VEML_PERS_8 = 0x11
} veml7700_persistence_t;

typedef enum veml7700_int_en
{
    VEML_INT_DISABLE = 0x00,
    VEML_INT_ENABLE = 0x01,
} veml7700_int_en_t;

typedef enum veml7700_als_sd
{
    VEML_ALS_SD_ON = 0x00,
    VEML_ALS_SD_OFF = 0x01
} veml7700_als_sd_t;

// typedef struct veml7700_params
// {
//     veml7700_gain_t gain;
//     veml7700_integration_t integration;
// } veml7700_params_t;

typedef struct veml7700_als_conf_reg
{
    veml7700_gain_t als_gain;
    veml7700_integration_t als_it;
    veml7700_persistence_t als_pers;
    veml7700_int_en_t als_int_en;
    veml7700_als_sd_t als_sd;
} veml7700_als_conf_reg_t;

typedef struct veml7700_dev
{
    i2c_port_t i2c_port;
    veml7700_als_conf_reg_t als_conf;
    bool auto_parameters;
    float als;
    float white;
} veml7700_dev_t;

esp_err_t veml7700_begin(veml7700_dev_t *veml, i2c_port_t i2c_port, bool auto_parameters);
esp_err_t veml7700_set_params(veml7700_dev_t *veml, veml7700_gain_t gain, veml7700_integration_t integration);
esp_err_t veml7700_read_als(veml7700_dev_t *veml);
esp_err_t veml7700_read_white(veml7700_dev_t *veml);
esp_err_t veml7700_enable(veml7700_dev_t *veml, bool enable);

#endif