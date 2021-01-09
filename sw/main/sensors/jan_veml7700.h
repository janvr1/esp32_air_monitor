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

typedef struct veml7700_params
{
    veml7700_gain_t gain;
    veml7700_integration_t integration;
} veml7700_params_t;

typedef struct veml7700_dev
{
    i2c_port_t i2c_port;
    veml7700_integration_t integration;
    veml7700_gain_t gain;
    bool auto_parameters;
    float als;
    float white;
} veml7700_dev_t;

veml7700_dev_t veml7700_begin(i2c_port_t i2c_port, bool auto_parameters);
void veml7700_set_params(veml7700_dev_t *veml, veml7700_gain_t gain, veml7700_integration_t integration);
void veml7700_read_als(veml7700_dev_t *veml);
void veml7700_read_white(veml7700_dev_t *veml);
void veml7700_enable(veml7700_dev_t *veml, bool enable, uint16_t *config);
esp_err_t veml7700_write(veml7700_dev_t *veml, uint8_t veml_cmd, uint16_t data);
esp_err_t veml7700_read(veml7700_dev_t *veml, uint8_t veml_cmd, uint16_t *data);
// esp_err_t veml7700_read2(veml7700_dev_t *veml, uint8_t veml_cmd, void *data, size_t size);

#endif