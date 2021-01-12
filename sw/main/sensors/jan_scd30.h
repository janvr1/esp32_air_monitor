#ifndef jan_scd30_h
#define jan_scd30_h

#include <driver/i2c.h>

#define SCD30_I2C_ADDRESS 0x61
#define SCD30_ALPHA 0.25

typedef struct scd30_dev
{
    i2c_port_t i2c_port;
    float temperature;
    float humidity;
    float co2;
    float co2_ewma;
    float alpha;
    uint16_t t_cutoff;
} scd30_dev_t;

typedef struct scd30_fw_ver
{
    uint8_t major;
    uint8_t minor;
} scd30_fw_ver_t;

esp_err_t scd30_begin(scd30_dev_t *scd, i2c_port_t i2c_port, uint16_t interval, uint16_t t_cutoff);
esp_err_t scd30_start_measurement(scd30_dev_t *scd, uint16_t pressure);
esp_err_t scd30_stop_measurement(scd30_dev_t *scd);
esp_err_t scd30_set_interval(scd30_dev_t *scd, uint16_t interval);
int16_t scd30_get_interval(scd30_dev_t *scd);
int8_t scd30_data_ready(scd30_dev_t *scd);
esp_err_t scd30_read_measurement(scd30_dev_t *scd);
esp_err_t scd30_set_asc(scd30_dev_t *scd, bool enabled);
int8_t scd30_get_asc(scd30_dev_t *scd);
esp_err_t scd30_set_frc(scd30_dev_t *scd, uint16_t frc_val);
int16_t scd30_get_frc(scd30_dev_t *scd);
esp_err_t scd30_set_temp_offset(scd30_dev_t *scd, float t_offset);
float scd30_get_temp_offset(scd30_dev_t *scd);
esp_err_t scd30_set_altitude_comp(scd30_dev_t *scd, uint16_t altitude);
int16_t scd30_get_altitude_comp(scd30_dev_t *scd);
esp_err_t scd30_get_fw_version(scd30_dev_t *scd, scd30_fw_ver_t *fw);
esp_err_t scd30_soft_reset(scd30_dev_t *scd);
void scd30_print_config(scd30_dev_t *scd);

#endif