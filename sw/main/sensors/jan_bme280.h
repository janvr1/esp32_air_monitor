#ifndef jan_bme280_h
#define jan_bme280_h

#include <driver/i2c.h>

typedef enum bme280_mode
{
    BME280_MODE_SLEEP = 0b00,
    BME280_MODE_NORMAL = 0b11,
    BME280_MODE_FORCED = 0b01
} bme280_mode_t;

typedef enum bme280_oversampling
{
    BME280_SAMPLING_NONE = 0b000,
    BME280_SAMPLING_X1 = 0b001,
    BME280_SAMPLING_X2 = 0b010,
    BME280_SAMPLING_X4 = 0b011,
    BME280_SAMPLING_X8 = 0b100,
    BME280_SAMPLING_X16 = 0b101
} bme280_oversampling_t;

typedef enum bme280_iir_filter
{
    BME280_FILTER_OFF = 0b000,
    BME280_FILTER_X2 = 0b001,
    BME280_FILTER_X4 = 0b010,
    BME280_FILTER_X8 = 0b011,
    BME280_FILTER_X16 = 0b100
} bme280_iir_filter_t;

typedef enum bme280_standby
{
    BME280_STANDBY_MS_0_5 = 0b000,
    BME280_STANDBY_MS_10 = 0b110,
    BME280_STANDBY_MS_20 = 0b111,
    BME280_STANDBY_MS_62_5 = 0b001,
    BME280_STANDBY_MS_125 = 0b010,
    BME280_STANDBY_MS_250 = 0b011,
    BME280_STANDBY_MS_500 = 0b100,
    BME280_STANDBY_MS_1000 = 0b101
} bme280_standby_t;

typedef struct bme280_params
{
    bme280_mode_t mode;
    bme280_standby_t standby_time;
    bme280_iir_filter_t iir_filter;
    bme280_oversampling_t oversampling_temp;
    bme280_oversampling_t oversampling_pres;
    bme280_oversampling_t oversampling_humi;
} bme280_params_t;

typedef struct bme280_calib_data
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
} bme280_calib_data_t;

typedef struct bme280_dev
{
    uint8_t i2c_addr;
    i2c_port_t i2c_port;
    bme280_calib_data_t calib_data;
    float temperature;
    float humidity;
    float pressure;
} bme280_dev_t;

bme280_dev_t bme280_begin(uint8_t i2c_addr, i2c_port_t i2c_port);
void bme280_load_calib_data(bme280_dev_t *bme);
void bme280_soft_reset(bme280_dev_t *bme);
void bme280_default_params(bme280_params_t *params);
void bme280_set_params(bme280_dev_t *bme, bme280_params_t *params);
void bme280_measurement(bme280_dev_t *bme);
#endif