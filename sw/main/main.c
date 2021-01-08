#include <stdio.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include "sensors/jan_bme280.h"

static const gpio_num_t I2C_GPIO_SDA = 32;
static const gpio_num_t I2C_GPIO_SCL = 33;
static const uint32_t i2c_frequency = 1000000;

void app_main(void)
{
    printf("Hello world!\n");

    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_GPIO_SDA,
        .scl_io_num = I2C_GPIO_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_frequency,
    };
    i2c_param_config(I2C_NUM_0, &i2c_conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    bme280_dev_t bme280 = bme280_begin(0x76, I2C_NUM_0);
    bme280_load_calib_data(&bme280);
    bme280_params_t bme280_params = {
        .mode = BME280_MODE_FORCED,
        .standby_time = BME280_STANDBY_MS_1000,
        .iir_filter = BME280_FILTER_X16,
        .oversampling_temp = BME280_SAMPLING_X4,
        .oversampling_pres = BME280_SAMPLING_X4,
        .oversampling_humi = BME280_SAMPLING_X4};
    bme280_set_params(&bme280, &bme280_params);

    while (1)
    {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        bme280_measurement(&bme280);
    }
    i2c_driver_delete(I2C_NUM_0);
}
