// System libraries
#include <stdio.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <mdns.h>
#include <lwip/apps/netbiosns.h>
#include <esp_http_server.h>

// Own libraries
#include "sensors/jan_bme280.h"
#include "sensors/jan_scd30.h"
#include "sensors/jan_veml7700.h"
#include "web_app/web_app.h"
#include "jan_nvs.h"
#include "jan_wifi.h"

#define APP_MDNS_HOSTNAME "esp32"
#define APP_MDNS_INSTANCE_NAME "esp32 air monitor"

// Logging tag
static const char *TAG = "MAIN";

// I2C parameters
static const gpio_num_t I2C_GPIO_SDA = 32;
static const gpio_num_t I2C_GPIO_SCL = 33;
static const uint32_t i2c_frequency = 50000;

// IIR filter parameters
#define CO2_INTERVAL 3.0
#define T_CUTOFF 60.0
#define PI 3.1416

void initialize_mdns(void)
{
    mdns_init();
    mdns_hostname_set(APP_MDNS_HOSTNAME);
    mdns_instance_name_set(APP_MDNS_INSTANCE_NAME);

    // mdns_txt_item_t serviceTxtData[] = {
    //     {"board", "esp32"},
    //     {"path", "/"}};

    // ESP_ERROR_CHECK(mdns_service_add("ESP32-WebServer", "_http", "_tcp", 80, serviceTxtData,
    //                                  sizeof(serviceTxtData) / sizeof(serviceTxtData[0])));

    netbiosns_init();
    netbiosns_set_name(APP_MDNS_HOSTNAME);
}

void app_main(void)
{
    printf("Hello world!\n");
    esp_err_t ret;

    //Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // nvs_set_wifi_ssid_pass("wifi4", "vsegrepozraku");

    // WiFi configuration
    esp_netif_t *netif;
    char ssid[32];
    char pass[64];
    bool connected = false;
    ret = nvs_get_wifi_ssid_pass(ssid, pass);
    if (ret == ESP_OK)
    {
        netif = wifi_init_sta(ssid, pass, &connected);
        if (!connected)
        {
            jan_wifi_deinit();
            netif = wifi_init_softap();
        }
    }
    else
        netif = wifi_init_softap();

    // mDNS configuration
    initialize_mdns();

    // I2C configuration
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

    // BME280 configuration
    bme280_dev_t bme280 = bme280_begin(BME280_I2C_ADDRESS, I2C_NUM_0);
    bme280_load_calib_data(&bme280);
    bme280_params_t bme280_params = {
        .mode = BME280_MODE_FORCED,
        .standby_time = BME280_STANDBY_MS_1000,
        .iir_filter = BME280_FILTER_X16,
        .oversampling_temp = BME280_SAMPLING_X4,
        .oversampling_pres = BME280_SAMPLING_X4,
        .oversampling_humi = BME280_SAMPLING_X4};
    bme280_set_params(&bme280, &bme280_params);

    // SCD30 configuration

    scd30_dev_t scd30 = scd30_begin(I2C_NUM_0, 3, 30);
    scd30_set_altitude_comp(&scd30, 300);
    scd30_start_measurement(&scd30, 0);
    // VEML7700 configuration
    veml7700_dev_t veml = veml7700_begin(I2C_NUM_0, true);
    // veml7700_set_params(&veml, VEML_GAIN_x1, VEML_IT_800_MS);

    // Web app configuration
    // httpd_handle_t server = NULL;
    web_app_init(netif, &bme280, &scd30, &veml);

    // Loop
    while (1)
    {
        vTaskDelay(6000 / portTICK_PERIOD_MS);

        bme280_measurement(&bme280);

        while (!scd30_data_ready(&scd30))
        {
            printf("Waiting for SCD30 data ready...");
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        scd30_read_measurement(&scd30);
        scd30_start_measurement(&scd30, bme280.pressure / 100);
        float offset = scd30.temperature - bme280.temperature;
        if (offset > 0)
        {
            scd30_set_temp_offset(&scd30, offset);
        }
        // vTaskDelay(200 / portTICK_PERIOD_MS);
        // scd30_print_config(&scd30);
        veml7700_read_als(&veml);
    }

    i2c_driver_delete(I2C_NUM_0);
}
