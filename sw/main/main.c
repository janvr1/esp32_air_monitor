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

    netbiosns_init();
    netbiosns_set_name(APP_MDNS_HOSTNAME);
}

void app_main(void)
{
    printf("Hello world!\n");
    esp_err_t ret;

    //Initialize NVS
    nvs_init();

    // WiFi configuration
    esp_netif_t *netif;
    char ssid[32];
    char pass[64];
    bool connected = false;
    ret = nvs_get_wifi_ssid_pass(ssid, pass);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (strlen(ssid) == 0 || strlen(pass) == 0)
    {
        ESP_LOGE(TAG, "Wifi SSID/pass is 0 length! Using AP mode...");
        ret = ESP_FAIL;
    }
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
    bme280_dev_t bme280;
    bme280_params_t bme280_params;
    bme280_default_params(&bme280_params);
    bme280_params.iir_filter = BME280_FILTER_X16;
    bme280_params.mode = BME280_MODE_NORMAL;
    bme280_params.standby_time = BME280_STANDBY_MS_1000;
    ret = bme280_begin(&bme280, &bme280_params, BME280_I2C_ADDRESS, I2C_NUM_0);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Error initializing BME280");

    // SCD30 configuration
    scd30_dev_t scd30;
    ret = scd30_begin(&scd30, I2C_NUM_0, 3, 30);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Error initializing SCD30");
    ESP_ERROR_CHECK_WITHOUT_ABORT(scd30_set_altitude_comp(&scd30, 300));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK_WITHOUT_ABORT(scd30_set_temp_offset(&scd30, 0));
    ESP_ERROR_CHECK_WITHOUT_ABORT(scd30_start_measurement(&scd30, 0));

    // VEML7700 configuration
    veml7700_dev_t veml;
    veml7700_begin(&veml, I2C_NUM_0, true);

    // Web app configuration
    // httpd_handle_t server = NULL;
    ret = web_app_init(netif, &bme280, &scd30, &veml);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Successfully started HTTP server");
    }

    uint32_t count = 0;
    int8_t data_ready = 0;
    // Loop
    while (1)
    {
        vTaskDelay(4000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Measurement count: %d", count);

        ESP_ERROR_CHECK_WITHOUT_ABORT(bme280_measurement(&bme280));

        ESP_ERROR_CHECK_WITHOUT_ABORT(veml7700_read_als(&veml));

        data_ready = scd30_data_ready(&scd30);
        while (data_ready != 1)
        {
            printf("Waiting for SCD30 data ready...");
            vTaskDelay(100 / portTICK_PERIOD_MS);
            if (data_ready < 0)
            {
                ESP_LOGE(TAG, "SCD30 data_ready=-1");
            }
            data_ready = scd30_data_ready(&scd30);
        }
        ESP_ERROR_CHECK_WITHOUT_ABORT(scd30_read_measurement(&scd30));

        count++;
    }

    i2c_driver_delete(I2C_NUM_0);
}
