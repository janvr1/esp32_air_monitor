// System libraries
#include <stdio.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <mdns.h>
#include <lwip/apps/netbiosns.h>
#include <esp_http_server.h>
#include <esp_task.h>
#include <math.h>
#include <freertos/queue.h>
#include <esp_sntp.h>

// Own libraries
#include "sensors/jan_bme280.h"
#include "sensors/jan_scd30.h"
#include "sensors/jan_veml7700.h"
#include "ledmatrix/led_matrix.h"
#include "web_app/web_app.h"
#include "jan_nvs.h"
#include "jan_wifi.h"

#define APP_MDNS_HOSTNAME "esp32"
#define APP_MDNS_INSTANCE_NAME "ESP32 air monitor"

// Logging tag
static const char *TAG = "MAIN";

// I2C parameters
static const gpio_num_t I2C_GPIO_SDA = 0;
static const gpio_num_t I2C_GPIO_SCL = 4;
static const uint32_t i2c_frequency = 50000;

// IIR filter parameters
#define CO2_INTERVAL 3.0
#define T_CUTOFF 30.0

// LED matrix pins
#define OE 27
#define LATCH 12
#define LINE_A 32
#define LINE_B 33
#define LINE_C 25
#define LINE_D 26

#define GUI_STR_TIME "%02d:%02d:%02d"
#define GUI_STR_DATE "%02d.%02d.%02d"
#define GUI_STR_T_RH "%02d`C %02d%%"
#define GUI_STR_CO2 "%*d ppm"
#define GUI_STR_CO2_HIGH "%d ppm"
#define GUI_STR_BLANK "        "

// Queues for communication between display task and main task
led_matrix_t lm;

void initialize_mdns(char *hostname)
{
    mdns_init();
    mdns_hostname_set(hostname);
    mdns_instance_name_set(APP_MDNS_INSTANCE_NAME);

    netbiosns_init();
    netbiosns_set_name(APP_MDNS_HOSTNAME);
}

void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}

TaskFunction_t display(void *pv)
{
    while (1)
    {
        lm_show_frame(&lm);
        vTaskDelay(1);
        if (lm.duty == 1023)
        {
            vTaskDelay(5 / portTICK_PERIOD_MS);
        }
    }
}

TaskFunction_t startup_animation(void)
{
    while (1)
    {
        for (int i = 0; i < 32; i++)
        {
            for (int j = 0; j < 64; j++)
            {
                lm_set_pixel(&lm, 1, j, i, LM_COLOR_RED);
                vTaskDelay(20 / portTICK_PERIOD_MS);
            }
        }
        lm_clear_frame(&lm);
    }
}

void app_main(void)
{
    printf("Hello world!\n");
    esp_err_t ret;

    esp_log_level_set("*", ESP_LOG_ERROR);
    setenv("TZ", "CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00", 1);
    tzset();

    ret = lm_init(&lm, LINE_A, LINE_B, LINE_C, LINE_D, OE, LATCH);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);

    TaskHandle_t display_task;
    xTaskCreatePinnedToCore(display, "task_display", 4096, NULL, 10, &display_task, 1);

    TaskHandle_t startup_task;
    xTaskCreate(startup_animation, "task_startup_animation", 1024, NULL, 0, &startup_task);

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

    wifi_mode_t wifimode;
    ret = esp_wifi_get_mode(&wifimode);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);

    vTaskDelete(startup_task);
    lm_clear_frame(&lm);

    if (wifimode == WIFI_MODE_STA)
    {
        esp_netif_ip_info_t ip_info;
        esp_netif_get_ip_info(netif, &ip_info);
        uint32_t ipaddr = ntohl(ip_info.ip.addr);
        uint8_t A = ipaddr >> 24;
        uint8_t B = (ipaddr >> 16) & 0xFF;
        uint8_t C = (ipaddr >> 8) & 0xFF;
        uint8_t D = ipaddr & 0xFF;
        char line1[9];
        char line2[9];
        sprintf(line1, "%*d.%d.", 3, A, B);
        sprintf(line2, "%*d.%d", 3, C, D);

        lm_draw_text(&lm, "IP ADDR:", 0, 0, LM_COLOR_RED);
        lm_draw_text(&lm, line1, 1, 0, LM_COLOR_RED);
        lm_draw_text(&lm, line2, 2, 0, LM_COLOR_RED);
    }
    else if (wifimode == WIFI_MODE_AP)
    {
        lm_draw_text(&lm, "AP", 0, 3, LM_COLOR_RED);
        lm_draw_text(&lm, "MODE", 1, 2, LM_COLOR_RED);
        lm_draw_text(&lm, "passwd:", 2, 0, LM_COLOR_RED);
        lm_draw_text(&lm, WIFI_AP_PASS, 3, 0, LM_COLOR_RED);
    }
    else
    {
        ESP_LOGW(TAG, "Unexpected wifi mode");
    }

    // If we are connected to wifi, update time from SNTP
    if (wifimode == WIFI_MODE_STA)
    {
        initialize_sntp();
        // int retry = 0;
        // const int retry_count = 5;
        // while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count)
        // {
        //     ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        //     vTaskDelay(1000 / portTICK_PERIOD_MS);
        // }
    }

    // mDNS configuration
    char hostname[64];
    nvs_get_device_name(hostname);
    if (strlen(hostname) == 0)
    {
        ESP_LOGW(TAG, "Hostname 0 length! using \"esp32\"");
        initialize_mdns("esp32");
    }
    else
    {
        ESP_LOGI(TAG, "Initializing mdns with hostname: %s", hostname);
        initialize_mdns(hostname);
    }

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
    bme280_params.mode = BME280_MODE_FORCED;
    bme280_params.standby_time = BME280_STANDBY_MS_1000;
    ret = bme280_begin(&bme280, &bme280_params, BME280_I2C_ADDRESS, I2C_NUM_0);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Error initializing BME280");

    // SCD30 configuration
    scd30_dev_t scd30;
    ret = scd30_begin(&scd30, I2C_NUM_0, CO2_INTERVAL, T_CUTOFF);
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

    float duty = 10;
    int co2 = 1234, t = 12, rh = 34;
    lm_color_t co2_color = LM_COLOR_GREEN;
    lm_color_t t_rh_color = LM_COLOR_BLUE;

    char *text_time = malloc(8 * sizeof(char) + 1);
    char *text_date = malloc(8 * sizeof(char) + 1);
    char *text_t_rh = malloc(8 * sizeof(char) + 1);
    char *text_co2 = malloc(8 * sizeof(char) + 1);

    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    vTaskDelay(4000 / portTICK_PERIOD_MS);
    lm_clear_frame(&lm);
    // Loop
    while (1)
    {
        ESP_ERROR_CHECK_WITHOUT_ABORT(veml7700_read_als(&veml));
        duty = 0.3 * veml.als + 0.7 * duty;
        if (duty > 1023)
        {
            duty = 1023;
        }
        lm_set_duty(&lm, round(duty));
        ESP_LOGI(TAG, "Duty: %f, %d", duty, lm.duty);

        if (co2 > 999 || duty < 3.5)
        {
            co2_color = LM_COLOR_RED;
            lm_draw_text(&lm, GUI_STR_BLANK, 3, 0, LM_COLOR_GREEN);
        }
        else
        {
            co2_color = LM_COLOR_GREEN;
            lm_draw_text(&lm, GUI_STR_BLANK, 3, 0, LM_COLOR_RED);
        }
        if (duty < 10)
        {
            t_rh_color = LM_COLOR_RED;
            lm_draw_text(&lm, GUI_STR_BLANK, 2, 0, LM_COLOR_BLUE);
        }
        else
        {
            t_rh_color = LM_COLOR_BLUE;
            lm_draw_text(&lm, GUI_STR_BLANK, 2, 0, LM_COLOR_RED);
        }

        time(&now);
        localtime_r(&now, &timeinfo);

        // sprintf(text_time, GUI_STR_TIME, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        strftime(text_time, 9, "%H:%M:%S", &timeinfo);
        lm_draw_text(&lm, text_time, 0, 0, LM_COLOR_RED);
        // sprintf(text_date, GUI_STR_DATE, timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year-2000);
        strftime(text_date, 9, "%d.%m.%y", &timeinfo);
        lm_draw_text(&lm, text_date, 1, 0, LM_COLOR_RED);
        sprintf(text_t_rh, GUI_STR_T_RH, t, rh);
        lm_draw_text(&lm, text_t_rh, 2, 0, t_rh_color);
        sprintf(text_co2, GUI_STR_CO2, 4, co2);
        lm_draw_text(&lm, text_co2, 3, 0, co2_color);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (count % 5 == 0)
        {
            ESP_LOGI(TAG, "Measurement count: %d", count);
            ESP_ERROR_CHECK_WITHOUT_ABORT(bme280_measurement(&bme280));
            t = (int)round(bme280.temperature);
            rh = (int)round(bme280.humidity);

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
            co2 = (int)round(scd30.co2);
        }
        count++;
    }

    i2c_driver_delete(I2C_NUM_0);
}
