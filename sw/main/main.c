// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

// System libraries
#include <stdio.h>
#include <math.h>
#include <mdns.h>
#include <lwip/apps/netbiosns.h>
#include <esp_log.h>
#include <esp_task.h>
#include <esp_sntp.h>
#include <esp_wifi.h>
#include <driver/i2c.h>

// Own libraries
#include <jan_bme280.h>
#include <jan_scd30.h>
#include <jan_veml7700.h>
#include <led_matrix.h>
#include <zrak_api.h>

#include "web_app/web_app.h"
#include "jan_nvs.h"
#include "jan_wifi.h"

// Logging tag
static const char *TAG = "MAIN";

// MDNS
#define APP_MDNS_INSTANCE_NAME "ESP32 air monitor"

// I2C parameters
#define I2C_GPIO_SDA 0
#define I2C_GPIO_SCL 4
#define I2C_FREQUENCY 50000

// IIR filter parameters (SCD30)
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
#define GUI_STR_BLANK "        "

// Led matrix handle
led_matrix_t lm;

// Function declarations
static void initialize_mdns(char *hostname);
static void initialize_sntp(void);
float calculate_duty(float lux);

// Task functions
static void display(void *pvParameters);
static void startup_animation(void *pvParameters);

void app_main(void)
{
    printf("Hello world!\n");
    esp_err_t ret;

    // esp_log_level_set(TAG, ESP_LOG_WARN);

    // I2C configuration
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_GPIO_SDA,
        .scl_io_num = I2C_GPIO_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQUENCY,
    };
    i2c_param_config(I2C_NUM_0, &i2c_conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    // BME280 configuration
    bme280_dev_t bme280;
    bme280_params_t bme280_params;
    bme280_default_params(&bme280_params);
    bme280_params.iir_filter = BME280_FILTER_X8;
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

    // Timezone confiugration
    setenv("TZ", "CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00", 1);
    tzset();

    // LED matrix configuration
    ret = lm_init(&lm, LINE_A, LINE_B, LINE_C, LINE_D, OE, LATCH);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);

    lm_draw_text(&lm, "Hello", 0, 1, LM_COLOR_GREEN);
    lm_draw_text(&lm, "world!", 2, 2, LM_COLOR_GREEN);

    // Create the task that refreshes the display
    TaskHandle_t display_task;
    xTaskCreatePinnedToCore(display, "task_display", 4096, NULL, 10, &display_task, 1);

    // Create the task that draws the startup animation
    TaskHandle_t startup_task;
    xTaskCreate(startup_animation, "task_startup_animation", 1024, NULL, 0, &startup_task);

    // Initialize NVS
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

    // Stop the startup animation
    vTaskDelete(startup_task);

    // Clear the frame
    lm_clear_frame(&lm);

    // Display the network information on screen
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
        lm_draw_text(&lm, line1, 0, 2, LM_COLOR_RED);
        lm_draw_text(&lm, line2, 1, 3, LM_COLOR_RED);
    }
    else if (wifimode == WIFI_MODE_AP)
    {
        lm_draw_text(&lm, "AP", 3, 0, LM_COLOR_RED);
        lm_draw_text(&lm, "MODE", 2, 1, LM_COLOR_RED);
        lm_draw_text(&lm, "passwd:", 0, 2, LM_COLOR_RED);
        lm_draw_text(&lm, WIFI_AP_PASS, 0, 3, LM_COLOR_RED);
    }
    else
    {
        ESP_LOGW(TAG, "Unexpected wifi mode");
    }

    // If we are connected to wifi, update time from SNTP
    if (wifimode == WIFI_MODE_STA)
        initialize_sntp();

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

    // Zrak API configuration
    bool zrak_api = false;
    bool zrak_api_sent = false;
    char zrak_user[64] = {};
    char zrak_pass[64] = {};
    uint32_t zrak_dev_id = 0;
    ret = nvs_get_zrak_api_credentials(zrak_user, zrak_pass, &zrak_dev_id);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (strlen(zrak_user) > 0 && strlen(zrak_pass) > 0 && zrak_dev_id != 0)
    {
        zrak_api = true;
        ESP_LOGI(TAG, "Zrak API activated");
    }
    else
    {
        ESP_LOGW(TAG, "Zraki API not activated");
    }
    ESP_LOGI(TAG, "Zrak API user: %s", zrak_user);
    ESP_LOGI(TAG, "Zrak API pass: %s", zrak_pass);
    ESP_LOGI(TAG, "Zrak API dev_id: %d", zrak_dev_id);

    // Web app configuration
    // httpd_handle_t server = NULL;
    ret = web_app_init(netif, &bme280, &scd30, &veml);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Successfully started HTTP server");
    }

    uint32_t count = 0;
    int8_t data_ready = 0;

    // Variables to display on screen
    float duty = 10;
    int co2 = 0, t = 0, rh = 0;
    lm_color_t co2_color = LM_COLOR_GREEN;
    lm_color_t t_rh_color = LM_COLOR_BLUE;

    char text_time[9] = {};
    char text_date[9] = {};
    char text_t_rh[9] = {};
    char text_co2[9] = {};

    // Time keeping
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    // Delay so the netowrk info stays on screen for longer
    vTaskDelay(4000 / portTICK_PERIOD_MS);
    lm_clear_frame(&lm);

    // Loop
    while (1)
    {
        // Read ambient light data and set duty cycle accordingly
        ESP_ERROR_CHECK_WITHOUT_ABORT(veml7700_read_als(&veml));
        duty = 0.3 * calculate_duty(veml.als) + 0.7 * duty;
        lm_set_duty(&lm, round(duty));
        ESP_LOGI(TAG, "Duty: %f, %d", round(duty), lm.duty);

        // If ambient light is low, change T and RH display to red color
        if (duty < 15)
        {
            t_rh_color = LM_COLOR_RED;
            lm_draw_text(&lm, GUI_STR_BLANK, 0, 2, LM_COLOR_BLUE);
            sprintf(text_t_rh, GUI_STR_T_RH, t, rh);
            lm_draw_text(&lm, text_t_rh, 0, 2, t_rh_color);
        }
        else
        {
            t_rh_color = LM_COLOR_BLUE;
            lm_draw_text(&lm, GUI_STR_BLANK, 0, 2, LM_COLOR_RED);
            sprintf(text_t_rh, GUI_STR_T_RH, t, rh);
            lm_draw_text(&lm, text_t_rh, 0, 2, t_rh_color);
        }

        // Read current time
        time(&now);
        localtime_r(&now, &timeinfo);

        // Update time on screen
        strftime(text_time, 9, "%H:%M:%S", &timeinfo);
        lm_draw_text(&lm, text_time, 0, 0, LM_COLOR_RED);
        strftime(text_date, 9, "%d.%m.%y", &timeinfo);
        lm_draw_text(&lm, text_date, 0, 1, LM_COLOR_RED);

        ESP_LOGI(TAG, "Count: %d", count / 5);

        // Every 5 seconds, read the sensor data
        if (count % 5 == 0)
        {
            ESP_LOGI(TAG, "Measurement count: %d", count / 5);

            // Read BME280
            ESP_ERROR_CHECK_WITHOUT_ABORT(bme280_measurement(&bme280));
            t = (int)round(bme280.temperature);
            rh = (int)round(bme280.humidity);

            // Wait for SCD30 data ready
            do
            {
                data_ready = scd30_data_ready(&scd30);
                if (data_ready < 0)
                    ESP_LOGI(TAG, "SCD30 data_ready -1");
                else if (data_ready < 1)
                    ESP_LOGI(TAG, "Waiting for SCD30 data to be ready");
                vTaskDelay(100 / portTICK_PERIOD_MS);
            } while (data_ready != 1);

            // Read SCD30 data
            ESP_ERROR_CHECK_WITHOUT_ABORT(scd30_read_measurement(&scd30));
            co2 = (int)round(scd30.co2);

            // If CO2 value is over 1499 ppm (or ambient light is very low), change its display color to red
            if (co2 > 1499 || duty < 3.5)
            {
                co2_color = LM_COLOR_RED;
                lm_draw_text(&lm, GUI_STR_BLANK, 0, 3, LM_COLOR_GREEN);
            }
            else
            {
                co2_color = LM_COLOR_GREEN;
                lm_draw_text(&lm, GUI_STR_BLANK, 0, 3, LM_COLOR_RED);
            }

            // Draw the sensor data on screen
            sprintf(text_t_rh, GUI_STR_T_RH, t, rh);
            lm_draw_text(&lm, text_t_rh, 0, 2, t_rh_color);
            sprintf(text_co2, GUI_STR_CO2, 4, co2);
            lm_draw_text(&lm, text_co2, 0, 3, co2_color);

            // Every half an hour, send Zrak API data to cloud, if enabled
            if (zrak_api && !zrak_api_sent && (timeinfo.tm_min == 30 || timeinfo.tm_min == 0))
            {
                zrak_send_measurements(zrak_user, zrak_pass, zrak_dev_id,
                                       bme280.temperature,
                                       bme280.humidity,
                                       bme280.pressure,
                                       scd30.co2,
                                       veml.als);

                zrak_api_sent = true;
            }
            else if (zrak_api && zrak_api_sent && (timeinfo.tm_min == 31 || timeinfo.tm_min == 1))
                zrak_api_sent = false;
        }
        count++;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void initialize_mdns(char *hostname)
{
    mdns_init();
    mdns_hostname_set(hostname);
    mdns_instance_name_set(APP_MDNS_INSTANCE_NAME);

    netbiosns_init();
    netbiosns_set_name(hostname);
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}

static void display(void *pvParameters)
{
    while (1)
    {
        lm_show_frame(&lm);
        vTaskDelay(1);
        if (lm.duty == 255)
        {
            vTaskDelay(5 / portTICK_PERIOD_MS);
        }
    }
}
static void startup_animation(void *pvParameters)
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

float calculate_duty(float lux)
{
    if (lux < 10)
        return lux * 2;
    else if (lux < 30)
        return 20 + lux;
    else if (lux < 100)
        return 50 + lux / 2;
    else if (lux < 620)
        return 100 + lux / 4;
    else
        return 255;
}