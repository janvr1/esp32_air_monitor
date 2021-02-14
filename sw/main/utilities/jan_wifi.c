#include "jan_wifi.h"

#include <esp_log.h>
#include <esp_err.h>
#include <string.h>

static const char *TAG = "JAN_WIFI";

static uint8_t s_wifi_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

void jan_wifi_deinit(void)
{
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_deinit());
    ESP_ERROR_CHECK(esp_event_loop_delete_default());
}

esp_netif_t *wifi_init_sta(char *ssid, char *pass, bool *connected)
{
    ESP_LOGD(TAG, "Started wifi_init_sta()");

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Add event handlers to later check if we connected successfully
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false},
        },
    };

    strlcpy((char *)wifi_config.sta.ssid, ssid, strlen(ssid) + 1);
    strlcpy((char *)wifi_config.sta.password, pass, strlen(pass) + 1);
    ESP_LOGD(TAG, "SSID: %s, password: %s", wifi_config.sta.ssid, wifi_config.sta.password);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Waiting until either the connection is established (WIFI_STA_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_STA_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_STA_CONNECTED_BIT | WIFI_STA_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler));
    vEventGroupDelete(s_wifi_event_group);
    if (bits & WIFI_STA_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "Successfully connected to SSID %s", ssid);
        *connected = true;
    }
    else if (bits & WIFI_STA_FAIL_BIT)
    {
        ESP_LOGW(TAG, "Failed to connect to SSID %s", ssid);
        *connected = false;
        // ESP_ERROR_CHECK(esp_netif_deinit());
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
    return netif;
}

esp_netif_t *wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *netif = esp_netif_create_default_wifi_ap();
    // ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    // Set the IP address of AP
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 32, 1);
    IP4_ADDR(&ip_info.gw, 192, 168, 32, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);

    // Stop the DHCP server so we can change IP
    esp_netif_dhcps_stop(netif);
    // Change IP
    esp_err_t retval = esp_netif_set_ip_info(netif, &ip_info);
    if (retval != ESP_OK)
    {
        ESP_LOGE("WIFIAP", "Error setting ip: %s", esp_err_to_name(retval));
    }
    // Start the DHCP server
    esp_netif_dhcps_start(netif);

    // Initialize WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // WiFi AP config
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = WIFI_AP_CHANNEL,
            .password = WIFI_AP_PASS,
            .max_connection = WIFI_AP_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK},
    };
    if (strlen(WIFI_AP_PASS) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    // Start WiFi
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi softAP created. SSID:%s password:%s channel:%d",
             WIFI_AP_SSID, WIFI_AP_PASS, WIFI_AP_CHANNEL);
    return netif;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    if (event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW(TAG, "Connection attempt unsuccessful");
        if (s_wifi_retry_num < WIFI_STA_MAX_RETRY)
        {
            esp_wifi_connect();
            s_wifi_retry_num++;
            ESP_LOGI(TAG, "Retrying WiFi connection...");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_STA_FAIL_BIT);
        }
    }
}

static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Successfully connected to WiFi AP. Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_STA_CONNECTED_BIT);
    }
}