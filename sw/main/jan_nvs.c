#include "jan_nvs.h"
#include <string.h>
#include <esp_log.h>
#include <esp_err.h>

static const char *TAG = "JAN_NVS";

esp_err_t nvs_get_wifi_ssid_pass(char *ssid, char *pass)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    size_t ssid_len = 32, pass_len = 64;
    ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = nvs_get_str(nvs_handle, "wifi_ssid", ssid, &ssid_len);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading SSID from NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = nvs_get_str(nvs_handle, "wifi_pass", pass, &pass_len);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading password from NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

esp_err_t nvs_set_wifi_ssid_pass(char *ssid, char *pass)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = nvs_set_str(nvs_handle, "wifi_ssid", ssid);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error writing SSID to NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = nvs_set_str(nvs_handle, "wifi_pass", pass);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error writing password to NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}