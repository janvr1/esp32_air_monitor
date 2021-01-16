#include "jan_nvs.h"
#include <string.h>
#include <esp_log.h>
#include <esp_err.h>

static const char *TAG = "JAN_NVS";

esp_err_t nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "Erasing NVS flash");
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

esp_err_t nvs_get_zrak_api_credentials(char *user, char *pass, uint32_t *dev_id)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    size_t max_len_user = 64;
    size_t max_len_pass = 64;

    ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS");
        return ret;
    }

    ret = nvs_get_str(nvs_handle, "zrak_user", user, &max_len_user);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading zrak api user from NVS");
        return ret;
    }

    ret = nvs_get_str(nvs_handle, "zrak_pass", pass, &max_len_pass);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading zrak api pass from NVS");
        return ret;
    }

    ret = nvs_get_u32(nvs_handle, "zrak_dev_id", dev_id);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading zrak api dev_id from NVS");
        return ret;
    }

    return ret;
}
esp_err_t nvs_set_zrak_api_credentials(char *user, char *pass, uint32_t dev_id)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;

    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS");
        return ret;
    }

    ret = nvs_set_str(nvs_handle, "zrak_user", user);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error writing zrak api user to NVS");
        return ret;
    }

    ret = nvs_set_str(nvs_handle, "zrak_pass", pass);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error writing zrak api pass to NVS");
        return ret;
    }

    ret = nvs_set_u32(nvs_handle, "zrak_dev_id", dev_id);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error writing zrak api dev_id to NVS");
        return ret;
    }

    return ret;
}

esp_err_t nvs_get_device_name(char *name)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    size_t max_len = 64;

    ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS");
        strlcpy(name, "ESP32", max_len);
        return ret;
    }

    ret = nvs_get_str(nvs_handle, "device_name", name, &max_len);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading device name from NVS");
        strlcpy(name, "ESP32", max_len);
        return ret;
    }
    return ret;
}
esp_err_t nvs_set_device_name(char *name)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;

    // Open NVS
    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS");
        return ret;
    }
    // Set ssid
    ret = nvs_set_str(nvs_handle, "device_name", name);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error writing device_name to NVS");
        return ret;
    }
    return ret;
}
esp_err_t nvs_get_device_location(char *location)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    size_t max_len = 64;

    ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS");
        strlcpy(location, "Planet Earth", max_len);
        return ret;
    }

    ret = nvs_get_str(nvs_handle, "device_location", location, &max_len);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading device location from NVS");
        strlcpy(location, "Planet Earth", max_len);
        return ret;
    }
    return ret;
}
esp_err_t nvs_set_device_location(char *location)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;

    // Open NVS
    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS");
        return ret;
    }
    // Set ssid
    ret = nvs_set_str(nvs_handle, "device_location", location);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error writing device location to NVS");
        return ret;
    }
    return ret;
}

esp_err_t nvs_get_wifi_ssid_pass(char *ssid, char *pass)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    size_t ssid_len = 32, pass_len = 64;

    ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS");
        return ret;
    }

    ret = nvs_get_str(nvs_handle, "wifi_ssid", ssid, &ssid_len);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading SSID from NVS");
        return ret;
    }

    ret = nvs_get_str(nvs_handle, "wifi_pass", pass, &pass_len);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading password from NVS");
        return ret;
    }
    return ESP_OK;
}

esp_err_t nvs_set_wifi_ssid_pass(char *ssid, char *pass)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;

    // Open NVS
    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS");
        return ret;
    }
    // Set ssid
    ret = nvs_set_str(nvs_handle, "wifi_ssid", ssid);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error writing SSID to NVS");
        return ret;
    }
    // Set password
    ret = nvs_set_str(nvs_handle, "wifi_pass", pass);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error writing password to NVS");
        return ret;
    }
    return ESP_OK;
}