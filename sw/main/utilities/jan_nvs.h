#ifndef JAN_NVS_H
#define JAN_NVS_H

#include <nvs_flash.h>

#define NVS_NAMESPACE "storage"

esp_err_t nvs_init(void);
esp_err_t nvs_get_wifi_ssid_pass(char *ssid, char *pass);
esp_err_t nvs_set_wifi_ssid_pass(char *ssid, char *pass);
esp_err_t nvs_get_device_name(char *name);
esp_err_t nvs_set_device_name(char *name);
esp_err_t nvs_get_device_location(char *location);
esp_err_t nvs_set_device_location(char *location);
esp_err_t nvs_get_zrak_api_credentials(char *user, char *pass, uint32_t *dev_id);
esp_err_t nvs_set_zrak_api_credentials(char *user, char *pass, uint32_t dev_id);

#endif