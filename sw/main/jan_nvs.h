#ifndef JAN_NVS_H
#define JAN_NVS_H

#include <nvs_flash.h>

#define NVS_NAMESPACE "storage"

esp_err_t nvs_get_wifi_ssid_pass(char *ssid, char *pass);
esp_err_t nvs_set_wifi_ssid_pass(char *ssid, char *pass);

#endif