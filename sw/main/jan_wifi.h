#ifndef JAN_WIFI_H
#define JAN_WIFI_H

#include <esp_wifi.h>
#include <freertos/event_groups.h>

#define WIFI_AP_SSID "esp32wifi"
#define WIFI_AP_PASS "qwertyuiop"
#define WIFI_AP_CHANNEL 5
#define WIFI_AP_MAX_STA_CONN 5

#define WIFI_STA_CONNECTED_BIT BIT0
#define WIFI_STA_FAIL_BIT BIT1
#define WIFI_STA_MAX_RETRY 2

esp_netif_t *wifi_init_softap(void);
esp_netif_t *wifi_init_sta(char *ssid, char *pass, bool *connected);
void jan_wifi_deinit(void);

#endif