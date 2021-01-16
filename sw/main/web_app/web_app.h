#include <esp_err.h>
#include <esp_vfs.h>
#include <esp_netif.h>
#include <esp_http_server.h>
#include "../sensors/jan_bme280.h"
#include "../sensors/jan_scd30.h"
#include "../sensors/jan_veml7700.h"

#define WEB_APP_BASE_PATH "/spiffs"
#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + 128)
#define CHUNK_BUFSIZE (10240)

typedef struct web_app_context
{
    esp_netif_t *netif;
    bme280_dev_t *bme;
    scd30_dev_t *scd;
    veml7700_dev_t *veml;
    void *buffer;
} web_app_context_t;

esp_err_t web_app_init(
    esp_netif_t *netif,
    bme280_dev_t *bme,
    scd30_dev_t *scd,
    veml7700_dev_t *veml);
void web_app_stop(httpd_handle_t *server);