#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "web_app.h"
#include <esp_log.h>
#include <string.h>
#include <esp_wifi.h>
#include <esp_spiffs.h>
#include <fcntl.h>
#include <cJSON.h>

#include "../sensors/jan_scd30.h"

#define CHECK_FILE_EXTENSION(filename, ext) (strcasecmp(&filename[strlen(filename) - strlen(ext)], ext) == 0)

static const char *TAG = "JAN_WEB_APP";

// static web_app_context_t web_ctx;

esp_err_t spiffs_init();
static esp_err_t get_file_handler(httpd_req_t *req);
static esp_err_t get_measurement_handler(httpd_req_t *req);
static esp_err_t get_info_handler(httpd_req_t *req);
static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filepath);

esp_err_t web_app_init(esp_netif_t *netif,
                       bme280_dev_t *bme,
                       scd30_dev_t *scd,
                       veml7700_dev_t *veml)
{
    // Return value
    esp_err_t ret;

    // Intialize flash storage
    ret = spiffs_init();
    if (ret != ESP_OK)
        return ret;

    // Intialize HTTP server
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_LOGI(TAG, "Starting HTTP Server");
    ret = httpd_start(&server, &config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error starting HTTP server: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register URI handlers
    web_app_context_t *web_ctx = calloc(1, sizeof(web_app_context_t));
    if (web_ctx == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for web_app_context");
        return ESP_FAIL;
    }
    web_ctx->scd = scd;
    web_ctx->bme = bme;
    web_ctx->veml = veml;
    web_ctx->netif = netif;

    httpd_uri_t get_measurements_uri = {
        .uri = "/api/measurements",
        .method = HTTP_GET,
        .handler = get_measurement_handler,
        .user_ctx = web_ctx};
    httpd_register_uri_handler(server, &get_measurements_uri);

    httpd_uri_t get_info_uri = {
        .uri = "/api/info",
        .method = HTTP_GET,
        .handler = get_info_handler,
        .user_ctx = web_ctx};
    httpd_register_uri_handler(server, &get_info_uri);

    httpd_uri_t get_file_uri = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = get_file_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &get_file_uri);

    return ESP_OK;
}

void web_app_stop(httpd_handle_t *server)
{
    esp_vfs_spiffs_unregister(NULL);
    ESP_LOGI(TAG, "SPIFFS unmounted");
    httpd_stop(server);
}

static esp_err_t get_measurement_handler(httpd_req_t *req)
{
    web_app_context_t *ctx = (web_app_context_t *)req->user_ctx;

    httpd_resp_set_type(req, "application/json");
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "CO2", ctx->scd->co2_ewma);
    cJSON_AddNumberToObject(json, "I", ctx->veml->als);
    cJSON_AddNumberToObject(json, "RH", ctx->bme->humidity);
    cJSON_AddNumberToObject(json, "T", ctx->bme->temperature);
    cJSON_AddNumberToObject(json, "p", ctx->bme->pressure);
    const char *measurements = cJSON_Print(json);
    httpd_resp_sendstr(req, measurements);
    free((void *)measurements);
    cJSON_Delete(json);
    return ESP_OK;
}

static esp_err_t get_info_handler(httpd_req_t *req)
{
    web_app_context_t *ctx = (web_app_context_t *)req->user_ctx;
    wifi_mode_t wifi_mode;
    wifi_ap_record_t ap_info;
    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(ctx->netif, &ip_info);
    uint8_t mac[6] = {};

    httpd_resp_set_type(req, "application/json");
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "Name", "esp32");
    cJSON_AddStringToObject(json, "Location", "Mars");

    esp_wifi_get_mode(&wifi_mode);
    if (wifi_mode == WIFI_MODE_STA)
    {
        esp_wifi_sta_get_ap_info(&ap_info);
        cJSON_AddStringToObject(json, "SSID", (char *)ap_info.ssid);
        esp_wifi_get_mac(WIFI_IF_STA, mac);
    }
    else
    {
        cJSON_AddStringToObject(json, "SSID", "Not connected to WiFi");
        esp_wifi_get_mac(WIFI_IF_AP, mac);
    }

    cJSON *mac_arr = cJSON_CreateArray();
    for (int i = 0; i < 6; i++)
    {
        cJSON_AddItemToArray(mac_arr, cJSON_CreateNumber(mac[i]));
    }
    cJSON_AddItemToObject(json, "MAC", mac_arr);
    cJSON_AddNumberToObject(json, "IP", ntohl(ip_info.ip.addr));
    cJSON_AddBoolToObject(json, "ASC", scd30_get_asc(ctx->scd));
    const char *info = cJSON_Print(json);
    httpd_resp_sendstr(req, info);
    free((void *)info);
    cJSON_Delete(json);
    return ESP_OK;
}

static esp_err_t get_file_handler(httpd_req_t *req)
{
    // Initialize filepath buffer
    char filepath[FILE_PATH_MAX];

    // Copy the base path into file path buffer
    strlcpy(filepath, WEB_APP_BASE_PATH, sizeof(filepath));

    //If uri is / then set file path to /index.html, otherwise set file path to the one in URI
    if (req->uri[strlen(req->uri) - 1] == '/')
        strlcat(filepath, "/index.html", sizeof(filepath));
    else
        strlcat(filepath, req->uri, sizeof(filepath));
    ESP_LOGD(TAG, "get_file_handler(): filepath=%s", req->uri);

    // Open the requested file
    int fd = open(filepath, O_RDONLY, 0);
    if (fd == -1)
    {
        ESP_LOGE(TAG, "Failed to open file: %s", filepath);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read the requested file");
        return ESP_FAIL;
    }

    // Set correct content type
    set_content_type_from_file(req, filepath);

    // Allocated buffer for reading chunks of file
    char *chunk = malloc(CHUNK_BUFSIZE);
    ssize_t read_bytes;
    do
    {
        // Read file in chunks into the chunk buffer
        read_bytes = read(fd, chunk, CHUNK_BUFSIZE);
        if (read_bytes == -1)
        {
            ESP_LOGE(TAG, "Failed to read file : %s", filepath);
        }
        else if (read_bytes > 0)
        {
            /* Send the buffer contents as HTTP response chunk */
            if (httpd_resp_send_chunk(req, chunk, read_bytes) != ESP_OK)
            {
                close(fd);
                ESP_LOGE(TAG, "File sending failed!");
                /* Abort sending file */
                httpd_resp_sendstr_chunk(req, NULL);
                /* Respond with 500 Internal Server Error */
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to serve the requested file");
                return ESP_FAIL;
            }
        }
    } while (read_bytes > 0);
    /* Close file after sending complete */
    close(fd);
    free(chunk);
    ESP_LOGI(TAG, "File sending complete");
    /* Respond with an empty chunk to signal HTTP response completion */
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

esp_err_t spiffs_init()
{
    // SPIFFS configuration
    ESP_LOGI(TAG, "Initializing SPIFFS");
    esp_vfs_spiffs_conf_t conf = {
        .base_path = WEB_APP_BASE_PATH,
        .partition_label = NULL,
        .max_files = 10,
        .format_if_mount_failed = false};

    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ret;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
    return ESP_OK;
}

/* Set HTTP response content type according to file extension */
static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filepath)
{
    const char *type = "text/plain";
    if (CHECK_FILE_EXTENSION(filepath, ".html"))
    {
        type = "text/html";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".js"))
    {
        type = "application/javascript";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".css"))
    {
        type = "text/css";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".png"))
    {
        type = "image/png";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".ico"))
    {
        type = "image/x-icon";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".svg"))
    {
        type = "text/xml";
    }
    return httpd_resp_set_type(req, type);
}