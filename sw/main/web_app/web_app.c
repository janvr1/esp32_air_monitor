// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "web_app.h"
#include <string.h>
#include <fcntl.h>
#include <cJSON.h>
#include <math.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_spiffs.h>
#include <esp_http_server.h>
#include <esp_heap_caps.h>
#include "../utilities/jan_nvs.h"


#define CHECK_FILE_EXTENSION(filename, ext) (strcasecmp(&filename[strlen(filename) - strlen(ext)], ext) == 0)

static const char *TAG = "JAN_WEB_APP";

// static web_app_context_t web_ctx;

esp_err_t spiffs_init();
static esp_err_t get_file_handler(httpd_req_t *req);
static esp_err_t get_measurement_handler(httpd_req_t *req);
static esp_err_t get_info_handler(httpd_req_t *req);
static esp_err_t post_dev_info_handler(httpd_req_t *req);
static esp_err_t post_wifi_handler(httpd_req_t *req);
static esp_err_t post_scd_asc_handler(httpd_req_t *req);
static esp_err_t post_scd_frc_handler(httpd_req_t *req);
static esp_err_t post_zrak_handler(httpd_req_t *req);
static esp_err_t read_req_data(httpd_req_t *req, char *buf, size_t maxlen);
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

    httpd_handle_t server = NULL;

    // Intialize HTTP server
    httpd_config_t conf_httpd = HTTPD_DEFAULT_CONFIG();
    conf_httpd.uri_match_fn = httpd_uri_match_wildcard;
    conf_httpd.core_id = 0;
    conf_httpd.task_priority = 5;

    ESP_LOGI(TAG, "Starting HTTP Server");
    ret = httpd_start(&server, &conf_httpd);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
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
    web_ctx->buffer = malloc(CHUNK_BUFSIZE);

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

    httpd_uri_t post_info_uri = {
        .uri = "/api/dev_info",
        .method = HTTP_POST,
        .handler = post_dev_info_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &post_info_uri);

    httpd_uri_t post_wifi_uri = {
        .uri = "/api/wifi",
        .method = HTTP_POST,
        .handler = post_wifi_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &post_wifi_uri);

    httpd_uri_t post_scd_asc_uri = {
        .uri = "/api/scd_asc",
        .method = HTTP_POST,
        .handler = post_scd_asc_handler,
        .user_ctx = web_ctx};
    httpd_register_uri_handler(server, &post_scd_asc_uri);

    httpd_uri_t post_scd_frc_uri = {
        .uri = "/api/scd_frc",
        .method = HTTP_POST,
        .handler = post_scd_frc_handler,
        .user_ctx = web_ctx};
    httpd_register_uri_handler(server, &post_scd_frc_uri);

    httpd_uri_t post_zrak_uri = {
        .uri = "/api/zrak",
        .method = HTTP_POST,
        .handler = post_zrak_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &post_zrak_uri);

    httpd_uri_t get_file_uri = {.uri = "/*", .method = HTTP_GET, .handler = get_file_handler, .user_ctx = web_ctx};
    httpd_register_uri_handler(server, &get_file_uri);

    return ESP_OK;
}

void web_app_stop(httpd_handle_t *server)
{
    esp_vfs_spiffs_unregister(NULL);
    ESP_LOGI(TAG, "SPIFFS unmounted");
    httpd_stop(server);
}

static esp_err_t post_zrak_handler(httpd_req_t *req)
{
    esp_err_t ret;

    size_t maxlen = 256;
    char *buf = malloc(maxlen);
    ret = read_req_data(req, buf, maxlen);

    if (ret == ESP_ERR_INVALID_SIZE)
    {
        ESP_LOGE(TAG, "Received request data is larger than provided buffer");
        free(buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Content too long");
        return ESP_FAIL;
    }
    if (ret == ESP_FAIL)
    {
        ESP_LOGE(TAG, "Failed to read the request data");
        free(buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read request data");
        return ESP_FAIL;
    }

    cJSON *json = cJSON_Parse(buf);

    if (cJSON_HasObjectItem(json, "user") && cJSON_HasObjectItem(json, "pass") && cJSON_HasObjectItem(json, "dev_id"))
    {
        if (strlen(cJSON_GetObjectItem(json, "user")->valuestring) > 63)
        {
            ESP_LOGE(TAG, "Error: Zrak API username too long");
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Error: Username too long");
            cJSON_Delete(json);
            free(buf);
            return ESP_FAIL;
        }

        if (strlen(cJSON_GetObjectItem(json, "pass")->valuestring) > 63)
        {
            ESP_LOGE(TAG, "Error: Zrak API password too long");
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Error: Password too long");
            cJSON_Delete(json);
            free(buf);
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "Updating zrak API user & pass");

        ret = nvs_set_zrak_api_credentials(cJSON_GetObjectItem(json, "user")->valuestring,
                                           cJSON_GetObjectItem(json, "pass")->valuestring,
                                           cJSON_GetObjectItem(json, "dev_id")->valueint);
        ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
        ESP_LOGD(TAG, "Zrak API user: %s", cJSON_GetObjectItem(json, "user")->valuestring);
        ESP_LOGD(TAG, "Zrak API pass: %s", cJSON_GetObjectItem(json, "pass")->valuestring);
        ESP_LOGD(TAG, "Zrak API dev_id: %d", cJSON_GetObjectItem(json, "dev_id")->valueint);

        if (ret == ESP_OK)
        {
            httpd_resp_sendstr(req, "Success: Updated Zrak API credentials successfully.\n");
        }
        else
        {
            httpd_resp_set_status(req, HTTPD_500);
            httpd_resp_sendstr(req, "Error: Failed to update Zrak API credentials.\n");
            cJSON_Delete(json);
            free(buf);
            return ESP_FAIL;
        }
    }
    else
    {
        ESP_LOGE(TAG, "Zrak API json missing fields");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Error: Missing one of the required fields");
        cJSON_Delete(json);
        free(buf);
        return ESP_FAIL;
    }

    cJSON_Delete(json);
    free(buf);

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    esp_restart();
}

static esp_err_t post_scd_frc_handler(httpd_req_t *req)
{
    char buf[32] = {};
    esp_err_t ret = httpd_req_get_url_query_str(req, buf, 32);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error getting the query string");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Error: Failed to parse query string");
        // free(buf);
        return ESP_FAIL;
    }
    ESP_LOGD(TAG, "Parsed URL query: %s", buf);
    char val[10] = {};
    ret = httpd_query_key_value(buf, "ref", val, 6);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error getting the query string");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Error: Failed to get query value");
        // free(buf);
        // free(val);
        return ESP_FAIL;
    }
    ESP_LOGD(TAG, "Parsed query value: %s", val);
    // free(buf);

    int ref_val = strtol(val, NULL, 10);
    ESP_LOGD(TAG, "Parsed FRC value: %d", ref_val);

    // free(val);
    web_app_context_t *ctx = (web_app_context_t *)req->user_ctx;
    ret = scd30_set_frc(ctx->scd, ref_val);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error performing FRC");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Error: Failed to perform FRC");
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, "Success: SCD calibrated");

    return ESP_OK;
}

static esp_err_t post_scd_asc_handler(httpd_req_t *req)
{
    char buf[32] = {};
    esp_err_t ret = httpd_req_get_url_query_str(req, buf, 32);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error getting the query string");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Error: Failed to parse query string");
        // free(buf);
        return ESP_FAIL;
    }
    ESP_LOGD(TAG, "Parsed URL query: %s", buf);
    char val[2] = {};
    ret = httpd_query_key_value(buf, "enable", val, 2);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error getting the query string");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Error: Failed to get query value");
        // free(buf);
        // free(val);
        return ESP_FAIL;
    }
    ESP_LOGD(TAG, "Parsed query value: %s", val);

    // free(buf);
    bool asc = false;
    if (val[0] == '1')
        asc = true;
    else if (val[0] == '0')
        asc = false;
    else
    {
        ESP_LOGE(TAG, "Invalid query string");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Error: Invalid query string");
        return ESP_FAIL;
    }
    // free(val);
    web_app_context_t *ctx = (web_app_context_t *)req->user_ctx;
    ret = scd30_set_asc(ctx->scd, asc);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error enabling SCD30 ASC");
        if (asc)
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Error: Failed to enable ASC");
        else
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Error: Failed to disable ASC");
        return ESP_FAIL;
    }

    if (asc)
        httpd_resp_sendstr(req, "Success: ASC enabled");
    else
        httpd_resp_sendstr(req, "Success: ASC disabled");

    return ESP_OK;
}

static esp_err_t post_wifi_handler(httpd_req_t *req)
{
    esp_err_t ret;

    size_t maxlen = 256;
    char *buf = malloc(maxlen);
    ret = read_req_data(req, buf, maxlen);

    if (ret == ESP_ERR_INVALID_SIZE)
    {
        ESP_LOGE(TAG, "Received request data is larger than provided buffer");
        free(buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Content too long");
        return ESP_FAIL;
    }
    if (ret == ESP_FAIL)
    {
        ESP_LOGE(TAG, "Failed to read the request data");
        free(buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read request data");
        return ESP_FAIL;
    }

    cJSON *json = cJSON_Parse(buf);

    if (cJSON_HasObjectItem(json, "ssid") && cJSON_HasObjectItem(json, "pass"))
    {
        if (strlen(cJSON_GetObjectItem(json, "ssid")->valuestring) > 63)
        {
            ESP_LOGE(TAG, "Error: WiFi SSID too long");
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Error: WiFi SSID too long");
            cJSON_Delete(json);
            free(buf);
            return ESP_FAIL;
        }

        if (strlen(cJSON_GetObjectItem(json, "pass")->valuestring) > 63)
        {
            ESP_LOGE(TAG, "Error: WiFi password too long");
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Error: WiFi password too long");
            cJSON_Delete(json);
            free(buf);
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Updating wifi ssid and password");

        ret = nvs_set_wifi_ssid_pass(cJSON_GetObjectItem(json, "ssid")->valuestring, cJSON_GetObjectItem(json, "pass")->valuestring);
        ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
        if (ret == ESP_OK)
        {
            httpd_resp_sendstr(req, "Success: Updated WiFi SSID and password successfully.\n");
        }
        else
        {
            httpd_resp_set_status(req, HTTPD_500);
            httpd_resp_sendstr(req, "Error: Failed to update WiFi SSID and password.\n");
            cJSON_Delete(json);
            free(buf);
            return ESP_FAIL;
        }
    }
    else
    {
        ESP_LOGE(TAG, "WiFi SSID or password missing from post data");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Error: Missing SSID or password");
        cJSON_Delete(json);
        free(buf);
        return ESP_FAIL;
    }

    cJSON_Delete(json);
    free(buf);

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    esp_restart();

    return ESP_OK;
}

static esp_err_t post_dev_info_handler(httpd_req_t *req)
{
    esp_err_t ret;

    size_t maxlen = 256;
    char *buf = malloc(maxlen);
    ret = read_req_data(req, buf, maxlen);

    if (ret == ESP_ERR_INVALID_SIZE)
    {
        ESP_LOGE(TAG, "Received request data is larger than provided buffer");
        free(buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Content too long");
        return ESP_FAIL;
    }
    if (ret == ESP_FAIL)
    {
        ESP_LOGE(TAG, "Failed to read the request data");
        free(buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read request data");
        return ESP_FAIL;
    }

    esp_err_t ret_name = ESP_OK, ret_loc = ESP_OK;

    cJSON *json = cJSON_Parse(buf);

    bool has_name = cJSON_HasObjectItem(json, "dev_name");
    bool has_location = cJSON_HasObjectItem(json, "dev_location");

    if (has_name)
    {
        if (strlen(cJSON_GetObjectItem(json, "dev_name")->valuestring) > 63)
        {
            ESP_LOGE(TAG, "Device name too long");
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Error: Device name too long");
            cJSON_Delete(json);
            free(buf);
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Updating device name");

        ret_name = nvs_set_device_name(cJSON_GetObjectItem(json, "dev_name")->valuestring);
        ESP_ERROR_CHECK_WITHOUT_ABORT(ret_name);
    }

    if (has_location)
    {
        if (strlen(cJSON_GetObjectItem(json, "dev_location")->valuestring) > 63)
        {
            ESP_LOGE(TAG, "Error: Device location too long");
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Error: Device location too long");
            cJSON_Delete(json);
            free(buf);
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Updating device location");

        ret_loc = nvs_set_device_location(cJSON_GetObjectItem(json, "dev_location")->valuestring);
        ESP_ERROR_CHECK_WITHOUT_ABORT(ret_loc);
    }

    if (has_name && (ret_name != ESP_OK))
        httpd_resp_set_status(req, HTTPD_500);
    if (has_location && (ret_loc != ESP_OK))
        httpd_resp_set_status(req, HTTPD_500);

    if (has_name && (ret_name == ESP_OK))
        httpd_resp_sendstr_chunk(req, "Success: Successfully updated device name\n");
    else if (has_name)
    {
        httpd_resp_sendstr_chunk(req, "Error: Failed to update device name (");
        httpd_resp_sendstr_chunk(req, esp_err_to_name(ret_name));
        httpd_resp_sendstr_chunk(req, ")\n");
    }
    if (has_location && (ret_loc == ESP_OK))
        httpd_resp_sendstr_chunk(req, "Success: Successfully updated device location\n");
    else if (has_location)
    {
        httpd_resp_sendstr_chunk(req, "Error: Failed to update device location (");
        httpd_resp_sendstr_chunk(req, esp_err_to_name(ret_name));
        httpd_resp_sendstr_chunk(req, ")\n");
    }

    httpd_resp_sendstr_chunk(req, "\0");

    cJSON_Delete(json);
    free(buf);

    return ESP_OK;
}

static esp_err_t get_measurement_handler(httpd_req_t *req)
{
    web_app_context_t *ctx = (web_app_context_t *)req->user_ctx;

    httpd_resp_set_type(req, "application/json");
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "CO2", round(ctx->scd->co2));
    cJSON_AddNumberToObject(json, "CO2ewma", round(ctx->scd->co2_ewma));
    cJSON_AddNumberToObject(json, "T_scd", round(ctx->scd->temperature * 10) / 10.0);
    cJSON_AddNumberToObject(json, "RH_scd", round(ctx->scd->humidity * 10) / 10.0);
    cJSON_AddNumberToObject(json, "I", round(ctx->veml->als * 10) / 10.0);
    cJSON_AddNumberToObject(json, "RH", round(ctx->bme->humidity * 10) / 10.0);
    cJSON_AddNumberToObject(json, "T", round(ctx->bme->temperature * 10) / 10.0);
    cJSON_AddNumberToObject(json, "p", round(ctx->bme->pressure));
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

    char dev_name[64];
    char dev_location[64];
    nvs_get_device_location(dev_location);
    nvs_get_device_name(dev_name);

    httpd_resp_set_type(req, "application/json");
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "name", dev_name);
    cJSON_AddStringToObject(json, "location", dev_location);
    cJSON_AddNumberToObject(json, "free_heap", esp_get_free_heap_size());
    esp_wifi_get_mode(&wifi_mode);
    if (wifi_mode == WIFI_MODE_STA)
    {
        esp_wifi_sta_get_ap_info(&ap_info);
        cJSON_AddStringToObject(json, "ssid", (char *)ap_info.ssid);
        esp_wifi_get_mac(WIFI_IF_STA, mac);
    }
    else
    {
        cJSON_AddStringToObject(json, "ssid", "Not connected to WiFi");
        esp_wifi_get_mac(WIFI_IF_AP, mac);
    }

    cJSON *mac_arr = cJSON_CreateArray();
    for (int i = 0; i < 6; i++)
    {
        cJSON_AddItemToArray(mac_arr, cJSON_CreateNumber(mac[i]));
    }
    cJSON_AddItemToObject(json, "mac", mac_arr);
    cJSON_AddNumberToObject(json, "ip", ntohl(ip_info.ip.addr));
    cJSON_AddBoolToObject(json, "asc", scd30_get_asc(ctx->scd));
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

    if (!strcmp(req->uri, "/settings") || !strcmp(req->uri, "/index"))
        strlcat(filepath, ".html", sizeof(filepath));

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
    web_app_context_t *ctx = (web_app_context_t *)req->user_ctx;
    char *chunk = ctx->buffer;
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
    // free(chunk);
    ESP_LOGI(TAG, "File sending complete");
    /* Respond with an empty chunk to signal HTTP response completion */
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t read_req_data(httpd_req_t *req, char *buf, size_t maxlen)
{
    int total_len = req->content_len;
    if (total_len >= maxlen)
        return ESP_ERR_INVALID_SIZE;
    int cur_len = 0;
    int received = 0;
    while (cur_len < total_len)
    {
        received = httpd_req_recv(req, buf + cur_len, total_len);
        if (received <= 0)
            return ESP_FAIL;
        cur_len += received;
    }
    buf[total_len] = '\0';
    ESP_LOGD(TAG, "Received request data: %s", buf);
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