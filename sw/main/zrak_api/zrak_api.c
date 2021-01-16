// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "zrak_api.h"
#include <esp_log.h>
#include <esp_http_client.h>
#include <freertos/task.h>
#include <cJSON.h>
#include <string.h>
#include <math.h>

static const char *TAG = "ZRAK_API";

static esp_err_t _http_event_handler(esp_http_client_event_t *evt);

void http_task(void *pvParameters)
{
    zrak_task_params_t *params = (zrak_task_params_t *)pvParameters;

    esp_err_t ret;
    esp_http_client_config_t config = {
        .url = "https://janvr.me",
        .method = HTTP_METHOD_POST,
        .auth_type = HTTP_AUTH_TYPE_BASIC,
        .timeout_ms = 10000,
        .username = params->user,
        .password = params->pass,
        .event_handler = _http_event_handler};

    ESP_LOGI(TAG, "User %s", params->user);
    ESP_LOGI(TAG, "Pass: %s", params->pass);
    esp_http_client_handle_t client = esp_http_client_init(&config);

    char url[64] = {};
    sprintf(url, ZRAK_API_URL, params->dev_id);
    esp_http_client_set_url(client, url);
    ESP_LOGD(TAG, "Url: %s", url);

    esp_http_client_set_header(client, "Content-Type", "application/json");

    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "CO2", round(params->CO2));
    cJSON_AddNumberToObject(json, "RH", round(params->RH * 10) / 10);
    cJSON_AddNumberToObject(json, "T", round(params->T * 10) / 10);
    cJSON_AddNumberToObject(json, "p", round(params->p / 100));
    cJSON_AddNumberToObject(json, "Ev", round(params->E * 10) / 10);

    const char *post_data = cJSON_Print(json);

    ret = esp_http_client_set_post_field(client, post_data, strlen(post_data));
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);

    ret = esp_http_client_perform(client);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);

    ret = esp_http_client_cleanup(client);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    free(params);
    vTaskDelete(NULL);
}

void zrak_send_measurements(char *user, char *pass, int dev_id, float T, float RH, float p, float CO2, float E)
{
    TaskHandle_t task_handle;
    zrak_task_params_t *params = malloc(sizeof(zrak_task_params_t));
    params->user = user;
    params->pass = pass;
    params->dev_id = dev_id;
    params->T = T;
    params->RH = RH;
    params->p = p;
    params->CO2 = CO2;
    params->E = E;

    xTaskCreate(http_task, "zrak_api_http_task", 4096, (void *)params, 0, &task_handle);
}

static esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        if (!esp_http_client_is_chunked_response(evt->client))
        {
            // Write out data
            ESP_LOGD(TAG, "%.*s", evt->data_len, (char *)evt->data);
        }
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    }
    return ESP_OK;
}