// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <stdint.h>
#include <esp_log.h>
#include <esp_err.h>
#include "jan_scd30.h"

static const char *TAG = "JAN_SCD30";

#define SCD30_I2C_TIMEOUT 200
#define SCD30_CMD_START_MEASURE 0x0010
#define SCD30_CMD_STOP_MEASURE 0x0104
#define SCD30_CMD_SET_INTERVAL 0x4600
#define SCD30_CMD_DATA_RDY 0x0202
#define SCD30_CMD_READ_MEAS 0x0300
#define SCD30_CMD_ASC 0x5306
#define SCD30_CMD_FRC 0x5204
#define SCD30_CMD_TEMP_OFFSET 0x5403
#define SCD30_CMD_ALTITUDE 0x5102
#define SCD30_CMD_FW 0xD100
#define SCD30_CMD_RESET 0xD304

uint8_t scd30_crc(uint8_t data[], size_t size);
esp_err_t scd30_write(scd30_dev_t *scd, uint16_t scd_cmd, void *data, size_t size);
esp_err_t scd30_read(scd30_dev_t *scd, void *data, size_t size);

scd30_dev_t scd30_begin(i2c_port_t i2c_port)
{
    scd30_dev_t scd30 = {
        .i2c_port = i2c_port,
        .temperature = 0.0,
        .humidity = 0.0,
        .co2 = 0.0,
    };
    struct scd30_fw_ver fw = scd30_get_fw_version(&scd30);
    ESP_LOGI(TAG, "SCD30 firmware version %d.%d", fw.major, fw.minor);
    return scd30;
}

void scd30_start_measurement(scd30_dev_t *scd, uint16_t pressure)
{
    if ((pressure > 1400) | (pressure < 700))
    {
        pressure = 0;
    }
    uint8_t data[3];
    data[0] = (pressure & 0xFF00) >> 8;
    data[1] = pressure & 0x00FF;
    data[2] = scd30_crc(data, 2);
    scd30_write(scd, SCD30_CMD_START_MEASURE, data, 3);
}

void scd30_stop_measurement(scd30_dev_t *scd)
{
    scd30_write(scd, SCD30_CMD_STOP_MEASURE, 0, 0);
}

void scd30_set_interval(scd30_dev_t *scd, uint16_t interval)
{
    if ((interval < 2) | (interval > 1800))
    {
        interval = 5;
    }
    uint8_t data[3];
    data[0] = (interval & 0xFF00) >> 8;
    data[1] = interval & 0x00FF;
    data[2] = scd30_crc(data, 2);
    scd30_write(scd, SCD30_CMD_SET_INTERVAL, data, 3);
}

uint16_t scd30_get_interval(scd30_dev_t *scd)
{
    // uint8_t test[2] = {0xBE, 0xEF};
    // uint8_t c = scd30_crc(test, 2);
    // ESP_LOGI(TAG, "CRC BEEF: 0x%02x", c);

    uint8_t data[3];
    scd30_write(scd, SCD30_CMD_SET_INTERVAL, 0, 0);
    scd30_read(scd, data, 3);
    uint16_t interval = ((uint16_t)data[0] << 8) | data[1];
    // ESP_LOGI(TAG, "Received interval %d", interval);
    if (data[2] != scd30_crc(data, 2))
    {
        ESP_LOGE(TAG, "CRC mismatch in get interval. Received: 0x%02x, calculated: 0x%02x", data[2], scd30_crc(data, 2));
        return 0;
    }
    return interval;
}

bool scd30_data_ready(scd30_dev_t *scd)
{
    scd30_write(scd, SCD30_CMD_DATA_RDY, 0, 0);
    uint8_t data[3];
    scd30_read(scd, data, 3);
    if (data[2] != scd30_crc(data, 2))
    {
        ESP_LOGE(TAG, "CRC mismatch in data ready. Received: 0x%02x, calculated: 0x%02x", data[2], scd30_crc(data, 2));
        return 0;
    }
    return (bool)data[1];
}

void scd30_read_measurement(scd30_dev_t *scd)
{
    scd30_write(scd, SCD30_CMD_READ_MEAS, 0, 0);
    uint8_t data[18];
    if (scd30_read(scd, data, 18) != ESP_OK)
    {
        return;
    };
    for (uint8_t x = 2; x < 17; x += 3)
    {
        if (data[x] != scd30_crc(&data[x - 2], 2))
        {
            ESP_LOGE(TAG, "CRC mismatch in read measurement at position %d. Received: 0x%02x, calculated: 0x%02x", x, data[x], scd30_crc(&data[x - 2], 2));
            return;
        }
    }
    uint32_t co2_int = (uint32_t)(((uint32_t)data[0] << 24) |
                                  ((uint32_t)data[1] << 16) |
                                  ((uint32_t)data[3] << 8) |
                                  ((uint32_t)data[4]));
    uint32_t t_int = (uint32_t)(((uint32_t)data[6] << 24) |
                                ((uint32_t)data[7] << 16) |
                                ((uint32_t)data[9] << 8) |
                                ((uint32_t)data[10]));
    uint32_t humi_int = (uint32_t)(((uint32_t)data[12] << 24) |
                                   ((uint32_t)data[13] << 16) |
                                   ((uint32_t)data[15] << 8) |
                                   ((uint32_t)data[16]));

    ESP_LOGD(TAG, "Integer temperature: %d", t_int);
    ESP_LOGD(TAG, "Integer humidity: %d", humi_int);
    ESP_LOGD(TAG, "Integer CO2: %d", co2_int);

    scd->temperature = *(float *)&t_int;
    scd->humidity = *(float *)&humi_int;
    scd->co2 = *(float *)&co2_int;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "******** Begin Sensirion SCD30 measurement ********");
    ESP_LOGI(TAG, "Temperature: %f °C", scd->temperature);
    ESP_LOGI(TAG, "Humidity: %f %%RH", scd->humidity);
    ESP_LOGI(TAG, "CO2: %f ppm", scd->co2);
    ESP_LOGI(TAG, "******** End Sensirion SCD30 measurement ********");
    ESP_LOGI(TAG, "");
}

void scd30_set_asc(scd30_dev_t *scd, bool enabled)
{
    uint8_t data[3];
    data[0] = 0;
    data[1] = enabled;
    data[2] = scd30_crc(data, 2);
    scd30_write(scd, SCD30_CMD_ASC, data, 3);
}

bool scd30_get_asc(scd30_dev_t *scd)
{
    scd30_write(scd, SCD30_CMD_ASC, 0, 0);
    uint8_t data[3];
    scd30_read(scd, data, 3);
    if (data[2] != scd30_crc(data, 2))
    {
        ESP_LOGE(TAG, "CRC mismatch in get ASC. Received: 0x%02x, calculated: 0x%02x", data[2], scd30_crc(data, 2));
        return false;
    }
    return (bool)data[1];
}

void scd30_set_frc(scd30_dev_t *scd, uint16_t frc_val)
{
    if ((frc_val < 400) | (frc_val > 2000))
    {
        ESP_LOGE(TAG, "FRC value out of range!");
        return;
    }
    uint8_t data[3];
    data[0] = frc_val >> 8;
    data[1] = frc_val & 0xFF;
    data[2] = scd30_crc(data, 2);
    scd30_write(scd, SCD30_CMD_FRC, data, 3);
}

uint16_t scd30_get_frc(scd30_dev_t *scd)
{
    scd30_write(scd, SCD30_CMD_FRC, 0, 0);
    uint8_t data[3];
    scd30_read(scd, data, 3);
    if (data[2] != scd30_crc(data, 2))
    {
        ESP_LOGE(TAG, "CRC mismatch in get FRC. Received: 0x%02x, calculated: 0x%02x", data[2], scd30_crc(data, 2));
        return 0;
    }
    return (uint16_t)((uint16_t)data[0] << 8 | (uint16_t)data[1]);
}

void scd30_set_temp_offset(scd30_dev_t *scd, float t_offset)
{
    uint16_t offset = (uint16_t)(t_offset * 100);
    uint8_t data[3];
    data[0] = offset >> 8;
    data[1] = offset & 0xFF;
    data[2] = scd30_crc(data, 2);
    scd30_write(scd, SCD30_CMD_TEMP_OFFSET, data, 3);
}

float scd30_get_temp_offset(scd30_dev_t *scd)
{
    scd30_write(scd, SCD30_CMD_TEMP_OFFSET, 0, 0);
    uint8_t data[3];
    scd30_read(scd, data, 3);
    if (data[2] != scd30_crc(data, 2))
    {
        ESP_LOGE(TAG, "CRC mismatch in get temperature offset. Received: 0x%02x, calculated: 0x%02x", data[2], scd30_crc(data, 2));
        return 0;
    }
    uint16_t offset = (uint16_t)((uint16_t)data[0] << 8 | (uint16_t)data[1]);
    ESP_LOGD(TAG, "Read temperature offset %d (int)", offset);
    return (float)offset / 100.0;
}

void scd30_set_altitude_comp(scd30_dev_t *scd, uint16_t altitude)
{
    uint8_t data[3];
    data[0] = altitude >> 8;
    data[1] = altitude & 0xFF;
    data[2] = scd30_crc(data, 2);
    scd30_write(scd, SCD30_CMD_ALTITUDE, data, 3);
}

uint16_t scd30_get_altitude_comp(scd30_dev_t *scd)
{
    scd30_write(scd, SCD30_CMD_ALTITUDE, 0, 0);
    uint8_t data[3];
    scd30_read(scd, data, 3);
    if (data[2] != scd30_crc(data, 2))
    {
        ESP_LOGE(TAG, "CRC mismatch in get altitude compensation. Received: 0x%02x, calculated: 0x%02x", data[2], scd30_crc(data, 2));
        return 0;
    }
    uint16_t altitude = (uint16_t)((uint16_t)data[0] << 8 | (uint16_t)data[1]);
    return altitude;
}

struct scd30_fw_ver scd30_get_fw_version(scd30_dev_t *scd)
{
    scd30_write(scd, SCD30_CMD_FW, 0, 0);
    struct scd30_fw_ver fw = {
        .major = 0,
        .minor = 0,
    };
    uint8_t data[3];
    scd30_read(scd, data, 3);
    if (data[2] != scd30_crc(data, 2))
    {
        ESP_LOGE(TAG, "CRC mismatch in get firmware version. Received: 0x%02x, calculated: 0x%02x", data[2], scd30_crc(data, 2));
        return fw;
    }
    fw.major = data[0];
    fw.minor = data[1];
    return fw;
}

void scd30_print_config(scd30_dev_t *scd)
{
    uint16_t interval = scd30_get_interval(scd);
    float t_off = scd30_get_temp_offset(scd);
    bool asc = scd30_get_asc(scd);
    uint16_t frc = scd30_get_frc(scd);
    uint16_t altitude = scd30_get_altitude_comp(scd);
    ESP_LOGI(TAG, "******** Begin Sensirion SCD30 config ********");
    ESP_LOGI(TAG, "Measurement interval: %d s", interval);
    ESP_LOGI(TAG, "Temperature offset: %f °C", t_off);
    ESP_LOGI(TAG, "Alititude compensation: %d m", altitude);
    ESP_LOGI(TAG, "Automatic Self Calibration: %d", asc);
    ESP_LOGI(TAG, "Forced Recalibration Value: %d", frc);
    ESP_LOGI(TAG, "******** End Sensirion SCD30 config ********");
}

void scd30_soft_reset(scd30_dev_t *scd)
{
    scd30_write(scd, SCD30_CMD_RESET, 0, 0);
}

esp_err_t scd30_write(scd30_dev_t *scd, uint16_t scd_cmd, void *data, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (SCD30_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true)); // Select I2C device
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, scd_cmd >> 8, true));                                // Command MSB
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, scd_cmd & 0xFF, true));                              // Command LSB
    if (size > 0)
    {
        ESP_ERROR_CHECK(i2c_master_write(cmd, data, size, true)); // Data bytes
    }
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t ret = i2c_master_cmd_begin(scd->i2c_port, cmd, SCD30_I2C_TIMEOUT / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading from SCD30: %d %s", ret, esp_err_to_name(ret));
    }
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t scd30_read(scd30_dev_t *scd, void *data, size_t size)
{
    vTaskDelay(5 / portTICK_PERIOD_MS);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (SCD30_I2C_ADDRESS << 1) | I2C_MASTER_READ, true));
    ESP_ERROR_CHECK(i2c_master_read(cmd, data, size, I2C_MASTER_LAST_NACK));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));

    esp_err_t ret = i2c_master_cmd_begin(scd->i2c_port, cmd, SCD30_I2C_TIMEOUT / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading from SCD30: %d %s", ret, esp_err_to_name(ret));
    }
    i2c_cmd_link_delete(cmd);
    return ret;
}

uint8_t scd30_crc(uint8_t data[], size_t size)
{
    uint8_t crc_val = 0xFF;  // initial value
    uint8_t crc_poly = 0x31; // CRC polynomial x^8 + x^5 + x^4 + 1

    for (int j = 0; j < 2; j++)
    {
        crc_val ^= data[j];
        for (int i = 0; i < 8; i++)
        {
            bool xor = crc_val & 0x80;
            crc_val = crc_val << 1;
            if (xor)
                crc_val ^= crc_poly;
        }
    }
    return crc_val;
}