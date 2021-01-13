#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "led_matrix.h"
#include "font_ibm.h"
#include <esp_err.h>
#include <esp_log.h>
#include <esp32/rom/ets_sys.h>
#include <esp_heap_caps.h>
#include <freertos/task.h>
#include <string.h>
#include <esp_task_wdt.h>
#include <driver/ledc.h>

static const char *TAG = "JAN_LM";

#define LM_DMA_CHAN 2
#define LM_PIN_NUM_MOSI 13
#define LM_PIN_NUM_SCLK 14
#define LM_OE_PWM_CHAN LEDC_CHANNEL_0

esp_err_t lm_output_disable(led_matrix_t *lm);
esp_err_t lm_output_enable(led_matrix_t *lm);
esp_err_t lm_output_disable_fixed(led_matrix_t *lm);
esp_err_t lm_output_enable_fixed(led_matrix_t *lm);
esp_err_t lm_latch_output(led_matrix_t *lm);
esp_err_t lm_select_line(led_matrix_t *lm, uint8_t line);
esp_err_t lm_transfer_line(led_matrix_t *lm, uint16_t line);

esp_err_t lm_init(led_matrix_t *lm,
                  gpio_num_t A,
                  gpio_num_t B,
                  gpio_num_t C,
                  gpio_num_t D,
                  gpio_num_t output_enable,
                  gpio_num_t latch)
{
    esp_err_t ret;

    lm->frame = heap_caps_calloc(3 * 32, sizeof(uint64_t), MALLOC_CAP_DMA);
    lm->line_A = A;
    lm->line_B = B;
    lm->line_C = C;
    lm->line_D = D;
    lm->output_enable = output_enable;
    lm->latch = latch;
    lm->ready = true;
    lm->duty = 512;

    // Configure GPIO outputs
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = ((1ULL << lm->line_A) |
                            (1ULL << lm->line_B) |
                            (1ULL << lm->line_C) |
                            (1ULL << lm->line_C) |
                            (1ULL << lm->line_D) |
                            (1ULL << lm->latch));
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    ret = gpio_config(&io_conf);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);

    // Output enable PWM configuration
    ledc_timer_config_t timercfg = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 78125,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ret = ledc_timer_config(&timercfg);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);

    ledc_channel_config_t chancfg = {
        .gpio_num = output_enable,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LM_OE_PWM_CHAN,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 127,
        .hpoint = 0,
    };
    ret = ledc_channel_config(&chancfg);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);

    // SPI configuration
    spi_bus_config_t spicfg = {
        .miso_io_num = -1,
        .mosi_io_num = LM_PIN_NUM_MOSI,
        .sclk_io_num = LM_PIN_NUM_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 3 * 8};

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .clock_speed_hz = SPI_MASTER_FREQ_40M, //Clock out at 20 MHz
        .mode = 0,                             //SPI mode 0
        .spics_io_num = -1,
        .queue_size = 32,
    };
    //Initialize the SPI bus
    ret = spi_bus_initialize(HSPI_HOST, &spicfg, LM_DMA_CHAN);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    //Attach the LCD to the SPI bus
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &lm->spidev);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);

    lm_output_disable(lm);

    return ret;
}

void lm_set_pixel(led_matrix_t *lm, int val, int x, int y, lm_color_t c)
{
    lm->ready = false;
    int idx = 3 * 8 * y + x / 8; // Index of byte the pixel belongs to;
    idx += 8 * c;                // Add the color
    uint8_t *ptr = lm->frame + idx;
    uint8_t shift = 7 - x % 8; // Bit shift needed to put 1 in correct position
    if (val)
        *ptr |= 1 << shift; // Set the bit
    else
        *ptr &= ~(1 << shift); // Clear the bit
    lm->ready = true;
}

void lm_draw_char(led_matrix_t *lm, unsigned char chr, int row, int pos, lm_color_t c)
{
    lm->ready = false;
    int idx = pos + 8 * c + 24 * 8 * row;
    uint8_t *ptr = lm->frame + idx;
    for (int i = 0; i < 8; i++)
    {
        *ptr = LM_FONT[chr][i];
        ptr += 8 * 3;
    }
    lm->ready = true;
}

esp_err_t lm_draw_text(led_matrix_t *lm, char *text, int row, int pos, lm_color_t c)
{
    if (row > 3 || pos > 7)
    {
        ESP_LOGE(TAG, "Text placement out of range: row=%d, pos=%d", row, pos);
        return ESP_FAIL;
    }
    while (lm->tx_in_progress)
        vTaskDelay(1);
    size_t len = strlen(text);
    lm->ready = false;
    for (int i = 0; i < len; i++)
    {
        if (pos % 8 == 0 && pos > 0)
        {
            pos = 0;
            row++;
        }
        if (row > 3)
        {
            ESP_LOGE(TAG, "Ran out of space on screen");
            return ESP_FAIL;
        }
        lm_draw_char(lm, text[i], row, pos, c);
        pos++;
    }
    lm->ready = true;
    return ESP_OK;
}

esp_err_t lm_show_frame(led_matrix_t *lm)
{
    // ESP_LOGD(TAG, "lm_show_frame() begin");
    esp_err_t ret;
    while (!lm->ready)
        vTaskDelay(1);
    lm->tx_in_progress = true;
    ret = spi_device_acquire_bus(lm->spidev, portMAX_DELAY);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    lm_output_disable(lm);
    for (uint8_t i = 0; i < 16; i++)
    {
        lm_transfer_line(lm, i);

        lm_output_disable(lm);
        ets_delay_us(lm->duty / 2);
        lm_latch_output(lm);
        lm_select_line(lm, i);
        lm_output_enable(lm);
        ets_delay_us(320);
    }
    ets_delay_us(100);
    lm_output_disable(lm);
    spi_device_release_bus(lm->spidev);
    lm->tx_in_progress = false;
    // ESP_LOGD(TAG, "lm_show_frame() end");
    return ret;
}

esp_err_t lm_transfer_line(led_matrix_t *lm, uint16_t line)
{
    // ESP_LOGD(TAG, "lm_transfer_line(%d) begin", line);
    esp_err_t ret;
    uint64_t *tx_0 = (uint64_t *)lm->frame + 3 * line;
    uint64_t *tx_1 = (uint64_t *)lm->frame + 3 * (line + 16);

    spi_transaction_t b1 = {
        .length = 64,
        .rxlength = 0,
        .rx_buffer = NULL,
        .tx_buffer = tx_1 + 2,
    };
    ret = spi_device_polling_start(lm->spidev, &b1, portMAX_DELAY);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    spi_transaction_t b0 = {
        .length = 64,
        .rxlength = 0,
        .rx_buffer = NULL,
        .tx_buffer = tx_0 + 2,
    };
    ret = spi_device_polling_end(lm->spidev, portMAX_DELAY);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    ret = spi_device_polling_start(lm->spidev, &b0, portMAX_DELAY);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);

    spi_transaction_t g1 = {
        .length = 64,
        .rxlength = 0,
        .rx_buffer = NULL,
        .tx_buffer = tx_1 + 1,
    };
    ret = spi_device_polling_end(lm->spidev, portMAX_DELAY);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    ret = spi_device_polling_start(lm->spidev, &g1, portMAX_DELAY);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    spi_transaction_t g0 = {
        .length = 64,
        .rxlength = 0,
        .rx_buffer = NULL,
        .tx_buffer = tx_0 + 1,
    };
    ret = spi_device_polling_end(lm->spidev, portMAX_DELAY);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    ret = spi_device_polling_start(lm->spidev, &g0, portMAX_DELAY);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    spi_transaction_t r1 = {
        .length = 64,
        .rxlength = 0,
        .rx_buffer = NULL,
        .tx_buffer = tx_1,
    };
    ret = spi_device_polling_end(lm->spidev, portMAX_DELAY);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    ret = spi_device_polling_start(lm->spidev, &r1, portMAX_DELAY);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    spi_transaction_t r0 = {
        .length = 64,
        .rxlength = 0,
        .rx_buffer = NULL,
        .tx_buffer = tx_0,
    };
    ret = spi_device_polling_end(lm->spidev, portMAX_DELAY);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    ret = spi_device_polling_start(lm->spidev, &r0, portMAX_DELAY);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    ret = spi_device_polling_end(lm->spidev, portMAX_DELAY);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);

    // ESP_LOGD(TAG, "lm_transfer_line(%d) end", line);

    return ret;
}

esp_err_t lm_deinit(led_matrix_t *lm)
{
    free(lm->frame);
    return ESP_OK;
}

void lm_set_duty(led_matrix_t *lm, uint16_t duty)
{
    if (duty > 1023)
        duty = 1023;
    lm->duty = (1023 - duty);
}

esp_err_t lm_output_disable(led_matrix_t *lm)
{
    esp_err_t ret = ledc_stop(LEDC_HIGH_SPEED_MODE, LM_OE_PWM_CHAN, 1);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    return ret;
}

esp_err_t lm_output_enable(led_matrix_t *lm)
{
    esp_err_t ret = ledc_set_duty(LEDC_HIGH_SPEED_MODE, LM_OE_PWM_CHAN, lm->duty);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    ret = ledc_update_duty(LEDC_HIGH_SPEED_MODE, LM_OE_PWM_CHAN);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);

    return ret;
}

esp_err_t lm_latch_output(led_matrix_t *lm)
{
    esp_err_t ret;
    ret = gpio_set_level(lm->latch, 0);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    ret = gpio_set_level(lm->latch, 1);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    // ets_delay_us(5);
    ret = gpio_set_level(lm->latch, 0);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    return ret;
}

esp_err_t lm_select_line(led_matrix_t *lm, uint8_t line)
{
    esp_err_t ret;
    ret = gpio_set_level(lm->line_A, line & 0x01);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    ret = gpio_set_level(lm->line_B, line & 0x02);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    ret = gpio_set_level(lm->line_C, line & 0x04);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    ret = gpio_set_level(lm->line_D, line & 0x08);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    return ret;
}

void lm_clear_frame(led_matrix_t *lm)
{
    memset(lm->frame, 0, 3 * 32 * sizeof(uint64_t));
}