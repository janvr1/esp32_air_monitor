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

static portMUX_TYPE my_mutex = portMUX_INITIALIZER_UNLOCKED;

esp_err_t lm_output_disable(led_matrix_t *lm);
esp_err_t lm_output_enable(led_matrix_t *lm);
esp_err_t lm_output_disable_fixed(led_matrix_t *lm);
esp_err_t lm_output_enable_fixed(led_matrix_t *lm);
esp_err_t lm_latch_output(led_matrix_t *lm);
esp_err_t lm_select_line(led_matrix_t *lm, uint8_t line);
esp_err_t lm_transfer_line(led_matrix_t *lm, uint16_t line);
uint16_t lm_get_row_offset(int row, lm_color_t color);

esp_err_t lm_init(led_matrix_t *lm,
                  gpio_num_t A,
                  gpio_num_t B,
                  gpio_num_t C,
                  gpio_num_t D,
                  gpio_num_t output_enable,
                  gpio_num_t latch)
{
    esp_err_t ret;

    // lm->frame = heap_caps_calloc(3 * 32, sizeof(uint64_t), MALLOC_CAP_DMA);
    lm->frame = malloc(3 * 32 * sizeof(uint64_t));
    lm_clear_frame(lm);
    lm->line_A = A;
    lm->line_B = B;
    lm->line_C = C;
    lm->line_D = D;
    lm->output_enable = output_enable;
    lm->latch = latch;
    lm->ready = true;
    lm->duty = 64;

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
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 312500,
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
        .duty = 64,
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
        .max_transfer_sz = 48};

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .clock_speed_hz = SPI_MASTER_FREQ_40M, //Clock out at 40 MHz
        .mode = 0,                             //SPI mode 0
        .spics_io_num = -1,
        .queue_size = 16,
    };
    //Initialize the SPI bus
    ret = spi_bus_initialize(HSPI_HOST, &spicfg, 0);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    //Attach the LCD to the SPI bus
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &lm->spidev);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);

    // spi_get_actual_clock()

    lm_output_disable(lm);

    return ret;
}

void lm_set_pixel(led_matrix_t *lm, int val, int x, int y, lm_color_t c)
{
    lm->ready = false;
    uint16_t offset = 1 + 6 * (y % 16) - y / 16 + c;
    uint64_t *row = lm->frame + offset;     // Address of the line (uint64_t) this pixel belongs to
    uint8_t *byte = (uint8_t *)row + x / 8; // Address of the byte this pixel belongs to
    uint8_t shift = 7 - x % 8;              // Bit shift needed to put the pixel in proper positon inside the byte
    if (val)
        *byte |= 1 << shift; // Set the bit
    else
        *byte &= ~(1 << shift); // Clear the bit
    lm->ready = true;
}

uint16_t lm_get_row_offset(int r, lm_color_t color)
{
    return 1 + 6 * (r % 16) - r / 16 + color;
}

void lm_draw_char(led_matrix_t *lm, unsigned char chr, int pos, int row, lm_color_t c)
{
    uint64_t *row_ptr = lm->frame + lm_get_row_offset(row * 8, c);
    uint8_t *byte = (uint8_t *)row_ptr + pos;
    for (int i = 0; i < 8; i++)
    {
        *byte = LM_FONT[chr][i];
        byte += 48;
    }
}

esp_err_t lm_draw_text(led_matrix_t *lm, char *text, int pos, int row, lm_color_t c)
{
    if (pos > 7 || row > 3)
    {
        ESP_LOGE(TAG, "Text placement out of range: row=%d, pos=%d", row, pos);
        return ESP_FAIL;
    }
    while (!lm->ready)
        vTaskDelay(1);
    size_t len = strlen(text);
    // portENTER_CRITICAL(&my_mutex);
    lm->ready = false;
    for (int i = 0; i < len; i++)
    {
        if (pos > 7)
        {
            pos = 0;
            row++;
        }
        if (row > 3)
        {
            ESP_LOGE(TAG, "Ran out of space on screen");
            // portEXIT_CRITICAL(&my_mutex);
            lm->ready = true;
            return ESP_FAIL;
        }
        lm_draw_char(lm, text[i], pos, row, c);
        pos++;
    }
    lm->ready = true;
    // portEXIT_CRITICAL(&my_mutex);
    return ESP_OK;
}

void lm_draw_char_xy(led_matrix_t *lm, unsigned char chr, int x, int y, lm_color_t c)
{
    lm->ready = false;
    uint16_t offset;
    uint64_t *row;
    uint64_t val;

    for (int i = 0; i < 8; i++)
    {
        val = (uint64_t)LM_FONT[chr][i];
        offset = lm_get_row_offset(y, c);
        row = lm->frame + offset;
        *row |= val << x;
        *row &= ~(val << x);
        y++;
    }
    lm->ready = true;
}

esp_err_t lm_draw_text_xy(led_matrix_t *lm, char *text, int x, int y, lm_color_t c)
{
    if (y > 24 || x > 56)
    {
        ESP_LOGE(TAG, "Text placement out of range: x=%d, y=%d", x, y);
        return ESP_FAIL;
    }
    while (!lm->ready)
        vTaskDelay(1);
    size_t len = strlen(text);
    // portENTER_CRITICAL(&my_mutex);
    lm->ready = false;
    for (int i = 0; i < len; i++)
    {
        lm_draw_char(lm, text[i], x, y, c);
        x += 8;
        if (x > 56)
        {
            x = 0;
            y += 8;
        }
        if (y > 24)
        {
            ESP_LOGE(TAG, "Ran out of space on screen");
            // portEXIT_CRITICAL(&my_mutex);
            return ESP_FAIL;
        }
    }
    lm->ready = true;
    // portEXIT_CRITICAL(&my_mutex);
    return ESP_OK;
}

esp_err_t lm_show_frame(led_matrix_t *lm)
{
    esp_err_t ret;
    while (!lm->ready)
        vTaskDelay(1);
    ret = spi_device_acquire_bus(lm->spidev, portMAX_DELAY);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);

    uint8_t t_off = 0.699 * lm->duty;
    uint8_t t_on = 178 - t_off;
    portENTER_CRITICAL(&my_mutex);
    lm->ready = false;
    lm_output_disable(lm);
    for (uint8_t i = 0; i < 16; i++)
    {
        lm_transfer_line(lm, i);
        lm_output_disable(lm);
        ets_delay_us(t_off);
        lm_latch_output(lm);
        lm_select_line(lm, i);
        lm_output_enable(lm);
        ets_delay_us(t_on);
    }
    lm_output_disable(lm);
    lm->ready = true;
    portEXIT_CRITICAL(&my_mutex);
    spi_device_release_bus(lm->spidev);
    return ret;
}

esp_err_t lm_transfer_line(led_matrix_t *lm, uint16_t line)
{
    esp_err_t ret;
    uint64_t *tx_buffer = lm->frame + line * 6;
    spi_transaction_t tx = {
        .length = 384,
        .rxlength = 0,
        .rx_buffer = NULL,
        .tx_buffer = (void *)tx_buffer};
    ret = spi_device_polling_transmit(lm->spidev, &tx);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    return ret;
}

esp_err_t lm_deinit(led_matrix_t *lm)
{
    free(lm->frame);
    return ESP_OK;
}

void lm_set_duty(led_matrix_t *lm, uint16_t duty)
{
    if (duty > 255)
        duty = 255;
    lm->duty = (255 - duty);
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