#ifndef JAN_LEDMATRIX_H
#define JAN_LEDMATRIX_H
#include <driver/gpio.h>
#include <driver/spi_master.h>

#define LM_WIDTH 64
#define LM_HEIGHT 32

typedef enum lm_color
{
    LM_COLOR_RED = 4,
    LM_COLOR_GREEN = 2,
    LM_COLOR_BLUE = 0,
} lm_color_t;

typedef struct led_matrix
{
    gpio_num_t line_A;
    gpio_num_t line_B;
    gpio_num_t line_C;
    gpio_num_t line_D;
    gpio_num_t output_enable;
    gpio_num_t latch;
    uint64_t *frame;
    bool ready;
    uint16_t duty;
    spi_device_handle_t spidev;

} led_matrix_t;

esp_err_t lm_init(led_matrix_t *lm,
                  gpio_num_t A,
                  gpio_num_t B,
                  gpio_num_t C,
                  gpio_num_t D,
                  gpio_num_t output_enable,
                  gpio_num_t latch);
esp_err_t lm_deinit(led_matrix_t *lm);
esp_err_t lm_show_frame(led_matrix_t *lm);
void lm_set_duty(led_matrix_t *lm, uint16_t duty);
void lm_set_pixel(led_matrix_t *lm, int val, int x, int y, lm_color_t c);
void lm_draw_char(led_matrix_t *lm, unsigned char chr, int pos, int row, lm_color_t c);
void lm_draw_char_xy(led_matrix_t *lm, unsigned char chr, int x, int y, lm_color_t c);
esp_err_t lm_draw_text(led_matrix_t *lm, char *text, int pos, int row, lm_color_t c);
esp_err_t lm_draw_text_xy(led_matrix_t *lm, char *text, int x, int y, lm_color_t c);

void lm_clear_frame(led_matrix_t *lm);
#endif