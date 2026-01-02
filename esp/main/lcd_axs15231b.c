#ifdef USE_LCD_ILI9341
#include "esp_log.h"
#include "esp_lcd_panel_io_interface.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_ili9341.h"
#include "driver/ledc.h"
#include "driver/spi_common.h"
#include "common.h"

static const char *TAG = "lcd";

#define TEST_LCD_BIT_PER_PIXEL          (16)
#define TEST_DELAY_TIME_MS              (3000)
#define TEST_READ_TIME_MS               (3000)
#define TEST_READ_PERIOD_MS             (30)

/* SPI Configuration */
#define TEST_LCD_SPI_H_RES              (320)
#define TEST_LCD_SPI_V_RES              (240)
#define TEST_LCD_SPI_HOST               (SPI2_HOST)
#define TEST_PIN_NUM_SPI_CS             (GPIO_NUM_5)
#define TEST_PIN_NUM_SPI_PCLK           (GPIO_NUM_18)
#define TEST_PIN_NUM_SPI_MOSI           (GPIO_NUM_19)
#define TEST_PIN_NUM_SPI_MISO           (GPIO_NUM_21)
#define TEST_PIN_NUM_SPI_RST            (GPIO_NUM_4)
#define TEST_PIN_NUM_SPI_DC             (GPIO_NUM_2)
#define TEST_PIN_NUM_SPI_BL             (GPIO_NUM_15)

static SemaphoreHandle_t refresh_finish = NULL;

#define LCD_LEDC_CH            1
static esp_err_t bsp_display_brightness_init(void)
{
    // Setup LEDC peripheral for PWM backlight control
    const ledc_channel_config_t LCD_backlight_channel = {
        .gpio_num = TEST_PIN_NUM_SPI_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LCD_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = 1,
        .duty = 0,
        .hpoint = 0
    };
    const ledc_timer_config_t LCD_backlight_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = 1,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&LCD_backlight_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&LCD_backlight_channel));

    return ESP_OK;
}

static esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    if (brightness_percent > 100) {
        brightness_percent = 100;
    }
    if (brightness_percent < 0) {
        brightness_percent = 0;
    }

    ESP_LOGI(TAG, "Setting LCD backlight: %d%%", brightness_percent);
    uint32_t duty_cycle = (1023 * brightness_percent) / 100; // LEDC resolution set to 10bits, thus: 100% = 1023
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH, duty_cycle));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH));

    return ESP_OK;
}

void pc_vga_step(void *o);

void lcd_draw(int x_start, int y_start, int x_end, int y_end, void *src)
{
    if (globals.panel) {
        ESP_ERROR_CHECK(
            esp_lcd_panel_draw_bitmap(
                globals.panel,
                x_start, y_start,
                x_end, y_end,
                src));
    }
}

void vga_task(void *arg)
{
    int core_id = esp_cpu_get_core_id();
    fprintf(stderr, "vga runs on core %d\n", core_id);

    ESP_LOGI(TAG, "Initialize BL");
    gpio_config_t init_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << TEST_PIN_NUM_SPI_BL),
    };
    ESP_ERROR_CHECK(gpio_config(&init_gpio_config));
    gpio_set_level(TEST_PIN_NUM_SPI_BL, 1);

    ESP_LOGI(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .mosi_io_num = TEST_PIN_NUM_SPI_MOSI,
        .miso_io_num = TEST_PIN_NUM_SPI_MISO,
        .sclk_io_num = TEST_PIN_NUM_SPI_PCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TEST_LCD_SPI_H_RES * TEST_LCD_SPI_V_RES * TEST_LCD_BIT_PER_PIXEL / 8,
        .flags = SPICOMMON_BUSFLAG_MASTER,
        .intr_flags = 0
    };
    ESP_ERROR_CHECK(spi_bus_initialize(TEST_LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = TEST_PIN_NUM_SPI_DC,
        .cs_gpio_num = TEST_PIN_NUM_SPI_CS,
        .pclk_hz = 40 * 1000 * 1000, // 40MHz
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };

    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)TEST_LCD_SPI_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install ILI9341 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = TEST_PIN_NUM_SPI_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = TEST_LCD_BIT_PER_PIXEL,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
    };
    
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));
    
    // Initialize the panel
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    
    // Set orientation (可根据需要调整)
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, true));
    
    // Turn on the display
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // Initialize backlight
    bsp_display_brightness_init();
    bsp_display_brightness_set(80); // 设置背光为80%

    globals.panel = panel_handle;
    
    // Wait for initialization signal if needed
    xEventGroupWaitBits(global_event_group,
                BIT0,
                pdFALSE,
                pdFALSE,
                portMAX_DELAY);

    ESP_LOGI(TAG, "ILI9341 LCD initialization complete");

    while (1) {
        pc_vga_step(globals.pc);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Cleanup (通常不会执行到这里)
    ESP_ERROR_CHECK(esp_lcd_panel_del(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_io_del(io_handle));
    ESP_ERROR_CHECK(spi_bus_free(TEST_LCD_SPI_HOST));
}
#endif
