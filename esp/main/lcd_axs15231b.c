#ifdef USE_LCD_ILI9341
#include "esp_log.h"
#include "esp_lcd_panel_io_interface.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_ili9341.h"
#include "driver/spi_common.h"
#include "common.h"

static const char *TAG = "lcd";

#define TEST_LCD_BIT_PER_PIXEL          (16)
#define TEST_LCD_SPI_H_RES              (320)
#define TEST_LCD_SPI_V_RES              (240)
#define TEST_LCD_SPI_HOST               (SPI2_HOST)
#define TEST_PIN_NUM_SPI_CS             (GPIO_NUM_5)
#define TEST_PIN_NUM_SPI_PCLK           (GPIO_NUM_6)
#define TEST_PIN_NUM_SPI_MOSI           (GPIO_NUM_7)
#define TEST_PIN_NUM_SPI_MISO           (GPIO_NUM_8)
#define TEST_PIN_NUM_SPI_RST            (GPIO_NUM_9)
#define TEST_PIN_NUM_SPI_DC             (GPIO_NUM_18)
// BLK引脚直连3.3V，无需在代码中控制

// 自定义初始化命令序列 - 固定为横屏90度方向
static const lcd_init_cmd_t lcd_init_cmds[] = {
    // 软件复位
    {0x01, (uint8_t []){0x00}, 0, 120},
    // 电源控制A
    {0xCB, (uint8_t []){0x39, 0x2C, 0x00, 0x34, 0x02}, 5, 0},
    // 电源控制B
    {0xCF, (uint8_t []){0x00, 0xC1, 0x30}, 3, 0},
    // 驱动时序控制A
    {0xE8, (uint8_t []){0x85, 0x00, 0x78}, 3, 0},
    // 驱动时序控制B
    {0xEA, (uint8_t []){0x00, 0x00}, 2, 0},
    // 电源控制1
    {0xED, (uint8_t []){0x64, 0x03, 0x12, 0x81}, 4, 0},
    // 电源控制2
    {0xF7, (uint8_t []){0x20}, 1, 0},
    // 泵比率控制
    {0xC0, (uint8_t []){0x23}, 1, 0},
    // 电源控制VRH
    {0xC1, (uint8_t []){0x10}, 1, 0},
    // VCM控制
    {0xC5, (uint8_t []){0x3E, 0x28}, 2, 0},
    // VCM控制2
    {0xC7, (uint8_t []){0x86}, 1, 0},
    // 内存访问控制 - 横屏90度: MY=1, MX=0, MV=1, ML=0, RGB=0, MH=0
    {0x36, (uint8_t []){0x68}, 1, 0},  // 0x68 = 0110 1000
    // 像素格式设置 - 16位像素
    {0x3A, (uint8_t []){0x55}, 1, 0},
    // 帧率控制
    {0xB1, (uint8_t []){0x00, 0x18}, 2, 0},
    // 显示功能控制
    {0xB6, (uint8_t []){0x08, 0x82, 0x27}, 3, 0},
    // 设置列地址 - 0-239
    {0x2A, (uint8_t []){0x00, 0x00, 0x00, 0xEF}, 4, 0},
    // 设置行地址 - 0-319
    {0x2B, (uint8_t []){0x00, 0x00, 0x01, 0x3F}, 4, 0},
    // 3Gamma功能禁用
    {0xF2, (uint8_t []){0x00}, 1, 0},
    // Gamma曲线选择
    {0x26, (uint8_t []){0x01}, 1, 0},
    // 正极性Gamma校正
    {0xE0, (uint8_t []){0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00}, 15, 0},
    // 负极性Gamma校正
    {0xE1, (uint8_t []){0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F}, 15, 0},
    // 退出睡眠模式
    {0x11, (uint8_t []){0x00}, 0, 120},
    // 开启显示
    {0x29, (uint8_t []){0x00}, 0, 100},
};

static SemaphoreHandle_t refresh_finish = NULL;

// 发送自定义初始化命令
static void send_init_commands(esp_lcd_panel_io_handle_t io_handle)
{
    ESP_LOGI(TAG, "Sending custom initialization commands");
    for (int i = 0; i < sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]); i++) {
        const lcd_init_cmd_t *cmd = &lcd_init_cmds[i];
        
        ESP_LOGI(TAG, "Sending command 0x%02X with %d bytes data", cmd->cmd, cmd->data_bytes);
        
        if (cmd->data_bytes > 0) {
            esp_lcd_panel_io_tx_param(io_handle, cmd->cmd, cmd->data, cmd->data_bytes);
        } else {
            esp_lcd_panel_io_tx_param(io_handle, cmd->cmd, NULL, 0);
        }
        
        if (cmd->delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(cmd->delay_ms));
        }
    }
}

void pc_vga_step(void *o);

void lcd_draw(int x_start, int y_start, int x_end, int y_end, void *src)
{
    if (globals.panel) {
        // 横屏90度方向：x=0-239, y=0-319
        if (x_start < 0) x_start = 0;
        if (y_start < 0) y_start = 0;
        if (x_end > 240) x_end = 240;  // 横屏时宽度为240
        if (y_end > 320) y_end = 320;  // 横屏时高度为320
        
        if (x_end > x_start && y_end > y_start) {
            ESP_ERROR_CHECK(
                esp_lcd_panel_draw_bitmap(
                    globals.panel,
                    x_start, y_start,
                    x_end, y_end,
                    src));
        }
    }
}

// 设置横屏90度方向
static void set_landscape_orientation(esp_lcd_panel_handle_t panel)
{
    ESP_LOGI(TAG, "Setting landscape orientation (90 degrees)");
    
    // 使用ESP-IDF内置函数设置方向
    // 横屏90度：交换XY轴，镜像Y轴
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, true));   // 交换X/Y
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, false, true)); // 镜像Y轴
    
    // 设置正确的显示区域（横屏：240x320）
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel, 0, 0));
}

void vga_task(void *arg)
{
    int core_id = esp_cpu_get_core_id();
    fprintf(stderr, "vga runs on core %d\n", core_id);

    ESP_LOGI(TAG, "Initialize LCD (BLK connected to 3.3V - always on)");

    ESP_LOGI(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .mosi_io_num = TEST_PIN_NUM_SPI_MOSI,
        .miso_io_num = TEST_PIN_NUM_SPI_MISO,
        .sclk_io_num = TEST_PIN_NUM_SPI_PCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 240 * 320 * 2, // 横屏尺寸：240x320，16位颜色
        .flags = SPICOMMON_BUSFLAG_MASTER,
        .intr_flags = 0
    };
    ESP_ERROR_CHECK(spi_bus_initialize(TEST_LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = TEST_PIN_NUM_SPI_DC,
        .cs_gpio_num = TEST_PIN_NUM_SPI_CS,
        .pclk_hz = 20 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)TEST_LCD_SPI_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Send custom initialization commands for landscape mode");
    send_init_commands(io_handle);

    ESP_LOGI(TAG, "Install ILI9341 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = TEST_PIN_NUM_SPI_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
    };
    
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));
    
    // 硬件复位
    ESP_LOGI(TAG, "Reset panel");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 初始化面板
    ESP_LOGI(TAG, "Initialize panel");
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    
    // 设置横屏90度方向
    set_landscape_orientation(panel_handle);
    
    // 开启显示
    ESP_LOGI(TAG, "Turn on display");
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    globals.panel = panel_handle;
    
    ESP_LOGI(TAG, "LCD initialization complete - Landscape mode (240x320), BLK always on");

    // 等待初始化信号
    xEventGroupWaitBits(global_event_group, BIT0, pdFALSE, pdFALSE, portMAX_DELAY);

    while (1) {
        pc_vga_step(globals.pc);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // 清理
    ESP_ERROR_CHECK(esp_lcd_panel_del(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_io_del(io_handle));
    ESP_ERROR_CHECK(spi_bus_free(TEST_LCD_SPI_HOST));
}
#endif