#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include <string.h>
#include "driver/i2c.h"
#include "lvgl.h"
#include "demos/lv_demos.h"

#define ST7796_SCLK   GPIO_NUM_18
#define ST7796_MOSI     GPIO_NUM_23
#define ST7796_CS       GPIO_NUM_5
#define ST7796_DC       GPIO_NUM_27
#define ST7796_RST      GPIO_NUM_26
#define BL_PIN          GPIO_NUM_33
#define TP_SDA      GPIO_NUM_21
#define TP_SCL      GPIO_NUM_22
#define TP_INT      GPIO_NUM_25
#define TP_RST      GPIO_NUM_32
// 所有应用层颜色统一使用 RGB565（R5-G6-B5）格式。
// 这些值可以直接传给 st7796_fill_color()；用于 LVGL 时通过
// lv_color_from_rgb565() 转换，以适配 LV_COLOR_16_SWAP。
#define RED_COLOR       0xF800
#define GREEN_COLOR     0x07E0
#define BLUE_COLOR      0x13BF  // 由原来的 RGB888 0x1677FF 量化得到
#define WHITE_COLOR     0xFFFF
#define BLACK_COLOR     0x0000
#define YELLOW_COLOR    0xFF00  // 由原来的 RGB888 0xFFE000 量化得到

#define FT6336U_ADDR 0x38
#define I2C_TP_PORT I2C_NUM_0

// #define LCD_MADCTL_PORTRAIT   0x48  // 竖屏
// #define ST7796_WIDTH    320
// #define ST7796_HEIGHT   480

// #define LCD_MADCTL_LANDSCAPE  0x28  // 横屏
#define ST7796_HEIGHT    320
#define ST7796_WIDTH   480
#define LCD_MADCTL_LANDSCAPE  0xE8  // 横屏 + 旋转 180°
#define ST7796_SPI_CLOCK_HZ (32 * 1000 * 1000)

volatile bool tp_touched_flag = false;

// 颜色切换列表
uint8_t color_idx = 0;
const uint16_t color_list[] = {GREEN_COLOR, RED_COLOR, BLUE_COLOR, WHITE_COLOR, BLACK_COLOR, YELLOW_COLOR};
#define COLOR_CNT (sizeof(color_list)/sizeof(color_list[0]))

// 新增消抖时间戳
spi_device_handle_t st7796_spi;
// LVGL显示缓冲区
static lv_color_t disp_buf[ST7796_WIDTH * 100];
static lv_disp_draw_buf_t draw_buf;
static lv_disp_drv_t disp_drv;

static inline lv_color_t lv_color_from_rgb565(uint16_t rgb565)
{
    uint8_t r = (uint8_t)(((rgb565 >> 11) & 0x1F) << 3);
    uint8_t g = (uint8_t)(((rgb565 >> 5) & 0x3F) << 2);
    uint8_t b = (uint8_t)((rgb565 & 0x1F) << 3);
    return lv_color_make(r, g, b);
}

// ESP32 SPI DMA 单次传输的安全上限是 4092 字节（不是 4096）。
// 超过该值时 spi_device_transmit 会返回 ESP_ERR_INVALID_ARG，原代码未检查该错误，
// 会导致像素数据没有发到屏幕，屏幕保持白色。
#define LVGL_SPI_MAX_CHUNK_BYTES 4092

// 导入字体
LV_FONT_DECLARE(weiruan);

static lv_indev_drv_t indev_drv;
static lv_indev_t *touch_indev;

static esp_err_t ft6336_write_reg(uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (FT6336U_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_TP_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t ft6336_read_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (FT6336U_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (FT6336U_ADDR << 1) | I2C_MASTER_READ, true);
    if(len > 1)
    {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_TP_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

void ft6336_init(void)
{
    // TP_RST
    gpio_config_t rst_conf = {0};
    rst_conf.pin_bit_mask = 1ULL << TP_RST;
    rst_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&rst_conf);
    gpio_set_level(TP_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(TP_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
    // I2C master 400K
    i2c_config_t conf = {0};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = TP_SDA;
    conf.scl_io_num = TP_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    i2c_param_config(I2C_TP_PORT, &conf);
    i2c_driver_install(I2C_TP_PORT, conf.mode, 0, 0, 0);
    // INT 下降沿中断
    gpio_config_t int_conf = {0};
    int_conf.pin_bit_mask = 1ULL << TP_INT;
    int_conf.mode = GPIO_MODE_INPUT;
    int_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    int_conf.intr_type = GPIO_INTR_NEGEDGE;
    gpio_config(&int_conf);
    gpio_install_isr_service(0);
    // gpio_isr_handler_add(TP_INT, tp_isr_handler, NULL);
}
// FT6336U 读取单点坐标，重点！寄存器0x03开始
bool ft6336_get_touch(uint16_t *x, uint16_t *y, uint8_t *point_cnt)
{
    uint8_t buf[5];
    esp_err_t ret = ft6336_read_reg(0x02, buf, 5);
    if(ret != ESP_OK)
    {
        *point_cnt = 0;
        return false;
    }
    *point_cnt = buf[0] & 0x0F;
    if(*point_cnt == 0) return false;
    uint8_t xh = buf[1] & 0x0F;
    uint8_t yh = buf[3] & 0x0F;
    *x = (xh << 8) | buf[2];
    *y = (yh << 8) | buf[4];
    return true;
}

void bl_init(void)
{
    gpio_config_t io_conf = {0};
    io_conf.pin_bit_mask = 1ULL << BL_PIN;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(BL_PIN, 1);
}

static void st7796_write_cmd(uint8_t cmd)
{
    gpio_set_level(ST7796_CS, 0);
    gpio_set_level(ST7796_DC, 0);
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    esp_err_t ret = spi_device_transmit(st7796_spi, &t);
    if (ret != ESP_OK) {
        printf("[LCD] command 0x%02X failed: %s\n", cmd, esp_err_to_name(ret));
    }
    gpio_set_level(ST7796_CS, 1);
}

static void st7796_write_data(uint8_t data)
{
    gpio_set_level(ST7796_CS, 0);
    gpio_set_level(ST7796_DC, 1);
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
    };
    esp_err_t ret = spi_device_transmit(st7796_spi, &t);
    if (ret != ESP_OK) {
        printf("[LCD] data 0x%02X failed: %s\n", data, esp_err_to_name(ret));
    }
    gpio_set_level(ST7796_CS, 1);
}

static void st7796_write_burst_data(const uint8_t *data, int len)
{
    gpio_set_level(ST7796_CS, 0);
    gpio_set_level(ST7796_DC, 1);
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
    };
    esp_err_t ret = spi_device_transmit(st7796_spi, &t);
    if (ret != ESP_OK) {
        printf("[LCD] pixel transfer (%d bytes) failed: %s\n", len, esp_err_to_name(ret));
    }
    gpio_set_level(ST7796_CS, 1);
}

static void st7796_set_orientation(uint8_t madctl)
{
    st7796_write_cmd(0x36);
    st7796_write_data(madctl);
}

void st7796_set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    st7796_write_cmd(0x2A);
    st7796_write_data(x1 >> 8);
    st7796_write_data(x1 & 0xFF);
    st7796_write_data(x2 >> 8);
    st7796_write_data(x2 & 0xFF);
    st7796_write_cmd(0x2B);
    st7796_write_data(y1 >> 8);
    st7796_write_data(y1 & 0xFF);
    st7796_write_data(y2 >> 8);
    st7796_write_data(y2 & 0xFF);
    st7796_write_cmd(0x2C);
}
void st7796_fill_color(uint16_t color)
{
    st7796_set_window(0, 0, ST7796_WIDTH - 1, ST7796_HEIGHT - 1);
    uint8_t buf[256];
    uint8_t c_h = color >> 8;
    uint8_t c_l = color & 0xFF;
    for(int i = 0; i < sizeof(buf); i += 2)
    {
        buf[i] = c_h;
        buf[i+1] = c_l;
    }
    int total_pixel = ST7796_HEIGHT * ST7796_WIDTH;
    int burst = sizeof(buf)/2;
    while(total_pixel > 0)
    {
        int w = (total_pixel > burst) ? burst : total_pixel;
        st7796_write_burst_data(buf, w*2);
        total_pixel -= w;
    }
}
void st7796_init(void)
{
    gpio_set_level(ST7796_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(ST7796_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(150));
    st7796_write_cmd(0x11);
    vTaskDelay(pdMS_TO_TICKS(120));
    st7796_write_cmd(0xF0);
    st7796_write_data(0xD3);
    st7796_write_cmd(0xF0);
    st7796_write_data(0x08);
    st7796_set_orientation(LCD_MADCTL_LANDSCAPE);
    st7796_write_cmd(0x3A);
    st7796_write_data(0x55);
    st7796_write_cmd(0xB4);
    st7796_write_data(0x00);
    st7796_write_cmd(0xB6);
    st7796_write_data(0x80);
    st7796_write_data(0x02);
    st7796_write_data(0x3B);
    st7796_write_cmd(0xC5);
    st7796_write_data(0x28);
    st7796_write_cmd(0x29);
    vTaskDelay(pdMS_TO_TICKS(50));
}
void spi_init(void)
{
    gpio_set_direction(ST7796_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(ST7796_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(ST7796_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(ST7796_CS, 1);
    spi_bus_config_t buscfg = {
        .mosi_io_num = ST7796_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = ST7796_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LVGL_SPI_MAX_CHUNK_BYTES,
    };
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        printf("[LCD] SPI bus init failed: %s\n", esp_err_to_name(ret));
        return;
    }
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = ST7796_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 7,
    };
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &st7796_spi);
    if (ret != ESP_OK) {
        printf("[LCD] SPI device init failed: %s\n", esp_err_to_name(ret));
        return;
    }
    printf("[LCD] SPI ready: %lu Hz, max transfer %d bytes\n",
           (unsigned long)ST7796_SPI_CLOCK_HZ, LVGL_SPI_MAX_CHUNK_BYTES);
}

void lv_flush_cb(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color_p)
{
    // 1. 计算本次刷新的矩形区域大小
    uint32_t width = area->x2 - area->x1 + 1;
    uint32_t height = area->y2 - area->y1 + 1;
    uint32_t total_pixels = width * height;

    static bool first_flush = true;
    if (first_flush) {
        printf("[LVGL] first flush: area=(%d,%d)-(%d,%d), pixels=%lu\n",
               area->x1, area->y1, area->x2, area->y2,
               (unsigned long)total_pixels);
        first_flush = false;
    }

    // 2. 设置 ST7796 的刷新窗口
    st7796_set_window(area->x1, area->y1,
                      area->x2, area->y2);

    // 3. 将 LVGL 像素数据分块发送。
    // LV_COLOR_16_SWAP 已在 sdkconfig 中开启，LVGL 缓冲区可直接按字节发送。
    const uint8_t *src = (const uint8_t *)color_p;

    uint32_t offset = 0;
    const uint32_t pixels_per_chunk =
        LVGL_SPI_MAX_CHUNK_BYTES / 2;

    while (offset < total_pixels) {
        uint32_t pixels = total_pixels - offset;

        if (pixels > pixels_per_chunk) {
            pixels = pixels_per_chunk;
        }

        // 4. 直接发送 LVGL 缓冲区，避免每个分块再 memcpy 一次。
        st7796_write_burst_data(src + offset * 2, pixels * 2);

        offset += pixels;
    }

    lv_disp_flush_ready(disp);
}

// LVGL 触摸输入回调（FT6336U）
static void lv_touch_read_cb(lv_indev_drv_t *drv,
                             lv_indev_data_t *data)
{
    static uint16_t last_tx = 0, last_ty = 0;
    uint16_t tx = 0, ty = 0;
    uint8_t point_cnt = 0;

    bool touched = ft6336_get_touch(&tx, &ty, &point_cnt) && point_cnt > 0;

    if (touched) {
        last_tx = tx;
        last_ty = ty;
        printf("[INDEV] REL X:%d Y:%d\n", tx, ty);
        data->point.x = ST7796_WIDTH-ty;
        data->point.y = tx;
        data->state = LV_INDEV_STATE_PR;
    } else {
        // printf("[INDEV] REL X:%d Y:%d\n", last_tx, last_ty);
        data->point.x = ST7796_WIDTH-last_ty;
        data->point.y = last_tx;
        data->state = LV_INDEV_STATE_REL;
    }
}

void lvgl_disp_init(void)
{
    // 使用已有的 480 x 100 缓冲区，减少整屏刷新时的 flush 回调次数。
    lv_disp_draw_buf_init(&draw_buf, disp_buf, NULL, ST7796_WIDTH * 100);
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = ST7796_WIDTH;
    disp_drv.ver_res = ST7796_HEIGHT;
    disp_drv.flush_cb = lv_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);
}

// 按钮颜色切换函数-临时
// 延迟执行背景颜色切换
static void background_change_timer_cb(lv_timer_t *timer)
{
    lv_obj_t *scr = lv_scr_act();

    lv_obj_set_style_bg_color(
        scr,
        lv_color_from_rgb565(color_list[color_idx]),
        LV_PART_MAIN
    );

    // 只执行一次
    lv_timer_del(timer);
}

// 按钮点击回调
static void btn_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }

    // 循环切换颜色索引
    color_idx = (color_idx + 1) % COLOR_CNT;

    // 延迟 150 ms 再修改背景
    lv_timer_create(background_change_timer_cb, 100, NULL);

    printf("Button clicked!\n");
}


void app_main(void)
{
    // 1. 初始化硬件
    bl_init();
    spi_init();
    st7796_init();
    ft6336_init();

    // 2. 初始化 LVGL
    lv_init();

    printf("LVGL version: %d.%d.%d\n",
           LVGL_VERSION_MAJOR,
           LVGL_VERSION_MINOR,
           LVGL_VERSION_PATCH);

    // 3. 注册显示驱动
    lvgl_disp_init();
    // 注册 LVGL 触摸输入设备
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lv_touch_read_cb;

    touch_indev = lv_indev_drv_register(&indev_drv);

    // lv_demo_widgets();

    // 4. 获取当前活动屏幕
    lv_obj_t *scr = lv_scr_act();

    // 5. 设置绿色背景
    lv_obj_set_style_bg_color(
        scr,
        lv_color_from_rgb565(GREEN_COLOR),
        LV_PART_MAIN
    );

    // 创建按钮
    lv_obj_t *btn = lv_btn_create(scr);

    // 设置按钮尺寸
    lv_obj_set_size(btn, 160, 70);

    // 居中显示
    lv_obj_center(btn);
    // lv_obj_set_pos(btn, 300, 230);

    lv_obj_add_event_cb(
        btn,
        btn_event_cb,
        LV_EVENT_CLICKED,
        NULL
    );


    // 设置蓝色背景
    lv_obj_set_style_bg_color(
        btn,
        lv_color_from_rgb565(BLUE_COLOR),
        LV_PART_MAIN
    );

    // 设置按钮不透明
    lv_obj_set_style_bg_opa(
        btn,
        LV_OPA_COVER,
        LV_PART_MAIN
    );

    // 设置圆角
    lv_obj_set_style_radius(
        btn,
        12,
        LV_PART_MAIN
    );

    // 创建按钮文字
    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, "切换");

    // 设置字体
    lv_obj_set_style_text_font(
        label,
        &weiruan,
        LV_PART_MAIN
    );


    // 设置文字颜色为白色
    lv_obj_set_style_text_color(
        label,
        lv_color_from_rgb565(WHITE_COLOR),
        LV_PART_MAIN
    );

    // 文字在按钮中央
    lv_obj_center(label);

    // 确保背景完全不透明
    lv_obj_set_style_bg_opa(
        scr,
        LV_OPA_COVER,
        LV_PART_MAIN
    );

    // 6. 处理 LVGL 的定时器和刷新任务
    while (1) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
        lv_tick_inc(10);
        // lv_area_t btn_area;
        // lv_obj_get_coords(btn, &btn_area);
        // printf("BTN: x1=%d y1=%d x2=%d y2=%d\n", btn_area.x1, btn_area.y1, btn_area.x2, btn_area.y2);
    }
}
