/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

 /* 颜色网站 https://rgbcolorpicker.com/565 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_task_wdt.h"  // 用于 WDT
#include "esp_err.h"
#include "esp_check.h"

/* Driver */
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"


/* LVGL */
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "lv_conf.h"
#include "lv_demos.h"
#include "lv_examples.h"
LV_FONT_DECLARE(Apple_Font)



/* LCD size */
#define RYAN_LCD_H_RES (240)
#define RYAN_LCD_V_RES (280)

/* LCD settings */
#define RYAN_LCD_SPI_NUM (SPI2_HOST)
#define RYAN_LCD_PIXEL_CLK_HZ (40 * 1000 * 1000)
#define RYAN_LCD_CMD_BITS (8)
#define RYAN_LCD_PARAM_BITS (8)
#define RYAN_LCD_COLOR_SPACE (ESP_LCD_COLOR_SPACE_BGR)
#define RYAN_LCD_BITS_PER_PIXEL (16)
#define RYAN_LCD_DRAW_BUFF_DOUBLE (1)
#define RYAN_LCD_DRAW_BUFF_HEIGHT (50)
#define RYAN_LCD_BL_ON_LEVEL (1)

/* LCD pins */
#define RYAN_LCD_GPIO_SCLK (GPIO_NUM_6)
#define RYAN_LCD_GPIO_MOSI (GPIO_NUM_7)
#define RYAN_LCD_GPIO_RST (GPIO_NUM_8)
#define RYAN_LCD_GPIO_DC (GPIO_NUM_4)
#define RYAN_LCD_GPIO_CS (GPIO_NUM_5)
#define RYAN_LCD_GPIO_BL (GPIO_NUM_15)


#define TOUCH_HOST I2C_NUM_0

#define RYAN_PIN_NUM_TOUCH_SCL (GPIO_NUM_10)
#define RYAN_PIN_NUM_TOUCH_SDA (GPIO_NUM_11)
#define RYAN_PIN_NUM_TOUCH_RST (GPIO_NUM_13)
#define RYAN_PIN_NUM_TOUCH_INT (GPIO_NUM_14)
static const char *TAG = "SMART_WATCH";

/* LCD_IO & LCD_PANEL */
static esp_lcd_panel_handle_t lcd_panel = NULL;
static esp_lcd_panel_io_handle_t lcd_io = NULL;

/* LVGL display and touch */
static lv_display_t *lvgl_disp = NULL;

uint16_t POINT_COLOR[RYAN_LCD_H_RES * RYAN_LCD_V_RES];

void fill_red_buffer(uint16_t color) {
    for (int i = 0; i < RYAN_LCD_H_RES * RYAN_LCD_V_RES; i++) {
        POINT_COLOR[i] = color;
    }
}
static esp_err_t app_lcd_init(void)
{
    esp_err_t ret = ESP_OK;

    /* LCD BackLight */
    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << RYAN_LCD_GPIO_BL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));
    gpio_set_level(RYAN_LCD_GPIO_BL, RYAN_LCD_BL_ON_LEVEL);

    /* Initialize SPI BUS */
    ESP_LOGI(TAG, "Initialize SPI BUS");
    const spi_bus_config_t spi_bus_config = {
        .mosi_io_num = RYAN_LCD_GPIO_MOSI,
        .sclk_io_num = RYAN_LCD_GPIO_SCLK,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = RYAN_LCD_H_RES * RYAN_LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(RYAN_LCD_SPI_NUM, &spi_bus_config, SPI_DMA_CH_AUTO), TAG, "SPI bus initialize failed");

    /* Initialize LCD IO */
    ESP_LOGI(TAG, "Initialize LCD IO");
    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = RYAN_LCD_GPIO_CS,
        .dc_gpio_num = RYAN_LCD_GPIO_DC,
        .lcd_cmd_bits = RYAN_LCD_CMD_BITS,
        .lcd_param_bits = RYAN_LCD_PARAM_BITS,
        .spi_mode = 0,
        .pclk_hz = RYAN_LCD_PIXEL_CLK_HZ,
        .trans_queue_depth = 10,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)RYAN_LCD_SPI_NUM, &io_config, &lcd_io), err, TAG, "New panel IO failed");

    /* Install LCD driver */
    ESP_LOGD(TAG, "Install LCD driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = RYAN_LCD_GPIO_RST,
        .color_space = RYAN_LCD_COLOR_SPACE,  // BGR
        .bits_per_pixel = RYAN_LCD_BITS_PER_PIXEL,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7789(lcd_io, &panel_config, &lcd_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(lcd_panel);
    esp_lcd_panel_init(lcd_panel);
    esp_lcd_panel_mirror(lcd_panel, true, true);
    esp_lcd_panel_disp_on_off(lcd_panel, true);

    /* LCD BackLight ON */
    ESP_ERROR_CHECK(gpio_set_level(RYAN_LCD_GPIO_BL, RYAN_LCD_BL_ON_LEVEL));

    esp_lcd_panel_set_gap(lcd_panel, 0, 20);
    esp_lcd_panel_invert_color(lcd_panel, true);
    fill_red_buffer(0xf800);
    esp_lcd_panel_draw_bitmap(lcd_panel, 0, 0, RYAN_LCD_H_RES, RYAN_LCD_V_RES, POINT_COLOR);
    // fill_red_buffer(0xfff0);
    // esp_lcd_panel_draw_bitmap(lcd_panel, 100, 100, RYAN_LCD_H_RES, RYAN_LCD_V_RES, POINT_COLOR);
    

err:
    if (ret != ESP_OK) {
        if (lcd_panel) esp_lcd_panel_del(lcd_panel);
        if (lcd_io) esp_lcd_panel_io_del(lcd_io);
        spi_bus_free(RYAN_LCD_SPI_NUM);
    }
    return ret;
}

lv_color_t Ryan_color(uint8_t R, uint8_t G, uint8_t B)
{
    return lv_color_make(R, B, G);
}

void lv_example_label_1(void)
{
    lv_obj_t * label1 = lv_label_create(lv_screen_active());
    lv_label_set_long_mode(label1, LV_LABEL_LONG_MODE_WRAP);     /*Break the long lines*/
    lv_label_set_recolor(label1, true);                      /*Enable re-coloring by commands in the text*/
    lv_label_set_text(label1, "#0000ff Re-color# #ff00ff words# #ff0000 of a# label, align the lines to the center "
                      "and wrap long text automatically.");
    lv_obj_set_width(label1, 150);  /*Set smaller width to make the lines wrap*/
    lv_obj_set_style_text_align(label1, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label1, LV_ALIGN_CENTER, 0, -40);

    lv_obj_t * label2 = lv_label_create(lv_screen_active());
    lv_label_set_long_mode(label2, LV_LABEL_LONG_MODE_SCROLL_CIRCULAR);     /*Circular scroll*/
    lv_obj_set_width(label2, 150);
    lv_label_set_text(label2, "It is a circularly scrolling text. ");
    lv_obj_align(label2, LV_ALIGN_CENTER, 0, 40);
}

/* LVGL Init */
static esp_err_t app_lvgl_init(void)
{
    /* Initialize LVGL */
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 4,       /* LVGL task priority */
        .task_stack = 4096,       /* LVGL task stack size */
        .task_affinity = -1,      /* LVGL task pinned to core (-1 is no affinity) */
        .task_max_sleep_ms = 500, /* Maximum sleep in LVGL task */
        .timer_period_ms = 5      /* LVGL timer tick period in ms */
    };
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port initialization failed");

    // /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_io,
        .panel_handle = lcd_panel,
        .buffer_size = RYAN_LCD_H_RES * RYAN_LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t),
        .double_buffer = RYAN_LCD_DRAW_BUFF_DOUBLE,
        .hres = RYAN_LCD_H_RES,
        .vres = RYAN_LCD_V_RES,
        .monochrome = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
        }};
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);
    /* 开启抗锯齿 */
    lv_display_set_antialiasing(lvgl_disp, true);

    if(lv_display_is_double_buffered(lvgl_disp))ESP_LOGI(TAG, "Double buffered");
    else ESP_LOGI(TAG, "Single buffered");

    return ESP_OK;
}

static void app_main_display(void)
{
    lv_obj_t *scr = lv_scr_act();

    /* Task lock */
    lvgl_port_lock(-1);


    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text_static(label, "Widgets demo");
    lv_obj_add_flag(label, LV_OBJ_FLAG_IGNORE_LAYOUT);
    
    lv_obj_t *label1 = lv_label_create(scr);
    lv_label_set_text_static(label1, "Widgets demo");
    lv_obj_add_flag(label1, LV_OBJ_FLAG_IGNORE_LAYOUT);

    lv_obj_t *label2 = lv_label_create(scr);
    lv_label_set_text_static(label2, "Widgets demo");
    lv_obj_add_flag(label2, LV_OBJ_FLAG_IGNORE_LAYOUT);

    lv_obj_t *label3 = lv_label_create(scr);
    lv_label_set_text_static(label3, "Widgets demo");
    lv_obj_add_flag(label3, LV_OBJ_FLAG_IGNORE_LAYOUT);

    lv_obj_t * my_button1 = lv_button_create(lv_screen_active());
    lv_obj_t * my_label1 = lv_label_create(my_button1);
    
    // LV_IMG_DECLARE(img_test3);
    // avatar = lv_img_create(scr);
    // lv_img_set_src(avatar, &img_test3);

    /* Task unlock */
    lvgl_port_unlock();
}

void lv_example_label_5(void)
{
    static lv_anim_t animation_template;
    static lv_style_t label_style;

    lv_anim_init(&animation_template);
    lv_anim_set_delay(&animation_template, 1000);           /*Wait 1 second to start the first scroll*/
    lv_anim_set_repeat_delay(&animation_template,
                             3000);    /*Repeat the scroll 3 seconds after the label scrolls back to the initial position*/

    /*Initialize the label style with the animation template*/
    lv_style_init(&label_style);
    lv_style_set_anim(&label_style, &animation_template);

    lv_obj_t * label1 = lv_label_create(lv_screen_active());
    lv_label_set_long_mode(label1, LV_LABEL_LONG_MODE_SCROLL_CIRCULAR);      /*Circular scroll*/
    lv_obj_set_width(label1, 150);
    lv_label_set_text(label1, "It is a circularly scrolling text. ");
    lv_obj_align(label1, LV_ALIGN_CENTER, 0, 40);
    lv_obj_add_style(label1, &label_style, LV_STATE_DEFAULT);           /*Add the style to the label*/
}

/* 创建任务句柄 */
static TaskHandle_t AppTaskCreate_Handle = NULL;



void IdeTask(void *pvParameters)
{
    while (1)
    {
        ESP_LOGI(TAG, "IDE Task");
        esp_task_wdt_reset();  // 重置 WDT
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // 延时 1s，让出 CPU
    }
}

void Main_Task(void *pvParameters)
{
    while (1)
    {
        ESP_LOGI(TAG, "Main Task");
        esp_task_wdt_reset();  // 重置 WDT
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // 延时 1s，让出 CPU
        
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Hello world!");

    // WDT 配置：超时 10s（比默认 5s 长，调试用）
    // esp_task_wdt_config_t twdt_config = {
    //     .timeout_ms = 10000,
    //     .idle_core_mask = 0,
    //     .trigger_panic = true
    // };
    // esp_task_wdt_init(&twdt_config);

    // 添加 IdeTask 到 WDT（main 不添加，或在循环中重置）
    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_42),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&gpio_conf);
    gpio_set_level(GPIO_NUM_42, 0);

    

    /* LCD HW initialization */
    ESP_ERROR_CHECK(app_lcd_init());



    // /* LVGL Init */
    ESP_ERROR_CHECK(app_lvgl_init());
    
    xTaskCreate(&IdeTask, "vTaskCode", 2048, NULL, 2, &AppTaskCreate_Handle);  // 优先级 2 > main 的 1
    /* 从任务创建成功 */
    if (AppTaskCreate_Handle != NULL) {
        ESP_LOGI(TAG, "Task created successfully");
        /* 只添加 IdeTask 到 WDT */
        esp_task_wdt_add(AppTaskCreate_Handle);  // 只添加 IdeTask
    }

    ESP_LOGI(TAG, "app_main ending...");  // 自然结束，让 IDLE 运行

    

    // // /* LVGL display */
    lv_example_label_5();

}

/* 颜色不对排查思路
1. 首先使用本身的LCD驱动的Bitmao来查看本身LCD驱动的颜色是否有反转
2. 测试没问题之后上LVGL的颜色是否有反转
3. 如果有反转，去lvgl的配置文件lv_conf.h中查看配置
* #define LV_COLOR_DEPTH 16
* #define LV_COLOR_16_SWAP 1
如果不起效果要去esp的config中的LVGL configuration中的Check this ito not use custom lv_conf,h关闭
*/