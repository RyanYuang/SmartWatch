/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/* 颜色网站 https://rgbcolorpicker.com/565 */

#include "HeadFiles.h"

// ============================================================================
// 全局变量定义
// ============================================================================

static const char *TAG = "SMART_WATCH";
uint16_t POINT_COLOR[RYAN_LCD_H_RES * RYAN_LCD_V_RES];



// ============================================================================
// 颜色处理函数
// ============================================================================

/* 填充颜色缓冲区 */
void fill_red_buffer(uint16_t color) {
    for (int i = 0; i < RYAN_LCD_H_RES * RYAN_LCD_V_RES; i++) {
        POINT_COLOR[i] = color;
    }
}

/* RGB颜色转换 */
lv_color_t Ryan_color(uint8_t R, uint8_t G, uint8_t B) {
    return lv_color_make(R, B, G);
}

// ============================================================================
// LVGL示例函数
// ============================================================================

/* LVGL标签示例 */
void lv_example_label_1(void) {
    lv_obj_t* label1 = lv_label_create(lv_screen_active());
    lv_label_set_long_mode(label1, LV_LABEL_LONG_MODE_WRAP);
    lv_label_set_recolor(label1, true);
    lv_label_set_text(label1, "#0000ff Re-color# #ff00ff words# #ff0000 of a# label, align the lines to the center "
                      "and wrap long text automatically.");
    lv_obj_set_width(label1, 150);
    lv_obj_set_style_text_align(label1, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label1, LV_ALIGN_CENTER, 0, -40);

    lv_obj_t * label2 = lv_label_create(lv_screen_active());
    lv_label_set_long_mode(label2, LV_LABEL_LONG_MODE_SCROLL_CIRCULAR);
    lv_obj_set_width(label2, 150);
    lv_label_set_text(label2, "It is a circularly scrolling text. ");
    lv_obj_align(label2, LV_ALIGN_CENTER, 0, 40);
}

/* 事件处理回调函数 */
static void short_click_event_cb(lv_event_t * e) {
    LV_LOG_USER("Short clicked");

    lv_obj_t * info_label = (lv_obj_t *) lv_event_get_user_data(e);
    lv_indev_t * indev = (lv_indev_t *) lv_event_get_param(e);
    uint8_t cnt = lv_indev_get_short_click_streak(indev);
    lv_label_set_text_fmt(info_label, "Short click streak: %u", cnt);
}

static void streak_event_cb(lv_event_t * e) {
    lv_obj_t * btn = lv_event_get_target_obj(e);
    lv_obj_t * label = lv_obj_get_child(btn, 0);
    const char * text = (const char *) lv_event_get_user_data(e);
    lv_label_set_text(label, text);
}

/* LVGL事件示例 */
void lv_example_event_streak(void) {
    lv_obj_t * info_label = lv_label_create(lv_screen_active());
    lv_label_set_text(info_label, "No events yet");

    lv_obj_t * btn = lv_button_create(lv_screen_active());
    lv_obj_set_size(btn, 100, 50);
    lv_obj_center(btn);
    lv_obj_add_event_cb(btn, short_click_event_cb, LV_EVENT_SHORT_CLICKED, info_label);
    lv_obj_add_event_cb(btn, streak_event_cb, LV_EVENT_SINGLE_CLICKED, (void *) "Single clicked");
    lv_obj_add_event_cb(btn, streak_event_cb, LV_EVENT_DOUBLE_CLICKED, (void *) "Double clicked");
    lv_obj_add_event_cb(btn, streak_event_cb, LV_EVENT_TRIPLE_CLICKED, (void *) "Triple clicked");

    lv_obj_t * label = lv_label_create(btn);
    lv_label_set_text(label, "Click me!");
    lv_obj_center(label);
}

// ============================================================================
// 任务函数
// ============================================================================

/* 调试任务 */
void IdeTask(void *pvParameters) {
    // UBaseType_t uxHighWaterMark;
    // while (1) {
    //     uxHighWaterMark = uxTaskGetStackHighWaterMark(RTC_Task_Handle);
    //     ESP_LOGI(TAG, "IdeTask HighWaterMark: %d", uxHighWaterMark);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
    // vTaskDelete(NULL);
}

/* 主任务 */
// void Main_Task(void *pvParameters) {
//     while (1) {
//         ESP_LOGI(TAG, "Main Task");
//         esp_task_wdt_reset();
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
//     vTaskDelete(NULL);
// }

/* BLE主机任务 */


// ============================================================================
// 主应用程序入口
// ============================================================================

void app_main(void) {
    esp_err_t ret = ESP_OK;
    
    // 初始化任务
    xTaskCreate(Init_Task, "Init_Task", 1024 * 5, NULL, 1, NULL);
    ESP_LOGI(TAG, "app_main initialization completed");
    
}

/* 
 * 颜色不对排查思路：
 * 1. 首先使用本身的LCD驱动的Bitmap来查看本身LCD驱动的颜色是否有反转
 * 2. 测试没问题之后上LVGL的颜色是否有反转
 * 3. 如果有反转，去lvgl的配置文件lv_conf.h中查看配置
 *    - #define LV_COLOR_DEPTH 16
 *    - #define LV_COLOR_16_SWAP 1
 * 如果不起效果要去esp的config中的LVGL configuration中的Check this to not use custom lv_conf.h关闭
 */