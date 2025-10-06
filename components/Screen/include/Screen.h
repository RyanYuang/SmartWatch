#ifndef SCREEN_H
#define SCREEN_H

#include <stdio.h>
#include <inttypes.h>

/* ESP-IDF Core */
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_check.h"
#include "esp_chip_info.h"
#include "esp_task_wdt.h"  // 用于 WDT

/* ESP-IDF Drivers */
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch_cst816s.h"

/* LVGL */
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "lv_conf.h"
#include "lv_demos.h"
#include "lv_examples.h"
#include "lvgl__lvgl/src/misc/lv_types.h"
#include "lvgl__lvgl/examples/porting/lv_port_indev.h"

/*PCF85063*/
#include "PCF85063.h"

/* 颜色定义 */
#define RYAN_CLOCK_BG_COLOR (lv_color_hex(0x003a57))  // 时钟背景色

/* LCD 尺寸定义 */
#define RYAN_LCD_H_RES (240)  // 水平分辨率
#define RYAN_LCD_V_RES (280)  // 垂直分辨率

/* LCD 配置参数 */
#define RYAN_LCD_SPI_NUM          (SPI2_HOST)           // SPI主机
#define RYAN_LCD_PIXEL_CLK_HZ     (40 * 1000 * 1000)    // 像素时钟频率
#define RYAN_LCD_CMD_BITS         (8)                   // 命令位数
#define RYAN_LCD_PARAM_BITS       (8)                   // 参数位数
#define RYAN_LCD_COLOR_SPACE      (ESP_LCD_COLOR_SPACE_RGB) // 颜色空间
#define RYAN_LCD_BITS_PER_PIXEL   (16)                  // 每像素位数
#define RYAN_LCD_DRAW_BUFF_DOUBLE (1)                   // 是否使用双缓冲
#define RYAN_LCD_DRAW_BUFF_HEIGHT (50)                  // 绘制缓冲区高度
#define RYAN_LCD_BL_ON_LEVEL      (1)                   // 背光开启电平

/* LCD 引脚定义 */
#define RYAN_LCD_GPIO_SCLK (GPIO_NUM_6)  // 时钟引脚
#define RYAN_LCD_GPIO_MOSI (GPIO_NUM_7)  // 数据输出引脚
#define RYAN_LCD_GPIO_RST  (GPIO_NUM_8)  // 复位引脚
#define RYAN_LCD_GPIO_DC   (GPIO_NUM_4)  // 数据/命令引脚
#define RYAN_LCD_GPIO_CS   (GPIO_NUM_5)  // 片选引脚
#define RYAN_LCD_GPIO_BL   (GPIO_NUM_15) // 背光引脚

/* 触摸屏配置 */
#define TOUCH_HOST              (I2C_NUM_0)  // 触摸I2C通道
#define RYAN_PIN_NUM_TOUCH_SCL  (GPIO_NUM_10) // 触摸SCL引脚
#define RYAN_PIN_NUM_TOUCH_SDA  (GPIO_NUM_11) // 触摸SDA引脚
#define RYAN_PIN_NUM_TOUCH_RST  (GPIO_NUM_13) // 触摸复位引脚
#define RYAN_PIN_NUM_TOUCH_INT  (GPIO_NUM_14) // 触摸中断引脚

/**
 * @brief 屏幕初始化函数
 * @return ESP_OK 初始化成功
 * @return 其他错误码 初始化失败
 */
esp_err_t Screen_Init(void);

/**
 * @brief 自定义智能手表屏幕函数
 * @return 无
 */
void My_Watch_Screen(void);

void Screen_Task(void *pvParameters);

/* 全局变量声明 */
extern lv_display_t *lvgl_disp;




#endif /* SCREEN_H */