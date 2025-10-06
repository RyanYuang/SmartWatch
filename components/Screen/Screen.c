#include <stdio.h>
#include "Screen.h"


/* ================================================== */
/* 全局变量声明                                       */
/* ================================================== */

LV_FONT_DECLARE(APPLE_SF_PRO_150)

/* LCD 相关句柄 */
static esp_lcd_panel_handle_t lcd_panel = NULL;
static esp_lcd_panel_io_handle_t lcd_io = NULL;
static const char *TAG = "SMART_WATCH";

/* LVGL 显示和触摸相关 */
lv_display_t *lvgl_disp = NULL;  // 全局显示设备
esp_lcd_touch_handle_t tp = NULL; // 触摸设备句柄
lv_indev_t *indev = NULL;        // 输入设备

/* 互斥锁 */
static SemaphoreHandle_t tp_mutex = NULL; // 触摸互斥锁

/* ================================================== */
/* LVGL全局变量初始化                                   */
/* ================================================== */
lv_obj_t* Watch_Screen = NULL;
lv_obj_t* Quick_Setting_Screen = NULL;
lv_obj_t* Menu_Screen = NULL;
lv_color_t* Last_Screeen_Color = NULL;
lv_obj_t* Light_slider = NULL;


/* ================================================== */
/* 时间显示相关                                        */
/* ================================================== */
lv_obj_t* Hour_Number = NULL;
lv_obj_t* Min_Number = NULL;
bool time_update = false;

/* ================================================== */
/* LCD 硬件初始化                                     */
/* ================================================== */

/* ================================================== */
/* LVGL回调函数                                        */
/* ================================================== */
void Light_Slider_Event_Handler(lv_event_t* event);
void Quick_Setting_Screen_Handler(lv_event_t* event);
static void Screen_Timer_Update(lv_timer_t* timer);
void Menu_Screen_Init(void);
esp_err_t LCD_HW_Init(void)
{
    esp_err_t ret = ESP_OK;

    /* 配置LCD背光引脚 */
    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << RYAN_LCD_GPIO_BL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));
    gpio_set_level(RYAN_LCD_GPIO_BL, RYAN_LCD_BL_ON_LEVEL);

    /* 初始化SPI总线 */
    ESP_LOGI(TAG, "Initialize SPI BUS");
    const spi_bus_config_t spi_bus_config = {
        .mosi_io_num = RYAN_LCD_GPIO_MOSI,
        .sclk_io_num = RYAN_LCD_GPIO_SCLK,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = RYAN_LCD_H_RES * RYAN_LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(RYAN_LCD_SPI_NUM, &spi_bus_config, SPI_DMA_CH_AUTO), 
                        TAG, "SPI bus initialize failed");

    /* 初始化LCD IO */
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
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)RYAN_LCD_SPI_NUM, 
                                               &io_config, &lcd_io), 
                     err, TAG, "New panel IO failed");

    /* 安装LCD驱动 */
    ESP_LOGD(TAG, "Install LCD driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = RYAN_LCD_GPIO_RST,
        .color_space = RYAN_LCD_COLOR_SPACE,  // BGR
        .bits_per_pixel = RYAN_LCD_BITS_PER_PIXEL,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7789(lcd_io, &panel_config, &lcd_panel), 
                     err, TAG, "New panel failed");

    /* 配置LCD面板 */
    esp_lcd_panel_reset(lcd_panel);
    esp_lcd_panel_init(lcd_panel);
    esp_lcd_panel_mirror(lcd_panel, true, true);
    esp_lcd_panel_disp_on_off(lcd_panel, true);

    /* 开启LCD背光 */
    ESP_ERROR_CHECK(gpio_set_level(RYAN_LCD_GPIO_BL, RYAN_LCD_BL_ON_LEVEL));

    /* 设置显示偏移和颜色反转 */
    esp_lcd_panel_set_gap(lcd_panel, 0, 20);
    esp_lcd_panel_invert_color(lcd_panel, true);
    
    /* 配置背光PWM */
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = RYAN_LCD_GPIO_BL,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&ledc_channel);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 8000);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

err:
    if (ret != ESP_OK) {
        if (lcd_panel) esp_lcd_panel_del(lcd_panel);
        if (lcd_io) esp_lcd_panel_io_del(lcd_io);
        spi_bus_free(RYAN_LCD_SPI_NUM);
    }
    return ret;
}

/* ================================================== */
/* LVGL 初始化                                        */
/* ================================================== */

static esp_err_t app_lvgl_init(void)
{
    /* 初始化LVGL */
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 1,       /* LVGL任务优先级 */
        .task_stack = 1024 * 5,       /* LVGL任务栈大小 */
        .task_affinity = 0,      /* LVGL任务核心绑定(-1表示不绑定) */
        .task_max_sleep_ms = 100, /* LVGL任务最大睡眠时间 */
        .timer_period_ms = 10      /* LVGL定时器周期 */
    };
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port initialization failed");
    
    /* 添加LCD屏幕 */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_io,
        .panel_handle = lcd_panel,
        .buffer_size = RYAN_LCD_H_RES * RYAN_LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t),
        .double_buffer = RYAN_LCD_DRAW_BUFF_DOUBLE,
        .hres = RYAN_LCD_H_RES,
        .vres = RYAN_LCD_V_RES,
        .monochrome = false,
        /* 旋转值必须与esp_lcd中屏幕的初始设置相同 */
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
        }
    };
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);
    
    /* 开启抗锯齿 */
    lv_display_set_antialiasing(lvgl_disp, true);

    /* 输出缓冲模式信息 */
    if(lv_display_is_double_buffered(lvgl_disp)) {
        ESP_LOGI(TAG, "Double buffered");
    } else {
        ESP_LOGI(TAG, "Single buffered");
    }

    return ESP_OK;
}

/* ================================================== */
/* 触摸处理相关                                       */
/* ================================================== */

/* 触摸中断回调函数 */
static void touch_cb(esp_lcd_touch_handle_t tp)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    /* 释放触摸互斥锁，通知任务有触摸事件 */
    xSemaphoreGiveFromISR(tp_mutex, &xHigherPriorityTaskWoken);
    
    /* 如果需要，进行任务切换 */
    if(xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

/* 触摸处理函数 */
void Tounch_Handler(lv_indev_t *indev, lv_indev_data_t *data)
{
    if (xSemaphoreTake(tp_mutex, 0) == pdTRUE) {
        /* 仅在ISR触发时读取数据 */
        esp_lcd_touch_read_data(tp);
      //   printf("touch x: %d, y: %d\n", tp->data.coords->x, tp->data.coords->y);
        
        /* 更新LVGL输入设备数据 */
        data->point.x = tp->data.coords->x;
        data->point.y = tp->data.coords->y;
        data->state = LV_INDEV_STATE_PRESSED;
    }
}

/* ================================================== */
/* 触摸硬件初始化                                     */
/* ================================================== */

esp_err_t Tounch_HW_Init(void)
{
    /* 创建触摸互斥锁 */
    tp_mutex = xSemaphoreCreateBinary();
    
    /* 初始化输入设备 */
    indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    
    /* 初始化触摸设备 */
    ESP_LOGI(TAG, "Initialize touch screen");

    /* 配置触摸I2C */
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = RYAN_PIN_NUM_TOUCH_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = RYAN_PIN_NUM_TOUCH_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100 * 1000,
    };
    
    /* I2C 初始化 */
    ESP_ERROR_CHECK(i2c_param_config(TOUCH_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(TOUCH_HOST, i2c_conf.mode, 0, 0, 0));

    /* 触摸面板 IO 初始化 */
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = {
        .dev_addr = 0x15,  // CST816S 默认地址
        .on_color_trans_done = 0,
        .user_ctx = 0,
        .control_phase_bytes = 1,
        .dc_bit_offset = 0,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 0,
        .flags = {
            .dc_low_on_data = 0,
            .disable_control_phase = 1,
        }
    };

    /* 创建触摸面板IO */
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)TOUCH_HOST, 
                                                &tp_io_config, &tp_io_handle), 
                        TAG, "Failed to create touch panel IO");

    /* 触摸控制器初始化 */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = RYAN_LCD_H_RES,
        .y_max = RYAN_LCD_V_RES,
        .rst_gpio_num = RYAN_PIN_NUM_TOUCH_RST,
        .int_gpio_num = RYAN_PIN_NUM_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
        .interrupt_callback = touch_cb,
    };

    ESP_LOGI(TAG, "Initialize touch controller");
    ESP_RETURN_ON_ERROR(esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &tp), 
                        TAG, "Failed to initialize touch controller");
    
    /* 设置触摸读取回调函数 */
    lv_indev_set_read_cb(indev, Tounch_Handler);
    for(int i = 0; i < 10; i++)
    {
        ESP_LOGI(TAG, "Touch OK!");
    }
    



    return ESP_OK;
}

/* ================================================== */
/* 屏幕总初始化                                       */
/* ================================================== */

esp_err_t Screen_Init(void)
{
    esp_err_t ret = ESP_OK;
    
    /* 初始化LCD硬件 */
    ret = LCD_HW_Init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD hardware initialization failed");
        return ret;
    }
    
    /* 初始化LVGL */
    ret = app_lvgl_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LVGL initialization failed");
        return ret;
    }
    
    /* 初始化触摸硬件 */
    ret = Tounch_HW_Init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Touch hardware initialization failed");
    }
    
    return ret;
}


static void My_Watch_Screen_Event_Handler(lv_event_t* event)
{
   lv_event_code_t code = lv_event_get_code(event);
   if(code == LV_EVENT_PRESSING)
   {
       ESP_LOGI(TAG, "pressing");
   }
   if(code == LV_EVENT_GESTURE)
   {
      lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_active());
      if(dir == LV_DIR_BOTTOM)
      {
          ESP_LOGI(TAG, "up");
          lv_screen_load(Watch_Screen);
          lv_screen_load_anim(Quick_Setting_Screen, LV_SCR_LOAD_ANIM_FADE_IN, 100, 0, NULL);
      }
   }
}
void Quick_Setting(void);
void My_Watch_Screen(void)
{
   /* 屏幕上拉触发 My_Watch_Screen_Event_Handler 事件 */
   Watch_Screen = lv_screen_active();
   lv_obj_t* screen  = Watch_Screen;

   lv_obj_add_event_cb(screen, My_Watch_Screen_Event_Handler, LV_EVENT_ALL,NULL);

    static lv_anim_t animation_template;
    static lv_style_t label_style;
    /* 设置背景颜色 */
    lv_obj_set_style_bg_color(screen, lv_color_hex(0xA4D5FF), LV_PART_MAIN);

    lv_led_set_brightness(screen, 0);

    /* 设置动画 */
    lv_anim_init(&animation_template);
    lv_anim_set_delay(&animation_template, 1000);           /*Wait 1 second to start the first scroll*/
    lv_anim_set_repeat_delay(&animation_template,
                             300);    /*Repeat the scroll 3 seconds after the label scrolls back to the initial position*/

    /*Initialize he label style with the animation template*/
    lv_style_init(&label_style);
    lv_style_set_anim(&label_style, &animation_template);
    lv_obj_set_style_text_color(screen, lv_color_hex(0x289BFF), LV_PART_MAIN);

    /* 添加小时 */
    Hour_Number = lv_label_create(screen);
    lv_obj_set_style_text_font(Hour_Number, &APPLE_SF_PRO_150, 0);
    lv_label_set_text_static(Hour_Number, "12");
    
    lv_obj_align(Hour_Number, LV_ALIGN_CENTER, 30, -60);

    Min_Number = lv_label_create(screen);
    lv_obj_set_style_text_font(Min_Number, &APPLE_SF_PRO_150, 0);
    lv_label_set_text_static(Min_Number, "30");
    lv_obj_align(Min_Number, LV_ALIGN_CENTER, 30, 70);
    
    lv_obj_t* Div_Label = lv_label_create(screen);
    lv_obj_set_style_text_font(Div_Label, &APPLE_SF_PRO_150, 0);
    lv_label_set_text_static(Div_Label, ":");
    lv_obj_align(Div_Label, LV_ALIGN_CENTER, -70, 60);

    lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    Quick_Setting();
    Menu_Screen_Init();
    lv_timer_create(&Screen_Timer_Update, 500, NULL);


}

void Quick_Setting(void)
{
   /* 创建一屏幕 */
   Quick_Setting_Screen = lv_obj_create(NULL);
   /* 设置该屏幕的背景颜色 */
   lv_obj_set_style_bg_color(Quick_Setting_Screen, lv_color_hex(0x000000), LV_PART_MAIN);

   /* 创建一个按钮图标表示上拉返回 */
//    lv_obj_t* Back_Button = lv_btn_create(Quick_Setting_Screen);
//    lv_obj_set_size(Back_Button, 50, 50);
//    lv_obj_align(Back_Button, LV_ALIGN_LEFT_MID, 10, 0);
//    lv_obj_t* Back_Button_Icon = lv_img_create(Back_Button);
//    lv_img_set_src(Back_Button_Icon, &ICON_BACK);
    lv_obj_t *Light_container = lv_obj_create(Quick_Setting_Screen);
    lv_obj_set_size(Light_container, RYAN_LCD_H_RES, 100);
    lv_obj_align(Light_container, LV_ALIGN_TOP_MID, 0, 30);
    lv_obj_set_style_bg_color(Light_container, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_border_color(Light_container, lv_color_hex(0x000000), LV_PART_MAIN);

   /* 创建一个label给滑动条 */
   lv_obj_t* Light_Setting_label = lv_label_create(Light_container);
   lv_obj_align(Light_Setting_label, LV_ALIGN_CENTER, 0, -30);
   lv_label_set_recolor(Light_Setting_label,true);
   lv_label_set_text(Light_Setting_label,"#FFFFFF Light");

   /* 创建一个屏亮滑动条 */
   Light_slider = lv_slider_create(Light_container);
   lv_obj_set_size(Light_slider, 180, 35);
   lv_obj_align(Light_slider, LV_ALIGN_CENTER, 0,0);
   lv_slider_set_range(Light_slider, 500, 8000);
   lv_slider_set_value(Light_slider, 5000, LV_ANIM_ON);
   /* 设置knob样式 */
   static lv_style_t knob_style;
   lv_style_init(&knob_style);
   lv_style_set_bg_opa(&knob_style, LV_OPA_0);
   lv_obj_add_style(Light_slider, &knob_style, LV_PART_KNOB);

   /* 添加回调函数 */
   lv_obj_add_event_cb(Light_slider, Light_Slider_Event_Handler, LV_EVENT_VALUE_CHANGED, NULL);
   /* 屏幕返回回调函数 */
   lv_obj_add_event_cb(Quick_Setting_Screen, Quick_Setting_Screen_Handler, LV_EVENT_ALL, NULL);

   
   
}

void Light_Slider_Event_Handler(lv_event_t * event)
{
    lv_event_code_t code = lv_event_get_code(event);
    if(code == LV_EVENT_VALUE_CHANGED)
    {
         int Light = lv_slider_get_value(Light_slider);
        ESP_LOGI(TAG, "Light Slider Value: %d", Light);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, Light);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }

}

void Quick_Setting_Screen_Handler(lv_event_t* event)
{
   lv_event_code_t code = lv_event_get_code(event);
   if(code == LV_EVENT_GESTURE)
   {
      lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_active());
      if(dir == LV_DIR_TOP)
      {
          ESP_LOGI(TAG, "up");
          lv_screen_load_anim(Watch_Screen, LV_SCR_LOAD_ANIM_FADE_IN, 100, 0, NULL);
      }
   }
}

void Screen_Task(void *pvParameters)
{
    
    while (1)
    {
        //  esp_task_wdt_reset();
        // printf("Screen Task\n");
        /* 从消息队列接收时间 */
        if (xQueueReceive(rtc_queue, &rtc_time, pdMS_TO_TICKS(100)) == pdTRUE) {
             time_update = true;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
        /* 重置看门狗定时器 */
        // esp_task_wdt_reset();
    }
}

static void Screen_Timer_Update(lv_timer_t* timer)
{
    if(time_update)
    {
           /* 更新时间显示 */
            printf("Time received: %02d:%02d:%02d\n", rtc_time.hours, rtc_time.minutes, rtc_time.seconds);
            lv_label_set_text_fmt(Hour_Number, "%02d", rtc_time.hours);
            lv_label_set_text_fmt(Min_Number, "%02d", rtc_time.minutes);
            time_update = false;
    }
    esp_task_wdt_reset();
}

void Menu_Screen_Init(void)
{
    /* 创建一屏幕 */
    Menu_Screen = lv_obj_create(NULL);
    /* 设置该屏幕的背景颜色 */
    lv_obj_set_style_bg_color(Menu_Screen, lv_color_hex(0x1E1E1E), LV_PART_MAIN);
    
    /* 创建菜单标题 */
    lv_obj_t* menu_title = lv_label_create(Menu_Screen);
    lv_label_set_text(menu_title, "Menu");
    lv_obj_set_style_text_color(menu_title, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_text_font(menu_title, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(menu_title, LV_ALIGN_TOP_MID, 0, 20);
    
    /* 创建菜单容器 */
    lv_obj_t* menu_container = lv_obj_create(Menu_Screen);
    lv_obj_set_size(menu_container, RYAN_LCD_H_RES - 40, RYAN_LCD_V_RES - 100);
    lv_obj_align(menu_container, LV_ALIGN_CENTER, 0, 20);
    lv_obj_set_style_bg_color(menu_container, lv_color_hex(0x2D2D2D), LV_PART_MAIN);
    lv_obj_set_style_border_width(menu_container, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(menu_container, 10, LV_PART_MAIN);
    
    /* 创建菜单项列表 */
    static const char* menu_items[] = {
        "Settings",
        "Health",
        "Music", 
        "Weather",
        "Calendar",
        "Calculator",
        "Stopwatch",
        "About"
    };
    
    // static const char* menu_icons[] = {
    //     "⚙️", "❤️", "🎵", "☀️", "📅", "🧮", "⏱️", "ℹ️"
    // };
    
    /* 创建菜单项 */
    for(int i = 0; i < 8; i++) {
        /* 创建菜单项容器 */
        lv_obj_t* menu_item = lv_obj_create(menu_container);
        lv_obj_set_size(menu_item, lv_pct(100), 60);
        lv_obj_align(menu_item, LV_ALIGN_TOP_MID, 0, i * 65);
        lv_obj_set_style_bg_color(menu_item, lv_color_hex(0x3A3A3A), LV_PART_MAIN);
        lv_obj_set_style_bg_color(menu_item, lv_color_hex(0x4A4A4A), LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_border_width(menu_item, 0, LV_PART_MAIN);
        lv_obj_set_style_radius(menu_item, 8, LV_PART_MAIN);
        lv_obj_set_style_pad_all(menu_item, 10, LV_PART_MAIN);
        
        /* 添加图标 */
        // lv_obj_t* icon_label = lv_label_create(menu_item);
        // lv_label_set_text(icon_label, menu_icons[i]);
        // lv_obj_set_style_text_font(icon_label, &lv_font_montserrat_14, LV_PART_MAIN);
        // lv_obj_align(icon_label, LV_ALIGN_LEFT_MID, 15, 0);
        
        /* 添加菜单项文本 */
        lv_obj_t* item_label = lv_label_create(menu_item);
        lv_label_set_text(item_label, menu_items[i]);
        lv_obj_set_style_text_color(item_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        lv_obj_set_style_text_font(item_label, &lv_font_montserrat_14, LV_PART_MAIN);
        lv_obj_align(item_label, LV_ALIGN_LEFT_MID, 60, 0);
        
        // /* 添加右侧箭头 */
        // lv_obj_t* arrow_label = lv_label_create(menu_item);
        // lv_label_set_text(arrow_label, "->");
        // lv_obj_set_style_text_font(arrow_label, &lv_font_montserrat_14, LV_PART_MAIN);
        // lv_obj_align(arrow_label, LV_ALIGN_RIGHT_MID, -15, 0);
        
        /* 添加点击事件 */
        // lv_obj_add_event_cb(menu_item, menu_item_event_handler, LV_EVENT_CLICKED, (void*)(intptr_t)i);
    }
    
    // /* 添加返回按钮 */
    // lv_obj_t* back_btn = lv_btn_create(Menu_Screen);
    // lv_obj_set_size(back_btn, 120, 40);
    // lv_obj_align(back_btn, LV_ALIGN_BOTTOM_MID, 0, -20);
    // lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x007AFF), LV_PART_MAIN);
    // lv_obj_set_style_radius(back_btn, 20, LV_PART_MAIN);
    
    // lv_obj_t* back_label = lv_label_create(back_btn);
    // lv_label_set_text(back_label, "返回");
    // lv_obj_set_style_text_color(back_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    // lv_obj_center(back_label);
    
    // lv_obj_add_event_cb(back_btn, menu_back_event_handler, LV_EVENT_CLICKED, NULL);
    
    // /* 添加屏幕返回事件（手势返回） */
    // lv_obj_add_event_cb(Menu_Screen, menu_screen_gesture_handler, LV_EVENT_GESTURE, NULL);
}

/* 菜单项点击事件处理函数 */
static void menu_item_event_handler(lv_event_t* e)
{
    lv_obj_t* obj = lv_event_get_target(e);
    int menu_index = (int)(intptr_t)lv_event_get_user_data(e);
    
    static const char* menu_items[] = {
        "设置", "健康", "音乐", "天气", "日历", "计算器", "秒表", "关于"
    };
    
    ESP_LOGI(TAG, "菜单项被点击: %s", menu_items[menu_index]);
    
    /* 根据菜单项执行不同操作 */
    switch(menu_index) {
        case 0: // 设置
            // 打开设置界面
            break;
        case 1: // 健康
            // 打开健康界面
            break;
        case 2: // 音乐
            // 打开音乐界面
            break;
        case 3: // 天气
            // 打开天气界面
            break;
        case 4: // 日历
            // 打开日历界面
            break;
        case 5: // 计算器
            // 打开计算器界面
            break;
        case 6: // 秒表
            // 打开秒表界面
            break;
        case 7: // 关于
            // 打开关于界面
            break;
    }
}

/* 返回按钮事件处理函数 */
static void menu_back_event_handler(lv_event_t* e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        ESP_LOGI(TAG, "返回按钮被点击");
        /* 返回到表盘界面 */
        lv_screen_load_anim(Watch_Screen, LV_SCR_LOAD_ANIM_FADE_OUT, 200, 0, NULL);
    }
}

/* 菜单屏幕手势处理函数 */
// static void menu_screen_gesture_handler(lv_event_t* e)
// {
//     lv_event_code_t code = lv_event_get_code(e);
//     if(code == LV_EVENT_GESTURE) {
//         lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_active());
//         if(dir == LV_DIR_LEFT) {
//             ESP_LOGI(TAG, "向左滑动，返回表盘");
//             lv_screen_load_anim(Watch_Screen, LV_SCR_LOAD_ANIM_SLIDE_RIGHT, 200, 0, NULL);
//         }
//     }
// }

