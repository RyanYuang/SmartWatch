#include "Key.h"
lv_indev_t *key_indev = NULL;
uint8_t key_state = 0;
uint8_t Key_Edge_Detect(void);

void KEY_Init(void)
{
    // GPIO配置
    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_0),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&gpio_conf);

    // //绑定LVGL
    // key_indev = lv_indev_create();
    // lv_indev_set_type(key_indev,LV_INDEV_TYPE_BUTTON);
    // lv_indev_set_read_cb(key_indev,Key_Read_Callback);
    //     static const lv_point_t key_points[2] = {
    //     {0, 0},
    // };
    // lv_indev_set_button_points(key_indev, key_points);
}

void Key_Task(void *pvParameters)
{
    static uint8_t screen_state = 0; // 屏幕状态：0-表盘，1-菜单
    while(1)
    {
        // 检测按键事件
        uint8_t key_event = Key_Edge_Detect();
        
        // 检测到按键按下（下降沿）
        if(key_event == 1)
        {
            // 切换屏幕状态
            screen_state = !screen_state;
            
            // 根据屏幕状态执行相应操作
            if(screen_state == 1)
            {
                // 切换到菜单屏幕
                lv_screen_load_anim(Menu_Screen, LV_SCR_LOAD_ANIM_FADE_IN, 100, 0, NULL);
                printf("切换到菜单屏幕\n");
            }
            else
            {
                // 切换到表盘屏幕
                lv_screen_load_anim(Watch_Screen, LV_SCR_LOAD_ANIM_FADE_IN, 100, 0, NULL);
                printf("切换到表盘屏幕\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));  
    }
}

// 改进的按键上升沿下降沿检测函数
uint8_t Key_Edge_Detect(void)
{
    static uint8_t key_prev_state = 1; // 默认高电平（按键未按下）
    static uint32_t last_press_time = 0;
    uint8_t key_event = 0;
    
    // 读取当前按键电平
    int key_current = gpio_get_level(GPIO_NUM_0);
    
    // 获取当前时间（用于消抖）
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // 检测下降沿（按键按下）
    if((key_prev_state == 1) && (key_current == 0))
    {
        // 消抖处理：确保按键稳定按下至少20ms
        if((current_time - last_press_time) > 20)
        {
            key_event = 1; // 按键按下事件
            last_press_time = current_time;
        }
    }
    // 检测上升沿（按键释放）
    else if((key_prev_state == 0) && (key_current == 1))
    {
        // 消抖处理：确保按键稳定释放至少20ms
        if((current_time - last_press_time) > 20)
        {
            key_event = 2; // 按键释放事件
            last_press_time = current_time;
        }
    }
    
    // 更新前一次状态
    key_prev_state = key_current;
    
    return key_event;
}