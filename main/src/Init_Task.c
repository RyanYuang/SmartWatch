#include "Init_Task.h"
#include "esp_lvgl_port.h"
#define TAG "Init_Task"

// 任务句柄
static TaskHandle_t RTC_Task_Handle = NULL;
static TaskHandle_t Screen_Task_Handle = NULL;
static TaskHandle_t BLE_Task_Handle = NULL;
static TaskHandle_t Key_Task_Handle = NULL;

//互斥锁
SemaphoreHandle_t rtc_mutex = NULL;


void Init_Task(void* pvParament)
{
    // 创建RTC更新列表
    rtc_update_queue = xQueueCreate(2,sizeof(struct rtc_time_t));
    if(rtc_update_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create RTC update queue");
        vTaskDelete(NULL);
    }
    // 创建RTC消息队列
    rtc_queue = xQueueCreate(10, sizeof(struct rtc_time_t));
    if (rtc_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create RTC queue");
        vTaskDelete(NULL);
    } 

    //创建互斥锁
    rtc_mutex = xSemaphoreCreateMutex();
    if(rtc_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create RTC mutex");
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "queue created");

    // GPIO配置
    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_42),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&gpio_conf);
    gpio_set_level(GPIO_NUM_42, 0);

    // 硬件初始化
    Screen_Init();
    BLE_Init();
    PCF85063_Init();
    // 按键初始化
    KEY_Init();
        
    // LVGL显示
    My_Watch_Screen();

    //创建RTC时钟更新任务
    xTaskCreate(RTC_Update_Task,"RTC_Update_Task",2048*2,NULL,1,&RTC_Update_Task_Handle);
    // 创建任务
    xTaskCreate(&RTC_Task, "RTC_Task", 2560, NULL, 2, &RTC_Task_Handle);
    // 创建屏幕任务
    xTaskCreate(&Screen_Task, "Screen_Task", 2560, NULL, 3, &Screen_Task_Handle);
    // 创建BLE任务
    xTaskCreate(&nimble_host_task, "BLE_Task", 1024 * 3, NULL, 3, &BLE_Task_Handle);
    // 创建按键任务
    xTaskCreate(&Key_Task, "Key_Task", 2560, NULL, 1, &Key_Task_Handle);
    
    


    //删除当前任务 
    vTaskDelete(NULL);

}