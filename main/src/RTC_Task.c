#include "RTC_Task.h"

TaskHandle_t RTC_Update_Task_Handle = NULL;
#define TAG "RTC_TASK"

void RTC_Update_Task(void *pvParameters)
{
    while(1)
    {
        // esp_task_wdt_reset();
        //阻塞等待BLE发送队列更新时间
        xQueueReceive(rtc_update_queue, &rtc_time, portMAX_DELAY);
        //获得互斥锁
        xSemaphoreTake(rtc_mutex, portMAX_DELAY);
        //更新RTC时间
        rtc_set_time(rtc_time.hours, rtc_time.minutes, rtc_time.seconds, rtc_time.day, rtc_time.month, rtc_time.year);
        //释放互斥锁
        xSemaphoreGive(rtc_mutex);
        //发送更新后的时间到消息队列
        xQueueSend(rtc_queue, &rtc_time, portMAX_DELAY);

        printf("RTC_Update_Task: %d-%d-%d %d:%d:%d\n", rtc_time.year, rtc_time.month, rtc_time.day, rtc_time.hours, rtc_time.minutes, rtc_time.seconds);
        
    }
}

void RTC_Task(void *pvParameters)
{
    esp_err_t ret = ESP_OK;

    if (rtc_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create RTC queue");
        return;
    }
    else
    {
        ESP_LOGE(TAG, "RTC queue created");
    }
    while (1)
    {
        ESP_LOGI(TAG, "RTC Task");
        //获取互斥锁
        xSemaphoreTake(rtc_mutex, portMAX_DELAY);
        ret = rtc_get_time(&rtc_time);
        //释放互斥锁
        xSemaphoreGive(rtc_mutex);
        if (ret == ESP_OK) {
            /* 发送时间到消息队列 */
            xQueueSend(rtc_queue, &rtc_time, portMAX_DELAY);
            ESP_LOGI(TAG, "Time sent to queue");
        }
        if (ret != ESP_OK) 
        {
            ESP_LOGE(TAG, "Failed to get time");
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
