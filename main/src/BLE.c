#include "BLE.h"
#define TAG "BLE"

/************************************************************************************/
/* 蓝牙相关  */
/************************************************************************************/



/* Library function declarations */
void ble_store_config_init(void);
/* Private functions */
/*
 *  Stack event callback functions
 *      - on_stack_reset is called when host resets BLE stack due to errors
 *      - on_stack_sync is called when host has synced with controller
 */
static void on_stack_reset(int reason) {
    /* On reset, print reset reason to console */
    ESP_LOGI(TAG, "nimble stack reset, reset reason: %d", reason);
    
}

static void on_stack_sync(void) {
    /* On sync, print to console */
    ESP_LOGI(TAG, "nimble stack synced");
    adv_init();
}

void nimble_host_task(void *param) 
{
    ESP_LOGI(TAG, "nimble host task has been started!");
    nimble_port_run();
    vTaskDelete(NULL);
}

esp_err_t BLE_Init(void)
{
    esp_err_t ret = nvs_flash_init();
 if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize nvs flash, error code: %d ", ret);
        return ret;
    }

    /* 初始化 NimBLE主机层协议 */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize nimble port, error code: %d ", ret);
        return ret;
    }

    ret = gap_init();
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "failed to initialize gap service, error code: %d ", ret);
        return ret;
    }

    /* GAP service 初始化 */
    ret = gatt_svc_init();
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "failed to initialize gap service, error code: %d ", ret);
    }

    /* 设定NimBLE主机协议栈的配置 */
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_store_config_init();
    return ret;
}

    //     /* 初始化 GAP服务  */
    // ble_svc_gap_init();

    // /* 设置GAP Device name */
    // ret = ble_svc_gap_device_name_set("Ryan Watch");
    // if(ret != ESP_OK) {
    //     ESP_LOGE(TAG, "failed to set device name, error code: %d ", ret);
    //     return ret;
    // }

