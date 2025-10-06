#ifndef HEAD_FILES_H
#define HEAD_FILES_H

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_task_wdt.h"  // 用于 WDT
#include "esp_err.h"
#include "esp_check.h"
#include "nvs_flash.h"
// #include "gap.h"
#include "esp_nimble_hci.h"
#include "esp_bt.h"
#include "nimble/nimble_port.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"


/* Driver */
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/ledc.h"
#include "esp_lcd_touch_cst816s.h"

/* LVGL */
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "lv_conf.h"
#include "lv_demos.h"
#include "lv_examples.h"
#include "lvgl__lvgl/src/misc/lv_types.h"
#include "lvgl__lvgl/examples/porting/lv_port_indev.h"

/* Code */
#include "Screen.h"
#include "PCF85063.h"
#include "Key.h"

/* BLE */
#include "gatt_svc.h"
#include "gap.h"
#include "BLE.h"

/* Task      */
#include "Init_Task.h"
#include "RTC_Task.h"

#define DEVICE_NAME "ESP32-SmartWatch"
extern lv_obj_t* Menu_Screen;
extern lv_obj_t* Watch_Screen;


#endif // HEAD_FILES_H
