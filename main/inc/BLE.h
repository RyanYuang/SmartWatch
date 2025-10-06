#ifndef BLE_H
#define BLE_H
#include "HeadFiles.h"

esp_err_t BLE_Init(void);
void nimble_host_task(void *param);

#endif // BLE_H