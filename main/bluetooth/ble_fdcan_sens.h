

#ifndef H_BLEHR_SENSOR_
#define H_BLEHR_SENSOR_

#include "nimble/ble.h"
#include "modlog/modlog.h"
#include <stdint.h>


#define GATT_CRYPT0S_UUID                           0x4321
#define GATT_CRYPT0S_MEASUREMENT_UUID               0x5678
#define GATT_CRYPT0S_SENSOR_LOC_UUID                0x2A38
#define GATT_DEVICE_INFO_UUID                       0x180A
#define GATT_MANUFACTURER_NAME_UUID                 0x2A29
#define GATT_MODEL_NUMBER_UUID                      0x2A24

extern uint16_t custom_data_handle;

struct ble_hs_cfg;
struct ble_gatt_register_ctxt;

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
int gatt_svr_init(void);

#endif
