
#include <stdint.h>
#include "esp_err.h"
#include "freertos/idf_additions.h"
#include "../i2c_devices/AHT20.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"

struct AHT20_t start_aht20(i2c_master_dev_handle_t dev_handle){

    uint8_t write_data[3] = {0xe1, 0x08, 0x00};
    struct AHT20_t aht_device;
    aht_device.status= i2c_master_transmit(dev_handle, write_data, sizeof(write_data), I2C_MASTER_TIMEOUT_MS);

    //In the case the calibration of the AHT20 Fails
    if (aht_device.status  != ESP_OK) {
        printf("AHT20 calibration failed: %s\n", esp_err_to_name(aht_device.status ));
    }
    // Small delay after calibration
    vTaskDelay(pdMS_TO_TICKS(10));
    return aht_device;
}


struct AHT20_t sample_aht20_raw(i2c_master_dev_handle_t dev_handle){
    uint8_t write_data[3] = {TRIGGER_MEASUREMENT, DATA0, DATA0};
    uint8_t data[6] = {0};
    struct AHT20_t aht_device;
    // Send measurement trigger command

    aht_device.status= i2c_master_transmit(dev_handle, write_data, sizeof(write_data), I2C_MASTER_TIMEOUT_MS);

    //Check the status of it, if failed return back
    if (aht_device.status != ESP_OK) {
        printf("AHT20 measurement trigger failed: %s\n", esp_err_to_name(aht_device.status));
        aht_device.failure = ESP_FAIL;
        return aht_device;
    }
    // Small delay to allow measurement
    vTaskDelay(pdMS_TO_TICKS(10));
    // Read measurement data

    aht_device.status = i2c_master_receive(dev_handle, data, sizeof(data), I2C_MASTER_TIMEOUT_MS);

    //Check the status of it, if failed return back
    if (aht_device.status != ESP_OK) {
        printf("AHT20 data read failed: %s\n", esp_err_to_name(aht_device.status));
        aht_device.failure = ESP_FAIL;
        return aht_device;
    }
    //Put the Raw data in the struct data called triggered_data
    for(uint8_t i = 0; i < sizeof(data); i++){
        aht_device.triggered_data[i] = data[i];
    }
    uint32_t h = (uint32_t)data[1];
    h <<= 8;
    h |= (uint32_t)data[2];
    h <<= 4;
    h |= (uint32_t)(data[3] >> 4);
    aht_device.humidity = ((float)h / 1048576.0f) * 100.0f;

    uint32_t tdata = (uint32_t)(data[3] & 0x0f);
    tdata <<= 8;
    tdata |= (uint32_t)data[4];
    tdata <<= 8;
    tdata |= (uint32_t)data[5];
    aht_device.temperature = ((float)tdata / 1048576.0f) * 200.0f - 50.0f;

    return aht_device;
}
