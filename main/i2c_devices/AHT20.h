#ifndef AH20_BASE_T
#define AH20_BASE_T

#include "esp_err.h"
#include <stdint.h>
#include "stdbool.h"
#include "driver/i2c_types.h"

// I2C Configuration
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SCL_IO           6       // GPIO pin for SCL
#define I2C_MASTER_SDA_IO           7       // GPIO pin for SDA
#define I2C_MASTER_FREQ_HZ          400000  // 400kHz
#define I2C_MASTER_TIMEOUT_MS       1000    // 1 second timeout
// External Device Slave Address
#define AHT20_ADDR                  0x38
#define TRIGGER_MEASUREMENT         0xAC
#define DATA0                       0x33
#define DATA1                       0x00

/*
    AHT20 Driver structure
    -> Triggered Data
    -> Union Humidity
    -> Union Temperature
    -> ESP Status
    -> Failure Byte
*/
struct AHT20_t{
    //Whole AHT20 data
    uint8_t triggered_data[6];
    //Humidity Union
    union{
        //Individual Bytes
        uint8_t humidity_u[4];
        //Converted actual measurement
        float humidity;
    };
    //Temperature Union
    union{
        //Individual Bytes
        uint8_t temperature_u[4];
        //Individual Bytes
        float temperature;
    };
    //Error Handling
    esp_err_t status;
    int8_t failure;
};

/*
    This starts up the AHT20 Device, and the pins are predefined for this device.
*/
struct AHT20_t start_aht20(i2c_master_dev_handle_t dev_handle);
/*
    This function doesnt return the specific temperature nor the humidity.
    Rather it returns the whole structure and with the whole 6 bytes, the value from the structure is "triggered_data"
*/
struct AHT20_t sample_aht20_raw(i2c_master_dev_handle_t dev_handle);

#endif
