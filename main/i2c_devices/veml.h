#ifndef VEML_BASE_T
#define VEML_BASE_T

#include <stdint.h>
#include "esp_err.h"
#include "stdbool.h"
#include "driver/i2c_types.h"
//I2C Slave Address
#define VELM_SLAVE_ADDRESS  0x10
//I2C Commands
#define VEML_ALS_CONF               0x00
#define VEML_ALS_THRESHOLD_HIGH     0x01
#define VEML_ALS_THREASHOLD_LOW     0x02
#define VEML_ALS_POWER_SAVE         0x03
#define VEML_ALS_DATA               0x04
#define VEML_WHITE_DATA             0x05
#define VEML_INTERUPT_STATUS        0x06
#define VEML_ID                     0x07

#define VEML_ID_VALUE               0x81
#define I2C_MASTER_TIMEOUT_MS       1000    // 1 second timeout

#define LUX_FC_COEFFICIENT 0.092903     /*!< Multiplier coefficient for lux-fc conversion */
#define FC_LUX_COEFFICIENT 10.7639      /*!< Multiplier coefficient for fc-lux conversion */

typedef enum{
    VEML_GAIN1 = 0x00,
    VEML_GAIN2 = 0x01,
    VEML_GAIN3 = 0x02,
    VEML_GAIN4 = 0x03,
}veml_gain_t;

typedef enum{
    VEML_IT_25MS = 0x0C,
    VEML_IT_50MS = 0x08,
    VEML_IT_100MS = 0x00,
    VEML_IT_200MS = 0x01,
    VEML_IT_400MS = 0x02,
    VEML_IT_800MS = 0x03,
}veml_it_t;

typedef enum{
    VELM_PERS1 = 0x00,
    VELM_PERS2 = 0x01,
    VELM_PERS4 = 0x02,
    VELM_PERS8 = 0x03,
}veml_pers_t;

typedef enum{
    VEML_PSM_1 = 0x00,
    VEML_PSM_2 = 0x01,
    VEML_PSM_3 = 0x02,
    VEML_PSM_4 = 0x03
}veml_psm_t;

typedef struct{
    i2c_master_dev_handle_t dev_handle;
    bool als_init;
    bool als_int_en;
    veml_gain_t gain;
    veml_it_t it;
    veml_pers_t pers;
}veml_conf_t;

typedef struct{
    float als;
    float white;
}veml_data_t;

uint8_t veml_check(veml_conf_t *veml);
esp_err_t veml_conf(veml_conf_t *sensor);
esp_err_t veml_power_save_mode(veml_conf_t *sensor, veml_psm_t power);
esp_err_t veml_read_lux(veml_conf_t *conf, float *lux);
esp_err_t veml_read_white(veml_conf_t *conf, uint16_t *raw_white);

#endif
