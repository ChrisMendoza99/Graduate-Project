#ifndef VEML_BASE_T
#define VEML_BASE_T

#include <stdint.h>
#include "esp_err.h"
#include "stdbool.h"
#include "driver/i2c_types.h"

/* -------------------------------------------------------------------------- */
/*                              I2C Device Address                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief I2C slave address of the VEML sensor (7-bit address)
 */
#define VELM_SLAVE_ADDRESS  0x10


/* -------------------------------------------------------------------------- */
/*                               Register Map                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Command register addresses for VEML sensor
 */
#define VEML_ALS_CONF               0x00    /*!< ALS configuration register */
#define VEML_ALS_THRESHOLD_HIGH     0x01    /*!< ALS interrupt high threshold register */
#define VEML_ALS_THREASHOLD_LOW     0x02    /*!< ALS interrupt low threshold register */
#define VEML_ALS_POWER_SAVE         0x03    /*!< ALS power saving mode register */
#define VEML_ALS_DATA               0x04    /*!< ALS measurement data register */
#define VEML_WHITE_DATA             0x05    /*!< White channel data register */
#define VEML_INTERUPT_STATUS        0x06    /*!< Interrupt status register */
#define VEML_ID                     0x07    /*!< Device ID register */

/**
 * @brief Expected device ID value (used for device check)
 */
#define VEML_ID_VALUE               0x81

/**
 * @brief Timeout for I2C master transactions in milliseconds
 */
#define I2C_MASTER_TIMEOUT_MS       1000    /*!< 1 second timeout */

/* -------------------------------------------------------------------------- */
/*                            Conversion Coefficients                         */
/* -------------------------------------------------------------------------- */

/**
 * @brief Conversion factors between lux and foot-candles.
 */
#define LUX_FC_COEFFICIENT 0.092903     /*!< Lux to foot-candle multiplier */
#define FC_LUX_COEFFICIENT 10.7639      /*!< Foot-candle to lux multiplier */


/* -------------------------------------------------------------------------- */
/*                              Enumerations                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief ALS gain selection
 *
 * Gain affects the sensitivity and maximum measurable lux range.
 */
typedef enum {
    VEML_GAIN1 = 0x00,   /*!< Gain: 1x (default) */
    VEML_GAIN2 = 0x01,   /*!< Gain: 2x */
    VEML_GAIN3 = 0x02,   /*!< Gain: 1/8x */
    VEML_GAIN4 = 0x03,   /*!< Gain: 1/4x */
} veml_gain_t;

/**
 * @brief ALS integration time selection
 *
 * Determines the measurement duration and sensitivity.
 */
typedef enum {
    VEML_IT_25MS  = 0x0C,  /*!< 25 ms integration time */
    VEML_IT_50MS  = 0x08,  /*!< 50 ms integration time */
    VEML_IT_100MS = 0x00,  /*!< 100 ms integration time (default) */
    VEML_IT_200MS = 0x01,  /*!< 200 ms integration time */
    VEML_IT_400MS = 0x02,  /*!< 400 ms integration time */
    VEML_IT_800MS = 0x03,  /*!< 800 ms integration time */
} veml_it_t;

/**
 * @brief ALS persistence protect setting
 *
 * Defines how many consecutive samples must exceed the threshold before triggering an interrupt.
 */
typedef enum {
    VELM_PERS1 = 0x00,  /*!< 1 sample */
    VELM_PERS2 = 0x01,  /*!< 2 consecutive samples */
    VELM_PERS4 = 0x02,  /*!< 4 consecutive samples */
    VELM_PERS8 = 0x03,  /*!< 8 consecutive samples */
} veml_pers_t;

/**
 * @brief Power saving mode settings
 *
 * Used for reducing power consumption at the expense of response time.
 */
typedef enum {
    VEML_PSM_1 = 0x00,  /*!< Mode 1: Fastest wake-up */
    VEML_PSM_2 = 0x01,  /*!< Mode 2 */
    VEML_PSM_3 = 0x02,  /*!< Mode 3 */
    VEML_PSM_4 = 0x03   /*!< Mode 4: Longest sleep duration */
} veml_psm_t;


/* -------------------------------------------------------------------------- */
/*                                Structures                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Configuration structure for VEML sensor
 *
 * Used to hold I2C device handle and user-defined configuration parameters.
 */
typedef struct {
    i2c_master_dev_handle_t dev_handle; /*!< I2C device handle obtained from ESP-IDF driver */
    bool als_init;                      /*!< ALS shutdown control (0=enabled, 1=shutdown) */
    bool als_int_en;                    /*!< ALS interrupt enable flag */
    veml_gain_t gain;                   /*!< Gain configuration */
    veml_it_t it;                       /*!< Integration time */
    veml_pers_t pers;                   /*!< Persistence setting */
} veml_conf_t;

/**
 * @brief Structure to store measured sensor data
 *
 * Used for storing lux and white channel values.
 */
typedef struct {
    float als;      /*!< Ambient light in lux */
    float white;    /*!< White channel intensity */
} veml_data_t;


/* -------------------------------------------------------------------------- */
/*                             Function Prototypes                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Read and verify the VEML device ID.
 *
 * @param veml Pointer to sensor configuration.
 * @return uint8_t Device ID value or ESP_FAIL if read fails.
 */
uint8_t veml_check(veml_conf_t *veml);

/**
 * @brief Write the configuration settings to the sensor.
 *
 * @param sensor Pointer to configuration structure.
 * @return esp_err_t ESP_OK on success, ESP_FAIL otherwise.
 */
esp_err_t veml_conf(veml_conf_t *sensor);

/**
 * @brief Set the sensor power-saving mode.
 *
 * @param sensor Pointer to configuration structure.
 * @param power Power-saving mode to set.
 * @return esp_err_t ESP_OK on success, ESP_FAIL otherwise.
 */
esp_err_t veml_power_save_mode(veml_conf_t *sensor, veml_psm_t power);

/**
 * @brief Read the ambient light level (ALS) in lux.
 *
 * @param conf Pointer to configuration structure.
 * @param lux Pointer to store calculated lux value.
 * @return esp_err_t ESP_OK on success, ESP_FAIL otherwise.
 */
esp_err_t veml_read_lux(veml_conf_t *conf, float *lux);

/**
 * @brief Read the white channel raw data.
 *
 * @param conf Pointer to configuration structure.
 * @param raw_white Pointer to store 16-bit raw white channel value.
 * @return esp_err_t ESP_OK on success, ESP_FAIL otherwise.
 */
esp_err_t veml_read_white(veml_conf_t *conf, uint16_t *raw_white);

#endif /* VEML_BASE_T */
