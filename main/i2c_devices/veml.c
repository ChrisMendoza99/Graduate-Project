#include <stdint.h>
#include "esp_err.h"
// #include "freertos/idf_additions.h"
#include "../i2c_devices/veml.h"
#include "driver/i2c_master.h"
#include "i2c_devices/AHT20.h"
#include "esp_log.h"

/**
 * @brief Check communication with the VEML sensor by reading its device ID.
 *
 * @param veml Pointer to the VEML configuration structure.
 * @return uint8_t Returns the ID value read from the sensor on success,
 *         or ESP_FAIL if communication failed or structure pointer is invalid.
 */
uint8_t veml_check(veml_conf_t *veml){
    uint8_t read_buf[2] = {0,0};
    uint8_t write_buf[1] = {VEML_ID};

    // Null pointer check
    if(!veml){
        return ESP_FAIL;
    }

    // Transmit the ID command and read the response
    esp_err_t check = i2c_master_transmit_receive(
                veml->dev_handle,
                write_buf,
                sizeof(write_buf),
                read_buf, sizeof(read_buf), I2C_MASTER_TIMEOUT_MS
    );

    // If transmission failed, log an error
    if(check != ESP_OK){
        ESP_LOGE("VEML ID CHECK", "Could not get the VEML ID");
        return ESP_FAIL;
    }

    // Return lower byte of ID (device ID register is 16-bit)
    return read_buf[0];
}

/**
 * @brief Configure the VEML sensor's operating parameters (gain, integration time, etc.).
 *
 * @param sensor Pointer to configuration structure with desired parameters.
 * @return esp_err_t ESP_OK if successful, ESP_FAIL if an error occurred.
 */
esp_err_t veml_conf(veml_conf_t *sensor){
    uint16_t reg = 0;
    if(!sensor){
        return ESP_FAIL;
    }
    // Construct the configuration register from bitfields
    reg |= (sensor->als_init & 0x01) << 0;      // ALS shutdown (0=on)
    reg |= (sensor->als_int_en & 0x01) << 1;    // Interrupt enable
    reg |= (sensor->pers & 0x03) << 4;          // Persistence protect number
    reg |= (sensor->it & 0x0F) << 6;            // Integration time
    reg |= (sensor->gain & 0x03) << 11;         // Gain
    // Write configuration register (command + 2 data bytes)
    uint8_t msg_write[3] = {VEML_ALS_CONF,(reg & 0xff), (reg >> 8)};
    esp_err_t check = i2c_master_transmit(
                sensor->dev_handle,
                msg_write,
                sizeof(msg_write),
                I2C_MASTER_TIMEOUT_MS
    );
    if(check != ESP_OK){
        ESP_LOGE("VEML ID CHECK", "Could not write to the Config Register!");
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * @brief Set the VEML sensor’s power saving mode.
 *
 * @param sensor Pointer to the sensor configuration structure.
 * @param power Desired power saving mode (VEML_PSM_1 to VEML_PSM_4).
 * @return esp_err_t ESP_OK if successful, ESP_FAIL otherwise.
 */
esp_err_t veml_power_save_mode(veml_conf_t *sensor, veml_psm_t power){
    uint8_t msg_write[2];
    msg_write[0] = VEML_ALS_POWER_SAVE;

    // Set power save mode according to selected enum
    switch(power){
        case VEML_PSM_1:
            msg_write[1] = (VEML_PSM_1 << 1 | 0x01);
            break;
        case VEML_PSM_2:
            msg_write[1] = (VEML_PSM_2 << 1 | 0x01);
            break;
        case VEML_PSM_3:
            msg_write[1] = (VEML_PSM_3 << 1 | 0x01);
            break;
        case VEML_PSM_4:
            msg_write[1] = (VEML_PSM_4 << 1 | 0x01);
            break;
    }

    // Transmit power-save command
    esp_err_t check = i2c_master_transmit(sensor->dev_handle, msg_write, sizeof(msg_write), I2C_MASTER_TIMEOUT_MS);
    if(check != ESP_OK){
        ESP_LOGE("VEML POWER MODE", "Could not write to the Power Mode Register!");
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * @brief (Placeholder) Read ALS (ambient light sensor) raw data.
 *
 * @param sensor Pointer to the sensor configuration structure.
 * @param lux Pointer to store converted lux value.
 * @return esp_err_t ESP_OK if successful, ESP_FAIL otherwise.
 *
 * @note Function is incomplete – currently reads ALS register but does not process lux value.
 */
esp_err_t veml_read_asl(veml_conf_t *sensor, float *lux){
    if(!sensor){
        return ESP_FAIL;
    }
    uint8_t cmd = VEML_ALS_DATA;
    uint8_t msg_rcv[2];

    // Read ALS data register (2 bytes)
    esp_err_t check = i2c_master_transmit_receive(sensor->dev_handle, &cmd, sizeof(cmd), msg_rcv, sizeof(msg_rcv), I2C_MASTER_TIMEOUT_MS);
    if(check != ESP_OK){
        ESP_LOGE("VEML POWER MODE", "Could not write to the Power Mode Register!");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief Get the measurement resolution (Lux per count) based on gain and integration time.
 *
 * @param gain Selected gain setting.
 * @param it Selected integration time.
 * @return float Resolution multiplier in lux/count.
 */
static float get_resolution(uint8_t gain, uint8_t it){
    // Resolution lookup table: [integration time][gain]
    const float res[6][4] = {
        {0.0042 ,0.0084 ,0.0336 ,0.0672 },
        {0.0084 ,0.0168 ,0.0672 ,0.1344},
        {0.0168 ,0.0336 ,0.1344 ,0.2688 },
        {0.0336 ,0.0672 ,0.2688 ,0.5376 },
        {0.0672 ,0.1344 ,0.5376 ,1.0752},
        {0.1344 ,0.2688 ,1.0752 ,2.1504}
    };
    uint8_t row = 0;

    // Map integration time enum to lookup table row
    switch (it) {
        case VEML_IT_800MS: row = 0; break;
        case VEML_IT_400MS: row = 1; break;
        case VEML_IT_200MS: row = 2; break;
        case VEML_IT_100MS: row = 3; break;
        case VEML_IT_50MS:  row = 4; break;
        case VEML_IT_25MS:  row = 5; break;
    }
    return res[row][gain];
}

/**
 * @brief Read the ambient light (ALS) value in lux.
 *
 * @param conf Pointer to sensor configuration.
 * @param lux Pointer to store calculated lux value.
 * @return esp_err_t ESP_OK if successful, ESP_FAIL otherwise.
 */
esp_err_t veml_read_lux(veml_conf_t *conf, float *lux){
    if(!conf){
        return ESP_FAIL;
    }
    uint8_t cmd = VEML_ALS_DATA;
    uint8_t rcv_data[2];

    // Request ALS data (2 bytes)
    esp_err_t check = i2c_master_transmit_receive(
                conf->dev_handle,
                &cmd,
                sizeof(cmd),
                rcv_data,
                sizeof(rcv_data),
                I2C_MASTER_TIMEOUT_MS
    );
    if(check != ESP_OK){
        return ESP_FAIL;
    }

    // Combine bytes into 16-bit raw ALS value
    uint16_t als_raw = (rcv_data[0] | rcv_data[1] << 8);

    // Compute resolution scaling factor
    float res = get_resolution(conf->gain, conf->it);

    // Convert to lux
    *lux = als_raw * res;

    return ESP_OK;
}

/**
 * @brief Read the "white" channel data from the VEML sensor.
 *
 * @param conf Pointer to configuration structure.
 * @param white Pointer to store 16-bit white channel raw data.
 * @return esp_err_t ESP_OK if successful, ESP_FAIL otherwise.
 */
esp_err_t veml_read_white(veml_conf_t *conf, uint16_t *white){
    if(!conf){
        return ESP_FAIL;
    }
    uint8_t cmd = VEML_WHITE_DATA;
    uint8_t rcv_data[2];

    // Request white channel data
    esp_err_t check = i2c_master_transmit_receive(
                conf->dev_handle,
                &cmd,
                sizeof(cmd),
                rcv_data,
                sizeof(rcv_data),
                I2C_MASTER_TIMEOUT_MS
    );
    if(check != ESP_OK){
        return ESP_FAIL;
    }

    // Combine into 16-bit raw value
    *white = (rcv_data[0] | rcv_data[1] << 8);

    return ESP_OK;
}
