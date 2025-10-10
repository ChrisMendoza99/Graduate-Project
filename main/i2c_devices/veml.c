#include <stdint.h>
#include "esp_err.h"
// #include "freertos/idf_additions.h"
#include "../i2c_devices/veml.h"
#include "driver/i2c_master.h"
#include "i2c_devices/AHT20.h"
#include "esp_log.h"

static int16_t from_le_bytes(uint8_t bytes[2]){
    return (int16_t)(bytes[0] | (bytes[1] << 8));
}

uint8_t veml_check(veml_conf_t *veml){
    uint8_t read_buf[2] = {0,0};
    uint8_t write_buf[1] = {VEML_ID};
    if(!veml){
        return ESP_FAIL;
    }
    esp_err_t check = i2c_master_transmit_receive(
                veml->dev_handle,
                write_buf,
                sizeof(write_buf),
                read_buf, sizeof(read_buf), I2C_MASTER_TIMEOUT_MS
    );
    if(check != ESP_OK){
        ESP_LOGE("VEML ID CHECK", "Could not get the VEML ID");
        return ESP_FAIL;
    }
    return read_buf[0];
}

esp_err_t veml_conf(veml_conf_t *sensor){
    uint16_t reg = 0;
    if(!sensor){
        return ESP_FAIL;
    }
    reg |= (sensor->als_init & 0x01) << 0;      // ALS shutdown (0=on)
    reg |= (sensor->als_int_en & 0x01) << 1;    // Interrupt enable
    reg |= (sensor->pers & 0x03) << 4;          // Persistence protect
    reg |= (sensor->it & 0x0F) << 6;            // Integration time
    reg |= (sensor->gain & 0x03) << 11;         // Gain
     ESP_LOGI("INFO","Reg => 0x%04x", reg);
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

esp_err_t veml_power_save_mode(veml_conf_t *sensor, veml_psm_t power){
    uint8_t msg_write[2];
    msg_write[0] = VEML_ALS_POWER_SAVE;
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
    esp_err_t check = i2c_master_transmit(sensor->dev_handle, msg_write, sizeof(msg_write), I2C_MASTER_TIMEOUT_MS);
    if(check != ESP_OK){
        ESP_LOGE("VEML POWER MODE", "Could not write to the Power Mode Register!");
        return ESP_FAIL;
    }
    return ESP_OK;
}



esp_err_t veml_read_asl(veml_conf_t *sensor, float *lux){
    if(!sensor){
        return ESP_FAIL;
    }
    uint8_t cmd = VEML_ALS_DATA;
    uint8_t msg_rcv[2];
    esp_err_t check = i2c_master_transmit_receive(sensor->dev_handle, &cmd, sizeof(cmd), msg_rcv, sizeof(msg_rcv), I2C_MASTER_TIMEOUT_MS);
    if(check != ESP_OK){
        ESP_LOGE("VEML POWER MODE", "Could not write to the Power Mode Register!");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static float get_resolution(uint8_t gain, uint8_t it){
    const float res[6][4] = {
        {0.0042 ,0.0084 ,0.0336 ,0.0672 },
        {0.0084 ,0.0168 ,0.0672 ,0.1344},
        {0.0168 ,0.0336 ,0.1344 ,0.2688 },
        {0.0336 ,0.0672 ,0.2688 ,0.5376 },
        {0.0672 ,0.1344 ,0.5376 ,1.0752},
        {0.1344 ,0.2688 ,1.0752 ,2.1504}
    };
    uint8_t row = 0;
    switch (it) {
        case VEML_IT_800MS:
            row = 0;
            break;
        case VEML_IT_400MS:
            row = 1;
            break;
        case VEML_IT_200MS:
            row = 2;
            break;
        case VEML_IT_100MS:
            row = 3;
            break;
        case VEML_IT_50MS:
            row = 4;
            break;
        case VEML_IT_25MS:
            row = 5;
            break;
    }

    return res[row][gain];
}


esp_err_t veml_read_lux(veml_conf_t *conf, float *lux){
    if(!conf){
        return ESP_FAIL;
    }
    uint8_t cmd = VEML_ALS_DATA;
    uint8_t rcv_data[2];
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
    uint16_t als_raw = (rcv_data[0] | rcv_data[1] << 8);
    float res = get_resolution(conf->gain, conf->it);
    *lux = als_raw * res;
    // ESP_LOGI("DATA", "Raw data => 0x%02x , 0x%02x",rcv_data[0],rcv_data[1]);

    return ESP_OK;
}

esp_err_t veml_read_white(veml_conf_t *conf, uint16_t *white){
    if(!conf){
        return ESP_FAIL;
    }
    uint8_t cmd = VEML_WHITE_DATA;
    uint8_t rcv_data[2];
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
    *white = (rcv_data[0] | rcv_data[1] << 8);

    return ESP_OK;
}
