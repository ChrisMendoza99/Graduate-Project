#include <stdint.h>
#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
// #include "esp_log_level.h"
#include "freertos/idf_additions.h"
#include "../i2c_devices/lsm6d.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
// #include "driver/i2c_types.h"
#include "i2c_devices/AHT20.h"

static int16_t from_le_bytes(uint8_t bytes[2]){
    return (int16_t)(bytes[0] | (bytes[1] << 8));
}

float general_accel_cal(int16_t raw_value, float scale){
    return G_ACCELERATION * (((float)raw_value*scale) / 1000.0);
}

float general_gyro_cal(int16_t raw_value, float dps_scale) {
    return dps_scale * (float)raw_value;
}

void lsm6d_setup(Lsm6ds032Device_t *device){
    uint8_t tx_msg[2] = {CTRL3_C, (BOOT | SW_RESET | IF_INC)};

    esp_err_t check = i2c_master_transmit(device->dev_handle, tx_msg, sizeof(tx_msg), I2C_MASTER_TIMEOUT_MS);
    if(check != ESP_OK){
        ESP_LOGE(I2C_LSM6D, "Could not startup the LSM6D IMU");
        return;
    }else{
        ESP_LOGI(I2C_LSM6D,"Device started!!");
    }
}

bool lsm6d_check(Lsm6ds032Device_t *device){
    uint8_t check_msg[1] = {0};
    uint8_t sml_msg[1] = {WHOAMI};
    i2c_master_transmit_receive(
        device->dev_handle,
        sml_msg, sizeof(sml_msg),
        check_msg, sizeof(check_msg),
        I2C_MASTER_TIMEOUT_MS
    );
    if(check_msg[0] != DEVICE_ID){
        ESP_LOGI(I2C_LSM6D,"Device not detected!!");
        return false;
    }else{
        ESP_LOGE(I2C_LSM6D, "LSM6DS032 Device detected!!");
    }
    return true;
}

void gyro_startup(Lsm6ds032Device_t *device){
    uint8_t check_msg[2] = {CTRL2_G, (device->odr_gyro | device->dps)};
    esp_err_t check = i2c_master_transmit(device->dev_handle, check_msg, sizeof(check_msg), I2C_MASTER_TIMEOUT_MS);
    if(check != ESP_OK){
        ESP_LOGE(I2C_LSM6D, "Could not startup the Gyro");
        return;
    }else{
        ESP_LOGI(I2C_LSM6D, "Gyro Started!");
    }
}
void accel_startup(Lsm6ds032Device_t *device){
    uint8_t check_msg[2] = {CTRL1_XL, (device->odr_accel | device->accelerometer)};
    esp_err_t check = i2c_master_transmit(device->dev_handle, check_msg, sizeof(check_msg), I2C_MASTER_TIMEOUT_MS);
    if(check != ESP_OK){
        ESP_LOGE(I2C_LSM6D, "Could not startup the Gyro");
        return;
    }else{
        ESP_LOGI(I2C_LSM6D, "Gyro Started!");
    }
}
void accel_read(Lsm6ds032Device_t *device, lsm6ds_imu_u *imu){
    uint8_t x_la_buffer[2] = {0,0};
    uint8_t y_la_buffer[2] = {0,0};
    uint8_t z_la_buffer[2] = {0,0};

    uint8_t sml_msg_XLA[1] = {OUT_X_LA};
    uint8_t sml_msg_YLA[1] = {OUT_Y_LA};
    uint8_t sml_msg_ZLA[1] = {OUT_Z_LA};

    i2c_master_transmit_receive(device->dev_handle, sml_msg_XLA, sizeof(sml_msg_XLA), x_la_buffer, sizeof(x_la_buffer), I2C_MASTER_TIMEOUT_MS);
    i2c_master_transmit_receive(device->dev_handle, sml_msg_YLA, sizeof(sml_msg_YLA), y_la_buffer, sizeof(y_la_buffer), I2C_MASTER_TIMEOUT_MS);
    i2c_master_transmit_receive(device->dev_handle, sml_msg_ZLA, sizeof(sml_msg_ZLA), z_la_buffer, sizeof(z_la_buffer), I2C_MASTER_TIMEOUT_MS);

    int16_t raw_x_value = from_le_bytes(x_la_buffer);
    int16_t raw_y_value = from_le_bytes(y_la_buffer);
    int16_t raw_z_value = from_le_bytes(z_la_buffer);

    float scale;
    switch (device->accelerometer) {
        case ACCEL_4G:  scale = 0.122; break;
        case ACCEL_8G:  scale = 0.244; break;
        case ACCEL_16G: scale = 0.488; break;
        default:        scale = 0.976; break;
    }

    imu->accel_x = general_accel_cal(raw_x_value, scale);
    imu->accel_y = general_accel_cal(raw_y_value, scale);
    imu->accel_z = general_accel_cal(raw_z_value, scale);
}

void gyro_read(Lsm6ds032Device_t *device, lsm6ds_imu_u *imu){
    uint8_t x_lg_buffer[2] = {0,0};
    uint8_t y_lg_buffer[2] = {0,0};
    uint8_t z_lg_buffer[2] = {0,0};

    uint8_t sml_msg_XLG[1] = {OUT_X_LG};
    uint8_t sml_msg_YLG[1] = {OUT_Y_LG};
    uint8_t sml_msg_ZLG[1] = {OUT_Z_LG};

    i2c_master_transmit_receive(
        device->dev_handle, sml_msg_XLG,
        sizeof(sml_msg_XLG), x_lg_buffer,
        sizeof(x_lg_buffer), I2C_MASTER_TIMEOUT_MS
    );
    i2c_master_transmit_receive(
        device->dev_handle, sml_msg_YLG,
        sizeof(sml_msg_YLG), y_lg_buffer,
        sizeof(y_lg_buffer), I2C_MASTER_TIMEOUT_MS
    );
    i2c_master_transmit_receive(
        device->dev_handle, sml_msg_ZLG,
        sizeof(sml_msg_ZLG), z_lg_buffer,
        sizeof(z_lg_buffer), I2C_MASTER_TIMEOUT_MS
    );

    int16_t raw_x_value = from_le_bytes(x_lg_buffer);
    int16_t raw_y_value = from_le_bytes(y_lg_buffer);
    int16_t raw_z_value = from_le_bytes(z_lg_buffer);

    float dps_scale;
    switch (device->dps) {
        case FS125:     dps_scale = 0.004375; break;
        case FS250:     dps_scale = 0.00875; break;
        case FS500:     dps_scale = 0.0175; break;
        case FS1000:    dps_scale = 0.035; break;
        default:        dps_scale = 0.070; break;
    }

    imu->gyro_x = general_gyro_cal(raw_x_value, dps_scale);
    imu->gyro_y = general_gyro_cal(raw_y_value, dps_scale);
    imu->gyro_z = general_gyro_cal(raw_z_value, dps_scale);

}
