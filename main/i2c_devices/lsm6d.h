
#ifndef LSM6D_BASE_T
#define LSM6D_BASE_T

#include <stdint.h>
#include "esp_err.h"
#include "stdbool.h"
#include "driver/i2c_types.h"
/* LSM Address */
#define LSM_ADDRESS     0x6A
#define WHOAMI          0x0F
#define DEVICE_ID       0x6C
/* Gyroscope X,Y,and Z Registers */
#define OUT_X_LG        0x22
#define OUT_X_HG        0x23
#define OUT_Y_LG        0x24
#define OUT_Y_HG        0x25
#define OUT_Z_LG        0x26
#define OUT_Z_HG        0x27

/* Acceleration X,Y,and Z Registers */
#define OUT_X_LA        0x28
#define OUT_X_HA        0x29
#define OUT_Y_LA        0x2A
#define OUT_Y_HA        0x2B
#define OUT_Z_LA        0x2C
#define OUT_Z_HA        0x2D

/* CTRL Registers */
#define CTRL1_XL        0x10
#define CTRL2_G         0x11
#define CTRL3_C         0x12

/* Output Data Rate */
#define ODR_FREQ0       0x10  // 1.6 Hz
#define ODR_FREQ1       0x20  // 12.5 Hz
#define ODR_FREQ2       0x30  // 26 Hz
#define ODR_FREQ3       0x40  // 52 Hz
#define ODR_FREQ4       0x50  // 104 Hz
#define ODR_FREQ5       0x60  // 208 Hz
#define ODR_FREQ6       0x70  // 417 Hz
#define ODR_FREQ7       0x80  // 833 Hz
#define ODR_FREQ8       0x90  // 1667 Hz
#define ODR_FREQ9       0xA0  // 3333 Hz
#define ODR_FREQ10      0xB0  // 6667 Hz

/* Degrees Per Second */
#define FS125           0x02
#define FS250           0x00
#define FS500           0x04
#define FS1000          0x08
#define FS2000          0x0C

/* Change in Velocity */
#define ACCEL_4G        0x00  // ±4 g
#define ACCEL_32G       0x04  // ±32 g
#define ACCEL_8G        0x08  // ±8 g
#define ACCEL_16G       0x0C  // ±16 g

/* Control Register 3 Constants */
#define SW_RESET        0x01
#define IF_INC          0x04
#define BOOT            0x80

/* Miscellaneous Constants */
#define G_ACCELERATION 9.81f

// I2C Configuration
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SCL_IO           6       // GPIO pin for SCL
#define I2C_MASTER_SDA_IO           7       // GPIO pin for SDA
#define I2C_MASTER_FREQ_HZ          400000  // 400kHz
#define I2C_MASTER_TIMEOUT_MS       1000    // 1 second timeout
#define I2C_LSM6D ("LSM6D")

typedef enum {
    GyroX,
    GyroY,
    GyroZ
} GyroCoordinate;

typedef enum {
    AccelX,
    AccelY,
    AccelZ
} AccelCoordinate;

typedef struct {
    i2c_master_dev_handle_t dev_handle;
    uint8_t address;
    uint8_t odr_accel;
    uint8_t odr_gyro;
    uint8_t dps;
    uint8_t accelerometer;
}Lsm6ds032Device_t;

typedef struct{
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
}lsm6ds_imu_t;

typedef union{
    struct{
        float accel_x, accel_y, accel_z;
        float gyro_x, gyro_y, gyro_z;
    };
    uint8_t lsm6ds_buff[24];
}lsm6ds_imu_u;

float general_accel_cal(int16_t raw_value, float scale);
float general_gyro_cal(int16_t raw_value, float dps_scale);
void accel_startup(Lsm6ds032Device_t *device);
void gyro_startup(Lsm6ds032Device_t *device);
void lsm6d_setup(Lsm6ds032Device_t *device);
void accel_read(Lsm6ds032Device_t *device, lsm6ds_imu_u *imu);
void gyro_read(Lsm6ds032Device_t *device, lsm6ds_imu_u *imu);
bool lsm6d_check(Lsm6ds032Device_t *device);

#endif
