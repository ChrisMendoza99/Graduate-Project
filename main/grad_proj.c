/*
 * ============================================================================
 * ESP32-C5 Multi-Sensor Hub with CAN (TWAI) Communication
 * ============================================================================
 * Description: Modular sensor hub supporting multiple I2C sensors with
 *              CAN bus communication for distributed sensing applications.
 *              With Ascon-128 encryption capabilities,
 *
 * Supported Sensors:
 *   - AHT20: Temperature & Humidity
 *   - LSM6DSO32: IMU (Accelerometer & Gyroscope)
 *   - VEML7700: Ambient Light Sensor
 *
 * By:
 *  Christopher A. Mendoza
 * ============================================================================
 */

/* ============================================================================
 * INCLUDES
 * ============================================================================ */

/* Standard Libraries */
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <sys/_types.h>
#include <sys/param.h>
#include <sys/types.h>

/* FreeRTOS & ESP-IDF Core */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/event_groups.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs_adv.h"
#include "host/ble_hs_id.h"
#include "host/ble_hs_mbuf.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "os/os_mbuf.h"
#include "portmacro.h"

/* ESP-IDF Drivers */
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"


/* CAN (TWAI) */
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_twai_types.h"
#include "hal/twai_types.h"

/* Bluetooth */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "ble_fdcan_sens.h"

/* LED Strip */
#include "led_strip.h"
#include "led_strip_rmt.h"
#include "led_strip_types.h"

/* SoC Specific */
#include "soc/clk_tree_defs.h"
#include "soc/gpio_num.h"

/* Custom Device Drivers */
#include "i2c_devices/AHT20.h"
#include "i2c_devices/lsm6d.h"
#include "i2c_devices/veml.h"
#include "canascon/canascon.h"
/* ============================================================================
 * CONFIGURATION MACROS
 * ============================================================================ */

/* ---------- Device Mode Selection ---------- */
/*
 * DEVICE_MODE:
 *   0 => Bluetooth Gateway (CAN receiver/aggregator)
 *   1 => Temperature & Humidity Sensor Node
 *   2 => IMU Sensor Node
 *   3 => Light Sensor Node
 */
#define DEVICE_MODE (0)

/*
 * I2C_DEVICE:
 *   0 => AHT20 (Temperature & Humidity)
 *   1 => LSM6DSO32 (IMU)
 *   2 => VEML7700 (Light Sensor)
 */
#define I2C_DEVICE (0)

/* ---------- LED Configuration ---------- */
#define LED_STRIP_RMT_RES_HZ    (10 * 1000 * 1000)  // 10 MHz
#define LED_INDEX               (0)
#define LED_GPIO                (27)
#define FULL_BRIGTNESS          (255)
#define HALF_BRIGTNESS          (127)
#define QUARTER_BRIGHTNESS      (63)
#define NO_BRIGTNESS            (0)

/* ---------- FreeRTOS Configuration ---------- */
#define TASK_SIZE               (1024 * 4)
#define QueueSize               (1)
#define MAX_SEMAPHORE_COUNT     (3)

/* ---------- CAN (TWAI) Configuration ---------- */
#define CAN_RX_PIN              (5)
#define CAN_TX_PIN              (4)
#define CAN_QUEUE_SIZE          (10)
#define TWAI_NOMINAL_BITRATE    (500000)
#define TWAI_TAG "TWAI Device"

/* ---------- I2C Configuration ---------- */
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_GPIO (3)
#define I2C_MASTER_SDA_GPIO (2)


/* ============================================================================
 * TYPE DEFINITIONS
 * ============================================================================ */

/* ---------- CAN Network IDs ---------- */
typedef enum {
    BLUETOOTH_ID = 0x200,
    TEMP_HUM_SENSOR_ID = 0x210,
    IMU_SENSOR_ID = 0x220,
    LIGHT_SENSOR_ID = 0x230
} can_id;

/* ---------- CAN Message Structure ---------- */
typedef struct {
    uint32_t id;
    uint8_t can_msg[64];
    size_t msg_size;
} can_messasge_t;

/* ============================================================================
 * GLOBAL VARIABLES
 * ============================================================================ */

/* ---------- Hardware Handles ---------- */
static led_strip_handle_t led_strip;
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;
twai_node_handle_t node_hdl = NULL;

/* ---------- FreeRTOS Objects ---------- */
TaskHandle_t aht_device = NULL;
static TimerHandle_t fdcan_tx_timer;
QueueHandle_t twai_queue_isr;
QueueHandle_t i2c_queue;

/* ========================================================================================================================================================
 * BLUETOOTH
 * ======================================================================================================================================================== */

static const char *device_name = "CRYPT0S_SENSOR_0.1";
static uint8_t fdcanble_addr_type;
static uint16_t conn_handle;
static void fdcanble_advertise(void);
static bool notify_state;
static uint8_t fd_can_data = 15;

static void tx_data_rate_stop(void){
    xTimerStop(fdcan_tx_timer, 1000/portTICK_PERIOD_MS);
}
//Reset heart rate measurement
static void tx_data_rate_reset(void){
    int rc;
    if(xTimerReset(fdcan_tx_timer, 1000/portTICK_PERIOD_MS) == pdPASS){
        rc = 0;
    }else{
        rc = 1;
    }
    assert(rc == 0);
}
static void fdcanble_tx_data(TimerHandle_t ev){
    static uint8_t fdcanble[2];
    int rc;
    struct os_mbuf *om;
    if(!notify_state){
        tx_data_rate_stop();
        fd_can_data = 15;
        return;
    }
    // fdcanble[0] = 0x05;
    fdcanble[0] = fd_can_data;

    fd_can_data++;
    if(fd_can_data == 160){
        fd_can_data = 15;
    }
    //Creata memory buffer from data
    om = ble_hs_mbuf_from_flat(fdcanble, sizeof(fdcanble));
    //Send notification to client
    rc = ble_gatts_notify_custom(conn_handle, custom_data_handle, om);
    assert(rc == 0);
    tx_data_rate_reset();
}



static int fdcanble_gap_event(struct ble_gap_event *event, void *arg){
    switch(event->type){
        case BLE_GAP_EVENT_CONNECT: { //Connection established, save conn_handle
            MODLOG_DFLT(INFO, "connection %s; status=%d\n",
                        event->connect.status == 0 ? "established" : "failed",
                        event->connect.status);
            if (event->connect.status != 0) {
                /* Connection failed; resume advertising */
                fdcanble_advertise();
            }
            conn_handle = event->connect.conn_handle;
            break;
        }
        case BLE_GAP_EVENT_DISCONNECT:{ //Connection lost, restart adverstising
            MODLOG_DFLT(INFO, "disconnect; reason=%d\n", event->disconnect.reason);
            //Connection terminated, resuming advertising
            fdcanble_advertise();
            break;
        }
        case BLE_GAP_EVENT_ADV_COMPLETE:{ //Advertising ended, restart adverstising
            MODLOG_DFLT(INFO, "adv complete\n");
            fdcanble_advertise();
            break;
        }
        case BLE_GAP_EVENT_SUBSCRIBE:{ //Client enabled notifications, start sending data
            MODLOG_DFLT(INFO, "subscribe event; cur_notify=%d\n value handle; "
                        "val_handle=%d\n",
                        event->subscribe.cur_notify, custom_data_handle);
            if(event->subscribe.attr_handle == custom_data_handle){
                notify_state = event->subscribe.cur_notify;
                tx_data_rate_reset();
            }else if(event->subscribe.attr_handle != custom_data_handle){
                notify_state = event->subscribe.cur_notify;
                tx_data_rate_stop();
            }
            break;
        }
        case BLE_GAP_EVENT_MTU:{ //MTU negotiated, Note max packet size
            MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d mtu=%d\n",
                        event->mtu.conn_handle,
                        event->mtu.value);
            break;
        }
    }
    return 0;
}


static void fdcanble_advertise(void){
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    /*
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info)
     *     o Advertising tx power
     *     o Device name
     */
    memset(&fields, 0, sizeof(fields));
     /*
      * Advertise two flags:
      *      o Discoverability in forthcoming advertisement (general)
      *      o BLE-only (BR/EDR unsupported)
      */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
      /*
       * Indicate that the TX power level field should be included; have the
       * stack fill this value automatically.  This is done by assigning the
       * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
       */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

       /* Begin advertising */
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(
        fdcanble_addr_type,
        NULL,
        BLE_HS_FOREVER,
        &adv_params,
        fdcanble_gap_event,
        NULL
    );
    if(rc != 0){
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}


static void fdcan_ble_on_sync(void){
    int rc;
    rc = ble_hs_id_infer_auto(0, &fdcanble_addr_type);
    assert(rc == 0);

    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(fdcanble_addr_type, addr_val, NULL);

    //Begin adverstising
    fdcanble_advertise();
}

static void fdcan_ble_on_reset(int reason){
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

static void fdcanble_host_task(void *param){
    ESP_LOGI("NimBLE_FDCAN_BLE_DATA", "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

 static void ble_init(){
    int rc;
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ret = nimble_port_init();


    /* Initialize the NimBLE host configuration */
    ble_hs_cfg.sync_cb = fdcan_ble_on_sync;
    ble_hs_cfg.reset_cb = fdcan_ble_on_reset;

    //Create a timer
    fdcan_tx_timer = xTimerCreate("fdcanble_tx_timer", pdMS_TO_TICKS(1000), pdTRUE, NULL, fdcanble_tx_data);

    rc = gatt_svr_init();
    assert(rc == 0);

    //Set the default device name
    rc = ble_svc_gap_device_name_set(device_name);
    assert(rc == 0);

    //Start the task
    nimble_port_freertos_init(fdcanble_host_task);
}


/* ============================================================================
 * TWAI (CAN) CALLBACK FUNCTIONS
 * ============================================================================ */

/* ---------- Transmission Completion Callback ---------- */
static IRAM_ATTR bool
twai_tx_done_callback(twai_node_handle_t handle,
                      const twai_tx_done_event_data_t *edata,
                      void *user_ctx) {
    if (!edata->is_tx_success) {
        ESP_EARLY_LOGW(TWAI_TAG, "Failed to transmit message, ID: 0x%X",
                       edata->done_tx_frame->header.id);
    }
    return false;
}

/* ---------- Bus Error Callback ---------- */
static IRAM_ATTR bool
twai_on_error_callback(twai_node_handle_t handle,
                       const twai_error_event_data_t *edata,
                       void *user_ctx) {
    ESP_EARLY_LOGW(TWAI_TAG, "TWAI node error: 0x%x", edata->err_flags.val);
    return false;
}

/* ---------- Receive Completion Callback ---------- */
static IRAM_ATTR bool
twai_rx_done_callback(twai_node_handle_t handle,
                      const twai_rx_done_event_data_t *edata,
                      void *user_ctx) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t rx_buffer[64];
    twai_frame_t frame = {
        .buffer = rx_buffer,
        .buffer_len = sizeof(rx_buffer),
    };

    can_messasge_t enqueue_can;

    if (twai_node_receive_from_isr(handle, &frame) != ESP_OK) {
        ESP_EARLY_LOGW(TWAI_TAG, "Failed to get message!");
    } else {
        ESP_EARLY_LOGI(TWAI_TAG, "ID=0x%03X, Len(DLC)=%d",
                       frame.header.id, frame.header.dlc);

        enqueue_can.id = frame.header.id;
        enqueue_can.msg_size = twaifd_dlc2len(frame.header.dlc);
        ESP_EARLY_LOGD("debug", "Actually size => %d\n", enqueue_can.msg_size);

        memcpy(enqueue_can.can_msg, frame.buffer, enqueue_can.msg_size);

        if (xQueueSendFromISR(twai_queue_isr, &enqueue_can,
                              &xHigherPriorityTaskWoken) != pdPASS) {
            ESP_EARLY_LOGE("Callback Error", "Could not fill buffer");
        }
    }
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }

    return false;
}

/* ---------- TWAI (CAN) Initialization ---------- */
void twai_init() {

    twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .rx = CAN_RX_PIN,
            .tx = CAN_TX_PIN,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC
        },
        .bit_timing = {
            .bitrate = TWAI_NOMINAL_BITRATE,
        },
        .fail_retry_cnt = 3,
        .tx_queue_depth = 5,
    };

    /* Register event callbacks */
    twai_event_callbacks_t callbacks = {
        .on_tx_done = twai_tx_done_callback,
        .on_rx_done = twai_rx_done_callback,
        .on_error = twai_on_error_callback,
    };

    /* Configure CAN ID filter based on device mode */
    twai_mask_filter_config_t general_mask;
#if DEVICE_MODE == 0
    general_mask.id = BLUETOOTH_ID;
    general_mask.mask = 0x7c0;  // 0b111_1100_0000
#elif DEVICE_MODE == 1
    general_mask.id = TEMP_HUM_SENSOR_ID;
    general_mask.mask = 0x7ff;
#elif DEVICE_MODE == 2
    general_mask.id = IMU_SENSOR_ID;
    general_mask.mask = 0x7ff;
    node_config.data_timing.bitrate = (TWAI_NOMINAL_BITRATE * 3);
#elif DEVICE_MODE == 3
    general_mask.id = LIGHT_SENSOR_ID;
    general_mask.mask = 0x7ff;
    node_config.data_timing.bitrate = (TWAI_NOMINAL_BITRATE * 3);
#else
#error "NO DEVICE SELECTED!"
#endif

    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &node_hdl));
    ESP_ERROR_CHECK(
        twai_node_register_event_callbacks(node_hdl, &callbacks, NULL));
    ESP_ERROR_CHECK(twai_node_config_mask_filter(node_hdl, 0, &general_mask));
    ESP_ERROR_CHECK(twai_node_enable(node_hdl));

    ESP_LOGI(TWAI_TAG, "TWAI Sender started successfully");
}
/* ============================================================================
 * SENSOR TASK FUNCTIONS
 * ============================================================================ */

/* ---------- AHT20 Temperature & Humidity Sensor Task ---------- */
static void aht_20_sensor(void *arg) {
    struct AHT20_t aht;
    uint8_t message[8];
    twai_frame_t i2c_msg = {
        .header.fdf = 1,
        .header.dlc = sizeof(message),
        .header.id = TEMP_HUM_SENSOR_ID,
        .buffer = message
    };

    aht = start_aht20(dev_handle);
    if (aht.status != ESP_OK) {
        ESP_LOGE("I2C DEVICE", "Could startup the AHT20 Device!");
        vTaskDelay(750 / portTICK_PERIOD_MS);
    }

    while (1) {
        aht = sample_aht20_raw(dev_handle);
        memcpy(&i2c_msg.buffer[0], &aht.humidity_u, sizeof(aht.humidity_u));
        memcpy(&i2c_msg.buffer[4], &aht.temperature_u, sizeof(aht.temperature_u));

        if ((twai_node_transmit(node_hdl, &i2c_msg, pdMS_TO_TICKS(100))) == ESP_OK && aht.status == ESP_OK){
            ESP_LOGI("AHT20 Device", "DATA -> Temperature %f | Humidity %f",
                     aht.temperature, aht.humidity);
            led_strip_set_pixel(led_strip, 0, 0, QUARTER_BRIGHTNESS, 0);
        } else {
            led_strip_set_pixel(led_strip, 0, QUARTER_BRIGHTNESS, 0, 0);
            ESP_LOGE("TWAI TX", "Could send AHT20 data!");
        }
        led_strip_refresh(led_strip);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


/* ---------- LSM6DSO32 IMU Sensor Task ---------- */
static void imu_sensor(void *arg) {
    lsm6ds_imu_u imu_data;
    uint8_t imu_buffer[24];

    Lsm6ds032Device_t lsm_config = {
        .accelerometer = ACCEL_4G,
        .odr_accel = ODR_FREQ5,
        .odr_gyro = ODR_FREQ5,
        .address = LSM_ADDRESS,
        .dps = FS500,
        .dev_handle = dev_handle
    };

    twai_frame_t imu_msg = {
        .header.fdf = 1,
        .header.id = IMU_SENSOR_ID,
        .buffer = imu_buffer,
        .buffer_len = sizeof(imu_buffer),
        .header.dlc = twaifd_len2dlc(sizeof(imu_buffer))
    };

    lsm6d_setup(&lsm_config);
    if (lsm6d_check(&lsm_config) == 1) {
        ESP_LOGI("I2C DEVICE", "Device found!");
    }

    accel_startup(&lsm_config);
    gyro_startup(&lsm_config);

    while (1) {
        /* Sample acceleration and gyroscope */
        accel_read(&lsm_config, &imu_data);
        gyro_read(&lsm_config, &imu_data);
        memcpy(&imu_buffer, imu_data.lsm6ds_buff, sizeof(imu_data.lsm6ds_buff));

        /* Transmit: {AX,AY,AZ,GX,GY,GZ} => 4 bytes × 6 = 24 bytes */
        if ((twai_node_transmit(node_hdl, &imu_msg, pdMS_TO_TICKS(100))) == ESP_OK) {
            ESP_LOGI("ACCEL", "Acceleration => AX: %f | AY: %f | AZ: %f",
                     imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
            ESP_LOGI("GYRO", "Gyroscope => GX: %f | GY: %f | GZ: %f",
                     imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);

            led_strip_set_pixel(led_strip, 0, 0, QUARTER_BRIGHTNESS, 0);
        } else {
            ESP_LOGE("TWAI TX", "Could not send IMU Data!");
            led_strip_set_pixel(led_strip, 0, QUARTER_BRIGHTNESS, 0, 0);
        }
        led_strip_refresh(led_strip);
        vTaskDelay(751 / portTICK_PERIOD_MS);
    }
}
/* ---------- VEML7700 Light Sensor Task ---------- */
static void veml_sensor_task(void *arg){

    float lux = 0.00;
    uint16_t white = 0;
    uint8_t veml_buffer[6];
    /* ---------- Configure VEML7700 Light Sensor ---------- */
    veml_conf_t sensor = {
        .dev_handle = dev_handle,
        .als_init = false,
        .als_int_en = false,
        .gain = VEML_GAIN4,
        .it = VEML_IT_25MS,
        .pers = VELM_PERS2,
    };
    twai_frame_t veml_msg = {
        .header.fdf = 1,
        .header.id = LIGHT_SENSOR_ID,
        .buffer = veml_buffer,
        .buffer_len = sizeof(veml_buffer),
        .header.dlc = twaifd_len2dlc(sizeof(veml_buffer))
    };

    // uint8_t key[16] = {
    //     0xA3, 0x7F, 0x1C, 0xD9, 0x4B, 0x02, 0xE8, 0x55,
    //     0x9A, 0x3D, 0x60, 0xF7, 0x8E, 0x21, 0xB4, 0xC0
    // };

    /* Verify sensor ID */
    uint8_t foo = veml_check(&sensor);
    ESP_LOGI("ID CHECK", "ID => 0x%x", foo);
    /* Apply configuration */
    veml_conf(&sensor);
    veml_power_save_mode(&sensor, VEML_PSM_3);

    while(1){
        veml_read_lux(&sensor, &lux);
        veml_read_white(&sensor, &white);
        memcpy(&veml_buffer[0], &lux, sizeof(lux));
        memcpy(&veml_buffer[4], &white, sizeof(white));
        if((twai_node_transmit(node_hdl, &veml_msg, pdMS_TO_TICKS(100)))== ESP_OK){
            ESP_LOGI("LUX DATA", "Lux => %.2f", lux);
            ESP_LOGI("WHITE DATA", "White = %u", white);
            led_strip_set_pixel(led_strip, 0, 0, QUARTER_BRIGHTNESS, 0);

        }else{
            ESP_LOGW("TRANSMIT ERROR","COULD NOT SEND DATA!!");
            led_strip_set_pixel(led_strip, 0, QUARTER_BRIGHTNESS, 0, 0);
        }
        led_strip_refresh(led_strip);
        // esp_err_t result = encrypt_transmit_msg(LIGHT_SENSOR_ID, node_hdl, veml_buffer, sizeof(veml_buffer), key);
        // if(result == ESP_OK){
        //     //Green
        //     led_strip_set_pixel(led_strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue)
        // }else{
        //     //Red
        // }
        vTaskDelay(502 / portTICK_PERIOD_MS);
    }
}

/* ============================================================================
 * CAN MESSAGE MANAGER
 * ============================================================================ */

 /* ---------- Message Router and Processor ---------- */
void messenger_manager(void *arg) {
    can_messasge_t queue_can;

    lsm6ds_imu_u rx_imu;
    //Constants
    // const size_t aht20_size = 8;
    // const size_t light_size = 8;
    // const size_t imu_size = 24;

    while (1) {
        if (xQueueReceive(twai_queue_isr, &queue_can, pdMS_TO_TICKS(1000)) == pdPASS) {
            switch (queue_can.id) {
                case TEMP_HUM_SENSOR_ID: {
                    float temp, hum;
                    memcpy(&hum, &queue_can.can_msg[0], sizeof(hum));
                    memcpy(&temp, &queue_can.can_msg[4], sizeof(temp));
                    printf("AHT20 Device: DATA -> Temperature %f | Humidity %f", temp, hum);
                    led_strip_set_pixel(led_strip, 0, 0, QUARTER_BRIGHTNESS, 0);
                    led_strip_refresh(led_strip);
                    break;
                }
                case IMU_SENSOR_ID: {
                    memcpy(rx_imu.lsm6ds_buff, &queue_can.can_msg,
                           sizeof(rx_imu.lsm6ds_buff));
                    printf("ACCEL: Acceleration => AX: %f | AY: %f | AZ: %f",
                             rx_imu.accel_x, rx_imu.accel_y, rx_imu.accel_z);
                    printf("GYRO (Receieved)L: Gyroscope => GX: %f | GY: %f | GZ: %f",
                             rx_imu.gyro_x, rx_imu.gyro_y, rx_imu.gyro_z);
                    led_strip_set_pixel(led_strip, 0, 0, QUARTER_BRIGHTNESS, 0);
                    led_strip_refresh(led_strip);
                    break;
                }
                case LIGHT_SENSOR_ID: {
                    float lux;
                    uint16_t white;
                    memcpy(&lux, &queue_can.can_msg[0], sizeof(lux));
                    memcpy(&white, &queue_can.can_msg[4], sizeof(white));
                    printf("LUX DATA: Lux => %.2f", lux);
                    printf("WHITE DATA: White = %u", white);
                    led_strip_set_pixel(led_strip, 0, 0, QUARTER_BRIGHTNESS, 0);
                    led_strip_refresh(led_strip);
                    break;
                }
                case 0x231:{
                    size_t msg = encrypted_expected_msg_size(13); //magic number I know
                    // printf("%d\n", msg);
                    uint8_t incoming_msg[msg];
                    uint8_t p[13]; //magic here too I know
                    uint8_t msg_size = 0;
                    uint8_t key[16] = { //temporary key
                        0xA3, 0x7F, 0x1C, 0xD9, 0x4B, 0x02, 0xE8, 0x55,
                        0x9A, 0x3D, 0x60, 0xF7, 0x8E, 0x21, 0xB4, 0xC0
                    };
                    memcpy(incoming_msg, queue_can.can_msg, msg);
                    decrypt_transmission(0x231, incoming_msg, sizeof(incoming_msg), p, &msg_size, key);
                    printf("Message Decrypted!\n");
                    printf("%s\n", p);
                    led_strip_set_pixel(led_strip, 0, 0, 0, QUARTER_BRIGHTNESS);
                    led_strip_refresh(led_strip);
                    break;
                }
            }
        }else{
            ESP_LOGI("Awaiting", "No new data... yet");
            led_strip_set_pixel(led_strip, 0, QUARTER_BRIGHTNESS, QUARTER_BRIGHTNESS, 0);
            led_strip_refresh(led_strip);
        }
    }
}

/* ============================================================================
 * INITIALIZATION FUNCTIONS
 * ============================================================================ */

/* ---------- LED Initialization ---------- */
static void init_led(void) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_STRIP_RMT_RES_HZ,
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(
        led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    assert(led_strip != NULL);
}

/* ---------- I2C Master Initialization ---------- */
static void i2c_master_init() {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_GPIO,
        .scl_io_num = I2C_MASTER_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    /* Configure device-specific I2C settings */
#if I2C_DEVICE == 0
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AHT20_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
#elif I2C_DEVICE == 1
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LSM_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
#elif I2C_DEVICE == 2
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VELM_SLAVE_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
#else
    ESP_LOGI("I2C SELECTION", "No I2C Device Selected\n");
#endif

    ESP_ERROR_CHECK(
        i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));
}

/* ---------- Task Initialization ---------- */
void tasks_init() {
    twai_queue_isr = xQueueCreate(CAN_QUEUE_SIZE, sizeof(can_messasge_t));

    if (twai_queue_isr == NULL) {
        ESP_LOGE("ERROR", "Could not create queue!!");
    }

#if DEVICE_MODE == 0
    // nimble_ble_init();
    xTaskCreate(messenger_manager, "Messenger Manager", 4069, NULL, 7, NULL);
#elif DEVICE_MODE == 1
    xTaskCreate(aht_20_sensor, "AHT20 Sensor Func", 4096, NULL, 5, &aht_device);
#elif DEVICE_MODE == 2
    xTaskCreate(imu_sensor, "LSM6DSO23 Sensor", 4096, NULL, 4, NULL);
#elif DEVICE_MODE == 3
    xTaskCreate(veml_sensor_task, "VEML7700 Sensor", 4096, NULL, 4, NULL);
#else
#error "NO DEVICE SELECTED!"
#endif
}

/* ========================================================================================================================================================
 * APPLICATION ENTRY POINT
 * ======================================================================================================================================================== */

void app_main(void) {
    /* ---------- Initialize Peripherals ---------- */
    init_led();
    twai_init();
    i2c_master_init();
    tasks_init();
    ble_init();


    // uint8_t message1[] = "Hello, World";
    // uint8_t key[16] = {
    //     0xA3, 0x7F, 0x1C, 0xD9, 0x4B, 0x02, 0xE8, 0x55,
    //     0x9A, 0x3D, 0x60, 0xF7, 0x8E, 0x21, 0xB4, 0xC0
    // };

    while(1){
        // printf("%s\n", message1);
        // encrypt_transmit_msg(0x231, node_hdl, message1, sizeof(message1), key);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }

}
