#include "esp_log.h"
#include "esp_twai.h"
#include "esp_random.h"
#include "esp_twai_types.h"
#include "freertos/idf_additions.h"
#include "hal/twai_types.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "esp32_ascon/encrypt.h"
#include "portmacro.h"

//I am conflicted, casue I think there is a bug somewhere in my code that
// causing my encrypt/decrypt to return a constant sized cipher/plaintext
// I am unsure, as to where it is occuring. for Now I will just do this
// directly since I do not trust it for now.
//
// The best thing I can do is just to use the direct api for ascon
// The first step is extracting whatever messasge is being sent,
// gather the nonce and the associated data.


#define ASCON_TAG_SIZE 16
#define CANFD_MAX_DATA_LEN 64

/* Encrypt the data, and send the data */
esp_err_t encrypt_transmit_msg(uint16_t can_id, twai_node_handle_t node_hdl, const uint8_t *message, size_t message_len) {
    // --- Constants ---
    const uint8_t key[16] = {
        0xA3, 0x7F, 0x1C, 0xD9, 0x4B, 0x02, 0xE8, 0x55,
        0x9A, 0x3D, 0x60, 0xF7, 0x8E, 0x21, 0xB4, 0xC0
    };
    const size_t TAG_SIZE = ASCON_TAG_SIZE;

    // --- Check message size ---
    if (message_len > CANFD_MAX_DATA_LEN) {
        ESP_LOGE("BUFFER ERROR", "Message size exceeds 64 bytes (CAN FD limit)!");
        return ESP_FAIL;
    }
    //--- TWAI Transmit Frame ---
    twai_frame_t msg = {
        .header.id = can_id,
        .header.fdf = 1
    };

    // --- Buffers ---
    uint8_t ciphertext[TAG_SIZE + message_len];
    uint8_t nonce[16] = {0};
    uint8_t ad[4] = {0};
    unsigned long long clen = 0;

    static uint8_t tx_buff[CANFD_MAX_DATA_LEN];
    memset(tx_buff, 0, sizeof(tx_buff));

    // --- Prepare nonce and associated data ---
    esp_fill_random(nonce, sizeof(nonce));
    uint8_t nonce_size = sizeof(nonce);


    uint32_t id = msg.header.id;
    ad[0] = (uint8_t)(id & 0xFF);
    ad[1] = (uint8_t)((id >> 8) & 0xFF);
    ad[2] = (uint8_t)((id >> 16) & 0xFF);
    ad[3] = (uint8_t)((id >> 24) & 0xFF);

    printf("%d\n", nonce_size);
    // --- Debug: Nonce and associated data ---
    for (uint8_t i = 0; i < sizeof(nonce); i++) {
        ESP_LOGI("NONCE DATA", "0x%02X", nonce[i]);
    }

    for (uint8_t i = 0; i < sizeof(ad); i++) {
        ESP_LOGI("ASSOCIATED DATA", "0x%02X", ad[i]);
    }

    ESP_LOGI("PRE-CIPHERTEXT SIZE", "%d", sizeof(ciphertext));

    // --- Encryption ---
    int ret = crypto_aead_encrypt(
        ciphertext, &clen,
        message, message_len,
        ad, sizeof(ad),
        NULL, nonce,
        key
    );

    if (ret != 0) {
        ESP_LOGE("ASCON ERROR", "Encryption failed!");
        return ESP_FAIL;
    }

    memcpy(&tx_buff[0], ciphertext, clen);
    memcpy(&tx_buff[clen], nonce, nonce_size);
    // printf("%d\n", nonce_size);
    // printf("%llu\n", clen);

    size_t frame_message_size = clen + nonce_size;

    printf("%d\n", frame_message_size);

    msg.buffer = tx_buff;
    msg.buffer_len = frame_message_size;
    msg.header.dlc = twaifd_len2dlc(frame_message_size);

    ESP_LOGI("CIPHERTEXT SIZE", "%llu", clen);

    // --- Debug: Ciphertext output ---
    for (uint8_t i = 0; i < clen; i++) {
        ESP_LOGI("CIPHERTEXT", "%d | 0x%02X", i,ciphertext[i]);
    }

    // --- Debug: Full Transmit Message output ---
    for (uint8_t i = 0; i < msg.buffer_len; i++) {
        ESP_LOGI("FULL TRANSMIT MESSAGE", "0x%02X", tx_buff[i]);
    }

    printf("%d\n", msg.buffer_len);

    // --- Transmit encrypted message ---
    // (You might later want to split ciphertext into multiple CAN FD frames if >64 bytes)
    if (twai_node_transmit(node_hdl, &msg, portMAX_DELAY) == ESP_OK) {
        ESP_LOGI("TRANSMIT SUCCESS", "Message sent!");
        return ESP_OK;
    }else{
        ESP_LOGE("TRANSMIT ERROR", "Could not send message!");
        return ESP_FAIL;
    }
    return ESP_OK;
}

/*TODO: decrypt the data, and send the data */
