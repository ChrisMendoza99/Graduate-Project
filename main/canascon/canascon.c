#include "esp_log.h"
#include "esp_twai.h"
#include "esp_random.h"
#include "esp_twai_types.h"
#include "freertos/idf_additions.h"
#include "hal/twai_types.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <sys/_types.h>
#include "esp32_ascon/encrypt.h"
#include "esp32_ascon/decrypt.h"
#include "portmacro.h"

// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------
#define ASCON_TAG_SIZE      (16)
#define ASCON_NONCE_SIZE    (16)
#define CANFD_MAX_DATA_LEN  (64)

// -----------------------------------------------------------------------------
// Helper Functions
// -----------------------------------------------------------------------------
size_t transmit_msg_size(size_t msg_size) {
    return msg_size + ASCON_TAG_SIZE;
}

size_t encrypted_expected_msg_size(uint8_t packet_size) {
    return packet_size + ASCON_TAG_SIZE + ASCON_NONCE_SIZE;
}



// -----------------------------------------------------------------------------
// Encrypt and Transmit
// -----------------------------------------------------------------------------
esp_err_t encrypt_transmit_msg(uint16_t can_id,
                               twai_node_handle_t node_hdl,
                               const uint8_t *message,
                               size_t message_len,
                               uint8_t *key)
{
    // --- Check message size ---
    if (message_len > CANFD_MAX_DATA_LEN) {
        ESP_LOGE("BUFFER ERROR", "Message size exceeds 64 bytes (CAN FD limit)!");
        return ESP_FAIL;
    }

    // --- TWAI Transmit Frame ---
    twai_frame_t msg = {
        .header.id  = can_id,
        .header.fdf = 1
    };

    // --- Buffers ---
    size_t encrypt_buff_size = transmit_msg_size(message_len);
    uint8_t ciphertext[encrypt_buff_size];
    uint8_t nonce[ASCON_NONCE_SIZE] = {0};
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

    // --- Combine Ciphertext + Nonce ---
    memcpy(&tx_buff[0], ciphertext, clen);
    memcpy(&tx_buff[clen], nonce, nonce_size);

    size_t frame_message_size = clen + nonce_size;

    msg.buffer     = tx_buff;
    msg.buffer_len = frame_message_size;
    msg.header.dlc = twaifd_len2dlc(frame_message_size);
    //-------------------------------------------------------
    printf("================Ciphertext + Tag================\n");
    for (int i = 0; i < clen; i++) {
      printf("%02x ", ciphertext[i]);
      if ((i + 1) % 16 == 0 || i + 1 == clen) {
        printf(" ");
        printf("\n");
      }
    }
    //-------------------------------------------------------
    printf("================Random Nonce #================\n");
    for (int i = 0; i < nonce_size; i++) {
      printf("%02x ", nonce[i]);
      if ((i + 1) % 16 == 0 || i + 1 == nonce_size) {
        printf(" ");
        printf("\n");
      }
    }
    //-------------------------------------------------------
    printf("================Full Encryption Message================\n");
    for (int i = 0; i < frame_message_size; i++) {
      printf("%02x ", tx_buff[i]);
      if ((i + 1) % 16 == 0 || i + 1 == frame_message_size) {
        printf(" ");
        printf("\n");
      }
    }
    //-------------------------------------------------------


    // --- Transmit encrypted message ---
    if (twai_node_transmit(node_hdl, &msg, portMAX_DELAY) == ESP_OK) {
        ESP_LOGI("TRANSMIT SUCCESS", "Message sent!");
        return ESP_OK;
    } else {
        ESP_LOGE("TRANSMIT ERROR", "Could not send message!");
        return ESP_FAIL;
    }

    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Decrypt Received Message
// -----------------------------------------------------------------------------
esp_err_t decrypt_transmission(uint16_t can_id,
                               const uint8_t *crypto_message,
                               size_t crypto_message_len,
                               uint8_t *plaintext,
                               uint8_t *msg_size,
                               uint8_t *key)
{
    uint8_t ad[4] = {0};

    // --- Validate message size ---
    if (crypto_message_len > CANFD_MAX_DATA_LEN) {
        ESP_LOGE("SIZE ERROR", "Message size exceeds 64 bytes (CAN FD limit)!");
        return ESP_FAIL;
    }

    // --- Separate ciphertext and nonce ---
    uint8_t ciphertext_size = crypto_message_len - ASCON_NONCE_SIZE;
    uint8_t ciphertext_msg[ciphertext_size];
    uint8_t nonce[ASCON_NONCE_SIZE];

    memcpy(ciphertext_msg, crypto_message, ciphertext_size);
    memcpy(nonce, &crypto_message[ciphertext_size], ASCON_NONCE_SIZE);

    printf("================Full Encryption Message================\n");
    for (int i = 0; i < crypto_message_len; i++) {
      printf("%02x ", ciphertext_msg[i]);
      if ((i + 1) % 16 == 0 || i + 1 == crypto_message_len) {
        printf(" ");
        printf("\n");
      }
    }
    //-------------------------------------------------------


    // --- Prepare associated data ---
    ad[0] = (uint8_t)(can_id & 0xFF);
    ad[1] = (uint8_t)((can_id >> 8) & 0xFF);
    ad[2] = (uint8_t)((can_id >> 16) & 0xFF);
    ad[3] = (uint8_t)((can_id >> 24) & 0xFF);

    unsigned long long message_len = 0;

    // --- Decrypt ---
    printf("\n");
    int ret = crypto_aead_decrypt(
        plaintext, &message_len,
        NULL, ciphertext_msg,
        ciphertext_size,
        ad, sizeof(ad),
        nonce, key
    );

    //-------------------------------------------------------



    if (ret != 0) {
        ESP_LOGE("ASCON", "Decryption/authentication failed!");
        return ESP_FAIL;
    }
    // ESP_LOGI("ASCON", "Decryption/authentication success!");
    *msg_size = (uint8_t)message_len;
    return ESP_OK;
}
