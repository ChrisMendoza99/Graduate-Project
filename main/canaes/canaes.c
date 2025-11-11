// #include "aes/esp_aes.h"
// #include "aes/esp_aes_gcm.h"
// #include "mbedtls/aes.h"
#include "esp_err.h"
#include "mbedtls/ccm.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_random.h"
#include "esp_twai_types.h"
#include "freertos/idf_additions.h"
#include "hal/twai_types.h"
#include "mbedtls/cipher.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/_types.h>
#include "canaes.h"


void aes_ccm_init(mbedtls_ccm_context *ctx, uint8_t *key){
    mbedtls_ccm_init(ctx);
    mbedtls_ccm_setkey(ctx, MBEDTLS_CIPHER_ID_AES, key, AES_128_KEY_SIZE);
}


esp_err_t encrypt_transmit_msg(uint16_t can_id, twai_node_handle_t node_hdl, const uint8_t *message, size_t message_len, uint8_t *key){

    int rc = mbedtls_ccm_encrypt_and_tag(
        mbedtls_ccm_context *ctx,
        size_t length,
        const unsigned char *iv,
        size_t iv_len,
        const unsigned char *ad,
        size_t ad_len,
        const unsigned char *input,
        unsigned char *output,
        unsigned char *tag,
        size_t tag_len
    );


    return ESP_OK;
}


esp_err_t decrypt_transmission(uint16_t can_id, const uint8_t *crypto_message, size_t crypto_message_len, uint8_t *plaintext, uint8_t *msg_size, uint8_t *key){

    int rc = mbedtls_ccm_auth_decrypt(
        mbedtls_ccm_context *ctx,
        size_t length,
        const unsigned char *iv,
        size_t iv_len,
        const unsigned char *ad,
        size_t ad_len,
        const unsigned char *input,
        unsigned char *output,
        const unsigned char *tag,
        size_t tag_len
    );
    if(rc == MBEDTLS_ERR_CCM_AUTH_FAILED){
        return ESP_FAIL;
    }
    return ESP_OK;
}
