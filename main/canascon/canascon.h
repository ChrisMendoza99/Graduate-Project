#ifndef CANASCON_BASE_T
#define CANASCON_BASE_T

#include "esp_twai.h"
#include <stdint.h>

/* Encrypt the data, and send the data */
esp_err_t encrypt_transmit_msg(uint16_t can_id, twai_node_handle_t node_hdl, const uint8_t *message, size_t message_len, uint8_t *key);
esp_err_t decrypt_transmission(uint16_t can_id, const uint8_t *crypto_message, size_t crypto_message_len, uint8_t *plaintext, uint8_t *msg_size,uint8_t *key);
size_t encrypted_expected_msg_size(uint8_t packet_size);
#endif
