#ifndef CANASCON_BASE_T
#define CANASCON_BASE_T

#include "esp_twai.h"
#include <stdint.h>

esp_err_t encrypt_transmit_msg(uint16_t can_id, twai_node_handle_t node_hdl, const uint8_t *message, size_t message_len);


#endif
