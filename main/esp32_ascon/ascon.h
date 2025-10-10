#ifndef ASCON_H_
#define ASCON_H_

#include <stdint.h>

typedef union {
  uint64_t x;
  struct {
    uint32_t l;
    uint32_t h;
  };
} uint32x2_t;

typedef union {
  uint64_t x[5];
  uint32x2_t w[5];
  uint8_t b[5][8];
} ascon_state_t;


/* Ascon Struct */
typedef struct{
    uint8_t plaintext[16];
    uint8_t associated_data[16];
    uint8_t nonce[16];
    uint8_t key[16];
    uint8_t ciphertext[32];
    unsigned long long clen;
    unsigned long long plen;
}ascon_t;



#endif  // ASCON_H_
