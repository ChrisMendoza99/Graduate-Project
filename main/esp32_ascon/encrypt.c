#include "core.h"
#include "encrypt.h"
#include "ascon.h"
#include "string.h"


int crypto_aead_encrypt(unsigned char* c, unsigned long long* clen,
                        const unsigned char* m, unsigned long long mlen,
                        const unsigned char* ad, unsigned long long adlen,
                        const unsigned char* nsec, const unsigned char* npub,
                        const unsigned char* k) {
  ascon_state_t s;
  (void)nsec;

  // set ciphertext size
  *clen = mlen + CRYPTO_ABYTES;

  ascon_core(&s, c, m, mlen, ad, adlen, npub, k, ASCON_ENC);

  // get tag
  int i;
  for (i = 0; i < CRYPTO_ABYTES; ++i) c[mlen + i] = *(s.b[3] + i);

  return 0;
}


/*Encrypt Wrapper */
int ascon_encrypt(ascon_t *encryption){

    int check = crypto_aead_encrypt(
        encryption->ciphertext, &encryption->clen,
        encryption->plaintext, sizeof(encryption->plaintext),
        encryption->associated_data, sizeof(encryption->associated_data),
        NULL, encryption->nonce,
        encryption->key);
    return check;
}
