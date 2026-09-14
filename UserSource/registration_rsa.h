#ifndef REGISTRATION_RSA_H
#define REGISTRATION_RSA_H

#include <stdint.h>

#define REGISTRATION_RSA_CIPHER_LENGTH 64U
#define REGISTRATION_RSA_PLAIN_LENGTH  30U

void RegistrationRSA_Decrypt(const uint8_t cipher[REGISTRATION_RSA_CIPHER_LENGTH],
  uint8_t plain[REGISTRATION_RSA_PLAIN_LENGTH]);

#endif