#ifndef REGISTRATION_RSA_H
#define REGISTRATION_RSA_H

#include <stdint.h>

/* 注册协议使用512位RSA密文，解密结果只取业务需要的30字节 */
#define REGISTRATION_RSA_CIPHER_LENGTH  64  // 密文字节数
#define REGISTRATION_RSA_PLAIN_LENGTH   30  // 解密输出字节数

void RegistrationRSA_Decrypt(const uint8_t cipher[REGISTRATION_RSA_CIPHER_LENGTH],
  uint8_t plain[REGISTRATION_RSA_PLAIN_LENGTH]);

#endif