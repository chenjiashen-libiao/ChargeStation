#include "registration_rsa.h"

#include <string.h>

#define BIGINT_MAX_LENGTH 35U

typedef struct
{
  uint32_t length;
  uint32_t value[BIGINT_MAX_LENGTH];
} BigInt;

static const uint8_t modulus[REGISTRATION_RSA_CIPHER_LENGTH] =
{
  0x0D, 0xF6, 0xE5, 0x6F, 0xEB, 0x1D, 0xDA, 0x8C,
  0xA8, 0x02, 0xCC, 0x77, 0xAB, 0x65, 0xA9, 0x2D,
  0xE8, 0xEC, 0xB6, 0x64, 0x3E, 0x2C, 0xF1, 0x5D,
  0xE6, 0xF9, 0x76, 0xE5, 0x82, 0xBE, 0x88, 0x48,
  0x80, 0x2D, 0x8D, 0x2C, 0x90, 0xAE, 0x9B, 0x58,
  0xFA, 0xBD, 0xCB, 0xC7, 0xFD, 0xA6, 0x00, 0x10,
  0xE0, 0x6A, 0x7F, 0x0F, 0x59, 0xA2, 0x25, 0xD5,
  0x38, 0x73, 0xC1, 0xEE, 0xF4, 0x82, 0x2D, 0x0B
};
static const uint8_t publicKey[3] = {0x01, 0x00, 0x01};

static void BigInt_Set(BigInt *number, uint64_t value);
static int BigInt_Compare(const BigInt *left, const BigInt *right);
static BigInt BigInt_Add(const BigInt *left, const BigInt *right);
static BigInt BigInt_Subtract(const BigInt *left, const BigInt *right);
static BigInt BigInt_MultiplyWord(const BigInt *number, uint32_t multiplier);
static BigInt BigInt_Mod(const BigInt *number, const BigInt *modulusValue);
static void BigInt_FromBytes(BigInt *number, const uint8_t *data, uint32_t length);
static void BigInt_ToBytes(const BigInt *number, uint8_t *data, uint32_t length);
static BigInt BigInt_ModExp(const BigInt *base, const BigInt *exponent,
  const BigInt *modulusValue);

void RegistrationRSA_Decrypt(const uint8_t cipher[REGISTRATION_RSA_CIPHER_LENGTH],
  uint8_t plain[REGISTRATION_RSA_PLAIN_LENGTH])
{
  BigInt exponent;
  BigInt modulusValue;
  BigInt cipherValue;
  BigInt plainValue;

  BigInt_FromBytes(&exponent, publicKey, sizeof(publicKey));
  BigInt_FromBytes(&modulusValue, modulus, sizeof(modulus));
  BigInt_FromBytes(&cipherValue, cipher, REGISTRATION_RSA_CIPHER_LENGTH);
  plainValue = BigInt_ModExp(&cipherValue, &exponent, &modulusValue);
  BigInt_ToBytes(&plainValue, plain, REGISTRATION_RSA_PLAIN_LENGTH);
}

static void BigInt_Set(BigInt *number, uint64_t value)
{
  memset(number, 0, sizeof(*number));
  number->length = (value > UINT32_MAX) ? 2U : 1U;
  number->value[0] = (uint32_t)value;
  number->value[1] = (uint32_t)(value >> 32);
}

static int BigInt_Compare(const BigInt *left, const BigInt *right)
{
  int32_t index;

  if (left->length != right->length)
  {
    return (left->length > right->length) ? 1 : -1;
  }

  for (index = (int32_t)left->length - 1; index >= 0; index--)
  {
    if (left->value[index] != right->value[index])
    {
      return (left->value[index] > right->value[index]) ? 1 : -1;
    }
  }

  return 0;
}

static BigInt BigInt_Add(const BigInt *left, const BigInt *right)
{
  BigInt result = *left;
  uint64_t sum;
  uint32_t carry = 0U;
  uint32_t index;

  if (result.length < right->length)
  {
    result.length = right->length;
  }

  for (index = 0U; index < result.length; index++)
  {
    sum = (uint64_t)result.value[index] + right->value[index] + carry;
    result.value[index] = (uint32_t)sum;
    carry = (uint32_t)(sum >> 32);
  }

  if ((carry != 0U) && (result.length < BIGINT_MAX_LENGTH))
  {
    result.value[result.length++] = carry;
  }

  return result;
}

static BigInt BigInt_Subtract(const BigInt *left, const BigInt *right)
{
  BigInt result = *left;
  uint64_t subtrahend;
  uint32_t borrow = 0U;
  uint32_t index;

  if (BigInt_Compare(left, right) <= 0)
  {
    BigInt_Set(&result, 0U);
    return result;
  }

  for (index = 0U; index < left->length; index++)
  {
    subtrahend = (uint64_t)right->value[index] + borrow;
    borrow = ((uint64_t)left->value[index] < subtrahend) ? 1U : 0U;
    result.value[index] = (uint32_t)((uint64_t)left->value[index] - subtrahend);
  }

  while ((result.length > 1U) && (result.value[result.length - 1U] == 0U))
  {
    result.length--;
  }

  return result;
}

static BigInt BigInt_MultiplyWord(const BigInt *number, uint32_t multiplier)
{
  BigInt result = *number;
  uint64_t product;
  uint32_t carry = 0U;
  uint32_t index;

  for (index = 0U; index < number->length; index++)
  {
    product = (uint64_t)number->value[index] * multiplier + carry;
    result.value[index] = (uint32_t)product;
    carry = (uint32_t)(product >> 32);
  }

  if ((carry != 0U) && (result.length < BIGINT_MAX_LENGTH))
  {
    result.value[result.length++] = carry;
  }

  return result;
}

static BigInt BigInt_Mod(const BigInt *number, const BigInt *modulusValue)
{
  BigInt result = *number;
  BigInt product;
  uint64_t dividend;
  uint64_t divisor;
  uint32_t quotient;
  uint32_t shift;
  uint32_t index;

  while (BigInt_Compare(&result, modulusValue) >= 0)
  {
    dividend = result.value[result.length - 1U];
    divisor = modulusValue->value[modulusValue->length - 1U];
    shift = result.length - modulusValue->length;

    if ((dividend == divisor) && (shift == 0U))
    {
      result = BigInt_Subtract(&result, modulusValue);
      break;
    }

    if ((dividend <= divisor) && (shift > 0U))
    {
      shift--;
      dividend = (dividend << 32) | result.value[result.length - 2U];
    }

    quotient = (uint32_t)(dividend / (divisor + 1U));
    product = BigInt_MultiplyWord(modulusValue, quotient);

    if (shift > 0U)
    {
      for (index = product.length; index > 0U; index--)
      {
        product.value[index + shift - 1U] = product.value[index - 1U];
      }
      memset(product.value, 0, shift * sizeof(product.value[0]));
      product.length += shift;
    }

    result = BigInt_Subtract(&result, &product);
  }

  return result;
}

static void BigInt_FromBytes(BigInt *number, const uint8_t *data, uint32_t length)
{
  uint32_t index;

  BigInt_Set(number, 0U);
  if (length > (BIGINT_MAX_LENGTH * sizeof(uint32_t)))
  {
    length = BIGINT_MAX_LENGTH * sizeof(uint32_t);
  }

  for (index = 0U; index < length; index++)
  {
    number->value[index >> 2] |= (uint32_t)data[index] << ((index & 3U) * 8U);
  }

  number->length = (length + 3U) >> 2;
  while ((number->length > 1U) && (number->value[number->length - 1U] == 0U))
  {
    number->length--;
  }
}

static void BigInt_ToBytes(const BigInt *number, uint8_t *data, uint32_t length)
{
  uint32_t outputLength = length;
  uint32_t index;

  memset(data, 0, length);
  if (outputLength > (number->length * sizeof(uint32_t)))
  {
    outputLength = number->length * sizeof(uint32_t);
  }

  for (index = 0U; index < outputLength; index++)
  {
    data[index] = (uint8_t)(number->value[index >> 2] >> ((index & 3U) * 8U));
  }
}

static BigInt BigInt_ModExp(const BigInt *base, const BigInt *exponent,
  const BigInt *modulusValue)
{
  BigInt result = *base;
  BigInt accumulator;
  BigInt product;
  uint32_t bitCount = (exponent->length - 1U) * 32U;
  uint32_t value = exponent->value[exponent->length - 1U];
  int32_t bit;
  uint32_t index;

  while (value != 0U)
  {
    value >>= 1;
    bitCount++;
  }

  for (bit = (int32_t)bitCount - 2; bit >= 0; bit--)
  {
    accumulator = BigInt_MultiplyWord(&result, result.value[result.length - 1U]);
    accumulator = BigInt_Mod(&accumulator, modulusValue);
    for (index = 1U; index < result.length; index++)
    {
      memmove(&accumulator.value[1], &accumulator.value[0],
        accumulator.length * sizeof(accumulator.value[0]));
      accumulator.value[0] = 0U;
      accumulator.length++;
      product = BigInt_MultiplyWord(&result, result.value[result.length - index - 1U]);
      accumulator = BigInt_Add(&accumulator, &product);
      accumulator = BigInt_Mod(&accumulator, modulusValue);
    }
    result = accumulator;

    if (((exponent->value[(uint32_t)bit >> 5] >> ((uint32_t)bit & 31U)) & 1U) != 0U)
    {
      accumulator = BigInt_MultiplyWord(base, result.value[result.length - 1U]);
      accumulator = BigInt_Mod(&accumulator, modulusValue);
      for (index = 1U; index < result.length; index++)
      {
        memmove(&accumulator.value[1], &accumulator.value[0],
          accumulator.length * sizeof(accumulator.value[0]));
        accumulator.value[0] = 0U;
        accumulator.length++;
        product = BigInt_MultiplyWord(base, result.value[result.length - index - 1U]);
        accumulator = BigInt_Add(&accumulator, &product);
        accumulator = BigInt_Mod(&accumulator, modulusValue);
      }
      result = accumulator;
    }
  }

  return result;
}