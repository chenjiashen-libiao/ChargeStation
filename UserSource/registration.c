#include "registration.h"

#include <string.h>
#include "../board_config.h"
#include "registration_rsa.h"

#define REGISTRATION_SN_LENGTH     7U
#define REGISTRATION_UID_LENGTH    12U

static uint8_t uidInFlash[REGISTRATION_UID_LENGTH];
static uint8_t snInFlash[REGISTRATION_SN_LENGTH];
static uint8_t isLegal;

HAL_StatusTypeDef Registration_Save(const uint8_t *data, uint16_t length)
{
  HAL_StatusTypeDef status;
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t pageError = 0U;
  uint32_t address;
  uint16_t value;

  if (length != REGISTRATION_RSA_CIPHER_LENGTH)
  {
    return HAL_ERROR;
  }

  status = HAL_FLASH_Unlock();
  if (status != HAL_OK)
  {
    return status;
  }

  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = BOARD_REGISTRATION_ADDRESS;
  erase.NbPages = 1U;
  status = HAL_FLASHEx_Erase(&erase, &pageError);

  if (status == HAL_OK)
  {
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
      BOARD_REGISTRATION_ADDRESS, length);
  }

  for (address = 0U; (address < length) && (status == HAL_OK); address += 2U)
  {
    value = (uint16_t)data[address] | ((uint16_t)data[address + 1U] << 8);
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
      BOARD_REGISTRATION_ADDRESS + sizeof(uint16_t) + address, value);
  }

  HAL_FLASH_Lock();
  if (status == HAL_OK)
  {
    Registration_Load();
  }
  return status;
}

uint8_t Registration_IsLegal(void)
{
  return isLegal;
}

const uint8_t *Registration_GetSN(void)
{
  return snInFlash;
}

void Registration_Load(void)
{
  const uint8_t *storedData = (const uint8_t *)BOARD_REGISTRATION_ADDRESS;
  const uint8_t *hardwareUID = (const uint8_t *)BOARD_UID_ADDRESS;
  uint16_t storedLength;
  uint8_t plainData[REGISTRATION_RSA_PLAIN_LENGTH];

  isLegal = 0U;
  memset(uidInFlash, 0xFF, sizeof(uidInFlash));
  memset(snInFlash, 0xFF, sizeof(snInFlash));
  memcpy(&storedLength, storedData, sizeof(storedLength));
  if (storedLength != REGISTRATION_RSA_CIPHER_LENGTH)
  {
    return;
  }

  RegistrationRSA_Decrypt(storedData + sizeof(storedLength), plainData);
  memcpy(uidInFlash, plainData, sizeof(uidInFlash));
  memcpy(snInFlash, plainData + sizeof(uidInFlash), sizeof(snInFlash));
  if (memcmp(uidInFlash, hardwareUID, sizeof(uidInFlash)) == 0)
  {
    isLegal = 1U;
  }
}