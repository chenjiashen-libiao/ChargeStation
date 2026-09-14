#include "CAN_CMD.h"

#include <stdint.h>
#include <string.h>
#include "../board_config.h"
#include "Drivers/can.h"
#include "registration.h"

#define CAN_ID_SN_SEND             0x312U
#define CAN_ID_SN_BIND             0x313U
#define CAN_ID_UID_GET             0x601U
#define CAN_ID_UID_SEND            0x602U
#define CAN_ID_IAP_ENTER           0x603U
#define CAN_ID_RESPONSE_OK         0x60AU
#define CAN_ID_UID_ENCRYPT_DATA    0x60BU
#define CAN_ID_SN_GET              0x60DU

#define CAN_CMD_CIPHER_LENGTH      64U

static uint8_t cipherData[CAN_CMD_CIPHER_LENGTH];
static uint16_t receivedLength;
static uint8_t receivingCipher;
static uint8_t deviceNumber = 0xFFU;

static void CAN_CMD_HandleFrame(uint32_t id, uint8_t length, const uint8_t *data);
static void CAN_CMD_SendUID(void);

void CAN_CMD_Process(void)
{
  uint32_t id;
  uint8_t length;
  uint8_t data[8];

  while (CAN_GetLastRx(&id, &length, data) != 0U)
  {
    CAN_CMD_HandleFrame(id, length, data);
  }
}

static void CAN_CMD_HandleFrame(uint32_t id, uint8_t length, const uint8_t *data)
{
  const uint8_t *sn = Registration_GetSN();
  uint16_t declaredLength;
  uint16_t copyLength;
  uint8_t response[2] = {0U, 0U};

  switch (id)
  {
    case CAN_ID_IAP_ENTER:
      if ((length >= 1U) && (data[0] == deviceNumber))
      {
        NVIC_SystemReset();
      }
      break;

    case CAN_ID_UID_GET:
      CAN_CMD_SendUID();
      break;

    case CAN_ID_UID_ENCRYPT_DATA:
      if (receivingCipher == 0U)
      {
        if (length < 2U)
        {
          break;
        }
        declaredLength = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
        if (declaredLength != CAN_CMD_CIPHER_LENGTH)
        {
          break;
        }
        receivedLength = 0U;
        receivingCipher = 1U;
        copyLength = length - 2U;
        memcpy(cipherData, data + 2U, copyLength);
        receivedLength = copyLength;
      }
      else
      {
        copyLength = length;
        if (copyLength > (CAN_CMD_CIPHER_LENGTH - receivedLength))
        {
          copyLength = CAN_CMD_CIPHER_LENGTH - receivedLength;
        }
        memcpy(cipherData + receivedLength, data, copyLength);
        receivedLength += copyLength;
      }

      if ((receivingCipher != 0U) && (receivedLength == CAN_CMD_CIPHER_LENGTH))
      {
        receivingCipher = 0U;
        receivedLength = 0U;
        if (Registration_Save(cipherData, sizeof(cipherData)) == HAL_OK)
        {
          CAN_SendDataId(CAN_ID_RESPONSE_OK, response, sizeof(response));
        }
      }
      break;

    case CAN_ID_SN_GET:
      if ((Registration_IsLegal() != 0U) && (length >= 1U) &&
        (data[0] == (uint8_t)BOARD_DEVICE_TYPE))
      {
        if ((length < 3U) || (data[1] == 0U) || ((data[1] & sn[4]) == data[2]))
        {
          CAN_CMD_SendSN();
        }
      }
      break;

    case CAN_ID_SN_BIND:
      if ((Registration_IsLegal() != 0U) && (length == 8U) &&
        (memcmp(data, sn, 7U) == 0))
      {
        deviceNumber = data[7];
      }
      break;

    default:
      break;
  }
}

static void CAN_CMD_SendUID(void)
{
  const uint8_t *hardwareUID = (const uint8_t *)BOARD_UID_ADDRESS;
  uint8_t data[8];

  memcpy(data, hardwareUID, sizeof(data));
  CAN_SendDataId(CAN_ID_UID_SEND, data, sizeof(data));
  memset(data, 0, sizeof(data));
  memcpy(data, hardwareUID + 8U, 4U);
  data[7] = (uint8_t)BOARD_DEVICE_TYPE;
  CAN_SendDataId(CAN_ID_UID_SEND, data, sizeof(data));
}

void CAN_CMD_SendSN(void)
{
  CAN_SendDataId(CAN_ID_SN_SEND, (uint8_t *)Registration_GetSN(), 7U);
}