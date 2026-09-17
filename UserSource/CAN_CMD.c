#include "CAN_CMD.h"

#include <stdint.h>
#include <string.h>
#include "../board_config.h"
#include "Drivers/can.h"
#include "registration.h"
#include "registration_rsa.h"
#include "wireless_protocol.h"

/* 充电桩CAN业务命令ID */
#define CAN_ID_SN_SEND                 0x312  // 上报设备序列号
#define CAN_ID_SN_BIND                 0x313  // 绑定设备编号
#define CAN_ID_CHANNEL_REQUEST         0x403  // 请求设备通道信息
#define CAN_ID_CHANNEL_SEND            0x404  // 上报设备通道信息
#define CAN_ID_UID_GET                 0x601  // 请求芯片UID
#define CAN_ID_UID_SEND                0x602  // 上报芯片UID
#define CAN_ID_IAP_ENTER               0x603  // 复位进入Boot升级流程
#define CAN_ID_RESPONSE_OK             0x60A  // 命令执行成功应答
#define CAN_ID_UID_ENCRYPT_DATA        0x60B  // 分帧下发RSA注册密文
#define CAN_ID_SN_GET                  0x60D  // 请求设备序列号
#define CAN_ID_WIRELESS_SET            0x433  // 设置无线地址

static uint8_t cipherData[REGISTRATION_RSA_CIPHER_LENGTH];
static uint16_t receivedLength;
static uint8_t receivingCipher;
static uint8_t deviceNumber = 0xFF;

static void CAN_CMD_HandleFrame(uint32_t id, uint8_t length, const uint8_t *data);
static void CAN_CMD_SendUID(void);
static void CAN_CMD_SendWirelessAddress(void);

void CAN_CMD_Process(void)
{ // 清空CAN接收队列并逐帧分发业务命令
  uint32_t id;
  uint8_t length;
  uint8_t data[8];

  while (CAN_GetLastRx(&id, &length, data) != 0)
  {
    CAN_CMD_HandleFrame(id, length, data);
  }
}

static void CAN_CMD_HandleFrame(uint32_t id, uint8_t length, const uint8_t *data)
{ // 按CAN命令ID处理注册、升级和无线地址配置
  const uint8_t *sn = Registration_GetSN();
  uint16_t declaredLength;
  uint16_t copyLength;
  uint32_t wirelessAddress;
  uint8_t response[2] = {0, 0};

  switch (id)
  {
    case CAN_ID_CHANNEL_REQUEST:
      if ((length >= 2) && (data[0] == deviceNumber) && (data[1] == 0))
      {
        CAN_CMD_SendWirelessAddress();
      }
      break;

    case CAN_ID_IAP_ENTER:
      if ((length >= 1) && (data[0] == deviceNumber))
      {
        NVIC_SystemReset();
      }
      break;

    case CAN_ID_UID_GET:
      CAN_CMD_SendUID();
      break;

    case CAN_ID_UID_ENCRYPT_DATA:
      // RSA密文跨多帧传输，首帧携带总长度，收齐后再统一写入Flash
      if (receivingCipher == 0)
      {
        if (length < 2)
        {
          break;
        }
        declaredLength = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
        if (declaredLength != REGISTRATION_RSA_CIPHER_LENGTH)
        {
          break;
        }
        receivedLength = 0;
        receivingCipher = 1;
        copyLength = length - 2;
        memcpy(cipherData, data + 2, copyLength);
        receivedLength = copyLength;
      }
      else
      {
        copyLength = length;
        if (copyLength > (REGISTRATION_RSA_CIPHER_LENGTH - receivedLength))
        {
          copyLength = REGISTRATION_RSA_CIPHER_LENGTH - receivedLength;
        }
        memcpy(cipherData + receivedLength, data, copyLength);
        receivedLength += copyLength;
      }

      if ((receivingCipher != 0) &&
        (receivedLength == REGISTRATION_RSA_CIPHER_LENGTH))
      {
        receivingCipher = 0;
        receivedLength = 0;
        if (Registration_Save(cipherData, sizeof(cipherData)) == HAL_OK)
        {
          CAN_SendDataId(CAN_ID_RESPONSE_OK, response, sizeof(response));
        }
      }
      break;

    case CAN_ID_SN_GET:
      if ((Registration_IsLegal() != 0) && (length >= 1) &&
        (data[0] == (uint8_t)BOARD_DEVICE_TYPE))
      {
        if ((length < 3) || (data[1] == 0) || ((data[1] & sn[4]) == data[2]))
        {
          CAN_CMD_SendSN();
        }
      }
      break;

    case CAN_ID_SN_BIND:
      if ((Registration_IsLegal() != 0) && (length == 8) &&
        (memcmp(data, sn, 7) == 0))
      {
        deviceNumber = data[7];
      }
      break;

    case CAN_ID_WIRELESS_SET:
      if ((length == 8) && (data[7] == deviceNumber))
      {
        wirelessAddress = (uint32_t)data[0] |
          ((uint32_t)data[1] << 8) |
          ((uint32_t)data[2] << 16) |
          ((uint32_t)data[3] << 24);
        if ((wirelessAddress != 0) &&
          (wirelessAddress != WirelessProtocol_GetAddress()))
        {
          WirelessProtocol_SaveAddress(wirelessAddress);
        }
      }
      break;

    default:
      break;
  }
}

static void CAN_CMD_SendUID(void)
{ // 将12字节芯片UID和设备类型拆成两帧发送
  const uint8_t *hardwareUID = (const uint8_t *)BOARD_UID_ADDRESS;
  uint8_t data[8];

  memcpy(data, hardwareUID, sizeof(data));
  CAN_SendDataId(CAN_ID_UID_SEND, data, sizeof(data));
  memset(data, 0, sizeof(data));
  memcpy(data, hardwareUID + 8, 4);
  data[7] = (uint8_t)BOARD_DEVICE_TYPE;
  CAN_SendDataId(CAN_ID_UID_SEND, data, sizeof(data));
}

static void CAN_CMD_SendWirelessAddress(void)
{ // 按BMSP通道0格式上报设备号和当前无线桩号
  uint32_t wirelessAddress = WirelessProtocol_GetAddress();
  uint8_t data[8] = {0};

  data[0] = deviceNumber;
  data[1] = 0;
  data[2] = (uint8_t)wirelessAddress;
  data[3] = (uint8_t)(wirelessAddress >> 8);
  data[4] = (uint8_t)(wirelessAddress >> 16);
  data[5] = (uint8_t)(wirelessAddress >> 24);
  CAN_SendDataId(CAN_ID_CHANNEL_SEND, data, sizeof(data));
}

void CAN_CMD_SendSN(void)
{ // 发送设备注册序列号
  CAN_SendDataId(CAN_ID_SN_SEND, (uint8_t *)Registration_GetSN(), 7);
}