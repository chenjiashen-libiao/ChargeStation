#include "wireless_upgrade.h"
#include "charge_control.h"
#include "Drivers/si4463.h"
#include "../board_config.h"
#include <stm32f1xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include <string.h>

#pragma GCC optimize ("Os")

/* 旧Bootloader约定：升级信息页0x08012800，升级镜像从0x08012C00开始 */
#define UPGRADE_INFO_ADDRESS          0x08012800U
#define UPGRADE_IMAGE_MIN_ADDRESS     0x08012C00U
#define UPGRADE_FLASH_END_ADDRESS     0x08020000U
#define UPGRADE_FLASH_PAGE_SIZE       0x400U
#define UPGRADE_FRAME_SIZE            8U

#define UPGRADE_COMMAND_STATUS        0x07
#define UPGRADE_COMMAND_ENTER         0x08
#define UPGRADE_COMMAND_EXIT          0x09
#define UPGRADE_COMMAND_WRITE         0x0A
#define UPGRADE_COMMAND_INTEGRITY     0x0B
#define UPGRADE_COMMAND_CHECK         0x0D
#define UPGRADE_COMMAND_EXECUTE       0x0E

static uint8_t upgradeActive;
static uint8_t upgradeReady;
static uint8_t broadcastWriteEnabled;
static uint32_t imageAddress;
static uint32_t imageLength;
static uint32_t imageChecksum;
static uint32_t calculatedChecksum;
static uint8_t writtenFrames[848];

static uint32_t WirelessUpgrade_ReadU32(const uint8_t *data);
static void WirelessUpgrade_WriteU32(uint8_t *data, uint32_t value);
static uint8_t WirelessUpgrade_SendStatus(const uint8_t *request, uint8_t channel,
  uint8_t status);
static uint8_t WirelessUpgrade_ValidateImage(uint32_t address, uint32_t length);
static HAL_StatusTypeDef WirelessUpgrade_EraseImage(void);
static HAL_StatusTypeDef WirelessUpgrade_WriteFrame(const uint8_t *request);
static uint32_t WirelessUpgrade_FindMissingAddress(void);
static uint32_t WirelessUpgrade_CalculateChecksum(void);
static HAL_StatusTypeDef WirelessUpgrade_WriteBootInfo(void);

void WirelessUpgrade_Init(void)
{ // 初始化无线升级状态
  upgradeActive = 0;
  upgradeReady = 0;
  broadcastWriteEnabled = 0;
  imageAddress = 0;
  imageLength = 0;
  imageChecksum = 0;
  calculatedChecksum = 0;
  memset(writtenFrames, 0xFF, sizeof(writtenFrames));
}

uint8_t WirelessUpgrade_Process(const uint8_t *request, uint8_t broadcast,
  uint8_t channel)
{ // 处理与参考工程一致的固定长无线升级命令
  uint8_t response[SI4463_PACKET_LENGTH];
  uint8_t command = request[4];
  uint8_t status;
  uint32_t missingAddress;

  if (broadcast)
  {
    if ((command == UPGRADE_COMMAND_WRITE) && broadcastWriteEnabled &&
      upgradeReady)
    {
      WirelessUpgrade_WriteFrame(request);
      return 1;
    }
    return 0;
  }

  switch (command)
  {
    case UPGRADE_COMMAND_STATUS:
      if (upgradeActive && !upgradeReady)
      {
        return 1;
      }
      WirelessUpgrade_SendStatus(request, channel, 1);
      return 1;

    case UPGRADE_COMMAND_ENTER:
      imageAddress = 0x08000000U | (uint32_t)request[5] |
        ((uint32_t)request[6] << 8) | ((uint32_t)request[7] << 16);
      imageLength = WirelessUpgrade_ReadU32(request + 8);
      imageChecksum = WirelessUpgrade_ReadU32(request + 12);
      status = WirelessUpgrade_ValidateImage(imageAddress, imageLength) ? 1 : 2;
      if (!WirelessUpgrade_SendStatus(request, channel, status) || (status != 1))
      {
        return 1;
      }
      upgradeActive = 1;
      upgradeReady = 0;
      broadcastWriteEnabled = 1;
      calculatedChecksum = 0;
      ChargeControl_SetUpgradeMode(1);
      memset(writtenFrames, 0xFF, sizeof(writtenFrames));
      if (WirelessUpgrade_EraseImage() == HAL_OK)
      {
        upgradeReady = 1;
      }
      return 1;

    case UPGRADE_COMMAND_EXIT:
      WirelessUpgrade_SendStatus(request, channel, 1);
      broadcastWriteEnabled = 0;
      return 1;

    case UPGRADE_COMMAND_WRITE:
      WirelessUpgrade_SendStatus(request, channel, 1);
      if (broadcastWriteEnabled && upgradeReady)
      {
        WirelessUpgrade_WriteFrame(request);
      }
      return 1;

    case UPGRADE_COMMAND_INTEGRITY:
      if (!upgradeActive || !upgradeReady)
      {
        return 1;
      }
      missingAddress = WirelessUpgrade_FindMissingAddress();
      memcpy(response, request, sizeof(response));
      memset(response + 5, 0, sizeof(response) - 5);
      WirelessUpgrade_WriteU32(response + 5, missingAddress);
      SI4463_SendPacket(response, sizeof(response), channel);
      if (missingAddress == 0)
      {
        calculatedChecksum = WirelessUpgrade_CalculateChecksum();
      }
      return 1;

    case UPGRADE_COMMAND_CHECK:
      status = (calculatedChecksum == imageChecksum) ? 1 : 0;
      memcpy(response, request, sizeof(response));
      memset(response + 5, 0, sizeof(response) - 5);
      response[5] = status;
      if (!status)
      {
        WirelessUpgrade_WriteU32(response + 6, calculatedChecksum);
      }
      SI4463_SendPacket(response, sizeof(response), channel);
      return 1;

    case UPGRADE_COMMAND_EXECUTE:
      status = (request[6] == (uint8_t)BOARD_DEVICE_TYPE) ? 1 : 2;
      if (calculatedChecksum != imageChecksum)
      {
        status = 0;
      }
      if ((status == 1) && (WirelessUpgrade_WriteBootInfo() != HAL_OK))
      {
        status = 0;
      }
      WirelessUpgrade_SendStatus(request, channel, status);
      if (status == 1)
      {
        osDelay(20);
        NVIC_SystemReset();
      }
      return 1;

    default:
      return 0;
  }
}

static uint32_t WirelessUpgrade_ReadU32(const uint8_t *data)
{ // 按协议小端序读取32位值
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
    ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void WirelessUpgrade_WriteU32(uint8_t *data, uint32_t value)
{ // 按协议小端序写入32位值
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
  data[2] = (uint8_t)(value >> 16);
  data[3] = (uint8_t)(value >> 24);
}

static uint8_t WirelessUpgrade_SendStatus(const uint8_t *request, uint8_t channel,
  uint8_t status)
{ // 返回旧协议状态字节
  uint8_t response[SI4463_PACKET_LENGTH];

  memcpy(response, request, sizeof(response));
  memset(response + 5, 0, sizeof(response) - 5);
  response[5] = status;
  return SI4463_SendPacket(response, sizeof(response), channel);
}

static uint8_t WirelessUpgrade_ValidateImage(uint32_t address, uint32_t length)
{ // 限制升级镜像只能写入128KB Flash的暂存区
  if ((address < UPGRADE_IMAGE_MIN_ADDRESS) ||
    ((address & (UPGRADE_FLASH_PAGE_SIZE - 1U)) != 0) ||
    (length == 0) || (length > (UPGRADE_FLASH_END_ADDRESS - address)))
  {
    return 0;
  }
  return 1;
}

static HAL_StatusTypeDef WirelessUpgrade_EraseImage(void)
{ // 按1KB页擦除升级镜像暂存区
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t pageError = 0;
  HAL_StatusTypeDef status;

  status = HAL_FLASH_Unlock();
  if (status != HAL_OK)
  {
    return status;
  }
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = imageAddress;
  erase.NbPages = (imageLength + UPGRADE_FLASH_PAGE_SIZE - 1U) /
    UPGRADE_FLASH_PAGE_SIZE;
  status = HAL_FLASHEx_Erase(&erase, &pageError);
  HAL_FLASH_Lock();
  return status;
}

static HAL_StatusTypeDef WirelessUpgrade_WriteFrame(const uint8_t *request)
{ // 解密并写入一帧8字节升级数据
  uint8_t data[UPGRADE_FRAME_SIZE];
  uint8_t temporary;
  uint32_t address = 0x08000000U | (uint32_t)request[5] |
    ((uint32_t)request[6] << 8) | ((uint32_t)request[7] << 16);
  uint32_t offset;
  uint32_t frameIndex;
  uint32_t firstWord;
  uint32_t secondWord;
  HAL_StatusTypeDef status;

  if ((address < imageAddress) || ((address & (UPGRADE_FRAME_SIZE - 1U)) != 0) ||
    ((address - imageAddress) >= imageLength) ||
    (address > (UPGRADE_FLASH_END_ADDRESS - UPGRADE_FRAME_SIZE)))
  {
    return HAL_ERROR;
  }
  firstWord = WirelessUpgrade_ReadU32(request + 8) - 0x13A07524U;
  secondWord = WirelessUpgrade_ReadU32(request + 12) - 0x25437BA2U;
  WirelessUpgrade_WriteU32(data, firstWord);
  WirelessUpgrade_WriteU32(data + 4, secondWord);
  temporary = data[0];
  data[0] = data[6];
  data[6] = temporary;
  temporary = data[2];
  data[2] = data[5];
  data[5] = temporary;

  status = HAL_FLASH_Unlock();
  if (status == HAL_OK)
  {
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address,
      WirelessUpgrade_ReadU32(data));
  }
  if (status == HAL_OK)
  {
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + 4,
      WirelessUpgrade_ReadU32(data + 4));
  }
  HAL_FLASH_Lock();
  if (status == HAL_OK)
  {
    offset = address - imageAddress;
    frameIndex = offset / UPGRADE_FRAME_SIZE;
    writtenFrames[frameIndex / 8] &= (uint8_t)~(1U << (frameIndex % 8));
  }
  return status;
}

static uint32_t WirelessUpgrade_FindMissingAddress(void)
{ // 返回首个缺失8字节帧的地址，全部收齐返回0
  uint32_t frameCount = (imageLength + UPGRADE_FRAME_SIZE - 1U) /
    UPGRADE_FRAME_SIZE;
  uint32_t frameIndex;

  for (frameIndex = 0; frameIndex < frameCount; frameIndex++)
  {
    if ((writtenFrames[frameIndex / 8] & (1U << (frameIndex % 8))) != 0)
    {
      return imageAddress + frameIndex * UPGRADE_FRAME_SIZE;
    }
  }
  return 0;
}

static uint32_t WirelessUpgrade_CalculateChecksum(void)
{ // 计算升级镜像原始字节累加和
  const uint8_t *data = (const uint8_t *)imageAddress;
  uint32_t checksum = 0;
  uint32_t index;

  for (index = 0; index < imageLength; index++)
  {
    checksum += data[index];
  }
  return checksum;
}

static HAL_StatusTypeDef WirelessUpgrade_WriteBootInfo(void)
{ // 写入旧Bootloader识别的镜像地址、长度和校验和
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t pageError = 0;
  HAL_StatusTypeDef status;

  status = HAL_FLASH_Unlock();
  if (status != HAL_OK)
  {
    return status;
  }
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = UPGRADE_INFO_ADDRESS;
  erase.NbPages = 1;
  status = HAL_FLASHEx_Erase(&erase, &pageError);
  if (status == HAL_OK)
  {
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
      UPGRADE_INFO_ADDRESS, imageAddress);
  }
  if (status == HAL_OK)
  {
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
      UPGRADE_INFO_ADDRESS + 4, imageLength);
  }
  if (status == HAL_OK)
  {
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
      UPGRADE_INFO_ADDRESS + 8, imageChecksum);
  }
  HAL_FLASH_Lock();
  return status;
}