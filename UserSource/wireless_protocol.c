#include "wireless_protocol.h"
#include "wireless_upgrade.h"
#include "charge_control.h"
#include "Drivers/adc.h"
#include "Drivers/si4463.h"
#include "board_config.h"
#include <stm32f1xx_hal.h>
#include <string.h>

/* Home信道固定长协议命令 */
#define WIRELESS_COMMAND_INFO           0x07    // 信息查询主命令
#define WIRELESS_COMMAND_UPGRADE        0x03    // 通用无线升级主命令
#define WIRELESS_SUBCOMMAND_INFO        0x00    // 充电桩设备信息查询
#define WIRELESS_SUBCOMMAND_STATUS      0x00    // 查询充电状态
#define WIRELESS_SUBCOMMAND_SWITCH      0x06    // 切换工作信道

/* 工作信道WifiV2状态协议 */
#define WIRELESS_WIFI_COMMAND_STATUS    0x7000  // 完整状态查询命令
#define WIRELESS_WORK_TIMEOUT_MS        10000   // 无有效通信后返回Home的时间
#define WIRELESS_STATUS_DATA_LENGTH     33      // 状态内容字节数
#define WIRELESS_RESPONSE_LENGTH        45      // 完整应答帧字节数

static uint8_t currentChannel;
static uint8_t variableMode;
static uint32_t backHomeDelay;
static uint32_t lastValidCommunication;
static uint32_t wirelessAddress;
static uint8_t homeChannel;

static uint8_t WirelessProtocol_AddressMatches(const uint8_t *frame);
static uint8_t WirelessProtocol_ProcessFixed(const uint8_t *frame, uint8_t length);
static void WirelessProtocol_ProcessVariable(const uint8_t *frame, uint8_t length);
static void WirelessProtocol_SendHomeStatus(const uint8_t *request);
static void WirelessProtocol_SendDeviceInfo(const uint8_t *request);
static uint8_t WirelessProtocol_SendVariableStatus(const uint8_t *request, uint8_t length);
static uint8_t WirelessProtocol_GetFlags(void);
static uint8_t WirelessProtocol_GetPhaseFlags(uint8_t phase);
static uint16_t WirelessProtocol_GetOutputVoltage(void);
static int16_t WirelessProtocol_GetTotalCurrent(void);
static void WirelessProtocol_PutU16(uint8_t *buffer, uint16_t value);
static void WirelessProtocol_PutS16(uint8_t *buffer, int16_t value);
static uint16_t WirelessProtocol_Checksum(const uint8_t *data, uint16_t length);
static uint16_t WirelessProtocol_ClampU16(int32_t value);
static int16_t WirelessProtocol_Current10mA(int32_t value);

void WirelessProtocol_Init(void)
{ // 初始化无线协议状态
  wirelessAddress = *(const uint32_t *)BOARD_WIRELESS_ADDRESS_STORAGE;
  if (wirelessAddress == 0xFFFFFFFF)
  {
    wirelessAddress = BOARD_WIRELESS_ADDRESS_DEFAULT;
  }
  homeChannel = SI4463_GetHomeChannel();
  currentChannel = homeChannel;
  variableMode = 0;
  backHomeDelay = 0;
  lastValidCommunication = HAL_GetTick();
  WirelessUpgrade_Init();
}

void WirelessProtocol_Process(void)
{ // 处理接收帧，并在工作信道静默超时后返回Home
  uint8_t frame[SI4463_VARIABLE_MAX_PAYLOAD];
  uint8_t length;
  uint8_t isVariable;
  uint8_t receiveStarted = 0;

  // Home信道使用固定长协议，工作信道按切换命令选择固定长或变长协议
  if (SI4463_GetReceivedFrame(frame, &length, &isVariable))
  {
    if (isVariable)
    {
      WirelessProtocol_ProcessVariable(frame, length);
    }
    else
    {
      receiveStarted = WirelessProtocol_ProcessFixed(frame, length);
    }
    if (!receiveStarted)
    {
      SI4463_EnterReceive();
    }
  }
  // 只有有效通信才更新时间，超时后回到Home信道，避免停留在临时工作信道
  if ((currentChannel != homeChannel) && (backHomeDelay != 0) &&
    ((uint32_t)(HAL_GetTick() - lastValidCommunication) >= backHomeDelay))
  {
    if (SI4463_SetFixedMode(homeChannel))
    {
      currentChannel = homeChannel;
      variableMode = 0;
      backHomeDelay = 0;
    }
  }
}

uint8_t WirelessProtocol_IsWorkChannel(void)
{ // 返回当前是否已切换到工作信道
  return currentChannel != homeChannel;
}

uint32_t WirelessProtocol_GetAddress(void)
{ // 返回从Flash加载的无线地址
  return wirelessAddress;
}

HAL_StatusTypeDef WirelessProtocol_SaveAddress(uint32_t address)
{ // 按参考工程地址擦页并保存本机无线地址
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t pageError = 0;
  uint32_t homeChannelConfig = *(const uint32_t *)BOARD_RF_HOME_CHANNEL_STORAGE;
  uint32_t frequencyConfig = *(const uint32_t *)BOARD_RF_FREQUENCY_STORAGE;
  uint32_t frequencyStepConfig = *(const uint32_t *)BOARD_RF_FREQUENCY_STEP_STORAGE;
  uint32_t txPowerConfig = *(const uint32_t *)BOARD_RF_TX_POWER_STORAGE;
  HAL_StatusTypeDef status;

  if (address == 0)
  {
    return HAL_ERROR;
  }
  status = HAL_FLASH_Unlock();
  if (status != HAL_OK)
  {
    return status;
  }
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = BOARD_WIRELESS_ADDRESS_STORAGE;
  erase.NbPages = 1;
  status = HAL_FLASHEx_Erase(&erase, &pageError);
  if (status == HAL_OK)
  {
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
      BOARD_WIRELESS_ADDRESS_STORAGE, address);
  }
  if (status == HAL_OK)
  {
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
      BOARD_RF_HOME_CHANNEL_STORAGE, homeChannelConfig);
  }
  if (status == HAL_OK)
  {
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
      BOARD_RF_FREQUENCY_STORAGE, frequencyConfig);
  }
  if (status == HAL_OK)
  {
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
      BOARD_RF_FREQUENCY_STEP_STORAGE, frequencyStepConfig);
  }
  if (status == HAL_OK)
  {
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
      BOARD_RF_TX_POWER_STORAGE, txPowerConfig);
  }
  HAL_FLASH_Lock();
  if (status == HAL_OK)
  {
    wirelessAddress = address;
  }
  return status;
}

static uint8_t WirelessProtocol_AddressMatches(const uint8_t *frame)
{ // 固定长和变长协议均使用地址的低、高字节
  return (frame[1] == (uint8_t)wirelessAddress) &&
    (frame[2] == (uint8_t)(wirelessAddress >> 8));
}

static uint8_t WirelessProtocol_ProcessFixed(const uint8_t *frame, uint8_t length)
{ // 处理Home状态查询和切信道命令
  uint8_t command;
  uint8_t broadcast;
  uint8_t requestedChannel;
  uint8_t requestedVariableMode;
  uint8_t modeChanged;

  if (length != SI4463_PACKET_LENGTH)
  {
    return 0;
  }
  broadcast = (frame[1] == 0xFF) && (frame[2] == 0xFF);
  if (!WirelessProtocol_AddressMatches(frame) && !broadcast)
  {
    return 0;
  }
  command = frame[0] >> 4;
  if (broadcast)
  { // 广播仅允许OTA数据帧，禁止广播触发其他设备操作
    if ((command == WIRELESS_COMMAND_UPGRADE) &&
      WirelessUpgrade_Process(frame, 1, currentChannel))
    {
      lastValidCommunication = HAL_GetTick();
    }
    return 0;
  }
  lastValidCommunication = HAL_GetTick();
  if ((command == WIRELESS_COMMAND_INFO) &&
    (frame[4] == WIRELESS_SUBCOMMAND_STATUS))
  {
    WirelessProtocol_SendHomeStatus(frame);
    return 0;
  }
  else if (command == WIRELESS_COMMAND_UPGRADE)
  {
    if (frame[4] == WIRELESS_SUBCOMMAND_INFO)
    {
      WirelessProtocol_SendDeviceInfo(frame);
      return 0;
    }
    if (frame[4] != WIRELESS_SUBCOMMAND_SWITCH)
    {
      WirelessUpgrade_Process(frame, 0, currentChannel);
      return 0;
    }
    // 先在原信道应答切换命令，发送成功后才改变射频信道和协议模式
    requestedChannel = frame[5];
    requestedVariableMode = frame[8] == 1;
    if (!SI4463_SendPacket(frame, length, currentChannel))
    {
      return 0;
    }
    modeChanged = requestedVariableMode ?
      SI4463_SetVariableMode(requestedChannel) :
      SI4463_SetFixedMode(requestedChannel);
    if (!modeChanged)
    {
      return SI4463_SetFixedMode(currentChannel);
    }
    currentChannel = requestedChannel;
    variableMode = requestedVariableMode;
    backHomeDelay = (currentChannel == homeChannel) ?
      0 : WIRELESS_WORK_TIMEOUT_MS;
    lastValidCommunication = HAL_GetTick();
    return 1;
  }
  return 0;
}

static void WirelessProtocol_ProcessVariable(const uint8_t *frame, uint8_t length)
{ // 只把校验通过的0x7000本机查询作为有效通信
  if (WirelessProtocol_SendVariableStatus(frame, length))
  {
    lastValidCommunication = HAL_GetTick();
  }
}

static void WirelessProtocol_SendHomeStatus(const uint8_t *request)
{ // 返回固定16字节Home精简状态
  uint8_t response[SI4463_PACKET_LENGTH];

  memcpy(response, request, 4);
  memset(response + 4, 0, SI4463_PACKET_LENGTH - 4);
  response[4] = 1;
  response[5] = WirelessProtocol_GetFlags();
  WirelessProtocol_PutU16(response + 6, WirelessProtocol_GetOutputVoltage());
  WirelessProtocol_PutS16(response + 8, WirelessProtocol_GetTotalCurrent());
  SI4463_SendPacket(response, sizeof(response), currentChannel);
}

static void WirelessProtocol_SendDeviceInfo(const uint8_t *request)
{ // 返回旧升级协议需要的设备、信道和版本信息
  extern const uint16_t g_ver;
  uint8_t response[SI4463_PACKET_LENGTH];

  memcpy(response, request, 4);
  memset(response + 4, 0, SI4463_PACKET_LENGTH - 4);
  response[4] = WIRELESS_SUBCOMMAND_INFO;
  response[5] = (uint8_t)wirelessAddress;
  response[6] = (uint8_t)(wirelessAddress >> 8);
  response[7] = (uint8_t)(wirelessAddress >> 16);
  response[8] = (uint8_t)(wirelessAddress >> 24);
  response[9] = homeChannel;
  response[10] = currentChannel;
  WirelessProtocol_PutU16(response + 11, BOARD_DEVICE_TYPE);
  response[14] = (uint8_t)g_ver;
  response[15] = (uint8_t)(g_ver >> 8);
  SI4463_SendPacket(response, sizeof(response), currentChannel);
}

static uint8_t WirelessProtocol_SendVariableStatus(const uint8_t *request, uint8_t length)
{ // 校验0x7000请求并返回三相完整状态
  uint8_t response[WIRELESS_RESPONSE_LENGTH];
  uint8_t *statusData;
  uint8_t phase;
  uint16_t contentLength;
  uint16_t checksum;
  uint16_t command;

  // 变长业务帧依次校验帧头、内容长度、校验和、设备地址和命令字
  if ((length < 10) || (request[0] != 0x55))
  {
    return 0;
  }
  contentLength = ((uint16_t)(request[1] & 0x03) << 8) | request[2];
  if (contentLength != (uint16_t)(length - 5))
  {
    return 0;
  }
  checksum = (uint16_t)request[3] | ((uint16_t)request[4] << 8);
  if (checksum != WirelessProtocol_Checksum(request + 5, contentLength))
  {
    return 0;
  }
  if ((request[5] != (uint8_t)wirelessAddress) ||
    (request[6] != (uint8_t)(wirelessAddress >> 8)))
  {
    return 0;
  }
  if ((request[1] & 0x0C) != 0)
  {
    return 0;
  }
  command = (uint16_t)request[7] | ((uint16_t)request[8] << 8);
  if (command != WIRELESS_WIFI_COMMAND_STATUS)
  {
    return 0;
  }

  memset(response, 0, sizeof(response));
  response[0] = 0xAA;
  response[1] = request[1] & 0xF0;
  response[2] = 40;
  memcpy(response + 5, request + 5, 5);
  response[10] = 0x0A;
  response[11] = WIRELESS_STATUS_DATA_LENGTH;
  statusData = response + 12;
  statusData[0] = 1;
  statusData[1] = WirelessProtocol_GetFlags();
  WirelessProtocol_PutU16(statusData + 2, WirelessProtocol_GetOutputVoltage());
  WirelessProtocol_PutS16(statusData + 4, WirelessProtocol_GetTotalCurrent());
  for (phase = 0; phase < CHARGE_PHASE_COUNT; phase++)
  {
    uint8_t *phaseData = statusData + 6 + phase * 9;
    WirelessProtocol_PutU16(phaseData, WirelessProtocol_ClampU16(g_out_Volt[phase]));
    WirelessProtocol_PutS16(phaseData + 2, WirelessProtocol_Current10mA(g_I[phase]));
    WirelessProtocol_PutU16(phaseData + 4, g_intvolt[phase]);
    WirelessProtocol_PutU16(phaseData + 6,
      WirelessProtocol_ClampU16(ChargeControl_GetTargetCurrent(phase) / 10));
    phaseData[8] = WirelessProtocol_GetPhaseFlags(phase);
  }
  checksum = WirelessProtocol_Checksum(response + 5, 40);
  response[3] = (uint8_t)checksum;
  response[4] = (uint8_t)(checksum >> 8);
  if (SI4463_SendVariablePacket(response, sizeof(response), currentChannel))
  {
    return 1;
  }
  return 0;
}

static uint8_t WirelessProtocol_GetFlags(void)
{ // 生成充电桩全局状态标志
  uint8_t flags = 0x40;

  if ((g_intvolt[0] < 5200) && (g_intvolt[1] < 5200) && (g_intvolt[2] < 5200))
  {
    flags |= 0x01;
  }
  if ((g_I[0] > 50000) || (g_I[0] < -50000) ||
    (g_I[1] > 50000) || (g_I[1] < -50000) ||
    (g_I[2] > 50000) || (g_I[2] < -50000))
  {
    flags |= 0x02;
  }
  if (ChargeControl_HasBreakFault())
  {
    flags |= 0x04;
  }
  if (ChargeControl_IsMosEnabled())
  {
    flags |= 0x08;
  }
  if (ChargeControl_IsMainEnabled())
  {
    flags |= 0x10;
  }
  return flags;
}

static uint8_t WirelessProtocol_GetPhaseFlags(uint8_t phase)
{ // 按参考协议生成单相实时状态标志
  uint8_t flags = 0;
  int32_t outputVoltage = g_out_Volt[phase];

  if (ChargeControl_IsPhaseEnabled(phase))
  {
    flags |= 0x01;
  }
  if ((outputVoltage > 200) && (outputVoltage < 5200))
  {
    flags |= 0x02;
  }
  if (g_intvolt[phase] > 5200)
  {
    flags |= 0x04;
  }
  if ((g_I[phase] > 50000) || (g_I[phase] < -50000))
  {
    flags |= 0x08;
  }
  if (ChargeControl_IsMainEnabled())
  {
    flags |= 0x10;
  }
  return flags;
}

static uint16_t WirelessProtocol_GetOutputVoltage(void)
{ // 以上报的三相实际输出电压平均值作为整体输出电压
  int32_t total = g_out_Volt[0] + g_out_Volt[1] + g_out_Volt[2];
  return WirelessProtocol_ClampU16(total / CHARGE_PHASE_COUNT);
}

static int16_t WirelessProtocol_GetTotalCurrent(void)
{ // 返回三相总电流，单位10mA
  return WirelessProtocol_Current10mA(g_I[0] + g_I[1] + g_I[2]);
}

static void WirelessProtocol_PutU16(uint8_t *buffer, uint16_t value)
{ // 按协议小端序写入无符号值
  buffer[0] = (uint8_t)value;
  buffer[1] = (uint8_t)(value >> 8);
}

static void WirelessProtocol_PutS16(uint8_t *buffer, int16_t value)
{ // 按协议小端序写入有符号值
  WirelessProtocol_PutU16(buffer, (uint16_t)value);
}

static uint16_t WirelessProtocol_Checksum(const uint8_t *data, uint16_t length)
{ // 计算WifiV2内容区累加和
  uint16_t checksum = 0;

  while (length > 0)
  {
    checksum += *data;
    data++;
    length--;
  }
  return checksum;
}

static uint16_t WirelessProtocol_ClampU16(int32_t value)
{ // 将采样值限制到协议无符号16位范围
  if (value < 0)
  {
    return 0;
  }
  if (value > 65535)
  {
    return 65535;
  }
  return (uint16_t)value;
}

static int16_t WirelessProtocol_Current10mA(int32_t value)
{ // 将mA转换为10mA并限制到有符号16位范围
  value /= 10;
  if (value > 32767)
  {
    return 32767;
  }
  if (value < -32768)
  {
    return -32768;
  }
  return (int16_t)value;
}