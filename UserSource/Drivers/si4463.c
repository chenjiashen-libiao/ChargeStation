#include "si4463.h"
#include "board_config.h"
#include "radio_config_Si4463.h"
#include "si446x_defs.h"
#include <../CMSIS_RTOS/cmsis_os.h>
#include <string.h>

#define SI4463_CTS_ATTEMPTS               20U
#define SI4463_TX_COMPLETE_ATTEMPTS       20U
#define SI4463_PACKET_RX_PENDING          0x10U
#define SI4463_PACKET_CRC_ERROR           0x08U
#define SI4463_PACKET_TX_COMPLETE         0x20U
#define SI4463_VARIABLE_CRC_SIZE          3U
#define SI4463_VARIABLE_MAX_FIELD2        63U
#define SI4463_VARIABLE_MIN_PAYLOAD       3U
#define SI4463_VARIABLE_MIN_FIELD2        6U

static const uint8_t configTable[] = RADIO_CONFIGURATION_DATA_ARRAY;
static const uint8_t config902Table[] = RADIO902_CONFIGURATION_DATA_ARRAY;

static SI4463_Info si4463Info;
static volatile uint8_t interruptPending;
static volatile uint8_t receivedFramePending;
static uint8_t receivedFrame[SI4463_VARIABLE_MAX_PAYLOAD];
static uint8_t receivedFrameLength;
static uint8_t receivedFrameVariable;
static uint8_t currentChannel;
static uint8_t variableMode;
static uint8_t homeChannel;
static uint8_t txPower;
static uint16_t frequencyBase;
static uint16_t frequencyStep;

static void SI4463_InitIo(void);
static void SI4463_SetMosiMode(void);
static void SI4463_SetIrqMode(void);
static void SI4463_Select(void);
static void SI4463_Deselect(void);
static uint8_t SI4463_ExchangeByte(uint8_t txData);
static uint8_t SI4463_WaitCts(void);
static uint8_t SI4463_Command(const uint8_t *command, uint8_t commandSize);
static uint8_t SI4463_ReadResponse(uint8_t *buffer, uint8_t size);
static uint8_t SI4463_ReadPartInfo(void);
static uint8_t SI4463_LoadConfig(const uint8_t *table);
static uint8_t SI4463_SetProperty(uint16_t property, uint8_t value);
static uint8_t SI4463_ConfigGpio(void);
static uint8_t SI4463_SetFrequency(void);
static uint8_t SI4463_SetTxPower(void);
static uint8_t SI4463_ChangeState(uint8_t state);
static uint8_t SI4463_ReadInterruptStatus(uint8_t *status);
static uint8_t SI4463_ResetRxFifo(void);
static uint8_t SI4463_ResetTxFifo(void);
static uint8_t SI4463_StartReceive(void);
static uint8_t SI4463_Transmit(const uint8_t *data, uint8_t length,
  uint8_t channel, uint8_t condition);
static uint8_t SI4463_ReadFixedPacket(uint8_t *data);
static uint8_t SI4463_ReadVariablePacket(uint8_t *data, uint8_t *length);
static uint32_t SI4463_ComputeCrc(const uint8_t *data, uint16_t length);
static uint8_t SI4463_TakeInterrupt(void);

uint8_t SI4463_Init(void)
{ // 按参考工程顺序初始化SI4463并进入Home信道固定长接收
  uint8_t status[9];

  memset(&si4463Info, 0, sizeof(si4463Info));
  homeChannel = SI4463_HOME_CHANNEL;
  txPower = SI4463_TX_POWER;
  frequencyBase = SI4463_FREQUENCY_BASE_MHZ;
  frequencyStep = SI4463_FREQUENCY_STEP_KHZ;
#if !BOARD_RF_TEST_FIXED_CONFIG
  if (*(const uint32_t *)BOARD_RF_HOME_CHANNEL_STORAGE != 0xFFFFFFFFU)
  {
    homeChannel = *(const uint8_t *)BOARD_RF_HOME_CHANNEL_STORAGE;
  }
  if (*(const uint32_t *)BOARD_RF_FREQUENCY_STORAGE != 0xFFFFFFFFU)
  {
    frequencyBase = *(const uint16_t *)BOARD_RF_FREQUENCY_STORAGE;
  }
  if (*(const uint32_t *)BOARD_RF_FREQUENCY_STEP_STORAGE != 0xFFFFFFFFU)
  {
    frequencyStep = *(const uint16_t *)BOARD_RF_FREQUENCY_STEP_STORAGE;
  }
  if (*(const uint32_t *)BOARD_RF_TX_POWER_STORAGE != 0xFFFFFFFFU)
  {
    txPower = *(const uint8_t *)BOARD_RF_TX_POWER_STORAGE;
  }
#endif
  SI4463_InitIo();
  osDelay(5);
  if (!SI4463_ReadPartInfo() ||
    ((si4463Info.PartNumber & 0xFFF0U) != 0x4460U) ||
    !SI4463_LoadConfig(configTable))
  {
    return 0U;
  }
  if ((frequencyBase > 849U) ||
    ((frequencyBase == 0U) && (si4463Info.RomId == 0x06U)))
  {
    if (!SI4463_LoadConfig(config902Table))
    {
      return 0U;
    }
  }
  if ((frequencyBase != 0U) && !SI4463_SetFrequency())
  {
    return 0U;
  }
  if (!SI4463_ConfigGpio() || !SI4463_SetTxPower() ||
    !SI4463_ReadInterruptStatus(status) ||
    !SI4463_SetFixedMode(homeChannel))
  {
    return 0U;
  }
  si4463Info.Ready = 1U;
  return 1U;
}

uint8_t SI4463_IsReady(void)
{ // 返回无线芯片初始化状态
  return si4463Info.Ready;
}

uint8_t SI4463_GetHomeChannel(void)
{ // 返回Flash配置的Home信道
  return homeChannel;
}

uint8_t SI4463_ReadCurrentChannel(uint8_t *channel)
{ // 读取SI4463硬件当前信道
  uint8_t command = REQUEST_DEVICE_STATE;
  uint8_t response[3];

  if ((channel == NULL) ||
    !SI4463_Command(&command, 1U) ||
    !SI4463_ReadResponse(response, sizeof(response)))
  {
    return 0U;
  }
  *channel = response[2];
  return 1U;
}

const SI4463_Info *SI4463_GetInfo(void)
{ // 返回芯片识别信息
  return &si4463Info;
}

uint8_t SI4463_SendPacket(const uint8_t *data, uint8_t length, uint8_t channel)
{ // 发送固定长度无线数据并恢复接收
  if (!si4463Info.Ready || (data == NULL) || (length != SI4463_PACKET_LENGTH))
  {
    return 0U;
  }
  return SI4463_Transmit(data, length, channel, 0U);
}

uint8_t SI4463_SendVariablePacket(const uint8_t *data, uint8_t length, uint8_t channel)
{ // 发送长度字节、CRC32低24位和有效数据
  uint8_t fifo[SI4463_VARIABLE_MAX_FIELD2 + 1U];
  uint8_t field2Length;
  uint8_t index;
  uint32_t crc;
  uint8_t result;

  if (!si4463Info.Ready || !variableMode || (data == NULL) ||
    (length < SI4463_VARIABLE_MIN_PAYLOAD) ||
    (length > SI4463_VARIABLE_MAX_PAYLOAD))
  {
    return 0U;
  }
  crc = SI4463_ComputeCrc(data, length);
  field2Length = length + SI4463_VARIABLE_CRC_SIZE;
  fifo[0] = field2Length;
  fifo[1] = (uint8_t)crc;
  fifo[2] = (uint8_t)(crc >> 8);
  fifo[3] = (uint8_t)(crc >> 16);
  for (index = 0U; index < length; index++)
  {
    fifo[index + 4U] = data[index];
  }
  if (!SI4463_SetProperty(PKT_FIELD_2_LENGTH_7_0, field2Length))
  {
    return 0U;
  }
  result = SI4463_Transmit(fifo, field2Length + 1U, channel, 0x60U);
  SI4463_SetProperty(PKT_FIELD_2_LENGTH_7_0, SI4463_VARIABLE_MAX_FIELD2);
  return result;
}

uint8_t SI4463_GetReceivedFrame(uint8_t *data, uint8_t *length, uint8_t *isVariable)
{ // 读取无线任务收到的最新帧及协议模式
  uint32_t interruptState;

  if ((data == NULL) || (length == NULL) || (isVariable == NULL))
  {
    return 0U;
  }
  interruptState = __get_PRIMASK();
  __disable_irq();
  if (!receivedFramePending)
  {
    if (interruptState == 0U)
    {
      __enable_irq();
    }
    return 0U;
  }
  memcpy(data, receivedFrame, receivedFrameLength);
  *length = receivedFrameLength;
  *isVariable = receivedFrameVariable;
  receivedFramePending = 0U;
  if (interruptState == 0U)
  {
    __enable_irq();
  }
  return 1U;
}

uint8_t SI4463_SetFixedMode(uint8_t channel)
{ // 配置固定16字节模式并进入指定信道接收
  uint8_t status[9];
  uint8_t hardwareChannel;

  if (!SI4463_ChangeState(3U) || !SI4463_ReadInterruptStatus(status) ||
    !SI4463_ResetRxFifo() ||
    !SI4463_SetProperty(PKT_CRC_CONFIG, 0x84U) ||
    !SI4463_SetProperty(PKT_LEN, 0x00U) ||
    !SI4463_SetProperty(PKT_LEN_FIELD_SOURCE, 0x00U) ||
    !SI4463_SetProperty(PKT_LEN_ADJUST, 0x00U) ||
    !SI4463_SetProperty(PKT_FIELD_1_LENGTH_12_8, 0x00U) ||
    !SI4463_SetProperty(PKT_FIELD_1_LENGTH_7_0, SI4463_PACKET_LENGTH) ||
    !SI4463_SetProperty(PKT_FIELD_1_CRC_CONFIG, 0xAAU) ||
    !SI4463_SetProperty(PKT_FIELD_2_LENGTH_12_8, 0x00U) ||
    !SI4463_SetProperty(PKT_FIELD_2_LENGTH_7_0, 0x00U) ||
    !SI4463_SetProperty(PKT_FIELD_2_CRC_CONFIG, 0x00U))
  {
    return 0U;
  }
  currentChannel = channel;
  variableMode = 0U;
  return SI4463_StartReceive() &&
    SI4463_ReadCurrentChannel(&hardwareChannel) &&
    (hardwareChannel == channel);
}

uint8_t SI4463_SetVariableMode(uint8_t channel)
{ // 配置V5双FIELD变长模式并进入指定信道接收
  uint8_t status[9];
  uint8_t hardwareChannel;

  if (!SI4463_ChangeState(3U) || !SI4463_ReadInterruptStatus(status) ||
    !SI4463_ResetRxFifo() ||
    !SI4463_SetProperty(PKT_CRC_CONFIG, 0x84U) ||
    !SI4463_SetProperty(PKT_LEN, 0x2AU) ||
    !SI4463_SetProperty(PKT_LEN_FIELD_SOURCE, 0x01U) ||
    !SI4463_SetProperty(PKT_LEN_ADJUST, 0x00U) ||
    !SI4463_SetProperty(PKT_FIELD_1_LENGTH_12_8, 0x00U) ||
    !SI4463_SetProperty(PKT_FIELD_1_LENGTH_7_0, 0x01U) ||
    !SI4463_SetProperty(PKT_FIELD_1_CRC_CONFIG, 0x00U) ||
    !SI4463_SetProperty(PKT_FIELD_2_LENGTH_12_8, 0x00U) ||
    !SI4463_SetProperty(PKT_FIELD_2_LENGTH_7_0, SI4463_VARIABLE_MAX_FIELD2) ||
    !SI4463_SetProperty(PKT_FIELD_2_CRC_CONFIG, 0x00U))
  {
    return 0U;
  }
  currentChannel = channel;
  variableMode = 1U;
  return SI4463_StartReceive() &&
    SI4463_ReadCurrentChannel(&hardwareChannel) &&
    (hardwareChannel == channel);
}

uint8_t SI4463_EnterReceive(void)
{ // 按当前协议模式恢复接收
  if (!si4463Info.Ready)
  {
    return 0U;
  }
  return SI4463_StartReceive();
}

void SI4463_Process(void)
{ // 处理无线中断并保持Home信道接收
  uint8_t status[9];
  uint8_t pending;
  uint8_t frameReceived = 0U;

  if (!si4463Info.Ready)
  {
    return;
  }
  pending = SI4463_TakeInterrupt();
  if (!pending)
  {
    if (HAL_GPIO_ReadPin(BOARD_SI4463_GPIO_PORT,
      BOARD_SI4463_MOSI_IRQ_PIN) != GPIO_PIN_RESET)
    {
      return;
    }
  }
  if (!SI4463_ReadInterruptStatus(status))
  {
    SI4463_StartReceive();
    return;
  }
  if (((status[3] & SI4463_PACKET_RX_PENDING) != 0U) &&
    ((status[3] & SI4463_PACKET_CRC_ERROR) == 0U))
  {
    if (variableMode)
    {
      if (SI4463_ReadVariablePacket(receivedFrame, &receivedFrameLength))
      {
        receivedFrameVariable = 1U;
        receivedFramePending = 1U;
        frameReceived = 1U;
      }
    }
    else if (SI4463_ReadFixedPacket(receivedFrame))
    {
      SI4463_ChangeState(5U);
      receivedFrameLength = SI4463_PACKET_LENGTH;
      receivedFrameVariable = 0U;
      receivedFramePending = 1U;
      frameReceived = 1U;
    }
  }
  if (!frameReceived)
  {
    SI4463_StartReceive();
  }
}

static void SI4463_InitIo(void)
{ // 初始化与参考工程一致的模拟SPI和NIRQ引脚
  GPIO_InitTypeDef gpioInit = {0};

  BOARD_SI4463_GPIO_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
  HAL_GPIO_WritePin(BOARD_SI4463_GPIO_PORT, BOARD_SI4463_CSN_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BOARD_SI4463_GPIO_PORT, BOARD_SI4463_SCK_PIN, GPIO_PIN_RESET);
  gpioInit.Pin = BOARD_SI4463_CSN_PIN | BOARD_SI4463_SCK_PIN;
  gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
  gpioInit.Pull = GPIO_NOPULL;
  gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BOARD_SI4463_GPIO_PORT, &gpioInit);
  gpioInit.Pin = BOARD_SI4463_MISO_PIN;
  gpioInit.Mode = GPIO_MODE_INPUT;
  gpioInit.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BOARD_SI4463_GPIO_PORT, &gpioInit);
  SI4463_SetIrqMode();
}

static void SI4463_SetMosiMode(void)
{ // SPI事务期间将PB5切换为MOSI输出
  GPIO_InitTypeDef gpioInit = {0};

  HAL_NVIC_DisableIRQ(BOARD_SI4463_IRQN);
  EXTI->IMR &= ~BOARD_SI4463_MOSI_IRQ_PIN;
  __HAL_GPIO_EXTI_CLEAR_IT(BOARD_SI4463_MOSI_IRQ_PIN);
  HAL_GPIO_WritePin(BOARD_SI4463_GPIO_PORT, BOARD_SI4463_MOSI_IRQ_PIN, GPIO_PIN_SET);
  gpioInit.Pin = BOARD_SI4463_MOSI_IRQ_PIN;
  gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
  gpioInit.Pull = GPIO_NOPULL;
  gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BOARD_SI4463_GPIO_PORT, &gpioInit);
}

static void SI4463_SetIrqMode(void)
{ // SPI空闲期间将PB5恢复为低有效NIRQ
  GPIO_InitTypeDef gpioInit = {0};
  uint8_t wasPending = (__HAL_GPIO_EXTI_GET_IT(BOARD_SI4463_MOSI_IRQ_PIN) != RESET);

  gpioInit.Pin = BOARD_SI4463_MOSI_IRQ_PIN;
  gpioInit.Mode = GPIO_MODE_IT_FALLING;
  gpioInit.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BOARD_SI4463_GPIO_PORT, &gpioInit);
  __HAL_GPIO_EXTI_CLEAR_IT(BOARD_SI4463_MOSI_IRQ_PIN);
  if (wasPending ||
    (HAL_GPIO_ReadPin(BOARD_SI4463_GPIO_PORT, BOARD_SI4463_MOSI_IRQ_PIN) == GPIO_PIN_RESET))
  {
    interruptPending = 1U;
  }
  EXTI->IMR |= BOARD_SI4463_MOSI_IRQ_PIN;
  HAL_NVIC_SetPriority(BOARD_SI4463_IRQN, 5, 0);
  HAL_NVIC_EnableIRQ(BOARD_SI4463_IRQN);
}

static void SI4463_Select(void)
{ // 开始模拟SPI事务
  SI4463_SetMosiMode();
  HAL_GPIO_WritePin(BOARD_SI4463_GPIO_PORT, BOARD_SI4463_CSN_PIN, GPIO_PIN_RESET);
}

static void SI4463_Deselect(void)
{ // 结束模拟SPI事务并恢复NIRQ
  HAL_GPIO_WritePin(BOARD_SI4463_GPIO_PORT, BOARD_SI4463_CSN_PIN, GPIO_PIN_SET);
  SI4463_SetIrqMode();
}

static uint8_t SI4463_ExchangeByte(uint8_t txData)
{ // 模拟SPI收发一个字节，CPOL=0、CPHA=1、MSB先行
  uint8_t bit;
  uint8_t rxData = 0U;

  HAL_GPIO_WritePin(BOARD_SI4463_GPIO_PORT, BOARD_SI4463_SCK_PIN, GPIO_PIN_RESET);
  for (bit = 0U; bit < 8U; bit++)
  {
    HAL_GPIO_WritePin(BOARD_SI4463_GPIO_PORT, BOARD_SI4463_MOSI_IRQ_PIN,
      ((txData & 0x80U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BOARD_SI4463_GPIO_PORT, BOARD_SI4463_SCK_PIN, GPIO_PIN_SET);
    rxData <<= 1;
    if (HAL_GPIO_ReadPin(BOARD_SI4463_GPIO_PORT, BOARD_SI4463_MISO_PIN) == GPIO_PIN_SET)
    {
      rxData |= 1U;
    }
    HAL_GPIO_WritePin(BOARD_SI4463_GPIO_PORT, BOARD_SI4463_SCK_PIN, GPIO_PIN_RESET);
    txData <<= 1;
  }
  return rxData;
}

static uint8_t SI4463_WaitCts(void)
{ // 等待芯片命令缓冲区就绪
  uint8_t attempt;
  uint8_t cts;

  for (attempt = 0U; attempt < SI4463_CTS_ATTEMPTS; attempt++)
  {
    SI4463_Select();
    SI4463_ExchangeByte(READ_CMD_BUFF);
    cts = SI4463_ExchangeByte(0xFFU);
    SI4463_Deselect();
    if (cts == 0xFFU)
    {
      return 1U;
    }
    osDelay(1);
  }
  return 0U;
}

static uint8_t SI4463_Command(const uint8_t *command, uint8_t commandSize)
{ // 向SI4463发送命令
  uint8_t index;

  if (!SI4463_WaitCts())
  {
    return 0U;
  }
  SI4463_Select();
  for (index = 0U; index < commandSize; index++)
  {
    SI4463_ExchangeByte(command[index]);
  }
  SI4463_Deselect();
  return 1U;
}

static uint8_t SI4463_ReadResponse(uint8_t *buffer, uint8_t size)
{ // 读取SI4463命令响应
  uint8_t index;

  if (!SI4463_WaitCts())
  {
    return 0U;
  }
  SI4463_Select();
  SI4463_ExchangeByte(READ_CMD_BUFF);
  for (index = 0U; index < size; index++)
  {
    buffer[index] = SI4463_ExchangeByte(0xFFU);
  }
  SI4463_Deselect();
  return 1U;
}

static uint8_t SI4463_ReadPartInfo(void)
{ // 读取并保存SI4463型号及ROM版本
  uint8_t command = PART_INFO;
  uint8_t response[9];

  if (!SI4463_Command(&command, 1U) || !SI4463_ReadResponse(response, sizeof(response)))
  {
    return 0U;
  }
  si4463Info.ChipRevision = response[1];
  si4463Info.PartNumber = ((uint16_t)response[2] << 8) | response[3];
  si4463Info.RomId = response[8];
  return 1U;
}

static uint8_t SI4463_LoadConfig(const uint8_t *table)
{ // 逐条下发WDS生成的配置表
  uint16_t offset = 0U;
  uint8_t length;

  while ((length = table[offset]) != 0U)
  {
    offset++;
    if (!SI4463_Command(table + offset, length))
    {
      return 0U;
    }
    offset += length;
  }
  return 1U;
}

static uint8_t SI4463_SetProperty(uint16_t property, uint8_t value)
{ // 写入一个SI4463属性
  uint8_t command[5];

  command[0] = SET_PROPERTY;
  command[1] = (uint8_t)(property >> 8);
  command[2] = 1U;
  command[3] = (uint8_t)property;
  command[4] = value;
  return SI4463_Command(command, sizeof(command));
}

static uint8_t SI4463_ConfigGpio(void)
{ // 配置模块GPIO2和GPIO3控制射频收发开关
  uint8_t command[8] = {GPIO_PIN_CFG, 0U, 0U, 0x20U, 0x21U, 0U, 0U, 0U};
  uint8_t response[8];

  return SI4463_Command(command, sizeof(command)) &&
    SI4463_ReadResponse(response, sizeof(response));
}

static uint8_t SI4463_SetFrequency(void)
{ // 按参考公式设置基频和信道间隔
  uint8_t command[10];
  uint8_t outDiv = (frequencyBase >= 850U) ? 1U : 2U;
  uint8_t integer = (uint8_t)(outDiv * frequencyBase / 15U - 1U);
  uint32_t fraction = (uint32_t)outDiv * frequencyBase * 524288U / 15U -
    (uint32_t)integer * 524288U;
  uint16_t step = (uint16_t)(524288U * outDiv * frequencyStep / 15000U);

  command[0] = SET_PROPERTY;
  command[1] = 0x40U;
  command[2] = 0x06U;
  command[3] = 0U;
  command[4] = integer;
  command[5] = (uint8_t)(fraction >> 16);
  command[6] = (uint8_t)(fraction >> 8);
  command[7] = (uint8_t)fraction;
  command[8] = (uint8_t)(step >> 8);
  command[9] = (uint8_t)step;
  return SI4463_Command(command, sizeof(command));
}

static uint8_t SI4463_SetTxPower(void)
{ // 设置与参考工程一致的发射功率
  uint8_t command[8] = {SET_PROPERTY, 0x22U, 0x04U, 0U,
    0x08U, txPower, 0U, 0x3DU};
  return SI4463_Command(command, sizeof(command));
}

static uint8_t SI4463_ChangeState(uint8_t state)
{ // 切换SI4463内部状态
  uint8_t command[2] = {CHANGE_STATE, state};
  return SI4463_Command(command, sizeof(command));
}

static uint8_t SI4463_ReadInterruptStatus(uint8_t *status)
{ // 读取并清除SI4463全部中断状态
  uint8_t command[4] = {GET_INT_STATUS, 0U, 0U, 0U};
  uint8_t index;
  uint8_t allOnes = 1U;

  if (!SI4463_Command(command, sizeof(command)) || !SI4463_ReadResponse(status, 9U))
  {
    return 0U;
  }
  for (index = 0U; index < 9U; index++)
  {
    if (status[index] != 0xFFU)
    {
      allOnes = 0U;
      break;
    }
  }
  return !allOnes && (status[3] != 0xFFU);
}

static uint8_t SI4463_ResetRxFifo(void)
{ // 清空接收FIFO
  uint8_t command[2] = {FIFO_INFO, 0x02U};
  return SI4463_Command(command, sizeof(command));
}

static uint8_t SI4463_ResetTxFifo(void)
{ // 清空发送FIFO
  uint8_t command[2] = {FIFO_INFO, 0x01U};
  return SI4463_Command(command, sizeof(command));
}

static uint8_t SI4463_StartReceive(void)
{ // 按当前信道和协议模式重新进入接收
  uint8_t command[8];

  if (!SI4463_ResetRxFifo())
  {
    return 0U;
  }
  command[0] = START_RX;
  command[1] = currentChannel;
  command[2] = 0U;
  command[3] = 0U;
  command[4] = variableMode ? 0U : SI4463_PACKET_LENGTH;
  command[5] = 0U;
  command[6] = variableMode ? 3U : 0U;
  command[7] = 3U;
  return SI4463_Command(command, sizeof(command));
}

static uint8_t SI4463_Transmit(const uint8_t *data, uint8_t length,
  uint8_t channel, uint8_t condition)
{ // 写入TX FIFO、启动发送并在完成后返回READY状态
  uint8_t command[5];
  uint8_t status[9];
  uint8_t attempts;
  uint8_t index;
  uint8_t completed = 0U;

  if (!SI4463_ResetTxFifo() || !SI4463_ReadInterruptStatus(status) || !SI4463_WaitCts())
  {
    return 0U;
  }
  SI4463_Select();
  SI4463_ExchangeByte(WRITE_TX_FIFO);
  for (index = 0U; index < length; index++)
  {
    SI4463_ExchangeByte(data[index]);
  }
  SI4463_Deselect();
  command[0] = START_TX;
  command[1] = channel;
  command[2] = condition;
  command[3] = 0U;
  command[4] = length;
  if (!SI4463_Command(command, sizeof(command)))
  {
    return 0U;
  }
  for (attempts = 0U; attempts < SI4463_TX_COMPLETE_ATTEMPTS; attempts++)
  {
    osDelay(1);
    if ((interruptPending != 0U) ||
      (HAL_GPIO_ReadPin(BOARD_SI4463_GPIO_PORT,
        BOARD_SI4463_MOSI_IRQ_PIN) == GPIO_PIN_RESET))
    {
      SI4463_TakeInterrupt();
      if (SI4463_ReadInterruptStatus(status) &&
        ((status[3] & SI4463_PACKET_TX_COMPLETE) != 0U))
      {
        completed = 1U;
        break;
      }
    }
  }
  SI4463_ChangeState(3U);
  SI4463_ReadInterruptStatus(status);
  return completed;
}

static uint8_t SI4463_ReadFixedPacket(uint8_t *data)
{ // 从RX FIFO读取一个固定16字节帧
  uint8_t index;

  if (!SI4463_WaitCts())
  {
    return 0U;
  }
  SI4463_Select();
  SI4463_ExchangeByte(READ_RX_FIFO);
  for (index = 0U; index < SI4463_PACKET_LENGTH; index++)
  {
    data[index] = SI4463_ExchangeByte(0xFFU);
  }
  SI4463_Deselect();
  return 1U;
}

static uint8_t SI4463_ReadVariablePacket(uint8_t *data, uint8_t *length)
{ // 读取并校验变长帧
  uint8_t fifo[SI4463_VARIABLE_MAX_FIELD2 + 1U];
  uint8_t field2Length;
  uint8_t index;
  uint32_t receivedCrc;
  uint32_t calculatedCrc;

  *length = 0U;
  if (!SI4463_WaitCts())
  {
    return 0U;
  }
  SI4463_Select();
  SI4463_ExchangeByte(READ_RX_FIFO);
  for (index = 0U; index <= SI4463_VARIABLE_MAX_FIELD2; index++)
  {
    fifo[index] = SI4463_ExchangeByte(0xFFU);
  }
  SI4463_Deselect();
  field2Length = fifo[0];
  if ((field2Length < SI4463_VARIABLE_MIN_FIELD2) ||
    (field2Length > SI4463_VARIABLE_MAX_FIELD2))
  {
    SI4463_ResetRxFifo();
    return 0U;
  }
  receivedCrc = (uint32_t)fifo[1] |
    ((uint32_t)fifo[2] << 8) | ((uint32_t)fifo[3] << 16);
  *length = field2Length - SI4463_VARIABLE_CRC_SIZE;
  calculatedCrc = SI4463_ComputeCrc(&fifo[4], *length);
  if ((calculatedCrc & 0x00FFFFFFU) != receivedCrc)
  {
    *length = 0U;
    SI4463_ResetRxFifo();
    return 0U;
  }
  memcpy(data, &fifo[4], *length);
  return 1U;
}

static uint32_t SI4463_ComputeCrc(const uint8_t *data, uint16_t length)
{ // 使用STM32硬件CRC计算与参考工程一致的CRC32
  uint16_t offset = 0U;
  uint32_t lastWord = 0U;
  uint8_t index;

  __HAL_RCC_CRC_CLK_ENABLE();
  CRC->CR = CRC_CR_RESET;
  while (length >= 4U)
  {
    CRC->DR = (uint32_t)data[offset] |
      ((uint32_t)data[offset + 1U] << 8) |
      ((uint32_t)data[offset + 2U] << 16) |
      ((uint32_t)data[offset + 3U] << 24);
    offset += 4U;
    length -= 4U;
  }
  for (index = 0U; index < length; index++)
  {
    lastWord |= (uint32_t)data[offset + index] << (index * 8U);
  }
  if (length > 0U)
  {
    CRC->DR = lastWord;
  }
  return CRC->DR;
}

static uint8_t SI4463_TakeInterrupt(void)
{ // 原子读取并清除NIRQ事件
  uint32_t interruptState = __get_PRIMASK();
  uint8_t pending;

  __disable_irq();
  pending = interruptPending;
  interruptPending = 0U;
  if (interruptState == 0U)
  {
    __enable_irq();
  }
  return pending;
}

void EXTI9_5_IRQHandler(void)
{ // 处理SI4463的PB5下降沿中断
  HAL_GPIO_EXTI_IRQHandler(BOARD_SI4463_MOSI_IRQ_PIN);
}

void HAL_GPIO_EXTI_Callback(uint16_t gpioPin)
{ // 记录SI4463中断，交由无线任务处理
  if (gpioPin == BOARD_SI4463_MOSI_IRQ_PIN)
  {
    interruptPending = 1U;
  }
}