#include "si4463.h"
#include "board_config.h"
#include "radio_config_Si4463.h"
#include "si446x_defs.h"
#include <../CMSIS_RTOS/cmsis_os.h>
#include <string.h>

/* SI4463命令等待及包处理中断标志 */
#define SI4463_PACKET_RX_PENDING        0x10  // 收到完整数据包
#define SI4463_PACKET_CRC_ERROR         0x08  // 射频包CRC错误
#define SI4463_PACKET_TX_COMPLETE       0x20  // 数据包发送完成

/* 变长帧FIELD2由3字节CRC和业务负载组成 */
#define SI4463_VARIABLE_CRC_SIZE        3   // 仅发送CRC32低24位
#define SI4463_VARIABLE_MAX_FIELD2      63  // SI4463接收FIELD2上限
#define SI4463_VARIABLE_MIN_PAYLOAD     3   // 最短有效业务负载

/* STM32F1 GPIOB CRL中PB5的4位配置字段 */
#define SI4463_MOSI_CONFIG_SHIFT        20
#define SI4463_MOSI_CONFIG_MASK         (0x0F << SI4463_MOSI_CONFIG_SHIFT)
#define SI4463_MOSI_OUTPUT_CONFIG       (0x03 << SI4463_MOSI_CONFIG_SHIFT)  // 推挽输出50MHz
#define SI4463_MOSI_INPUT_CONFIG        (0x08 << SI4463_MOSI_CONFIG_SHIFT)  // 上拉/下拉输入

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
static uint32_t spiInterruptState;

static void SI4463_InitIo(void);
static void SI4463_SetMosiMode(void);
static void SI4463_SetIrqMode(void);
static void SI4463_Select(void);
static void SI4463_Deselect(void);
__STATIC_INLINE void SI4463_SpiDelay(void);
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
static uint8_t SI4463_ReadDeviceState(uint8_t *state);
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
{ // 初始化SI4463并进入Home信道固定长接收
  uint8_t status[9];
  uint8_t attempt;

  memset(&si4463Info, 0, sizeof(si4463Info));
  homeChannel = 0;
  txPower = 0x7F;
  frequencyBase = 0;
  frequencyStep = 250;
  if (*(const uint32_t *)BOARD_RF_HOME_CHANNEL_STORAGE != 0xFFFFFFFF)
  {
    homeChannel = *(const uint8_t *)BOARD_RF_HOME_CHANNEL_STORAGE;
  }
  if (*(const uint32_t *)BOARD_RF_FREQUENCY_STORAGE != 0xFFFFFFFF)
  {
    frequencyBase = *(const uint16_t *)BOARD_RF_FREQUENCY_STORAGE;
  }
  if (*(const uint32_t *)BOARD_RF_FREQUENCY_STEP_STORAGE != 0xFFFFFFFF)
  {
    frequencyStep = *(const uint16_t *)BOARD_RF_FREQUENCY_STEP_STORAGE;
  }
  if (*(const uint32_t *)BOARD_RF_TX_POWER_STORAGE != 0xFFFFFFFF)
  {
    txPower = *(const uint8_t *)BOARD_RF_TX_POWER_STORAGE;
  }
  SI4463_InitIo();
  osDelay(100);                        // 等待无线模块电源和晶振稳定
  for (attempt = 0; attempt < 3; attempt++)
  { // 上电较慢时仅重试安全的只读识别命令
    if (SI4463_ReadPartInfo() &&
      ((si4463Info.PartNumber & 0xFFF0) == 0x4460))
    {
      break;
    }
    osDelay(20);                       // 芯片识别重试间隔
  }
  if ((attempt == 3) || !SI4463_LoadConfig(configTable))
  {
    return 0;
  }
  if ((frequencyBase > 849) ||
    ((frequencyBase == 0) && (si4463Info.RomId == 0x06)))
  {
    if (!SI4463_LoadConfig(config902Table))
    {
      return 0;
    }
  }
  if ((frequencyBase != 0) && !SI4463_SetFrequency())
  {
    return 0;
  }
  if (!SI4463_ConfigGpio() || !SI4463_SetTxPower() ||
    !SI4463_ReadInterruptStatus(status) ||
    !SI4463_SetFixedMode(homeChannel))
  {
    return 0;
  }
  si4463Info.Ready = 1;
  return 1;
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
    !SI4463_Command(&command, 1) ||
    !SI4463_ReadResponse(response, sizeof(response)))
  {
    return 0;
  }
  *channel = response[2];
  return 1;
}

const SI4463_Info *SI4463_GetInfo(void)
{ // 返回芯片识别信息
  return &si4463Info;
}

uint8_t SI4463_SendPacket(const uint8_t *data, uint8_t length, uint8_t channel)
{ // 发送固定长度无线数据并恢复接收
  if (!si4463Info.Ready || (data == NULL) || (length != SI4463_PACKET_LENGTH))
  {
    return 0;
  }
  return SI4463_Transmit(data, length, channel, 0);
}

uint8_t SI4463_SendVariablePacket(const uint8_t *data, uint8_t length, uint8_t channel)
{ // 发送长度字节、CRC32低24位和有效数据
  uint8_t fifo[SI4463_VARIABLE_MAX_FIELD2 + 1];
  uint8_t field2Length;
  uint8_t index;
  uint32_t crc;
  uint8_t result;

  if (!si4463Info.Ready || !variableMode || (data == NULL) ||
    (length < SI4463_VARIABLE_MIN_PAYLOAD) ||
    (length > SI4463_VARIABLE_MAX_PAYLOAD))
  {
    return 0;
  }
  crc = SI4463_ComputeCrc(data, length);
  field2Length = length + SI4463_VARIABLE_CRC_SIZE;
  fifo[0] = field2Length;
  fifo[1] = (uint8_t)crc;
  fifo[2] = (uint8_t)(crc >> 8);
  fifo[3] = (uint8_t)(crc >> 16);
  for (index = 0; index < length; index++)
  {
    fifo[index + 4] = data[index];
  }
  if (!SI4463_SetProperty(PKT_FIELD_2_LENGTH_7_0, field2Length))
  {
    return 0;
  }
  result = SI4463_Transmit(fifo, field2Length + 1, channel, 0x60);
  SI4463_SetProperty(PKT_FIELD_2_LENGTH_7_0, SI4463_VARIABLE_MAX_FIELD2);
  return result;
}

uint8_t SI4463_GetReceivedFrame(uint8_t *data, uint8_t *length, uint8_t *isVariable)
{ // 读取无线任务收到的最新帧及协议模式
  uint32_t interruptState;

  if ((data == NULL) || (length == NULL) || (isVariable == NULL))
  {
    return 0;
  }
  interruptState = __get_PRIMASK();
  __disable_irq();
  if (!receivedFramePending)
  {
    if (interruptState == 0)
    {
      __enable_irq();
    }
    return 0;
  }
  memcpy(data, receivedFrame, receivedFrameLength);
  *length = receivedFrameLength;
  *isVariable = receivedFrameVariable;
  receivedFramePending = 0;
  if (interruptState == 0)
  {
    __enable_irq();
  }
  return 1;
}

uint8_t SI4463_SetFixedMode(uint8_t channel)
{ // 配置固定16字节模式并进入指定信道接收
  uint8_t status[9];
  uint8_t hardwareChannel;

  if (!SI4463_ChangeState(3) || !SI4463_ReadInterruptStatus(status) ||
    !SI4463_ResetRxFifo() ||
    !SI4463_SetProperty(PKT_CRC_CONFIG, 0x84) ||
    !SI4463_SetProperty(PKT_LEN, 0x00) ||
    !SI4463_SetProperty(PKT_LEN_FIELD_SOURCE, 0x00) ||
    !SI4463_SetProperty(PKT_LEN_ADJUST, 0x00) ||
    !SI4463_SetProperty(PKT_FIELD_1_LENGTH_12_8, 0x00) ||
    !SI4463_SetProperty(PKT_FIELD_1_LENGTH_7_0, SI4463_PACKET_LENGTH) ||
    !SI4463_SetProperty(PKT_FIELD_1_CRC_CONFIG, 0xAA) ||
    !SI4463_SetProperty(PKT_FIELD_2_LENGTH_12_8, 0x00) ||
    !SI4463_SetProperty(PKT_FIELD_2_LENGTH_7_0, 0x00) ||
    !SI4463_SetProperty(PKT_FIELD_2_CRC_CONFIG, 0x00))
  {
    return 0;
  }
  currentChannel = channel;
  variableMode = 0;
  return SI4463_StartReceive() &&
    SI4463_ReadCurrentChannel(&hardwareChannel) &&
    (hardwareChannel == channel);
}

uint8_t SI4463_SetVariableMode(uint8_t channel)
{ // 配置V5双FIELD变长模式并进入指定信道接收
  uint8_t status[9];
  uint8_t hardwareChannel;

  if (!SI4463_ChangeState(3) || !SI4463_ReadInterruptStatus(status) ||
    !SI4463_ResetRxFifo() ||
    !SI4463_SetProperty(PKT_CRC_CONFIG, 0x84) ||
    !SI4463_SetProperty(PKT_LEN, 0x2A) ||
    !SI4463_SetProperty(PKT_LEN_FIELD_SOURCE, 0x01) ||
    !SI4463_SetProperty(PKT_LEN_ADJUST, 0x00) ||
    !SI4463_SetProperty(PKT_FIELD_1_LENGTH_12_8, 0x00) ||
    !SI4463_SetProperty(PKT_FIELD_1_LENGTH_7_0, 0x01) ||
    !SI4463_SetProperty(PKT_FIELD_1_CRC_CONFIG, 0x00) ||
    !SI4463_SetProperty(PKT_FIELD_2_LENGTH_12_8, 0x00) ||
    !SI4463_SetProperty(PKT_FIELD_2_LENGTH_7_0, SI4463_VARIABLE_MAX_FIELD2) ||
    !SI4463_SetProperty(PKT_FIELD_2_CRC_CONFIG, 0x00))
  {
    return 0;
  }
  currentChannel = channel;
  variableMode = 1;
  return SI4463_StartReceive() &&
    SI4463_ReadCurrentChannel(&hardwareChannel) &&
    (hardwareChannel == channel);
}

uint8_t SI4463_EnterReceive(void)
{ // 按当前协议模式恢复接收
  if (!si4463Info.Ready)
  {
    return 0;
  }
  return SI4463_StartReceive();
}

void SI4463_Process(void)
{ // 处理无线中断并保持Home信道接收
  uint8_t status[9];
  uint8_t pending;
  uint8_t frameReceived = 0;
  uint8_t state;
  static uint32_t lastRxCheckTick;

  if (!si4463Info.Ready)
  {
    return;
  }
  // NIRQ可能由EXTI记录，也可能在PB5恢复输入后重新采样，两条路径共同避免漏报
  pending = SI4463_TakeInterrupt();
  if (!pending)
  {
    if (HAL_GPIO_ReadPin(BOARD_SI4463_GPIO_PORT,
      BOARD_SI4463_MOSI_IRQ_PIN) != GPIO_PIN_RESET)
    {
      if ((uint32_t)(HAL_GetTick() - lastRxCheckTick) >= 50)
      {
        lastRxCheckTick = HAL_GetTick();
        if (SI4463_ReadDeviceState(&state) && (state != 8) &&
          (interruptPending == 0) &&
          (HAL_GPIO_ReadPin(BOARD_SI4463_GPIO_PORT,
            BOARD_SI4463_MOSI_IRQ_PIN) != GPIO_PIN_RESET))
        {
          SI4463_StartReceive();
        }
      }
      return;
    }
  }
  if (!SI4463_ReadInterruptStatus(status))
  {
    SI4463_StartReceive();
    return;
  }
  if (((status[3] & SI4463_PACKET_RX_PENDING) != 0) &&
    ((status[3] & SI4463_PACKET_CRC_ERROR) == 0))
  {
    if (variableMode)
    {
      if (SI4463_ReadVariablePacket(receivedFrame, &receivedFrameLength))
      {
        receivedFrameVariable = 1;
        receivedFramePending = 1;
        frameReceived = 1;
      }
    }
    else if (SI4463_ReadFixedPacket(receivedFrame))
    {
      SI4463_ChangeState(5);
      receivedFrameLength = SI4463_PACKET_LENGTH;
      receivedFrameVariable = 0;
      receivedFramePending = 1;
      frameReceived = 1;
    }
  }
  if ((status[3] & SI4463_PACKET_TX_COMPLETE) != 0)
  {
    SI4463_ResetTxFifo();
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
  gpioInit.Pin = BOARD_SI4463_MOSI_IRQ_PIN;
  gpioInit.Mode = GPIO_MODE_IT_FALLING;
  gpioInit.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BOARD_SI4463_GPIO_PORT, &gpioInit);
  __HAL_GPIO_EXTI_CLEAR_IT(BOARD_SI4463_MOSI_IRQ_PIN);
  HAL_NVIC_SetPriority(BOARD_SI4463_IRQN, 5, 0);
  HAL_NVIC_EnableIRQ(BOARD_SI4463_IRQN);
}

static void SI4463_SetMosiMode(void)
{ // SPI事务期间将PB5切换为MOSI输出
  // PB5复用为MOSI和低有效NIRQ，切换输出前必须屏蔽并清除对应EXTI
  EXTI->IMR &= ~BOARD_SI4463_MOSI_IRQ_PIN;
  HAL_NVIC_DisableIRQ(BOARD_SI4463_IRQN);
  __HAL_GPIO_EXTI_CLEAR_IT(BOARD_SI4463_MOSI_IRQ_PIN);
  BOARD_SI4463_GPIO_PORT->BSRR = BOARD_SI4463_MOSI_IRQ_PIN;
  MODIFY_REG(BOARD_SI4463_GPIO_PORT->CRL, SI4463_MOSI_CONFIG_MASK,
    SI4463_MOSI_OUTPUT_CONFIG);
}

static void SI4463_SetIrqMode(void)
{ // SPI空闲期间将PB5恢复为低有效NIRQ
  // 恢复输入后重新检查引脚电平，补记SPI事务期间到达的无线事件
  BOARD_SI4463_GPIO_PORT->BSRR = BOARD_SI4463_MOSI_IRQ_PIN;
  MODIFY_REG(BOARD_SI4463_GPIO_PORT->CRL, SI4463_MOSI_CONFIG_MASK,
    SI4463_MOSI_INPUT_CONFIG);
  if ((__HAL_GPIO_EXTI_GET_IT(BOARD_SI4463_MOSI_IRQ_PIN) == RESET) &&
    ((BOARD_SI4463_GPIO_PORT->IDR & BOARD_SI4463_MOSI_IRQ_PIN) == 0))
  {
    interruptPending = 1;
  }
  EXTI->IMR |= BOARD_SI4463_MOSI_IRQ_PIN;
  HAL_NVIC_EnableIRQ(BOARD_SI4463_IRQN);
}

static void SI4463_Select(void)
{ // 开始模拟SPI事务
  // 单笔CS事务内禁止ADC等中断插入，保证GPIO模拟SPI时钟连续
  spiInterruptState = __get_PRIMASK();
  __disable_irq();
  SI4463_SetMosiMode();
  BOARD_SI4463_GPIO_PORT->BRR = BOARD_SI4463_CSN_PIN;
}

static void SI4463_Deselect(void)
{ // 结束模拟SPI事务并恢复NIRQ
  BOARD_SI4463_GPIO_PORT->BSRR = BOARD_SI4463_CSN_PIN;
  SI4463_SetIrqMode();
  // 仅恢复进入事务前处于开启状态的全局中断
  if (spiInterruptState == 0)
  {
    __enable_irq();
  }
}

__STATIC_INLINE void SI4463_SpiDelay(void)
{ // 固定模拟SPI边沿间隔，不受编译优化影响
  __NOP();
  __NOP();
  __NOP();
  __NOP();
}

static uint8_t SI4463_ExchangeByte(uint8_t txData)
{ // 模拟SPI收发一个字节，CPOL=0、CPHA=1、MSB先行
  uint8_t bit;
  uint8_t rxData = 0;
  GPIO_TypeDef *gpio = BOARD_SI4463_GPIO_PORT;

  gpio->BRR = BOARD_SI4463_SCK_PIN;
  for (bit = 0; bit < 8; bit++)
  {
    if ((txData & 0x80) != 0)
    {
      gpio->BSRR = BOARD_SI4463_MOSI_IRQ_PIN;
    }
    else
    {
      gpio->BRR = BOARD_SI4463_MOSI_IRQ_PIN;
    }
    SI4463_SpiDelay();
    gpio->BSRR = BOARD_SI4463_SCK_PIN;
    SI4463_SpiDelay();
    rxData <<= 1;
    if ((gpio->IDR & BOARD_SI4463_MISO_PIN) != 0)
    {
      rxData |= 1;
    }
    gpio->BRR = BOARD_SI4463_SCK_PIN;
    SI4463_SpiDelay();
    txData <<= 1;
  }
  return rxData;
}

static uint8_t SI4463_WaitCts(void)
{ // 等待芯片命令缓冲区就绪
  uint8_t attempt;
  uint8_t cts;

  for (attempt = 0; attempt < 20; attempt++)
  {
    SI4463_Select();
    SI4463_ExchangeByte(READ_CMD_BUFF);
    cts = SI4463_ExchangeByte(0xFF);
    SI4463_Deselect();
    if (cts == 0xFF)
    {
      return 1;
    }
    if (attempt < 19)
    {
      osDelay(1);
    }
  }
  return 0;
}

static uint8_t SI4463_Command(const uint8_t *command, uint8_t commandSize)
{ // 向SI4463发送命令
  uint8_t index;

  if (!SI4463_WaitCts())
  {
    return 0;
  }
  SI4463_Select();
  for (index = 0; index < commandSize; index++)
  {
    SI4463_ExchangeByte(command[index]);
  }
  SI4463_Deselect();
  return 1;
}

static uint8_t SI4463_ReadResponse(uint8_t *buffer, uint8_t size)
{ // 读取SI4463命令响应
  uint8_t index;

  if (!SI4463_WaitCts())
  {
    return 0;
  }
  SI4463_Select();
  SI4463_ExchangeByte(READ_CMD_BUFF);
  for (index = 0; index < size; index++)
  {
    buffer[index] = SI4463_ExchangeByte(0xFF);
  }
  SI4463_Deselect();
  return 1;
}

static uint8_t SI4463_ReadPartInfo(void)
{ // 读取并保存SI4463型号及ROM版本
  uint8_t command = PART_INFO;
  uint8_t response[9];

  if (!SI4463_Command(&command, 1) || !SI4463_ReadResponse(response, sizeof(response)))
  {
    return 0;
  }
  si4463Info.ChipRevision = response[1];
  si4463Info.PartNumber = ((uint16_t)response[2] << 8) | response[3];
  si4463Info.RomId = response[8];
  return 1;
}

static uint8_t SI4463_LoadConfig(const uint8_t *table)
{ // 逐条下发WDS生成的配置表
  uint16_t offset = 0;
  uint8_t length;

  while ((length = table[offset]) != 0)
  {
    offset++;
    if (!SI4463_Command(table + offset, length))
    {
      return 0;
    }
    offset += length;
  }
  return 1;
}

static uint8_t SI4463_SetProperty(uint16_t property, uint8_t value)
{ // 写入一个SI4463属性
  uint8_t command[5];

  command[0] = SET_PROPERTY;
  command[1] = (uint8_t)(property >> 8);
  command[2] = 1;
  command[3] = (uint8_t)property;
  command[4] = value;
  return SI4463_Command(command, sizeof(command));
}

static uint8_t SI4463_ConfigGpio(void)
{ // 配置模块GPIO2和GPIO3控制射频收发开关
  uint8_t command[8] = {GPIO_PIN_CFG, 0, 0, 0x20, 0x21, 0, 0, 0};
  uint8_t response[8];

  return SI4463_Command(command, sizeof(command)) &&
    SI4463_ReadResponse(response, sizeof(response));
}

static uint8_t SI4463_SetFrequency(void)
{ // 按参考公式设置基频和信道间隔
  uint8_t command[10];
  uint8_t outDiv = (frequencyBase >= 850) ? 1 : 2;
  uint8_t integer = (uint8_t)(outDiv * frequencyBase / 15 - 1);
  uint32_t fraction = (uint32_t)outDiv * frequencyBase * 524288 / 15 -
    (uint32_t)integer * 524288;
  uint16_t step = (uint16_t)((uint32_t)524288 * outDiv * frequencyStep / 15000);

  command[0] = SET_PROPERTY;
  command[1] = 0x40;
  command[2] = 0x06;
  command[3] = 0;
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
  uint8_t command[8] = {SET_PROPERTY, 0x22, 0x04, 0,
    0x08, txPower, 0, 0x3D};
  return SI4463_Command(command, sizeof(command));
}

static uint8_t SI4463_ChangeState(uint8_t state)
{ // 切换SI4463内部状态
  uint8_t command[2] = {CHANGE_STATE, state};
  return SI4463_Command(command, sizeof(command));
}

static uint8_t SI4463_ReadDeviceState(uint8_t *state)
{
  uint8_t command = REQUEST_DEVICE_STATE;
  uint8_t response[3];

  if (!SI4463_Command(&command, 1) ||
    !SI4463_ReadResponse(response, sizeof(response)))
  {
    return 0;
  }
  *state = response[1] & 0x0F;
  return 1;
}

static uint8_t SI4463_ReadInterruptStatus(uint8_t *status)
{ // 读取并清除SI4463全部中断状态
  uint8_t command[4] = {GET_INT_STATUS, 0, 0, 0};
  uint8_t index;
  uint8_t allOnes = 1;

  if (!SI4463_Command(command, sizeof(command)) || !SI4463_ReadResponse(status, 9))
  {
    return 0;
  }
  for (index = 0; index < 9; index++)
  {
    if (status[index] != 0xFF)
    {
      allOnes = 0;
      break;
    }
  }
  return !allOnes && (status[3] != 0xFF);
}

static uint8_t SI4463_ResetRxFifo(void)
{ // 清空接收FIFO
  uint8_t command[2] = {FIFO_INFO, 0x02};
  return SI4463_Command(command, sizeof(command));
}

static uint8_t SI4463_ResetTxFifo(void)
{ // 清空发送FIFO
  uint8_t command[2] = {FIFO_INFO, 0x01};
  return SI4463_Command(command, sizeof(command));
}

static uint8_t SI4463_StartReceive(void)
{ // 按当前信道和协议模式重新进入接收
  uint8_t command[8];

  if (!SI4463_ResetRxFifo())
  {
    return 0;
  }
  command[0] = START_RX;
  command[1] = currentChannel;
  command[2] = 0;
  command[3] = 0;
  command[4] = variableMode ? 0 : SI4463_PACKET_LENGTH;
  command[5] = 0;
  command[6] = variableMode ? 3 : 0;
  command[7] = 3;
  return SI4463_Command(command, sizeof(command));
}

static uint8_t SI4463_Transmit(const uint8_t *data, uint8_t length,
  uint8_t channel, uint8_t condition)
{ // 写入TX FIFO、启动发送并在完成后返回READY状态
  uint8_t command[5];
  uint8_t status[9];
  uint8_t attempts;
  uint8_t index;
  uint8_t completed = 0;

  if (!SI4463_ResetTxFifo() || !SI4463_ReadInterruptStatus(status) || !SI4463_WaitCts())
  {
    return 0;
  }
  SI4463_Select();
  SI4463_ExchangeByte(WRITE_TX_FIFO);
  for (index = 0; index < length; index++)
  {
    SI4463_ExchangeByte(data[index]);
  }
  SI4463_Deselect();
  command[0] = START_TX;
  command[1] = channel;
  command[2] = condition;
  command[3] = 0;
  command[4] = length;
  if (!SI4463_Command(command, sizeof(command)))
  {
    return 0;
  }
  for (attempts = 0; attempts < 20; attempts++)  // 最多等待约20ms发送完成
  {
    osDelay(1);
    if ((interruptPending != 0) ||
      (HAL_GPIO_ReadPin(BOARD_SI4463_GPIO_PORT,
        BOARD_SI4463_MOSI_IRQ_PIN) == GPIO_PIN_RESET))
    {
      SI4463_TakeInterrupt();
      if (SI4463_ReadInterruptStatus(status) &&
        ((status[3] & SI4463_PACKET_TX_COMPLETE) != 0))
      {
        completed = 1;
        break;
      }
    }
  }
  SI4463_ChangeState(3);
  SI4463_ReadInterruptStatus(status);
  return completed;
}

static uint8_t SI4463_ReadFixedPacket(uint8_t *data)
{ // 从RX FIFO读取一个固定16字节帧
  uint8_t index;

  if (!SI4463_WaitCts())
  {
    return 0;
  }
  SI4463_Select();
  SI4463_ExchangeByte(READ_RX_FIFO);
  for (index = 0; index < SI4463_PACKET_LENGTH; index++)
  {
    data[index] = SI4463_ExchangeByte(0xFF);
  }
  SI4463_Deselect();
  return 1;
}

static uint8_t SI4463_ReadVariablePacket(uint8_t *data, uint8_t *length)
{ // 读取并校验变长帧
  uint8_t fifo[SI4463_VARIABLE_MAX_FIELD2 + 1];
  uint8_t field2Length;
  uint8_t index;
  uint32_t receivedCrc;
  uint32_t calculatedCrc;

  *length = 0;
  if (!SI4463_WaitCts())
  {
    return 0;
  }
  SI4463_Select();
  SI4463_ExchangeByte(READ_RX_FIFO);
  field2Length = SI4463_ExchangeByte(0xFF);
  if ((field2Length >=
      (SI4463_VARIABLE_MIN_PAYLOAD + SI4463_VARIABLE_CRC_SIZE)) &&
    (field2Length <= SI4463_VARIABLE_MAX_FIELD2))
  {
    fifo[0] = field2Length;
    for (index = 1; index <= field2Length; index++)
    {
      fifo[index] = SI4463_ExchangeByte(0xFF);
    }
  }
  SI4463_Deselect();
  if ((field2Length <
      (SI4463_VARIABLE_MIN_PAYLOAD + SI4463_VARIABLE_CRC_SIZE)) ||
    (field2Length > SI4463_VARIABLE_MAX_FIELD2))
  {
    SI4463_ResetRxFifo();
    return 0;
  }
  receivedCrc = (uint32_t)fifo[1] |
    ((uint32_t)fifo[2] << 8) | ((uint32_t)fifo[3] << 16);
  *length = field2Length - SI4463_VARIABLE_CRC_SIZE;
  calculatedCrc = SI4463_ComputeCrc(&fifo[4], *length);
  if ((calculatedCrc & 0x00FFFFFF) != receivedCrc)
  {
    *length = 0;
    SI4463_ResetRxFifo();
    return 0;
  }
  memcpy(data, &fifo[4], *length);
  return 1;
}

static uint32_t SI4463_ComputeCrc(const uint8_t *data, uint16_t length)
{ // 使用STM32硬件CRC计算与参考工程一致的CRC32
  uint16_t offset = 0;
  uint32_t lastWord = 0;
  uint8_t index;

  __HAL_RCC_CRC_CLK_ENABLE();
  CRC->CR = CRC_CR_RESET;
  while (length >= 4)
  {
    CRC->DR = (uint32_t)data[offset] |
      ((uint32_t)data[offset + 1] << 8) |
      ((uint32_t)data[offset + 2] << 16) |
      ((uint32_t)data[offset + 3] << 24);
    offset += 4;
    length -= 4;
  }
  for (index = 0; index < length; index++)
  {
    lastWord |= (uint32_t)data[offset + index] << (index * 8);
  }
  if (length > 0)
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
  interruptPending = 0;
  if (interruptState == 0)
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
    interruptPending = 1;
  }
}