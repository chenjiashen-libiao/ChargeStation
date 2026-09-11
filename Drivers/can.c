#include "can.h"

#if BOARD_CAN_ENABLE

CAN_HandleTypeDef hcan;

static CAN_TxHeaderTypeDef txHeader;
static CAN_RxHeaderTypeDef rxHeader;
static uint8_t rxData[8];
static volatile uint8_t rxFlag = 0;

static void CAN_GPIO_Init(void);
static void CAN_FilterConfig(void);

void CAN_Init(void)
{
  CAN_GPIO_Init();

  BOARD_CAN_CLK_ENABLE();

  hcan.Instance = BOARD_CAN_INSTANCE;
  hcan.Init.Prescaler = BOARD_CAN_PRESCALER;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    while (1) {}
  }

  CAN_FilterConfig();

  /* 使能 CAN 中断 */
  HAL_NVIC_SetPriority(BOARD_CAN_RX_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(BOARD_CAN_RX_IRQn);
  HAL_NVIC_SetPriority(BOARD_CAN_SCE_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(BOARD_CAN_SCE_IRQn);

  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF | CAN_IT_ERROR);
  HAL_CAN_Start(&hcan);
}

HAL_StatusTypeDef CAN_SendByte(uint8_t data)
{
  return CAN_SendData(&data, 1);
}

HAL_StatusTypeDef CAN_SendData(uint8_t *data, uint8_t len)
{
  return CAN_SendDataId(0x0100, data, len);
}

HAL_StatusTypeDef CAN_SendDataId(uint32_t id, uint8_t *data, uint8_t len)
{
  uint32_t mailbox;
  uint32_t timeout = HAL_GetTick() + 10;

  if (len > 8)
  {
    len = 8;
  }

  txHeader.StdId = id;
  txHeader.ExtId = 0;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = len;
  txHeader.TransmitGlobalTime = DISABLE;

  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
  {
    if (HAL_GetTick() > timeout)
    {
      return HAL_TIMEOUT;
    }
  }

  return HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &mailbox);
}

uint8_t CAN_GetLastRx(uint32_t *id, uint8_t *len, uint8_t *dataBuf)
{
  uint8_t i;

  if (rxFlag == 0)
  {
    return 0;
  }

  if (id != NULL)
  {
    *id = (rxHeader.IDE == CAN_ID_STD) ? rxHeader.StdId : rxHeader.ExtId;
  }

  if (len != NULL)
  {
    *len = rxHeader.DLC;
  }

  if (dataBuf != NULL)
  {
    for (i = 0; i < rxHeader.DLC; i++)
    {
      dataBuf[i] = rxData[i];
    }
  }

  rxFlag = 0;
  return 1;
}

uint32_t CAN_GetErrorStatus(void)
{
  return HAL_CAN_GetError(&hcan);
}

uint8_t CAN_IsBusOff(void)
{
  return ((hcan.Instance->ESR & CAN_ESR_BOFF) != 0) ? 1 : 0;
}

HAL_StatusTypeDef CAN_ReInitMode(uint32_t mode)
{
  HAL_CAN_Stop(&hcan);
  hcan.Init.Mode = mode;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_CAN_Start(&hcan);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcanInst)
{
  if (HAL_CAN_GetRxMessage(hcanInst, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
  {
    rxFlag = 1;
  }
}

static void CAN_GPIO_Init(void)
{
  GPIO_InitTypeDef gpio_init = {0};

  BOARD_CAN_GPIO_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();
  BOARD_CAN_REMAP();

  gpio_init.Pin = BOARD_CAN_RX_PIN;
  gpio_init.Mode = GPIO_MODE_INPUT;
  gpio_init.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BOARD_CAN_RX_PORT, &gpio_init);

  gpio_init.Pin = BOARD_CAN_TX_PIN;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BOARD_CAN_TX_PORT, &gpio_init);
}

static void CAN_FilterConfig(void)
{
  CAN_FilterTypeDef filter;

  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan, &filter);
}

/* CAN 中断处理入口 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan);
}

void CAN1_SCE_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan);
}

/**
  * @brief  CAN 回环测试（不需要外部设备）
  * @retval 1=成功, 0=失败
  */
uint8_t CAN_LoopbackTest(void)
{
  uint8_t txData[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
  uint8_t rxBuf[8];
  uint32_t rxId;
  uint8_t rxLen;
  uint32_t timeout;

  /* 切换到回环模式 */
  HAL_CAN_Stop(&hcan);
  hcan.Init.Mode = CAN_MODE_LOOPBACK;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    return 0;
  }
  HAL_CAN_Start(&hcan);

  /* 清除之前的接收标志 */
  rxFlag = 0;

  /* 发送测试帧 */
  if (CAN_SendDataId(0x7FF, txData, 8) != HAL_OK)
  {
    CAN_ReInitMode(CAN_MODE_NORMAL);
    return 0;
  }

  /* 等待接收 */
  timeout = HAL_GetTick() + 100;
  while (rxFlag == 0)
  {
    if (HAL_GetTick() > timeout)
    {
      CAN_ReInitMode(CAN_MODE_NORMAL);
      return 0;  /* 超时 */
    }
  }

  /* 验证数据 */
  if (!CAN_GetLastRx(&rxId, &rxLen, rxBuf))
  {
    CAN_ReInitMode(CAN_MODE_NORMAL);
    return 0;
  }

  if (rxId != 0x7FF || rxLen != 8)
  {
    CAN_ReInitMode(CAN_MODE_NORMAL);
    return 0;
  }

  for (int i = 0; i < 8; i++)
  {
    if (rxBuf[i] != txData[i])
    {
      CAN_ReInitMode(CAN_MODE_NORMAL);
      return 0;
    }
  }

  /* 恢复正常模式 */
  CAN_ReInitMode(CAN_MODE_NORMAL);
  return 1;  /* 测试通过 */
}

#endif
