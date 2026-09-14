#include "can.h"
#include <string.h>

CAN_HandleTypeDef hcan;

static CAN_TxHeaderTypeDef txHeader;

#define CAN_RX_QUEUE_SIZE 16U

typedef struct
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
} CAN_RxFrame;

static CAN_RxFrame rxQueue[CAN_RX_QUEUE_SIZE];
static volatile uint8_t rxWriteIndex;
static volatile uint8_t rxReadIndex;

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
  uint32_t interruptState;
  uint8_t i;
  CAN_RxFrame *frame;

  interruptState = __get_PRIMASK();
  __disable_irq();
  if (rxReadIndex == rxWriteIndex)
  {
    if (interruptState == 0U)
    {
      __enable_irq();
    }
    return 0;
  }

  frame = &rxQueue[rxReadIndex];
  if (id != NULL)
  {
    *id = (frame->header.IDE == CAN_ID_STD) ? frame->header.StdId : frame->header.ExtId;
  }

  if (len != NULL)
  {
    *len = frame->header.DLC;
  }

  if (dataBuf != NULL)
  {
    for (i = 0; i < frame->header.DLC; i++)
    {
      dataBuf[i] = frame->data[i];
    }
  }

  rxReadIndex = (rxReadIndex + 1U) % CAN_RX_QUEUE_SIZE;
  if (interruptState == 0U)
  {
    __enable_irq();
  }
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
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
  uint8_t nextWriteIndex;

  if (HAL_CAN_GetRxMessage(hcanInst, CAN_RX_FIFO0, &header, data) == HAL_OK)
  {
    nextWriteIndex = (rxWriteIndex + 1U) % CAN_RX_QUEUE_SIZE;
    if (nextWriteIndex != rxReadIndex)
    {
      rxQueue[rxWriteIndex].header = header;
      memcpy(rxQueue[rxWriteIndex].data, data, sizeof(data));
      rxWriteIndex = nextWriteIndex;
    }
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

void USB_LP_CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan);
}

void CAN1_SCE_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan);
}

uint8_t CAN_LoopbackTest(void)
{
  uint8_t txData[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
  uint8_t rxBuf[8];
  uint32_t rxId;
  uint8_t rxLen;
  uint32_t timeout;

  HAL_CAN_Stop(&hcan);
  hcan.Init.Mode = CAN_MODE_LOOPBACK;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    return 0;
  }
  HAL_CAN_Start(&hcan);

  rxReadIndex = 0U;
  rxWriteIndex = 0U;

  if (CAN_SendDataId(0x7FF, txData, 8) != HAL_OK)
  {
    CAN_ReInitMode(CAN_MODE_NORMAL);
    return 0;
  }

  timeout = HAL_GetTick() + 100;
  while (rxReadIndex == rxWriteIndex)
  {
    if (HAL_GetTick() > timeout)
    {
      CAN_ReInitMode(CAN_MODE_NORMAL);
      return 0;
    }
  }

  if (!CAN_GetLastRx(&rxId, &rxLen, rxBuf))
  {
    CAN_ReInitMode(CAN_MODE_NORMAL);
    return 0;
  }

  CAN_ReInitMode(CAN_MODE_NORMAL);
  return (rxId == 0x7FF && rxLen == 8 && memcmp(txData, rxBuf, 8) == 0) ? 1 : 0;
}

