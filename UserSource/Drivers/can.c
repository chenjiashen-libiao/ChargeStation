#include "can.h"
#include <string.h>

static CAN_HandleTypeDef hcan;

static CAN_TxHeaderTypeDef txHeader;

/* 中断接收队列最多缓存15帧，保留一个槽位区分队满和队空 */
#define CAN_RX_QUEUE_SIZE              16

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
{ // 初始化CAN1、全接收过滤器及接收中断
  CAN_GPIO_Init();

  BOARD_CAN_CLK_ENABLE();

  hcan.Instance = BOARD_CAN_INSTANCE;
  hcan.Init.Prescaler = BOARD_CAN_PRESCALER;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
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

HAL_StatusTypeDef CAN_SendDataId(uint32_t id, uint8_t *data, uint8_t len)
{ // 发送标准数据帧，邮箱忙时最多等待10ms
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
{ // 从中断写入的环形队列中原子取出最早一帧
  uint32_t interruptState;
  uint8_t i;
  CAN_RxFrame *frame;

  interruptState = __get_PRIMASK();
  __disable_irq();
  if (rxReadIndex == rxWriteIndex)
  {
    if (interruptState == 0)
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

  rxReadIndex = (rxReadIndex + 1) % CAN_RX_QUEUE_SIZE;
  if (interruptState == 0)
  {
    __enable_irq();
  }
  return 1;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcanInst)
{ // 在接收中断中将报文写入软件环形队列
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
  uint8_t nextWriteIndex;

  if (HAL_CAN_GetRxMessage(hcanInst, CAN_RX_FIFO0, &header, data) == HAL_OK)
  {
    nextWriteIndex = (rxWriteIndex + 1) % CAN_RX_QUEUE_SIZE;
    if (nextWriteIndex != rxReadIndex)
    {
      rxQueue[rxWriteIndex].header = header;
      memcpy(rxQueue[rxWriteIndex].data, data, sizeof(data));
      rxWriteIndex = nextWriteIndex;
    }
  }
}

static void CAN_GPIO_Init(void)
{ // 配置CAN重映射及收发引脚
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
{ // 配置32位掩码过滤器接收全部CAN报文
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
{ // 转交CAN FIFO0接收中断
  HAL_CAN_IRQHandler(&hcan);
}

void CAN1_SCE_IRQHandler(void)
{ // 转交CAN状态变化和错误中断
  HAL_CAN_IRQHandler(&hcan);
}

