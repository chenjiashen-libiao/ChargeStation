#include "uart.h"

#if BOARD_UART_ENABLE

UART_HandleTypeDef huart;

static void UART_GPIO_Init(void);

void UART_Init(void)
{ // 初始化可选的USART1接口
  UART_GPIO_Init();
  BOARD_UART_CLK_ENABLE();

  huart.Instance = BOARD_UART_INSTANCE;
  huart.Init.BaudRate = BOARD_UART_BAUD_RATE;
  huart.Init.WordLength = UART_WORDLENGTH_8B;
  huart.Init.StopBits = UART_STOPBITS_1;
  huart.Init.Parity = UART_PARITY_NONE;
  huart.Init.Mode = UART_MODE_TX_RX;
  huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart) != HAL_OK)
  {
    while (1)
    {
    }
  }
}

HAL_StatusTypeDef UART_Send(const uint8_t *data, uint16_t length, uint32_t timeout)
{ // 在指定超时时间内阻塞发送数据
  return HAL_UART_Transmit(&huart, (uint8_t *)data, length, timeout);
}

HAL_StatusTypeDef UART_Receive(uint8_t *data, uint16_t length, uint32_t timeout)
{ // 在指定超时时间内阻塞接收数据
  return HAL_UART_Receive(&huart, data, length, timeout);
}

static void UART_GPIO_Init(void)
{ // 配置USART重映射及收发引脚
  GPIO_InitTypeDef gpio_init = {0};

  BOARD_UART_GPIO_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();
  BOARD_UART_REMAP();

  gpio_init.Pin = BOARD_UART_TX_PIN;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOARD_UART_TX_PORT, &gpio_init);

  gpio_init.Pin = BOARD_UART_RX_PIN;
  gpio_init.Mode = GPIO_MODE_INPUT;
  gpio_init.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BOARD_UART_RX_PORT, &gpio_init);
}

#endif
