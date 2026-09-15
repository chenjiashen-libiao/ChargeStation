#include "si4463_io.h"

static volatile uint8_t interruptPending;

void SI4463_IO_Init(void)
{ // 初始化SI4463模拟SPI和低有效中断引脚
  GPIO_InitTypeDef gpioInit = {0};

  BOARD_SI4463_GPIO_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  HAL_GPIO_WritePin(BOARD_SI4463_GPIO_PORT,
    BOARD_SI4463_CSN_PIN | BOARD_SI4463_SCK_PIN,
    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BOARD_SI4463_GPIO_PORT, BOARD_SI4463_CSN_PIN, GPIO_PIN_SET);

  gpioInit.Pin = BOARD_SI4463_CSN_PIN | BOARD_SI4463_SCK_PIN;
  gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
  gpioInit.Pull = GPIO_NOPULL;
  gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BOARD_SI4463_GPIO_PORT, &gpioInit);

  gpioInit.Pin = BOARD_SI4463_MISO_PIN;
  gpioInit.Mode = GPIO_MODE_INPUT;
  gpioInit.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BOARD_SI4463_GPIO_PORT, &gpioInit);

  SI4463_IO_SetIrqMode();
}

void SI4463_IO_SetMosiMode(void)
{ // SPI事务期间将PB5切换为MOSI输出
  GPIO_InitTypeDef gpioInit = {0};

  HAL_NVIC_DisableIRQ(BOARD_SI4463_IRQN);
  __HAL_GPIO_EXTI_CLEAR_IT(BOARD_SI4463_MOSI_IRQ_PIN);
  HAL_GPIO_WritePin(BOARD_SI4463_GPIO_PORT, BOARD_SI4463_MOSI_IRQ_PIN, GPIO_PIN_SET);

  gpioInit.Pin = BOARD_SI4463_MOSI_IRQ_PIN;
  gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
  gpioInit.Pull = GPIO_NOPULL;
  gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BOARD_SI4463_GPIO_PORT, &gpioInit);
}

void SI4463_IO_SetIrqMode(void)
{ // SPI空闲期间将PB5切换为NIRQ下降沿输入
  GPIO_InitTypeDef gpioInit = {0};

  gpioInit.Pin = BOARD_SI4463_MOSI_IRQ_PIN;
  gpioInit.Mode = GPIO_MODE_IT_FALLING;
  gpioInit.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BOARD_SI4463_GPIO_PORT, &gpioInit);

  __HAL_GPIO_EXTI_CLEAR_IT(BOARD_SI4463_MOSI_IRQ_PIN);
  HAL_NVIC_SetPriority(BOARD_SI4463_IRQN, 5, 0);
  HAL_NVIC_EnableIRQ(BOARD_SI4463_IRQN);

  if (HAL_GPIO_ReadPin(BOARD_SI4463_GPIO_PORT, BOARD_SI4463_MOSI_IRQ_PIN) == GPIO_PIN_RESET)
  {
    interruptPending = 1U;
  }
}

uint8_t SI4463_IO_TakeInterrupt(void)
{ // 读取并清除无线中断事件
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
{ // 记录SI4463中断，后续由无线任务消费
  if (gpioPin == BOARD_SI4463_MOSI_IRQ_PIN)
  {
    interruptPending = 1U;
  }
}