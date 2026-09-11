#include "SPI.h"

#if BOARD_SPI_ENABLE

static SPI_HandleTypeDef hspi2;

static void SPI_GPIO_Init(void);

void SPI_Init(void)
{
  SPI_GPIO_Init();
  SPI_CS_High();

  BOARD_SPI_CLK_ENABLE();

  hspi2.Instance = BOARD_SPI_INSTANCE;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = BOARD_SPI_CPOL;
  hspi2.Init.CLKPhase = BOARD_SPI_CPHA;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = BOARD_SPI_BAUD_PRESCALER;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;

  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    while (1)
    {
    }
  }
}

void SPI_CS_Low(void)
{
  HAL_GPIO_WritePin(BOARD_SPI_CS_PORT, BOARD_SPI_CS_PIN, GPIO_PIN_RESET);
}

void SPI_CS_High(void)
{
  HAL_GPIO_WritePin(BOARD_SPI_CS_PORT, BOARD_SPI_CS_PIN, GPIO_PIN_SET);
}

uint8_t SPI_TransferByte(uint8_t data)
{
  uint8_t rx_data = 0U;

  if (HAL_SPI_TransmitReceive(&hspi2, &data, &rx_data, 1, 100) != HAL_OK)
  {
    return 0U;
  }

  return rx_data;
}

static void SPI_GPIO_Init(void)
{
  GPIO_InitTypeDef gpio_init = {0};

  BOARD_SPI_GPIO_CLK_ENABLE();

  HAL_GPIO_WritePin(BOARD_SPI_CS_PORT, BOARD_SPI_CS_PIN, GPIO_PIN_SET);

  gpio_init.Pin = BOARD_SPI_CS_PIN;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOARD_SPI_CS_PORT, &gpio_init);

  gpio_init.Pin = BOARD_SPI_SCK_PIN | BOARD_SPI_MOSI_PIN;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOARD_SPI_SCK_PORT, &gpio_init);

  gpio_init.Pin = BOARD_SPI_MISO_PIN;
  gpio_init.Mode = GPIO_MODE_INPUT;
  gpio_init.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOARD_SPI_MISO_PORT, &gpio_init);
}

#endif