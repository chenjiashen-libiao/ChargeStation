#include "adc.h"
#include "board_config.h"
#include "../charge_control.h"

volatile uint16_t g_ADC_ConvertedValue[CHARGE_ADC_CHANNEL_COUNT];
volatile int32_t g_out_Volt[CHARGE_PHASE_COUNT];
volatile int32_t g_I[CHARGE_PHASE_COUNT];
volatile uint16_t g_intvolt[CHARGE_PHASE_COUNT];
volatile int32_t g_ref_1v2_vol;
volatile int32_t g_temp;

static ADC_HandleTypeDef hadc1;
static DMA_HandleTypeDef hdmaAdc1;

static void ADC_GPIO_Init(void);
static HAL_StatusTypeDef ADC_ConfigChannel(uint32_t channel, uint32_t rank);
static void ADC_UpdateMeasurements(void);

void ADC_Init(void)
{ // 初始化与参考充电桩一致的ADC1十一通道DMA循环采样
  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_ADC_CONFIG(RCC_ADCPCLK2_DIV4);
  ADC_GPIO_Init();

  hdmaAdc1.Instance = BOARD_ADC_DMA_INSTANCE;
  hdmaAdc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdmaAdc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdmaAdc1.Init.MemInc = DMA_MINC_ENABLE;
  hdmaAdc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdmaAdc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdmaAdc1.Init.Mode = DMA_CIRCULAR;
  hdmaAdc1.Init.Priority = DMA_PRIORITY_MEDIUM;
  if (HAL_DMA_Init(&hdmaAdc1) != HAL_OK)
  {
    while (1) {}
  }

  hadc1.Instance = BOARD_ADC_INSTANCE;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = CHARGE_ADC_CHANNEL_COUNT;
  __HAL_LINKDMA(&hadc1, DMA_Handle, hdmaAdc1);
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    while (1) {}
  }

  if ((ADC_ConfigChannel(ADC_CHANNEL_3, ADC_REGULAR_RANK_1) != HAL_OK) ||
    (ADC_ConfigChannel(ADC_CHANNEL_4, ADC_REGULAR_RANK_2) != HAL_OK) ||
    (ADC_ConfigChannel(ADC_CHANNEL_5, ADC_REGULAR_RANK_3) != HAL_OK) ||
    (ADC_ConfigChannel(ADC_CHANNEL_0, ADC_REGULAR_RANK_4) != HAL_OK) ||
    (ADC_ConfigChannel(ADC_CHANNEL_1, ADC_REGULAR_RANK_5) != HAL_OK) ||
    (ADC_ConfigChannel(ADC_CHANNEL_2, ADC_REGULAR_RANK_6) != HAL_OK) ||
    (ADC_ConfigChannel(ADC_CHANNEL_6, ADC_REGULAR_RANK_7) != HAL_OK) ||
    (ADC_ConfigChannel(ADC_CHANNEL_7, ADC_REGULAR_RANK_8) != HAL_OK) ||
    (ADC_ConfigChannel(ADC_CHANNEL_8, ADC_REGULAR_RANK_9) != HAL_OK) ||
    (ADC_ConfigChannel(ADC_CHANNEL_VREFINT, ADC_REGULAR_RANK_10) != HAL_OK) ||
    (ADC_ConfigChannel(ADC_CHANNEL_TEMPSENSOR, ADC_REGULAR_RANK_11) != HAL_OK))
  {
    while (1) {}
  }

  SET_BIT(ADC1->CR2, ADC_CR2_TSVREFE);
  HAL_NVIC_SetPriority(BOARD_ADC_DMA_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(BOARD_ADC_DMA_IRQn);

  if ((HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) ||
    (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_ADC_ConvertedValue,
      CHARGE_ADC_CHANNEL_COUNT) != HAL_OK))
  {
    while (1) {}
  }
}

static void ADC_GPIO_Init(void)
{ // 配置三相电压和电流采样引脚为模拟输入
  GPIO_InitTypeDef gpioInit = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  gpioInit.Mode = GPIO_MODE_ANALOG;
  gpioInit.Pull = GPIO_NOPULL;
  gpioInit.Pin = BOARD_ADC_GPIOA_PINS;
  HAL_GPIO_Init(GPIOA, &gpioInit);
  gpioInit.Pin = BOARD_ADC_GPIOB_PINS;
  HAL_GPIO_Init(GPIOB, &gpioInit);
}

static HAL_StatusTypeDef ADC_ConfigChannel(uint32_t channel, uint32_t rank)
{ // 配置ADC规则组中的单个采样通道
  ADC_ChannelConfTypeDef channelConfig = {0};

  channelConfig.Channel = channel;
  channelConfig.Rank = rank;
  channelConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  return HAL_ADC_ConfigChannel(&hadc1, &channelConfig);
}

static void ADC_UpdateMeasurements(void)
{ // 按参考工程公式换算三相采样值
  uint8_t phase;

  for (phase = 0U; phase < CHARGE_PHASE_COUNT; phase++)
  {
    g_out_Volt[phase] = (int32_t)g_ADC_ConvertedValue[phase + 3U] * 6600 / 4096;
    g_intvolt[phase] = (uint16_t)((uint32_t)g_ADC_ConvertedValue[phase + 6U] * 6600U / 4096U);
    g_I[phase] = ((int32_t)g_ADC_ConvertedValue[phase] * 6600 / 4096 - 2500) * 25;
  }

  if (g_ADC_ConvertedValue[9] != 0U)
  {
    g_ref_1v2_vol = (1200 * 4096) / g_ADC_ConvertedValue[9];
  }
  else
  {
    g_ref_1v2_vol = 1200;
  }
  g_temp = ((1420 - ((int32_t)g_ADC_ConvertedValue[10] * g_ref_1v2_vol) / 4096) * 100) / 435 + 25;
}

void DMA1_Channel1_IRQHandler(void)
{ // 处理ADC1 DMA循环采样完成中断
  HAL_DMA_IRQHandler(&hdmaAdc1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{ // 完成一轮十一通道采样后更新测量值
  if (hadc->Instance == BOARD_ADC_INSTANCE)
  {
    ADC_UpdateMeasurements();
    ChargeControl_Update();
  }
}