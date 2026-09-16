#include "charge_pwm.h"
#include "../../board_config.h"

#pragma GCC optimize ("Os")

static const uint16_t phaseEnableMask[3] = {0x0500U, 0x0050U, 0x0005U};

void ChargePWM_Init(void)
{ // 按参考工程初始化三相互补PWM并保持功率输出关闭
  GPIO_InitTypeDef gpioInit = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  gpioInit.Pin = GPIO_PIN_7;
  gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
  gpioInit.Pull = GPIO_NOPULL;
  gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpioInit);

  gpioInit.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  gpioInit.Mode = GPIO_MODE_AF_PP;
  HAL_GPIO_Init(GPIOA, &gpioInit);
  gpioInit.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOB, &gpioInit);

  RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST;
  RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST;
  DBGMCU->CR |= DBGMCU_CR_DBG_TIM1_STOP;
  TIM1->PSC = 0U;
  TIM1->ARR = CHARGE_PWM_PERIOD;
  TIM1->RCR = 0U;
  TIM1->CCR1 = CHARGE_PWM_INITIAL_COMPARE;
  TIM1->CCR2 = CHARGE_PWM_INITIAL_COMPARE;
  TIM1->CCR3 = CHARGE_PWM_INITIAL_COMPARE;
  TIM1->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE |
    TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;
  TIM1->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;
  TIM1->CR2 = 0x7FFCU;
  TIM1->CCER = 0U;
  TIM1->BDTR = TIM_BDTR_OSSR | TIM_BDTR_OSSI | TIM_BDTR_BKE | 0x20U;
  TIM1->CR1 = TIM_CR1_CMS_0 | TIM_CR1_ARPE | TIM_CR1_CKD_1;
  TIM1->EGR = TIM_EGR_UG;
  TIM1->CR1 |= TIM_CR1_CEN;
}

void ChargePWM_EnableMain(void)
{ // 打开TIM1主输出
  TIM1->BDTR |= TIM_BDTR_MOE;
}

void ChargePWM_DisableMain(void)
{ // 关闭TIM1主输出
  TIM1->BDTR &= ~TIM_BDTR_MOE;
}

void ChargePWM_EnablePhase(uint8_t phase)
{ // 打开指定相的主输出和互补输出
  if (phase < 3U)
  {
    TIM1->CCER |= phaseEnableMask[phase];
  }
}

void ChargePWM_DisablePhase(uint8_t phase)
{ // 关闭指定相的主输出和互补输出
  if (phase < 3U)
  {
    TIM1->CCER &= ~phaseEnableMask[phase];
  }
}

void ChargePWM_DisableAllPhases(void)
{ // 关闭全部三相支路
  TIM1->CCER &= ~0x0555U;
}

void ChargePWM_SetCompare(uint8_t phase, uint16_t compare)
{ // 按U/V/W对应CCR3/CCR2/CCR1写入PWM比较值
  if (phase == 0U)
  {
    TIM1->CCR3 = compare;
  }
  else if (phase == 1U)
  {
    TIM1->CCR2 = compare;
  }
  else if (phase == 2U)
  {
    TIM1->CCR1 = compare;
  }
}

uint8_t ChargePWM_IsMainEnabled(void)
{ // 返回TIM1主输出是否开启
  return (TIM1->BDTR & TIM_BDTR_MOE) != 0U;
}

uint8_t ChargePWM_IsPhaseEnabled(uint8_t phase)
{ // 返回指定相的主输出和互补输出是否开启
  return (phase < 3U) && ((TIM1->CCER & phaseEnableMask[phase]) != 0U);
}

uint8_t ChargePWM_HasBreakFault(void)
{ // 返回TIM1硬件刹车标志
  return (TIM1->SR & TIM_SR_BIF) != 0U;
}

void ChargePWM_ClearBreakFault(void)
{ // 清除TIM1硬件刹车标志
  TIM1->SR &= ~TIM_SR_BIF;
}

void ChargePWM_SetMosEnabled(uint8_t enabled)
{ // PB7低电平打开输出MOS
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,
    enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

uint8_t ChargePWM_IsMosEnabled(void)
{ // 返回输出MOS是否开启
  return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET;
}
