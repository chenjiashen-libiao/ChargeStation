#include "charge_control.h"
#include "Drivers/adc.h"
#include "Drivers/charge_pwm.h"
#include <../CMSIS_RTOS/cmsis_os.h>

#pragma GCC optimize ("Os")

#define CHARGE_PHASE_COUNT               3U
#define CHARGE_OUTPUT_VALID_MIN_MV       200
#define CHARGE_OUTPUT_VALID_MAX_MV       5200
#define CHARGE_LOW_BATTERY_MAX_MV        4100
#define CHARGE_REDUCE_CURRENT_MV         5400
#define CHARGE_INPUT_VALID_MIN_MV        5200U
#define CHARGE_OVERCURRENT_MA            50000
#define CHARGE_PRECHARGE_CURRENT_MA      10000
#define CHARGE_TEST_CURRENT_MA           5000
#define CHARGE_MAX_CURRENT_MA            30000
#define CHARGE_CURRENT_INTEGRAL_DIV      100000
#define CHARGE_CURRENT_INTEGRAL_MAX      70000000
#define CHARGE_CURRENT_INTEGRAL_MIN      (-70000000)

static volatile int32_t targetCurrent[CHARGE_PHASE_COUNT];
static volatile int32_t currentIntegral[CHARGE_PHASE_COUNT];
static volatile int32_t currentCorrection[CHARGE_PHASE_COUNT];
static volatile int32_t pwmFeedforward[CHARGE_PHASE_COUNT];
static volatile uint16_t pwmOutput[CHARGE_PHASE_COUNT];
static volatile uint16_t outputMask;

static uint8_t ChargeControl_IsOutputValid(uint8_t phase);
static uint8_t ChargeControl_AnyOutputValid(void);
static uint8_t ChargeControl_AnyPhaseEnabled(void);
static uint8_t ChargeControl_AllOutputsLow(void);
static uint8_t ChargeControl_AnyOutputHigh(int32_t voltage);
static void ChargeControl_SetAllTargets(int32_t current);
static void ChargeControl_EnablePhase(uint8_t phase);
static void ChargeControl_DisablePhase(uint8_t phase);
static void ChargeControl_DisableOutputs(void);
static void ChargeControl_BuildOutputMask(void);
static void ChargeControl_Precharge(void);
static void ChargeControl_TestResistance(void);
static void ChargeControl_StartCharging(void);
static void ChargeControl_UpdateCurrentLoop(uint8_t phase);

void ChargeControl_Init(void)
{ // 初始化功率控制并保持全部输出关闭
  uint8_t phase;

  ChargePWM_Init();
  ChargePWM_SetMosEnabled(0U);
  ChargePWM_DisableMain();
  ChargePWM_DisableAllPhases();
  outputMask = 0U;
  for (phase = 0U; phase < CHARGE_PHASE_COUNT; phase++)
  {
    targetCurrent[phase] = 0;
    currentIntegral[phase] = 0;
    currentCorrection[phase] = 0;
    pwmFeedforward[phase] = CHARGE_PWM_MIN_COMPARE;
    pwmOutput[phase] = CHARGE_PWM_MIN_COMPARE;
    ChargePWM_SetCompare(phase, CHARGE_PWM_MIN_COMPARE);
  }
}

void ChargeControl_Task(void const *argument)
{ // 按参考工程执行电池检测、内阻测试和充电循环
  uint8_t reduceCount;
  uint8_t phase;

  (void)argument;
  osDelay(20U);
  for (;;)
  {
    ChargeControl_SetAllTargets(0);
    osDelay(200U);
    ChargeControl_DisableOutputs();

    while (!ChargeControl_AnyOutputValid())
    {
      ChargePWM_SetMosEnabled(0U);
      osDelay(1200U);
    }

    ChargePWM_SetMosEnabled(1U);
    osDelay(500U);
    ChargeControl_BuildOutputMask();
    if (outputMask == 0U)
    {
      ChargePWM_SetMosEnabled(0U);
      continue;
    }

    ChargeControl_Precharge();
    ChargeControl_TestResistance();
    ChargeControl_StartCharging();

    reduceCount = 0U;
    for (;;)
    {
      osDelay(301U);
      if (ChargeControl_AnyOutputHigh(CHARGE_REDUCE_CURRENT_MV))
      {
        for (phase = 0U; phase < CHARGE_PHASE_COUNT; phase++)
        {
          targetCurrent[phase] -= 5000;
          if (targetCurrent[phase] <= 5000)
          {
            targetCurrent[phase] = 6000;
          }
        }
        osDelay(50U);
        reduceCount++;
      }
      if (reduceCount > 4U)
      {
        break;
      }
      if (ChargePWM_HasBreakFault())
      {
        ChargePWM_ClearBreakFault();
        osDelay(5000U);
        break;
      }
      if (!ChargePWM_IsMainEnabled() || !ChargeControl_AnyPhaseEnabled() ||
        !ChargePWM_IsMosEnabled())
      {
        osDelay(5000U);
        break;
      }
    }
  }
}

void ChargeControl_Update(void)
{ // 每轮ADC采样后执行保护、前馈和三相电流积分控制
  uint8_t phase;

  if ((g_intvolt[0] < CHARGE_INPUT_VALID_MIN_MV) &&
    (g_intvolt[1] < CHARGE_INPUT_VALID_MIN_MV) &&
    (g_intvolt[2] < CHARGE_INPUT_VALID_MIN_MV))
  {
    ChargePWM_DisableMain();
    ChargePWM_SetMosEnabled(0U);
  }

  for (phase = 0U; phase < CHARGE_PHASE_COUNT; phase++)
  {
    if ((g_I[phase] > CHARGE_OVERCURRENT_MA) ||
      (g_I[phase] < -CHARGE_OVERCURRENT_MA))
    {
      ChargeControl_DisablePhase(phase);
    }

    if (g_intvolt[phase] > 0U)
    {
      pwmFeedforward[phase] = g_out_Volt[phase] *
        (CHARGE_PWM_PERIOD + 1U) / g_intvolt[phase];
    }
    else
    {
      pwmFeedforward[phase] = CHARGE_PWM_MIN_COMPARE;
      ChargeControl_DisablePhase(phase);
    }

    if (ChargePWM_IsMainEnabled() && ChargePWM_IsPhaseEnabled(phase))
    {
      ChargeControl_UpdateCurrentLoop(phase);
    }
    else
    {
      currentIntegral[phase] = 0;
      currentCorrection[phase] = 0;
      if (pwmFeedforward[phase] < CHARGE_PWM_MIN_COMPARE)
      {
        pwmFeedforward[phase] = CHARGE_PWM_MIN_COMPARE;
      }
      if (pwmFeedforward[phase] > CHARGE_PWM_MAX_COMPARE)
      {
        pwmFeedforward[phase] = CHARGE_PWM_MAX_COMPARE;
      }
      pwmOutput[phase] = (uint16_t)pwmFeedforward[phase];
    }
    ChargePWM_SetCompare(phase, pwmOutput[phase]);
  }
}

int32_t ChargeControl_GetTargetCurrent(uint8_t phase)
{ // 返回指定相目标电流，单位mA
  return (phase < CHARGE_PHASE_COUNT) ? targetCurrent[phase] : 0;
}

uint8_t ChargeControl_IsPhaseEnabled(uint8_t phase)
{ // 返回指定相PWM支路状态
  return ChargePWM_IsPhaseEnabled(phase);
}

uint8_t ChargeControl_IsMainEnabled(void)
{ // 返回PWM主输出状态
  return ChargePWM_IsMainEnabled();
}

uint8_t ChargeControl_IsMosEnabled(void)
{ // 返回输出MOS状态
  return ChargePWM_IsMosEnabled();
}

uint8_t ChargeControl_HasBreakFault(void)
{ // 返回硬件刹车故障状态
  return ChargePWM_HasBreakFault();
}

static uint8_t ChargeControl_IsOutputValid(uint8_t phase)
{ // 判断指定相是否检测到有效电池电压
  return (g_out_Volt[phase] > CHARGE_OUTPUT_VALID_MIN_MV) &&
    (g_out_Volt[phase] < CHARGE_OUTPUT_VALID_MAX_MV);
}

static uint8_t ChargeControl_AnyOutputValid(void)
{ // 判断任意一相是否检测到有效电池电压
  uint8_t phase;

  for (phase = 0U; phase < CHARGE_PHASE_COUNT; phase++)
  {
    if (ChargeControl_IsOutputValid(phase))
    {
      return 1U;
    }
  }
  return 0U;
}

static uint8_t ChargeControl_AnyPhaseEnabled(void)
{ // 判断任意一相PWM支路是否开启
  uint8_t phase;

  for (phase = 0U; phase < CHARGE_PHASE_COUNT; phase++)
  {
    if (ChargePWM_IsPhaseEnabled(phase))
    {
      return 1U;
    }
  }
  return 0U;
}

static uint8_t ChargeControl_AllOutputsLow(void)
{ // 判断三相输出是否都处于低电压预充范围
  uint8_t phase;

  for (phase = 0U; phase < CHARGE_PHASE_COUNT; phase++)
  {
    if ((g_out_Volt[phase] <= CHARGE_OUTPUT_VALID_MIN_MV) ||
      (g_out_Volt[phase] >= CHARGE_LOW_BATTERY_MAX_MV))
    {
      return 0U;
    }
  }
  return 1U;
}

static uint8_t ChargeControl_AnyOutputHigh(int32_t voltage)
{ // 判断任意一相输出电压是否超过给定阈值
  uint8_t phase;

  for (phase = 0U; phase < CHARGE_PHASE_COUNT; phase++)
  {
    if (g_out_Volt[phase] > voltage)
    {
      return 1U;
    }
  }
  return 0U;
}

static void ChargeControl_SetAllTargets(int32_t current)
{ // 设置三相目标电流
  uint8_t phase;

  for (phase = 0U; phase < CHARGE_PHASE_COUNT; phase++)
  {
    targetCurrent[phase] = current;
  }
}

static void ChargeControl_EnablePhase(uint8_t phase)
{ // 仅打开已通过电压检查的相
  static const uint16_t masks[CHARGE_PHASE_COUNT] = {0x0001U, 0x0010U, 0x0100U};

  if ((phase < CHARGE_PHASE_COUNT) && ((outputMask & masks[phase]) != 0U))
  {
    ChargePWM_EnablePhase(phase);
  }
}

static void ChargeControl_DisablePhase(uint8_t phase)
{ // 关闭指定相PWM支路
  ChargePWM_DisablePhase(phase);
}

static void ChargeControl_DisableOutputs(void)
{ // 关闭主输出和全部三相支路
  ChargePWM_DisableMain();
  ChargePWM_DisableAllPhases();
}

static void ChargeControl_BuildOutputMask(void)
{ // 根据三相电池电压和输入电源生成输出允许位
  static const uint16_t masks[CHARGE_PHASE_COUNT] = {0x0001U, 0x0010U, 0x0100U};
  uint8_t phase;

  outputMask = 0U;
  for (phase = 0U; phase < CHARGE_PHASE_COUNT; phase++)
  {
    if (ChargeControl_IsOutputValid(phase) &&
      (g_intvolt[phase] > CHARGE_INPUT_VALID_MIN_MV))
    {
      outputMask |= masks[phase];
    }
  }
}

static void ChargeControl_Precharge(void)
{ // 对三相低压电池按参考逻辑执行10A预充
  uint16_t count;
  uint8_t breakFlag;

  if (!ChargeControl_AllOutputsLow())
  {
    return;
  }
  do
  {
    breakFlag = 0U;
    ChargeControl_EnablePhase(0U);
    ChargeControl_EnablePhase(1U);
    ChargeControl_EnablePhase(2U);
    ChargeControl_SetAllTargets(CHARGE_PRECHARGE_CURRENT_MA);
    osDelay(1U);
    ChargePWM_EnableMain();
    osDelay(10U);
    for (count = 0U; count < 1000U; count++)
    {
      osDelay(25U);
      if (ChargeControl_AnyOutputHigh(5000))
      {
        breakFlag = 1U;
        break;
      }
    }
    osDelay(10U);
    ChargeControl_DisableOutputs();
    ChargeControl_SetAllTargets(0);
    osDelay(2000U);
    if (breakFlag)
    {
      break;
    }
  }
  while (ChargeControl_AllOutputsLow());
}

static void ChargeControl_TestResistance(void)
{ // 逐相以5A测试压升并计算正式充电目标电流
  int32_t beginVoltage;
  int32_t resistanceVoltage;
  uint8_t phase;

  for (phase = 0U; phase < CHARGE_PHASE_COUNT; phase++)
  {
    osDelay(1U);
    beginVoltage = g_out_Volt[phase];
    ChargeControl_EnablePhase(phase);
    targetCurrent[phase] = CHARGE_TEST_CURRENT_MA;
    osDelay(10U);
    ChargePWM_EnableMain();
    osDelay(800U);
    resistanceVoltage = g_out_Volt[phase] - beginVoltage;
    ChargeControl_DisablePhase(phase);
    osDelay(10U);
    ChargePWM_DisableMain();
    osDelay(10U);

    if (resistanceVoltage > 300)
    {
      resistanceVoltage = 300;
    }
    if (resistanceVoltage < 0)
    {
      resistanceVoltage = 0;
    }
    targetCurrent[phase] = -100 * resistanceVoltage + 36000;
    if (g_out_Volt[phase] > CHARGE_OUTPUT_VALID_MAX_MV)
    {
      targetCurrent[phase] = 5001;
    }
    else
    {
      if (g_temp <= 10)
      {
        targetCurrent[phase] -= 15000;
      }
      if (g_temp <= 0)
      {
        targetCurrent[phase] -= 5000;
      }
      if (targetCurrent[phase] > CHARGE_MAX_CURRENT_MA)
      {
        targetCurrent[phase] = CHARGE_MAX_CURRENT_MA;
      }
      if (targetCurrent[phase] <= CHARGE_TEST_CURRENT_MA)
      {
        static const uint16_t masks[CHARGE_PHASE_COUNT] = {0x0001U, 0x0010U, 0x0100U};
        outputMask &= (uint16_t)~masks[phase];
      }
    }
  }
}

static void ChargeControl_StartCharging(void)
{ // 打开有效相并启动TIM1主输出
  uint8_t phase;

  for (phase = 0U; phase < CHARGE_PHASE_COUNT; phase++)
  {
    ChargeControl_EnablePhase(phase);
    osDelay(1U);
  }
  osDelay(10U);
  ChargePWM_EnableMain();
  osDelay(10U);
}

static void ChargeControl_UpdateCurrentLoop(uint8_t phase)
{ // 按参考积分算法更新指定相PWM
  int32_t pwm;

  currentIntegral[phase] += targetCurrent[phase] - g_I[phase];
  if (currentIntegral[phase] > CHARGE_CURRENT_INTEGRAL_MAX)
  {
    currentIntegral[phase] = CHARGE_CURRENT_INTEGRAL_MAX;
  }
  if (currentIntegral[phase] < CHARGE_CURRENT_INTEGRAL_MIN)
  {
    currentIntegral[phase] = CHARGE_CURRENT_INTEGRAL_MIN;
  }
  currentCorrection[phase] = currentIntegral[phase] / CHARGE_CURRENT_INTEGRAL_DIV;
  pwm = pwmFeedforward[phase] + currentCorrection[phase];
  if (pwm > CHARGE_PWM_MAX_COMPARE)
  {
    pwm = CHARGE_PWM_MAX_COMPARE;
  }
  if (pwm < CHARGE_PWM_MIN_COMPARE)
  {
    pwm = CHARGE_PWM_MIN_COMPARE;
  }
  pwmOutput[phase] = (uint16_t)pwm;
}
