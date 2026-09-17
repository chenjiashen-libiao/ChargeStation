#include "charge_control.h"
#include "Drivers/adc.h"
#include "Drivers/charge_pwm.h"
#include <../CMSIS_RTOS/cmsis_os.h>

#pragma GCC optimize ("Os")

/* 充电判定与控制阈值 */
#define CHARGE_OUTPUT_VALID_MIN_MV      200       // 电池接入判定下限，单位mV
#define CHARGE_VALID_MAX_MV             5200      // 输入有效及输出保护边界，单位mV
#define CHARGE_LOW_BATTERY_MAX_MV       4100      // 允许继续预充的电压上限，单位mV
#define CHARGE_OVERCURRENT_MA           50000     // 单相过流保护阈值，单位mA
#define CHARGE_TEST_CURRENT_MA          5000      // 内阻测试目标电流，单位mA
#define CHARGE_CURRENT_INTEGRAL_LIMIT   70000000  // 电流环积分绝对值上限

static volatile int32_t targetCurrent[CHARGE_PHASE_COUNT];
static volatile int32_t currentIntegral[CHARGE_PHASE_COUNT];
static volatile int32_t currentCorrection[CHARGE_PHASE_COUNT];
static volatile int32_t pwmFeedforward[CHARGE_PHASE_COUNT];
static volatile uint16_t pwmOutput[CHARGE_PHASE_COUNT];
static volatile uint16_t outputMask;
static uint8_t feedforwardCounter;
static const uint16_t phaseMasks[CHARGE_PHASE_COUNT] = {0x0001, 0x0010, 0x0100};

static uint8_t ChargeControl_IsOutputValid(uint8_t phase);
static uint8_t ChargeControl_AnyOutputValid(void);
static uint8_t ChargeControl_AnyPhaseEnabled(void);
static uint8_t ChargeControl_AllOutputsLow(void);
static uint8_t ChargeControl_AllOutputsBelowPrechargeLimit(void);
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
  ChargePWM_SetMosEnabled(0);
  ChargePWM_DisableMain();
  ChargePWM_DisableAllPhases();
  outputMask = 0;
  feedforwardCounter = 0;
  for (phase = 0; phase < CHARGE_PHASE_COUNT; phase++)
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
  osDelay(20);
  for (;;)
  {
    // 每轮依次执行电池检测、输入确认、预充、内阻测试和正式充电
    ChargeControl_SetAllTargets(0);
    osDelay(200);
    ChargeControl_DisableOutputs();

    while (!ChargeControl_AnyOutputValid())
    {
      ChargePWM_SetMosEnabled(0);
      osDelay(1200);
    }

    ChargePWM_SetMosEnabled(1);
    osDelay(500);
    ChargeControl_BuildOutputMask();
    if (outputMask == 0)
    {
      ChargePWM_SetMosEnabled(0);
      continue;
    }

    ChargeControl_Precharge();
    ChargeControl_TestResistance();
    ChargeControl_StartCharging();

    reduceCount = 0;
    for (;;)
    {
      // 输出超过5.4V时逐步降流，连续五次后结束本轮充电
      osDelay(301);
      if (ChargeControl_AnyOutputHigh(5400))
      {
        for (phase = 0; phase < CHARGE_PHASE_COUNT; phase++)
        {
          targetCurrent[phase] -= 5000;
          if (targetCurrent[phase] <= 5000)
          {
            targetCurrent[phase] = 6000;
          }
        }
        osDelay(50);
        reduceCount++;
      }
      if (reduceCount > 4)
      {
        break;
      }
      if (ChargePWM_HasBreakFault())
      {
        ChargePWM_ClearBreakFault();
        osDelay(5000);
        break;
      }
      if (!ChargePWM_IsMainEnabled() || !ChargeControl_AnyPhaseEnabled() ||
        !ChargePWM_IsMosEnabled())
      {
        osDelay(5000);
        break;
      }
    }
  }
}

void ChargeControl_Update(void)
{ // 每轮ADC采样后执行保护、前馈和三相电流积分控制
  uint8_t updateFeedforward;
  uint8_t phase;

  // 欠压和过流保护每轮执行，耗时较大的电压前馈每16轮更新一次
  feedforwardCounter++;
  updateFeedforward = feedforwardCounter >= 16;
  if (updateFeedforward)
  {
    feedforwardCounter = 0;
  }

  if ((g_intvolt[0] < CHARGE_VALID_MAX_MV) &&
    (g_intvolt[1] < CHARGE_VALID_MAX_MV) &&
    (g_intvolt[2] < CHARGE_VALID_MAX_MV))
  {
    ChargePWM_DisableMain();
    ChargePWM_SetMosEnabled(0);
  }

  for (phase = 0; phase < CHARGE_PHASE_COUNT; phase++)
  {
    if ((g_I[phase] > CHARGE_OVERCURRENT_MA) ||
      (g_I[phase] < -CHARGE_OVERCURRENT_MA))
    {
      ChargeControl_DisablePhase(phase);
    }

    if (updateFeedforward && (g_intvolt[phase] > 0))
    {
      pwmFeedforward[phase] = g_out_Volt[phase] *
        (CHARGE_PWM_PERIOD + 1) / g_intvolt[phase];
    }
    else if (g_intvolt[phase] == 0)
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
    (g_out_Volt[phase] < CHARGE_VALID_MAX_MV);
}

static uint8_t ChargeControl_AnyOutputValid(void)
{ // 判断任意一相是否检测到有效电池电压
  uint8_t phase;

  for (phase = 0; phase < CHARGE_PHASE_COUNT; phase++)
  {
    if (ChargeControl_IsOutputValid(phase))
    {
      return 1;
    }
  }
  return 0;
}

static uint8_t ChargeControl_AnyPhaseEnabled(void)
{ // 判断任意一相PWM支路是否开启
  uint8_t phase;

  for (phase = 0; phase < CHARGE_PHASE_COUNT; phase++)
  {
    if (ChargePWM_IsPhaseEnabled(phase))
    {
      return 1;
    }
  }
  return 0;
}

static uint8_t ChargeControl_AllOutputsLow(void)
{ // 判断三相输出是否都满足首次进入预充的电压范围
  uint8_t phase;

  for (phase = 0; phase < CHARGE_PHASE_COUNT; phase++)
  {
    if ((g_out_Volt[phase] <= CHARGE_OUTPUT_VALID_MIN_MV) ||
      (g_out_Volt[phase] >= CHARGE_LOW_BATTERY_MAX_MV))
    {
      return 0;
    }
  }
  return 1;
}

static uint8_t ChargeControl_AllOutputsBelowPrechargeLimit(void)
{ // 按参考流程判断三相是否继续预充
  uint8_t phase;

  for (phase = 0; phase < CHARGE_PHASE_COUNT; phase++)
  {
    if (g_out_Volt[phase] >= CHARGE_LOW_BATTERY_MAX_MV)
    {
      return 0;
    }
  }
  return 1;
}

static uint8_t ChargeControl_AnyOutputHigh(int32_t voltage)
{ // 判断任意一相输出电压是否超过给定阈值
  uint8_t phase;

  for (phase = 0; phase < CHARGE_PHASE_COUNT; phase++)
  {
    if (g_out_Volt[phase] > voltage)
    {
      return 1;
    }
  }
  return 0;
}

static void ChargeControl_SetAllTargets(int32_t current)
{ // 设置三相目标电流
  uint8_t phase;

  for (phase = 0; phase < CHARGE_PHASE_COUNT; phase++)
  {
    targetCurrent[phase] = current;
  }
}

static void ChargeControl_EnablePhase(uint8_t phase)
{ // 仅打开已通过电压检查的相
  if ((phase < CHARGE_PHASE_COUNT) && ((outputMask & phaseMasks[phase]) != 0))
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
  uint8_t phase;

  outputMask = 0;
  for (phase = 0; phase < CHARGE_PHASE_COUNT; phase++)
  {
    if (ChargeControl_IsOutputValid(phase) &&
      (g_intvolt[phase] > CHARGE_VALID_MAX_MV))
    {
      outputMask |= phaseMasks[phase];
    }
  }
}

static void ChargeControl_Precharge(void)
{ // 对三相低压电池按参考逻辑执行10A预充
  uint16_t count;
  uint8_t breakFlag;

  // 只有三相均处于首次预充电压范围才进入，避免对异常电池直接加压
  if (!ChargeControl_AllOutputsLow())
  {
    return;
  }
  do
  {
    breakFlag = 0;
    ChargeControl_EnablePhase(0);
    ChargeControl_EnablePhase(1);
    ChargeControl_EnablePhase(2);
    ChargeControl_SetAllTargets(10000);
    osDelay(1);
    ChargePWM_EnableMain();
    osDelay(10);
    for (count = 0; count < 1000; count++)
    {
      osDelay(25);
      if (ChargeControl_AnyOutputHigh(5000))
      {
        breakFlag = 1;
        break;
      }
    }
    osDelay(10);
    ChargePWM_DisableMain();
    osDelay(10);
    ChargePWM_DisableAllPhases();
    ChargeControl_SetAllTargets(0);
    osDelay(2000);
    if (breakFlag)
    {
      break;
    }
  }
  while (ChargeControl_AllOutputsBelowPrechargeLimit());
}

static void ChargeControl_TestResistance(void)
{ // 逐相以5A测试压升并计算正式充电目标电流
  int32_t beginVoltage;
  int32_t resistanceVoltage;
  uint8_t phase;

  for (phase = 0; phase < CHARGE_PHASE_COUNT; phase++)
  {
    osDelay(1);
    beginVoltage = g_out_Volt[phase];
    ChargeControl_EnablePhase(phase);
    targetCurrent[phase] = CHARGE_TEST_CURRENT_MA;
    osDelay(10);
    ChargePWM_EnableMain();
    osDelay(800);
    resistanceVoltage = g_out_Volt[phase] - beginVoltage;
    ChargeControl_DisablePhase(phase);
    osDelay(10);
    ChargePWM_DisableMain();
    osDelay(10);

    // 压升决定基础目标电流，温度和输出电压进一步限制允许电流
    if (resistanceVoltage > 300)
    {
      resistanceVoltage = 300;
    }
    if (resistanceVoltage < 0)
    {
      resistanceVoltage = 0;
    }
    targetCurrent[phase] = -100 * resistanceVoltage + 36000;
    if (g_out_Volt[phase] > CHARGE_VALID_MAX_MV)
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
      if (targetCurrent[phase] > 30000)
      {
        targetCurrent[phase] = 30000;
      }
      if (targetCurrent[phase] <= CHARGE_TEST_CURRENT_MA)
      {
        outputMask &= (uint16_t)~phaseMasks[phase];
      }
    }
  }
}

static void ChargeControl_StartCharging(void)
{ // 打开有效相并启动TIM1主输出
  uint8_t phase;

  for (phase = 0; phase < CHARGE_PHASE_COUNT; phase++)
  {
    ChargeControl_EnablePhase(phase);
    osDelay(1);
  }
  osDelay(10);
  ChargePWM_EnableMain();
  osDelay(10);
}

static void ChargeControl_UpdateCurrentLoop(uint8_t phase)
{ // 按参考积分算法更新指定相PWM
  int32_t pwm;

  // 输入电压前馈提供基础占空比，电流误差积分只负责补偿偏差
  currentIntegral[phase] += targetCurrent[phase] - g_I[phase];
  if (currentIntegral[phase] > CHARGE_CURRENT_INTEGRAL_LIMIT)
  {
    currentIntegral[phase] = CHARGE_CURRENT_INTEGRAL_LIMIT;
  }
  if (currentIntegral[phase] < -CHARGE_CURRENT_INTEGRAL_LIMIT)
  {
    currentIntegral[phase] = -CHARGE_CURRENT_INTEGRAL_LIMIT;
  }
  currentCorrection[phase] = currentIntegral[phase] / 100000;
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
