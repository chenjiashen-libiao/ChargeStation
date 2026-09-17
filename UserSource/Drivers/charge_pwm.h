#ifndef CHARGE_PWM_H
#define CHARGE_PWM_H

#include <stdint.h>

/* TIM1中心对齐PWM参数，32MHz时钟下频率约22.22kHz */
#define CHARGE_PWM_PERIOD              719  // 自动重装值
#define CHARGE_PWM_INITIAL_COMPARE     550  // 初始化比较值
#define CHARGE_PWM_MIN_COMPARE         50   // 控制环最小比较值
#define CHARGE_PWM_MAX_COMPARE         718  // 控制环最大比较值

void ChargePWM_Init(void);
void ChargePWM_EnableMain(void);
void ChargePWM_DisableMain(void);
void ChargePWM_EnablePhase(uint8_t phase);
void ChargePWM_DisablePhase(uint8_t phase);
void ChargePWM_DisableAllPhases(void);
void ChargePWM_SetCompare(uint8_t phase, uint16_t compare);
uint8_t ChargePWM_IsMainEnabled(void);
uint8_t ChargePWM_IsPhaseEnabled(uint8_t phase);
uint8_t ChargePWM_HasBreakFault(void);
void ChargePWM_ClearBreakFault(void);
void ChargePWM_SetMosEnabled(uint8_t enabled);
uint8_t ChargePWM_IsMosEnabled(void);

#endif
