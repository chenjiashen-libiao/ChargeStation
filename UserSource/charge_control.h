#ifndef CHARGE_CONTROL_H
#define CHARGE_CONTROL_H

#include <stdint.h>

void ChargeControl_Init(void);
void ChargeControl_Task(void const *argument);
void ChargeControl_Update(void);
void ChargeControl_SetUpgradeMode(uint8_t enabled);
int32_t ChargeControl_GetTargetCurrent(uint8_t phase);
uint8_t ChargeControl_IsPhaseEnabled(uint8_t phase);
uint8_t ChargeControl_IsMainEnabled(void);
uint8_t ChargeControl_IsMosEnabled(void);
uint8_t ChargeControl_HasBreakFault(void);

#endif
