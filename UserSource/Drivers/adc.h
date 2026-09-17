#ifndef CHARGE_ADC_H
#define CHARGE_ADC_H

#include <stdint.h>

/* 充电采样数组尺寸 */
#define CHARGE_ADC_CHANNEL_COUNT       11  // 九路三相采样、VREFINT和温度
#define CHARGE_PHASE_COUNT             3   // U、V、W三相

extern volatile uint16_t g_ADC_ConvertedValue[CHARGE_ADC_CHANNEL_COUNT];
extern volatile int32_t g_out_Volt[CHARGE_PHASE_COUNT];
extern volatile int32_t g_I[CHARGE_PHASE_COUNT];
extern volatile uint16_t g_intvolt[CHARGE_PHASE_COUNT];
extern volatile int32_t g_ref_1v2_vol;
extern volatile int32_t g_temp;

void ADC_Init(void);

#endif