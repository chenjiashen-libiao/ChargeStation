#ifndef CHARGE_ADC_H
#define CHARGE_ADC_H

#include <stdint.h>

#define CHARGE_ADC_CHANNEL_COUNT 11U
#define CHARGE_PHASE_COUNT       3U

extern volatile uint16_t g_ADC_ConvertedValue[CHARGE_ADC_CHANNEL_COUNT];
extern volatile int32_t g_out_Volt[CHARGE_PHASE_COUNT];
extern volatile int32_t g_I[CHARGE_PHASE_COUNT];
extern volatile uint16_t g_intvolt[CHARGE_PHASE_COUNT];
extern volatile int32_t g_ref_1v2_vol;
extern volatile int32_t g_temp;

void ADC_Init(void);

#endif