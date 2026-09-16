#ifndef WIRELESS_PROTOCOL_H
#define WIRELESS_PROTOCOL_H

#include <stdint.h>
#include <stm32f1xx_hal.h>

void WirelessProtocol_Init(void);
void WirelessProtocol_Process(void);
uint8_t WirelessProtocol_IsWorkChannel(void);
uint32_t WirelessProtocol_GetAddress(void);
HAL_StatusTypeDef WirelessProtocol_SaveAddress(uint32_t address);

#endif