#ifndef WIRELESS_UPGRADE_H
#define WIRELESS_UPGRADE_H

#include <stdint.h>

void WirelessUpgrade_Init(void);
uint8_t WirelessUpgrade_Process(const uint8_t *request, uint8_t broadcast,
  uint8_t channel);

#endif