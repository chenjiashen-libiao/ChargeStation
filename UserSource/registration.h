#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <stdint.h>
#include <stm32f1xx_hal.h>

void Registration_Load(void);
HAL_StatusTypeDef Registration_Save(const uint8_t *data, uint16_t length);
uint8_t Registration_IsLegal(void);
const uint8_t *Registration_GetSN(void);

#endif