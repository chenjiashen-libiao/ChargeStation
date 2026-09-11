#ifndef UART_H
#define UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "board_config.h"

#if BOARD_UART_ENABLE

extern UART_HandleTypeDef huart;

void UART_Init(void);
HAL_StatusTypeDef UART_Send(const uint8_t *data, uint16_t length, uint32_t timeout);
HAL_StatusTypeDef UART_Receive(uint8_t *data, uint16_t length, uint32_t timeout);

#endif

#ifdef __cplusplus
}
#endif

#endif
