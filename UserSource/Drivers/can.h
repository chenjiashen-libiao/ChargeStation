#ifndef CAN_H
#define CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "../../board_config.h"

void CAN_Init(void);
HAL_StatusTypeDef CAN_SendDataId(uint32_t id, uint8_t *data, uint8_t len);
uint8_t CAN_GetLastRx(uint32_t *id, uint8_t *len, uint8_t *dataBuf);

#ifdef __cplusplus
}
#endif

#endif