#ifndef CAN_H
#define CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "../../board_config.h"

extern CAN_HandleTypeDef hcan;

void CAN_Init(void);
HAL_StatusTypeDef CAN_SendByte(uint8_t data);
HAL_StatusTypeDef CAN_SendData(uint8_t *data, uint8_t len);
HAL_StatusTypeDef CAN_SendDataId(uint32_t id, uint8_t *data, uint8_t len);
uint8_t CAN_GetLastRx(uint32_t *id, uint8_t *len, uint8_t *dataBuf);
uint32_t CAN_GetErrorStatus(void);
uint8_t CAN_IsBusOff(void);
HAL_StatusTypeDef CAN_ReInitMode(uint32_t mode);
uint8_t CAN_LoopbackTest(void);  /* 回环测试 */

#ifdef __cplusplus
}
#endif

#endif