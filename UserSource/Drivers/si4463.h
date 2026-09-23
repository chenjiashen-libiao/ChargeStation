#ifndef SI4463_H
#define SI4463_H

#include <stdint.h>

/* 无线协议对外可用的负载长度 */
#define SI4463_PACKET_LENGTH            16  // Home信道固定帧长度
#define SI4463_VARIABLE_MAX_PAYLOAD     60  // 工作信道最大业务负载

typedef struct
{
	uint16_t PartNumber;
	uint8_t ChipRevision;
	uint8_t RomId;
	uint8_t Ready;
} SI4463_Info;

uint8_t SI4463_Init(void);
uint8_t SI4463_IsReady(void);
uint8_t SI4463_GetHomeChannel(void);
uint8_t SI4463_ReadCurrentChannel(uint8_t *channel);
const SI4463_Info *SI4463_GetInfo(void);
uint8_t SI4463_SendPacket(const uint8_t *data, uint8_t length, uint8_t channel);
uint8_t SI4463_SendVariablePacket(const uint8_t *data, uint8_t length, uint8_t channel);
uint8_t SI4463_GetReceivedFrame(uint8_t *data, uint8_t *length, uint8_t *isVariable);
uint8_t SI4463_SetFixedMode(uint8_t channel);
uint8_t SI4463_SetVariableMode(uint8_t channel);
uint8_t SI4463_EnterReceive(void);
void SI4463_Process(void);
void SI4463_SetSharedLed(uint8_t enabled);

#endif