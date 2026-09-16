#ifndef SI4463_H
#define SI4463_H

#include <stdint.h>

#define SI4463_PACKET_LENGTH              16U
#define SI4463_VARIABLE_MAX_PAYLOAD       60U
#define SI4463_HOME_CHANNEL               0U
#define SI4463_FREQUENCY_BASE_MHZ         0U
#define SI4463_FREQUENCY_STEP_KHZ         250U
#define SI4463_TX_POWER                   0x7FU

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

#endif