#ifndef SPI_H
#define SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "../../board_config.h"

#if BOARD_SPI_ENABLE

void SPI_Init(void);
void SPI_CS_Low(void);
void SPI_CS_High(void);
uint8_t SPI_TransferByte(uint8_t data);

#endif

#ifdef __cplusplus
}
#endif

#endif