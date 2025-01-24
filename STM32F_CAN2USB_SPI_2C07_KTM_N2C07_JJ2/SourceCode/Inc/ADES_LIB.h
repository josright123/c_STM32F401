
#ifndef __ADES_LIB_H_
#define __ADES_LIB_H_

#include "main.h"

extern void LIB_CalculateSystemTime(uint8_t *sysTime);
extern uint8_t LIB_TxRxBq20z45(uint8_t rw, uint8_t device, uint8_t cmd, uint8_t length, uint8_t *data);
extern uint8_t LIB_MemByteReadWrite(uint8_t rw, uint8_t device, uint8_t *cmd, uint8_t length, uint8_t *data);
extern uint8_t LIB_Tx_I2C3_SIR(uint8_t device, uint8_t cmd, uint8_t length, uint8_t *data); 
extern uint8_t LIB_MemRead(uint8_t rw, uint8_t device, uint8_t cmd, uint8_t length, uint8_t *data);
extern uint8_t CRC_Check(uint8_t *crc_all, uint8_t length);
#endif  // __ADES_LIB_H_
