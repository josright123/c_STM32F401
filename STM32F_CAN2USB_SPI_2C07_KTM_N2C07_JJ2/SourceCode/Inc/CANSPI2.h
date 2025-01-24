#ifndef __CAN_SPI2_H
#define	__CAN_SPI2_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"
typedef union {
  struct {
    uint8_t idType;
    uint32_t id;
    uint8_t dlc;
    uint8_t data0;
    uint8_t data1;
    uint8_t data2;
    uint8_t data3;
    uint8_t data4;
    uint8_t data5;
    uint8_t data6;
    uint8_t data7;
  } frame;
  uint8_t array[14];
} uCAN2_MSG;

#define dSTANDARD_CAN_MSG_ID_2_0B 1
#define dEXTENDED_CAN_MSG_ID_2_0B 2

extern bool CANSPI2_Initialize(void);
extern void CANSPI2_Sleep(void);
extern uint8_t CANSPI2_Transmit(uCAN2_MSG *tempCanMsg);
extern uint8_t CANSPI2_Receive(uCAN2_MSG *tempCanMsg);
extern uint8_t CANSPI2_messagesInBuffer(void);
extern uint8_t CANSPI2_isBussOff(void);
extern uint8_t CANSPI2_isRxErrorPassive(void);
extern uint8_t CANSPI2_isTxErrorPassive(void);

#endif	/* __CAN_SPI2_H */
