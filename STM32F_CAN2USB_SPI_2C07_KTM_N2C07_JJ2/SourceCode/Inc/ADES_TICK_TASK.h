
#ifndef __ADES_TICK_TASK_H_
#define __ADES_TICK_TASK_H_

#include "main.h"

extern void TSK_ADCProcess(void);
extern void TSK_PowerProcess(void);
extern void TSK_HT66StateMachine(void);
extern void TSK_BatteryHT66StateMachine(void);
extern void TSK_VerifyBattery(void);
extern void TSK_CalculateSystemTime(void);
extern void TSK_FlashLed(uint8_t setFlashCount);
extern void TSK_FanFunction(void);
extern void TSK_BatteryAlgorithm(void);
extern void TSK_ReadHt66Parameter(void);
extern void TSK_CheckBatteryDischargeFunction(void);
extern void TSK_BQ25756Action(void);
extern void TSK_Bq24745Action(void);
extern void TSK_Bq20z45Action(void);
extern void TSK_Bq78350Action(void);
extern void TSK_TMS320F28032Action(void);
extern void TSK_MCP2515Action(void);
extern void I2C3_To_SIR_Write_Block(void);
extern void EEPROM_Action(void);
// FSP 650
extern void TSK_FSP650ReplayAll(void);

// Root 
extern void TSK_RootFunction(void);

#endif  // __ADES_TICK_TASK_H_
