
#ifndef __ADES_GLOBAL_VAR_H_
#define __ADES_GLOBAL_VAR_H_

#include "stm32f4xx_hal_adc.h" 
#include "stm32f4xx_hal_tim.h" 

#include "main.h"
#include "ADES_CFG.h"

// SYSTEM
extern uint32_t g_ulSysTick1MS;
extern uint32_t g_ulSysTick1MSFlag;
extern uint32_t g_ulSysTick1Sec;
extern uint16_t g_ausSysTickTaskRing[TICK_TASK_QUEUE_QUANTITY];
extern uint8_t  g_ucSysRingPtr;
extern uint8_t  g_ucSysRingTaskPtr;
extern uint8_t  g_ucSysTime[7];
extern uint8_t  g_ucSysMode; // For FSP500, 1:fsp550-70mua 0:fsp450-60mua
extern double   g_dSysWatt;

// Root
extern uint8_t  g_ucRootFlag;
extern uint8_t  g_ucRootFSP400ChargeManual;
extern uint8_t  g_ucRootFSP400FanManual;

// RESET
extern uint8_t g_ucResetFlag;

// LED
extern uint8_t  g_ucLedSysStatus;
extern uint8_t  g_ucLedFlashCount;
extern uint8_t  g_ucLedIdle;

// SYSTEM FLAG
extern uint32_t g_ulSysTickTaskFlag; 

// POWER & BATTERY
extern uint8_t   g_PowerOCP1;
extern uint8_t   g_PowerOCP2;
extern uint8_t   g_PowerOCP3;
extern uint8_t   g_PowerOCP4;
extern uint32_t  g_Mosfail_delay;
extern uint8_t   g_Mosfail_act;
extern uint32_t  VCC22;
extern uint32_t  VBATT;
extern uint8_t   g_PowerOVP1;
extern uint8_t   g_PowerOVP2;
extern uint8_t   g_PowerOVP3;
extern uint8_t   g_PowerUVP1;
extern uint8_t   g_PowerUVP2;
extern uint8_t   g_PowerOTP;
extern uint8_t   g_PowerOTP_Counter;
extern uint8_t   g_PowerUp;
//extern uint8_t   g_ACPowerUp;
//extern uint8_t   g_BATPowerUp;
extern uint8_t   g_ucBatteryError;
extern uint8_t   g_ucPwStateMachine;
extern uint8_t   g_ucAcFailFlag;           // charge flag 0 for charge process
extern uint8_t   g_ucBatteryType;
extern uint8_t   g_ucBatteryChargeMode;
extern uint8_t   g_ucBatteryChargeCC2CV; // 0:CC, 1:CV
extern uint8_t   g_ucBatteryChargeState; // 0:No Charge, 1:Normal Charge, 2: Fast Charge
extern uint8_t   g_ucBatteryChargeCount;
extern uint32_t  g_ulBatteryTime;
extern uint8_t   g_ucBatteryPercentage;
extern uint32_t   g_ucBatteryVoltage;
extern uint32_t   g_ucBatteryCurrent;
extern uint32_t   g_ucBatteryAverageCurrent;
extern uint8_t   g_ucBatteryLowState; // 1:Low, >2:Critical low
extern uint32_t  g_ulBatteryChargeCurrentLimit;

extern uint16_t   g_ucBatteryRemainingCapacity;
extern uint16_t   g_ucBatteryFullChargeCapacity;
extern uint16_t   g_ucBatteryAbsoluteStateOfCharge;
extern uint16_t   g_ucBatteryREMAINING_DISCHARGE_TIME;
extern uint16_t   g_ucBatteryCELL_VOLTAGE_1;
extern uint16_t   g_ucBatteryCELL_VOLTAGE_2;
extern uint16_t   g_ucBatteryCELL_VOLTAGE_3;
extern uint16_t   g_ucBatteryCELL_VOLTAGE_4;
extern uint16_t   g_ucBatteryCELL_VOLTAGE_5;

extern uint8_t   g_ucBatteryLowCurrentTimer;
extern uint8_t   g_ucBatteryChargeStartLiPo;
extern uint8_t   g_ucBatteryChargerShutdownVCC;
extern uint8_t   g_ucBatteryDischargeOcpFlag;
extern uint8_t   g_ucBatteryInsertEstimateTime;

//ROCK Info HT66 Battery_Full and if busy
extern uint16_t   g_ucBatteryFullFlag;
extern uint8_t    g_ucBatteryFullRechargeFlagCount;
extern uint8_t    g_ucBatteryRechargeFlag;

extern uint32_t  g_ulBatteryChargeTime;
extern uint16_t g_ulNormalBatteryChargeTime;
extern uint16_t  g_ulPreBatteryChargeTime;
extern uint32_t  g_ulBatteryChargerDelay; 
extern uint32_t  g_ulBatteryStopChargerTimer;
extern uint8_t  g_ulBatteryStopChargerErrCount; 
extern uint8_t  g_ulBatteryStopChargerErrCount_2;// fist 4 bit  for ocp times ,  last for 1 time counter
extern uint8_t   g_ucBatteryChargerState; 


extern uint8_t   g_ucBatterySystemOnCharge;

extern uint8_t   g_ucBatteryCheckFlag;
extern uint8_t   g_ucBatteryHaltChargeFlag;

extern uint8_t   g_ucVoltageMode;
extern uint32_t  g_ulAdc0AlarmLimit;
extern uint32_t  g_ulAdc1AlarmLimit;
extern uint32_t  g_ulAdc2AlarmLimit;
extern uint32_t  g_ulAdc2AlarmLimit1Pack;
extern uint32_t  g_ulAdc2AlarmLimit2Pack;
extern uint32_t  g_ulAdc3AlarmLimit;
extern uint32_t  g_ulAdc4AlarmLimit;
extern uint32_t  g_ulAdc5AlarmLimit;
extern uint32_t  g_ulAdc6AlarmLimitH;
extern uint32_t  g_ulAdc6AlarmLimitL;
extern uint32_t  g_ulAdc8AlarmLimitH;
extern uint32_t  g_ulAdc8AlarmLimitL;

// CHECK BATTERY DISCHARGE FUNCTION
extern uint8_t   g_ChkBatDischargeFun;
extern uint8_t   g_ChkBatDischargeFunCount;

// BQ20Z45 ACTION
extern uint8_t   g_ucBqStateMachine;
extern uint8_t   g_ucBqStateMachineStep;
extern uint8_t   g_ucBqStateMachineBatteryChannel;
extern uint8_t   g_ucBqStateMachineSemaphore;
extern uint8_t   g_ausBqBatteryState[4][2];
extern uint8_t   g_ausBqBatteryRemainingDischargingTime[2];
extern uint8_t   g_ausBqBatteryTemperature[4][2];
extern uint8_t   g_ausBqBatteryVoltage[4][2];
extern uint8_t   g_ausBqBatteryCurrent[4][2];
extern uint8_t   g_ausBqBatteryRelativeStateOfCharge[4][1];
extern uint8_t   g_ausBqBatteryAbsoluteStateOfCharge[4][1];
extern uint8_t   g_ausBqBatteryRemainingCapacity[4][2];
extern uint8_t   g_ausBqBatteryFullChargeCapacity[4][2];
extern uint32_t  g_ulBqChargingCurrent;
extern uint32_t  g_ulBqChargingVoltage;
extern uint32_t  g_ulOLDChargingVoltage;
extern uint32_t  g_ulBqBatteryTemperature;
extern uint8_t   g_ausBqBatteryCellVoltage8[2];
extern uint8_t   g_ausBqBatteryCellVoltage7[2];
extern uint8_t   g_ausBqBatteryCellVoltage6[2];
extern uint8_t   g_ausBqBatteryCellVoltage5[2];
extern uint8_t   g_ausBqBatteryCellVoltage4[4][2];
extern uint8_t   g_ausBqBatteryCellVoltage3[4][2];
extern uint8_t   g_ausBqBatteryCellVoltage2[4][2];
extern uint8_t   g_ausBqBatteryCellVoltage1[4][2];
extern uint32_t  g_ausBqBatteryUpdateSysTime[4][4];
extern uint8_t   g_ausBqBatteryDeviceChemistry[5];
extern uint8_t   g_ausBqBatteryAverageCurrent[2];
extern uint8_t   g_ausBqBatteryStatus[2];
extern uint8_t   g_ucCanCmdflag;
extern uint8_t   g_ucBqBatteryQuantity;
extern uint8_t   g_ucBqBatteryMultiMode;
extern uint8_t   g_ucBqBatteryMultiSelectChannel;
extern uint8_t   g_ucBqBatteryMultiChargerSemaphore;
extern uint8_t   g_ucBqBatteryMultiReadLagTime;
extern uint8_t   g_ucBqBatteryMultiCellExist;
extern uint8_t   g_ucBqBatteryMultiRetryCount;
extern uint8_t   g_ucBqBatteryMultiChargeGoal;
extern uint32_t  g_ulBqBatteryDelayTimer;

// BQ24745 ACTION 
extern uint32_t  g_ulBq24745DelayCmdTimer;
extern uint32_t  g_ulBq24745PrechargeTimer;
extern uint32_t  g_ulBq24745DelayTimer;
extern uint8_t  g_ucBq24745ChargerMode;
extern uint8_t  g_ucBq24745ChargerModeFlag[4];

// BQ24725A ACTION
extern uint8_t	 g_ucBQ24725AChargerOptions;

//CANSPI ACTION
extern uint8_t    g_ausCANID[2];
extern uint32_t   g_ulCANID;
extern uint8_t		g_CAN_channel;
extern uint8_t		g_ausCANDATA[64];
extern uint8_t    g_ausCAN1_DATA1[13];
extern uint8_t    g_ausCAN1_DATA2[13];
extern uint8_t    g_ausCAN1_DATA3[13];
extern uint8_t    g_ausCAN1_DATA4[13];
extern uint8_t    g_ausCAN2_DATA1[13];
extern uint8_t    g_ausCAN2_DATA2[13];
extern uint8_t    g_ausCAN2_DATA3[13];
extern uint8_t    g_ausCAN2_DATA4[13];
extern uint8_t		Read_CAN_Flag;
extern uint8_t    g_WriteCANID[3];
extern uint8_t		g_WriteCANDATA[64];
extern uint8_t		g_CANSPI_Receive;
extern uint8_t		g_PC_CANSPI_Receive;
extern uint8_t		g_rxCANcounter;
// USBD_HID
extern USBD_HandleTypeDef  g_hUSBDDevice;
extern uint8_t g_USBD_HID_OutFlag;
extern uint8_t g_USBD_HID_InFlag; 
extern uint8_t g_USBD_HID_DataOutBuf[USBD_HID_DATA_OUT_BUF_SIZE]; 
extern uint8_t g_USBD_HID_OutStream[USBD_HID_DATA_OUT_BUF_SIZE]; 
extern uint8_t g_USBD_HID_InStream[USBD_HID_DATA_OUT_BUF_SIZE]; 

// BBU 

extern uint8_t CHARGER_MODE; 
extern uint8_t Precharge_MODE; 
extern uint8_t OLD_CHARGER_MODE; 
extern uint8_t current_mode;   // Mode1
extern uint8_t old_LED_MODE;
// ADC
extern ADC_HandleTypeDef g_AdcHandle;
extern __IO uint16_t g_ausADCReg[18];
extern __IO uint16_t g_ausADCRegTemp[18];
extern uint16_t g_ausADCValue[9][1];
extern uint8_t g_ausADCValueOffset[9][2];
extern uint16_t g_sAdcVDDA; 
extern uint8_t g_ucAdcUVPDelay;
//rock set charge current value
extern uint16_t g_Battery_resmbus;
extern uint16_t g_Battery_resmbus_counter;
//
extern uint32_t g_ulPinStatus; 

extern uint16_t adc_value_pa3;
extern uint16_t adc_value_pa4;
extern uint16_t adc_value_pa5;
extern uint16_t adc_value_pa6;
// FAN PWM
extern TIM_HandleTypeDef g_TimHandle;
extern uint32_t g_ulFanDutyCycle, g_ulFanFrequency;
extern uint8_t  g_ucFanStopFlag;
extern uint8_t  g_ucFanPowerOffStopFlag;
//Rock Fan Line PWM Value
extern uint16_t  g_ucFanPwmControlValue;


// BEEP PWM
extern TIM_HandleTypeDef g_TimHandle_Beep;
extern TIM_HandleTypeDef g_TimHandle_Beep2;

extern uint8_t g_ucBeepPeriod;

//TIM2
extern TIM_HandleTypeDef htim2, htim4;

// TEMPERATURE
extern uint32_t g_ulTemperature[2];

// I2C
extern I2C_HandleTypeDef g_I2c1Handle, g_I2c3Handle;
extern uint8_t g_ucI2CTxBuffer[4];
extern uint8_t g_ucI2CRxBuffer[1];
extern uint8_t g_ucHT66ActionFlagMCU;                         
extern uint8_t g_ucHT66ActionFlagApp;                        
extern uint8_t g_ucHT66Semaphore;
extern uint8_t g_ucHT66StateMachine;
extern uint8_t g_ucHT66Command[4];
extern uint8_t g_ucHT66CommandMcu[9];
extern uint8_t g_ucHT66FlashReadParaFlag;
extern uint8_t g_ucHT66Flash0x40;
extern uint8_t g_ucHT66CmdDoubleFlag;

extern uint8_t g_ucBatteryI2CTxBuffer[4];
extern uint8_t g_ucBatteryHT66ActionFlagMCU;                
extern uint8_t g_ucBatteryHT66ActionFlagApp;                
extern uint8_t g_ucBatteryHT66Semaphore;
extern uint8_t g_ucBatteryHT66StateMachine;
extern uint8_t g_ucBatteryHT66Command[4];
extern uint8_t g_ucBatteryHT66CommandMcu[4];
extern uint8_t g_ucBatteryHT66CmdDoubleFlag;
//Rock add second I2C 
extern uint8_t g_ucBatteryHT66ActionDelayFlag;
//Rock add delay for battery i2c at start
extern uint8_t g_battery_act;
extern uint8_t g_battery_i2c;
//Benjamin add delay for battery pack i2c
extern uint16_t g_battery_delay;
extern uint32_t g_ulbatteryDelayTimer;
//Benjamin add delay for battery charger i2c write
extern uint16_t g_charger_delay;
extern uint16_t voutin;
extern uint16_t vboost2;
extern uint32_t Ben_Value_32_1;
extern uint32_t Ben_Value_32_2;
extern uint8_t g_Led_act;
extern uint8_t g_Led_delay;
extern uint8_t g_USB_act;

//Rock reini battery i2c
extern uint8_t g_ucReIniI2C1_Step;
extern uint32_t Rock_value_32;
extern uint16_t g_Rock_value[15];

//Rock add I2C shift flag
extern uint8_t g_ucHT66ActionFlagShift;
extern uint16_t TSK_BatteryHT66StateMachineCounter;
//Rock dealeay intit for en288
extern uint8_t DeleayFlag;



extern uint8_t g_PreChargeCurrentOnceCheck;
extern uint8_t g_PreChargeCurrentCounterGate;
extern uint8_t g_PreChargeCurrentPeriod;
extern uint16_t g_PreChargeCurrentGate;
extern uint8_t g_OcpCounter;
extern uint8_t g_OCP_happen;
extern uint8_t g_LEC_Counter;
//22 to 26   26 to 29
extern uint8_t g_change_voltage;


// UART
extern UART_HandleTypeDef g_UartHandle;

// TEST
extern uint32_t g_ulRFU;
extern uint8_t g_ucDebug[4];

// FSP 650
extern uint8_t g_ucFSP650Cmd;
extern uint8_t g_ucFSP650Countdown;
extern uint8_t STR_TX_Buffer[15];
extern uint8_t g_OfflineAuto;
extern uint8_t g_ReadEEPROMAddress[2];
extern uint8_t EEPROM_RX_Buffer[12];
extern uint8_t g_Auto_CAN_Broadcast;
extern uint8_t g_CANFlash;
extern uint8_t g_CAN1_receive;
extern uint8_t g_CAN2_receive;
extern uint8_t SIR_counter;
//

extern uint8_t g_ChargerVoltageFlag;  //
extern uint8_t g_ChargerVoltageFlag2;
extern uint8_t g_ucBatteryChargerStateCounter;

extern uint8_t CRC_ALL_Data[15];
extern uint32_t CRC_Data;
extern uint8_t  CRC_buffer[35];
extern uint32_t CRC8[16];
extern uint8_t CHARGER_Voltage_counter;
extern uint8_t CHARGER_0Current_counter;
extern uint8_t CHARGER_Voltage;
extern uint8_t CHARGER_26V_TIME;

extern uint16_t CHARGER_one_puls;
extern uint8_t g_Battery_Low;
extern uint16_t g_AdcAndValue;
extern uint16_t g_AdcAndValueCounter;

extern uint8_t Charger_power_50us_TIME;
extern uint8_t g_ulBatteryChargerDelay_offset;
extern uint8_t g_BI_ACT_Delay;

// SPI
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

#endif  // __ADES_GLOBAL_VAR_H_
