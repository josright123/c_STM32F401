
/* Includes ------------------------------------------------------------------*/
#include "main.h"

// SYSTEM
uint32_t g_ulSysTick1MS  = 989;
uint32_t g_ulSysTick1MSFlag  = 0;
uint32_t g_ulSysTick1Sec = 0;
uint16_t g_ausSysTickTaskRing[TICK_TASK_QUEUE_QUANTITY];
uint8_t  g_ucSysRingPtr = 0;
uint8_t  g_ucSysRingTaskPtr = 0;
uint8_t  g_ucSysTime[7] = {0x14, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};
uint8_t  g_ucSysMode = 0; // For FSP500, 1:fsp550-70mua 0:fsp450-60mua
double   g_dSysWatt = 0;


uint16_t LowPowerCnt = 1000;
uint8_t  RUNNING_PROGRAM = 0;

// Root
uint8_t  g_ucRootFlag = 0;
uint8_t  g_ucRootFSP400ChargeManual = 0;
uint8_t  g_ucRootFSP400FanManual = 0;

// RESET
uint8_t g_ucResetFlag = 0;

// LED
uint8_t  g_ucLedSysStatus = LED_SYS_NO_BATTERY;
uint8_t  g_ucLedFlashCount = 0;
uint8_t  g_ucLedIdle = LED_IDLE_SEC;

// SYSTEM FLAG
uint32_t g_ulSysTickTaskFlag = 0; 

// POWER & BATTERY
uint8_t   g_PowerOCP1 = 0;
uint8_t   g_PowerOCP2 = 0;
uint8_t   g_PowerOCP3 = 0;
uint8_t   g_PowerOCP4 = 0;
uint32_t  g_Mosfail_delay = 0;
uint8_t   g_Mosfail_act = 0;
uint32_t  VCC22 = 0;
uint32_t  VBATT = 0;
uint8_t   g_PowerOVP1 = 0;
uint8_t   g_PowerOVP2 = 0;
uint8_t   g_PowerOVP3 = 0;
uint8_t   g_PowerUVP1 = 0;
uint8_t   g_PowerUVP2 = 0;
uint8_t   g_PowerOTP  = 0;
uint8_t   g_PowerOTP_Counter = 0;
uint8_t   g_PowerUp = 0;
//uint8_t   g_ACPowerUp = 0;
//uint8_t   g_BATPowerUp = 0;
uint8_t   g_ucBatteryError = 0;
uint8_t   g_ucPwStateMachine = PW_STEP_0;
uint8_t   g_ucAcFailFlag = 0;                // charge flag 0 for charge process
uint8_t   g_ucBatteryType = NO_BATTERY;
uint8_t   g_ucBatteryChargeMode = 3;
uint8_t   g_ucBatteryChargeCC2CV = 0; // 0:CC, 1:CV
uint8_t   g_ucBatteryChargeState = CHARGE_STOP; // 0:Charge Off, 1:Normal Charge, 2: Fast Charge
uint8_t   g_ucBatteryChargeCount = 0;
uint32_t  g_ulBatteryTime = 0;
uint8_t   g_ucBatteryPercentage = 0;
uint8_t   g_ucBatteryLowState = 0; // 1:Low, >2:Critical low
uint32_t   g_ucBatteryVoltage = 0;
uint32_t   g_ucBatteryCurrent = 0;

uint32_t   g_ucBatteryAverageCurrent = 0;
uint16_t   g_ucBatteryRemainingCapacity = 0;
uint16_t   g_ucBatteryFullChargeCapacity = 0;
uint16_t   g_ucBatteryAbsoluteStateOfCharge = 0;
uint16_t   g_ucBatteryREMAINING_DISCHARGE_TIME = 0;
 uint16_t   g_ucBatteryCELL_VOLTAGE_1=0;
 uint16_t   g_ucBatteryCELL_VOLTAGE_2=0;
 uint16_t   g_ucBatteryCELL_VOLTAGE_3=0;
 uint16_t   g_ucBatteryCELL_VOLTAGE_4=0;
 uint16_t   g_ucBatteryCELL_VOLTAGE_5=0;

//bit 0 1 2 3 10 13 un-use
//4   1 for battery no power   (discharge)
//5   1 for battery full       (charge)
//6   1 for discahrging       
//7   1 for working normal  
//8   1 for remaining time low
//9   1 for remainingcapacity low
//11  1 battery critical low   (capacity or voltage ?)
//12  1 over temperature 
//14  1 for battery charge stop
//15  1 for ocp ()
uint16_t g_ucBatteryStatus = 0 ; // 
//Rock SIR TX BUFBER
uint8_t STR_TX_Buffer[15];
uint8_t SIR_counter = 0;
uint8_t g_OfflineAuto = 0;
uint8_t g_ReadEEPROMAddress[2];
uint8_t EEPROM_RX_Buffer[12];
uint8_t g_Auto_CAN_Broadcast = 0;
uint8_t g_CANFlash = 0;
uint8_t g_CAN1_receive = 0;
uint8_t g_CAN2_receive = 0;
uint8_t   g_ucBatteryLowCurrentTimer = 0;//Digits ones for battery full /Digits tens for battery fail
uint8_t   g_ucBatteryChargeStartLiPo = 0;
uint8_t   g_ucBatteryChargerShutdownVCC = 0;
uint8_t   g_ucBatteryDischargeOcpFlag = 0;
uint8_t   g_ucBatteryInsertEstimateTime = BATTERY_ALG_TIMER_15_SEC;

//ROCK Info HT66 Battery_Full and if busy
uint16_t   g_ucBatteryFullFlag = 0;
uint8_t    g_ucBatteryFullRechargeFlagCount = 20;
uint8_t    g_ucBatteryRechargeFlag;
uint32_t  g_ulBatteryChargeTime = 0;                                    // 5HR
uint16_t  g_ulNormalBatteryChargeTime = 0;                              // for battery 0.5 A CC mode (normal mode)  
uint16_t  g_ulPreBatteryChargeTime = 1000;                              // for pluse chargeing // 1000 = init / 0~360 ocunt up
uint32_t  g_ulBatteryChargerDelay = 5010;
uint32_t  g_ulBatteryStopChargerTimer = 0;  
uint8_t  g_ulBatteryStopChargerErrCount = 0; 
uint8_t  g_ulBatteryStopChargerErrCount_2 = 0; 
uint8_t   g_ucBatteryChargerState = 0; // 0           0   0                  0      0      0             00 
                                       // 30 voltage  otp enable percentage  BIT15  5HR    ocp(light)    Err rate(count down time)
																			 //                 enable charge percentage(>90%)   ocp / battery fail (couldn't charge)
uint8_t   g_ucBatterySystemOnCharge = 0;

uint8_t   g_ucBatteryCheckFlag = 0;
uint8_t   g_ucBatteryHaltChargeFlag = 0;

#ifdef FSP400 
// >>> FSP 400 >>>>>>
uint8_t   g_ucVoltageMode = 0; // 0 = 18 V, 1 = 24 V, 2 = 15 V 
uint32_t  g_ulAdc0AlarmLimit;
uint32_t  g_ulAdc2AlarmLimit1Pack;
uint32_t  g_ulAdc2AlarmLimit2Pack;
uint32_t  g_ulAdc3AlarmLimit;

uint32_t  g_ulAdc1AlarmLimit; 
uint32_t  g_ulAdc5AlarmLimit;

uint32_t  g_ulAdc4AlarmLimit;
uint32_t  g_ulAdc6AlarmLimitH;
uint32_t  g_ulAdc6AlarmLimitL;
uint32_t  g_ulAdc8AlarmLimitH;
uint32_t  g_ulAdc8AlarmLimitL;

// <<<<<< FSP 400 <<<
#else
uint8_t   g_ucVoltageMode = 0;
uint32_t  g_ulAdc0AlarmLimit;
uint32_t  g_ulAdc1AlarmLimit;
uint32_t  g_ulAdc2AlarmLimit;
uint32_t  g_ulAdc3AlarmLimit;
uint32_t  g_ulAdc4AlarmLimit;
uint32_t  g_ulAdc5AlarmLimit;
uint32_t  g_ulAdc6AlarmLimitH;
uint32_t  g_ulAdc8AlarmLimitH;	
#endif

// CHECK BATTERY DISCHARGE FUNCTION
uint8_t   g_ChkBatDischargeFun = 0;
uint8_t   g_ChkBatDischargeFunCount = 0; // Check correct time

// BQ20Z45 ACTION
#ifdef FSP400 
uint8_t   g_ucBqStateMachine = BQ20Z45_CHECK_TCA9546A;
#else
uint8_t   g_ucBqStateMachine = BQ78350_SBS_TEMPERATURE;
uint8_t   g_ucBqStateMachineStep = 0;
#endif
uint8_t   g_ucBqStateMachineBatteryChannel = 0;
uint8_t   g_ucBqStateMachineSemaphore = 0;
// g_ausBqBatteryState[*][0] = 0:Normal, 1:Abnormal
// g_ausBqBatteryState[*][1] = Error count
uint8_t   g_ausBqBatteryState[4][2] = {0xFF}; 
uint8_t   g_ucCanCmdflag = 0;
uint8_t   g_ausBqBatteryTemperature[4][2] = {0};
uint8_t   g_ausBqBatteryVoltage[4][2] = {0};
uint8_t   g_ausBqBatteryCurrent[4][2] = {0};
uint8_t   g_ausBqBatteryAverageCurrent[2] = {0};
uint8_t   g_ausBqBatteryStatus[2] = {0};
uint8_t   g_ausBqBatteryRemainingDischargingTime[2] = {0};
uint8_t   g_ausBqBatteryRelativeStateOfCharge[4][1] = {0};
uint8_t   g_ausBqBatteryAbsoluteStateOfCharge[4][1] = {0};
uint8_t   g_ausBqBatteryRemainingCapacity[4][2] = {0};
uint8_t   g_ausBqBatteryFullChargeCapacity[4][2] = {0};
uint32_t  g_ulBqChargingCurrent = 0;
uint32_t  g_ulBqChargingVoltage = 0;
uint32_t  g_ulOLDChargingVoltage = 0;
uint32_t  g_ulBqBatteryTemperature = 0;
uint8_t   g_ausBqBatteryCellVoltage8[2] = {0};
uint8_t   g_ausBqBatteryCellVoltage7[2] = {0};
uint8_t   g_ausBqBatteryCellVoltage6[2] = {0};
uint8_t   g_ausBqBatteryCellVoltage5[2] = {0};
uint8_t   g_ausBqBatteryCellVoltage4[4][2] = {0};
uint8_t   g_ausBqBatteryCellVoltage3[4][2] = {0};
uint8_t   g_ausBqBatteryCellVoltage2[4][2] = {0};
uint8_t   g_ausBqBatteryCellVoltage1[4][2] = {0};
uint32_t  g_ausBqBatteryUpdateSysTime[4][4] = {0};
uint8_t   g_ausBqBatteryDeviceChemistry[5] = {0};


uint8_t   g_ucBqBatteryQuantity = 0;
uint8_t   g_ucBqBatteryMultiMode = 0;
uint8_t   g_ucBqBatteryMultiSelectChannel = 0;
uint8_t   g_ucBqBatteryMultiChargerSemaphore = 0;
uint8_t   g_ucBqBatteryMultiReadLagTime = 0;
uint8_t   g_ucBqBatteryMultiCellExist = 0;
uint8_t   g_ucBqBatteryMultiRetryCount = 0;
uint8_t   g_ucBqBatteryMultiChargeGoal = 0;
uint32_t  g_ulBqBatteryDelayTimer = 0;

// BQ24745 ACTION 
uint32_t  g_ulBq24745DelayCmdTimer = 0;
uint32_t  g_ulBq24745PrechargeTimer = 0;
uint32_t  g_ulBq24745DelayTimer = 0;
uint8_t  g_ucBq24745ChargerMode = 0;
uint8_t  g_ucBq24745ChargerModeFlag[4] = {0};

// BQ24725A ACTION
uint8_t		g_ucBQ24725AChargerOptions = 0;

//TMS320F28032 ACTION
uint8_t   g_ausCANID[2] = {0};
uint32_t  g_ulCANID = 0;
uint8_t   g_CAN_channel = 0;
uint8_t   g_ausCANDATA[64] = {0};
uint8_t   g_ausCAN1_DATA1[13] = {0};
uint8_t   g_ausCAN1_DATA2[13] = {0};
uint8_t   g_ausCAN1_DATA3[13] = {0};
uint8_t   g_ausCAN1_DATA4[13] = {0};
uint8_t   g_ausCAN2_DATA1[13] = {0};
uint8_t   g_ausCAN2_DATA2[13] = {0};
uint8_t   g_ausCAN2_DATA3[13] = {0};
uint8_t   g_ausCAN2_DATA4[13] = {0};
uint8_t		Read_CAN_Flag = 0;
uint8_t   g_WriteCANID[3] = {0};
uint8_t   g_WriteCANDATA[64] = {0};
uint8_t		g_CANSPI_Receive = 0;
uint8_t		g_PC_CANSPI_Receive = 0;
uint8_t 	g_rxCANcounter = 0;

// USBD_HID
USBD_HandleTypeDef  g_hUSBDDevice;
uint8_t g_USBD_HID_OutFlag = 0; 
uint8_t g_USBD_HID_InFlag = 0; 
uint8_t g_USBD_HID_DataOutBuf[USBD_HID_DATA_OUT_BUF_SIZE]; 
uint8_t g_USBD_HID_OutStream[USBD_HID_DATA_OUT_BUF_SIZE];  // PC to MCU 
uint8_t g_USBD_HID_InStream[USBD_HID_DATA_OUT_BUF_SIZE];   // MCU to PC


// BBU 

uint8_t CHARGER_MODE = 0; 
uint8_t Precharge_MODE = 0; 
uint8_t OLD_CHARGER_MODE = 9; 
uint8_t current_mode = Standby_LED;   // Mode1
uint8_t old_LED_MODE = 99;

// ADC


ADC_HandleTypeDef g_AdcHandle;
__IO uint16_t g_ausADCReg[18] = {0};
__IO uint16_t g_ausADCRegTemp[18] = {0};
uint16_t g_ausADCValue[9][1] = {0}; // [0]:Value, [1]:Offset     
uint8_t g_ausADCValueOffset[9][2] = {0}; // [0]:+, [1]:-    
uint16_t g_sAdcVDDA = 0;  
uint8_t g_ucAdcUVPDelay = 0;   
//rock set charge current value
uint16_t g_Battery_resmbus;
uint16_t g_Battery_resmbus_counter;

uint16_t adc_value_pa3 = 0;
uint16_t adc_value_pa4 = 0;
uint16_t adc_value_pa5 = 0;
uint16_t adc_value_pa6 = 0;
// FAN PWM
TIM_HandleTypeDef g_TimHandle;
uint32_t g_ulFanDutyCycle, g_ulFanFrequency;
uint8_t  g_ucFanStopFlag;
uint8_t  g_ucFanPowerOffStopFlag;
uint16_t  g_ucFanPwmControlValue;

// BEEP PWM
TIM_HandleTypeDef g_TimHandle_Beep2;
TIM_HandleTypeDef g_TimHandle_Beep;
uint8_t g_ucBeepPeriod = 0;

//TIM2
TIM_HandleTypeDef htim2, htim4;

// TEMPERATURE
uint32_t g_ulTemperature[2];

// I2C
I2C_HandleTypeDef g_I2c1Handle, g_I2c3Handle;
uint8_t g_ucI2CTxBuffer[4];
uint8_t g_ucI2CRxBuffer[1];
uint8_t g_ucHT66ActionFlagMCU = 0;                         // MCU routine task  
uint8_t g_ucHT66ActionFlagApp = 0;                         // Application task  
uint8_t g_ucHT66Semaphore = 0;
uint8_t g_ucHT66StateMachine = 0;
uint8_t g_ucHT66Command[4];
uint8_t g_ucHT66CommandMcu[9];
uint8_t g_ucHT66FlashReadParaFlag = 0;
uint8_t g_ucHT66Flash0x40 = 0;
uint8_t g_ucHT66CmdDoubleFlag = 0;

uint8_t g_ucBatteryI2CTxBuffer[4];
uint8_t g_ucBatteryHT66ActionFlagMCU = 0;                  // MCU routine task  
uint8_t g_ucBatteryHT66ActionFlagApp = 0;                  // Application task  
uint8_t g_ucBatteryHT66Semaphore = 0;
uint8_t g_ucBatteryHT66StateMachine = 0;
uint8_t g_ucBatteryHT66Command[4];
uint8_t g_ucBatteryHT66CommandMcu[4];
uint8_t g_ucBatteryHT66CmdDoubleFlag = 0;
//Rock add second i2c flag
uint8_t g_ucBatteryHT66ActionDelayFlag = 0;
//Rock add delay for battery i2c at start
uint8_t g_battery_act = 0;
uint8_t g_battery_i2c = 0;
//Benjamin add delay for battery pack i2c
uint16_t g_battery_delay = 0;
uint32_t g_ulbatteryDelayTimer = 0;
//Benjamin add delay for battery charger i2c write
uint16_t g_charger_delay = 0;
//Rock add I2C shift flag
uint8_t g_ucHT66ActionFlagShift = 0;
uint16_t TSK_BatteryHT66StateMachineCounter = 50;
//Rock deleay initfor en288
uint8_t DeleayFlag = 0;
//Rock
uint32_t Rock_value_32;
uint16_t g_Rock_value[15];
//Ben
uint32_t Ben_Value_32_1;
uint32_t Ben_Value_32_2;
uint8_t g_Led_act = 0;
uint8_t g_Led_delay = 0;
uint8_t g_USB_act = 0;

uint16_t voutin = 0;
uint16_t vboost2 = 0;
uint8_t g_PreChargeCurrentOnceCheck = 10;
uint8_t g_PreChargeCurrentCounterGate = 0;
uint8_t g_PreChargeCurrentPeriod = 0;
uint16_t g_PreChargeCurrentGate = 210;
uint8_t g_OcpCounter = 0;
uint8_t g_OCP_happen = 0;
uint8_t g_LEC_Counter = 0;

//22 to 26   26 to 29
uint8_t g_change_voltage = 0;
//Rock REini battery i2c
uint8_t g_ucReIniI2C1_Step = 0;

// UART
UART_HandleTypeDef g_UartHandle;

//
uint32_t g_ulPinStatus; 

// TEST
uint32_t g_ulRFU;
uint8_t g_ucDebug[4];

// FSP 650
uint8_t g_ucFSP650Cmd = 0;
uint8_t g_ucFSP650Countdown = 60;

//rock enable charger
uint8_t g_ChargerVoltageFlag = 0;
uint8_t g_ChargerVoltageFlag2 = 0;

uint8_t g_ucBatteryChargerStateCounter = 0;
//

uint8_t CRC_ALL_Data[15];
uint32_t CRC_Data =0;
uint8_t  CRC_buffer[35];
uint32_t CRC8[16] = {0x000, 0x107, 0x20E, 0x309, 0x41C, 0x51b, 0x612, 0x715, 0x838, 0x93f, 0xa36, 0xb31, 0xc24, 0xd23, 0xe2a, 0xf2d }; 

uint8_t CHARGER_Voltage_counter = 0;
uint8_t CHARGER_0Current_counter = 0; // faster charger

uint8_t CHARGER_Voltage = 0;
uint8_t CHARGER_26V_TIME = 20;

uint16_t CHARGER_one_puls = 0;
//0  undo
//1 ~ 35535  puls
// >  35535  lock

uint8_t g_Battery_Low = 0;
// reset ADC

uint16_t g_AdcAndValue = 0;
uint16_t g_AdcAndValueCounter = 0;

//
uint8_t Charger_power_50us_TIME = 255;
uint8_t g_ulBatteryChargerDelay_offset = 0;
uint8_t g_BI_ACT_Delay = 0;

// SPI
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
