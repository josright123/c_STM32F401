
#include "main.h"
#include "CANSPI2.h"
uint8_t g_loop_t = 0;

void EN_charging(uint8_t EN_CHG);
extern void set_led_mode(uint8_t mode);
//
void TSK_ADCProcess(void)
{
     /* RM200
      * g_ausADCValue[0][0] =  g_ausADCReg[0];  PA0  VBATT_IN2
      * g_ausADCValue[1][0] =  g_ausADCReg[2];  PA1  +24V_CSNS_AUX
      * g_ausADCValue[2][0] =  g_ausADCReg[4];  PA2  +24V_PRNCSNS_AUX
      * g_ausADCValue[3][0] =  g_ausADCReg[6];  PA3  DATA_TEMP2
      * g_ausADCValue[4][0] =  g_ausADCReg[8];  PA4  ACDC_VOUT_IN2
      * g_ausADCValue[5][0] = g_ausADCReg[10];  PA5  +12V1_CSNS_AUX
      * g_ausADCValue[6][0] = g_ausADCReg[12];  PA6  DCDC_MOSFAIL (BAT_ID)
      * g_ausADCValue[7][0] = g_ausADCReg[14];  PA7  FAN SPEED
      * g_ausADCValue[8][0] = g_ausADCReg[16];  PB0  ENATXPOWER_N_AUX (NO USE
      */
    // averger ADC IN
    g_ausADCValue[0][0] = (g_ausADCValue[0][0] * FACTOR_X1 + (g_ausADCReg[0] + g_ausADCValueOffset[0][0] - g_ausADCValueOffset[0][1]) * FACTOR_X2) / 10;
    g_ausADCValue[1][0] = (g_ausADCValue[1][0] * FACTOR_X1 + (g_ausADCReg[2] + g_ausADCValueOffset[1][0] - g_ausADCValueOffset[1][1]) * FACTOR_X2) / 10;
    g_ausADCValue[2][0] = (g_ausADCValue[2][0] * FACTOR_X1 + (g_ausADCReg[4] + g_ausADCValueOffset[2][0] - g_ausADCValueOffset[2][1]) * FACTOR_X2) / 10;
    g_ausADCValue[3][0] = (g_ausADCValue[3][0] * FACTOR_X1 + (g_ausADCReg[6] + g_ausADCValueOffset[3][0] - g_ausADCValueOffset[3][1]) * FACTOR_X2) / 10;
    g_ausADCValue[4][0] = (g_ausADCValue[4][0] * FACTOR_X1 + (g_ausADCReg[8] + g_ausADCValueOffset[4][0] - g_ausADCValueOffset[4][1]) * FACTOR_X2) / 10;
    g_ausADCValue[5][0] = (g_ausADCValue[5][0] * FACTOR_X1 + (g_ausADCReg[10] + g_ausADCValueOffset[5][0] - g_ausADCValueOffset[5][1]) * FACTOR_X2) / 10;
    g_ausADCValue[6][0] = (g_ausADCValue[6][0] * FACTOR_X1 + (g_ausADCReg[12] + g_ausADCValueOffset[6][0] - g_ausADCValueOffset[6][1]) * FACTOR_X2) / 10;
    g_ausADCValue[7][0] = (g_ausADCValue[7][0] * FACTOR_X1 + (g_ausADCReg[14] + g_ausADCValueOffset[7][0] - g_ausADCValueOffset[7][1]) * FACTOR_X2) / 10;
    g_ausADCValue[8][0] = (g_ausADCValue[8][0] * FACTOR_X1 + (g_ausADCReg[16] + g_ausADCValueOffset[8][0] - g_ausADCValueOffset[8][1]) * FACTOR_X2) / 10;

    // Temperature
    g_ulTemperature[0] = g_ausADCValue[3][0];      // Sensor
    g_ulTemperature[1] = g_ausADCValue[6][0];      // MCU
		
    // ADC start convert 
    HWF_ADCStartConvert();
}

//
void TSK_PowerProcess(void)
{
    if (g_ulSysTickTaskFlag & SYS_TICK_TASK_6)
    {
        //
        g_ulSysTickTaskFlag -= SYS_TICK_TASK_6;

        // Battery exsit or not
        if (g_ucBatteryType == NO_BATTERY)
        {
            g_ucLedSysStatus = LED_SYS_NO_BATTERY;
            return;
        }
        //
        if (g_ucPwStateMachine == PW_STEP_4)
            return;

        //
        HAL_NVIC_DisableIRQ(EXTI2_IRQn);
        HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
        switch (g_ucPwStateMachine)
        {
            // Check [AC_FAIL_N_MCU1]
            case PW_STEP_0:
                if (AC_FAIL_N_MCU1 > 0)
                    g_ucPwStateMachine = PW_STEP_1;
                else
                    ;//g_ucLedSysStatus = LED_SYS_AC_FAIL;
                break;

            // Check [ACINOK_N_MCU1]
            case PW_STEP_1:
                if (ACINOK_N_MCU1 == 0)
                    g_ucPwStateMachine = PW_STEP_2;
                else
                    ;//g_ucLedSysStatus = LED_SYS_ACINOK;
                break;

            // Enable [EN_CHARGER_POWER_H]
            case PW_STEP_2:
                //EN_CHARGER_POWER_H;
                g_ucPwStateMachine = PW_STEP_3;
                //g_ucLedSysStatus = LED_BAT_FULL;
                break;

            // Enable [EN_CHARGER_H]
            case PW_STEP_3:
                //EN_CHARGER_H;
                g_ucPwStateMachine = PW_STEP_4;
                break;

            // Finish process
            case PW_STEP_4:
                g_ucAcFailFlag = 0;
                break;

            default:
                g_ucPwStateMachine = PW_STEP_0;
                break;
        }
        HAL_NVIC_EnableIRQ(EXTI2_IRQn);
        //HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        //HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    }
}

//
void TSK_HT66StateMachineErr(void)
{
    if (g_ucHT66Semaphore == 2) // Application 
    {
        g_ucHT66ActionFlagApp = 0;
        g_USBD_HID_InStream[1] = 1; // Fail
        g_USBD_HID_InFlag = 1;
    }
    else
        g_ucHT66ActionFlagMCU = 0;
    g_ucHT66StateMachine = HT66_STEP_0;
    g_ucHT66Semaphore = 0;
}

//
void TSK_HT66StateMachine(void)
{
    uint8_t device;

    if (g_ucHT66Semaphore > 0)
    {
        //  
        device = g_ucI2CTxBuffer[0];
        //
        switch (g_ucHT66StateMachine)
        {
            // Send [COMM]
            case HT66_STEP_0:
                HWF_ReInitI2C3Pin();
                if (HAL_I2C_Master_Transmit(&g_I2c3Handle, (uint16_t)device, (uint8_t*)&g_ucI2CTxBuffer[1], 1, 3) == HAL_OK) // Length = 1, Wait time = 3ms
                    g_ucHT66StateMachine = HT66_STEP_1;
                else
                    TSK_HT66StateMachineErr();
                break;

            // Send [ADDR]
            case HT66_STEP_1:
                HWF_ReInitI2C3Pin();
                if (HAL_I2C_Master_Transmit(&g_I2c3Handle, (uint16_t)device, (uint8_t*)&g_ucI2CTxBuffer[2], 1, 3) == HAL_OK) // Length = 1, Wait time = 3ms
                    g_ucHT66StateMachine = HT66_STEP_2;
                else
                    TSK_HT66StateMachineErr();
                break;

            // Send [DATA]
            case HT66_STEP_2:
                HWF_ReInitI2C3Pin();
                if (HAL_I2C_Master_Transmit(&g_I2c3Handle, (uint16_t)device, (uint8_t*)&g_ucI2CTxBuffer[3], 1, 3) == HAL_OK) // Length = 1, Wait time = 3ms
                {
                    if ((g_ucI2CTxBuffer[1] == HT66_CMD_0x54) || (g_ucI2CTxBuffer[1] == HT66_CMD_0x5D))
                    {
                        if (g_ucHT66Semaphore == 2) // Application 
                        {
                            g_ucHT66ActionFlagApp = 0;
                            g_USBD_HID_InFlag = 1;
                        }
                        else
                        {
                            g_ucHT66ActionFlagMCU = 0;
                            if (g_ucHT66CmdDoubleFlag == 1)
                            {
                                g_ucHT66ActionFlagMCU = 1;
                                g_ucHT66CmdDoubleFlag = 0;
                                break;
                            }

                        }
                        g_ucHT66StateMachine = HT66_STEP_0;
                        g_ucHT66Semaphore = 0; // sending end
                    }
                    else
                        g_ucHT66StateMachine = HT66_STEP_3;

                }
                else
                    TSK_HT66StateMachineErr();
                break;

            // Receive [DATA]
            case HT66_STEP_3:
                HWF_ReInitI2C3Pin();
                if (HAL_I2C_Master_Receive(&g_I2c3Handle, (uint16_t)device, (uint8_t*)&g_ucI2CRxBuffer[0], 1, 3) == HAL_OK) // Length = 1, Wait time = 3ms
                {
                    if (g_ucHT66Semaphore == 2) // Application 
                    {
                        g_ucHT66ActionFlagApp = 0;
                        g_USBD_HID_InStream[2] = g_ucI2CRxBuffer[0];
                        g_USBD_HID_InFlag = 1;
                        if (g_ucI2CTxBuffer[2] == HT66_ADDR_0x40)
                            g_ucHT66Flash0x40 = g_ucI2CRxBuffer[0];
                    }
                    else
                    {
                        g_ucHT66ActionFlagShift = 0;
                        if (g_ucI2CTxBuffer[2] == HT66_ADDR_0x40)
                            g_ucHT66Flash0x40 = g_ucI2CRxBuffer[0];
                        if (g_ucI2CTxBuffer[2] == HT66_ADDR_0x7B || g_ucI2CTxBuffer[2] == HT66_ADDR_0x7C)
                        {
                            g_ucBatteryI2CTxBuffer[1]++;
                            g_ucBatteryI2CTxBuffer[0] = g_ucI2CRxBuffer[0];
                        }
                    }
                    g_ucHT66StateMachine = HT66_STEP_0;
                    g_ucHT66Semaphore = 0;
                }
                else
                    TSK_HT66StateMachineErr();
                break;

            //
            default:
                break;
        }
    }
    else
    {

        if (g_ucHT66ActionFlagApp == 1)
        {
            g_ucHT66Semaphore = 2; // From Application request 
            g_ucHT66StateMachine = HT66_STEP_0;
            memcpy(g_ucI2CTxBuffer, g_ucHT66Command, 4);
        }
        else if (g_ucHT66ActionFlagMCU == 1)
        {
            if (g_ucHT66CommandMcu[4] == 1) // set double flag
            {
                g_ucHT66CommandMcu[4] = 0;
                g_ucHT66CmdDoubleFlag = 1;
            }
            g_ucHT66Semaphore = 3; // From MCU request
            g_ucHT66StateMachine = HT66_STEP_0;
            memcpy(g_ucI2CTxBuffer, g_ucHT66CommandMcu, 4);
        }
        else if (g_ucHT66ActionFlagShift == 1)
        {
            g_ucHT66Semaphore = 3; // From MCU request
            g_ucHT66StateMachine = HT66_STEP_0;
            memcpy(g_ucI2CTxBuffer, &g_ucHT66CommandMcu[5], 4);
        }

    }
}

//
void TSK_BatteryHT66StateMachineErr(void)
{

}

//
void TSK_BatteryHT66StateMachine(void)  // for check if ht66 I2C Shift
{
    if (g_ulSysTickTaskFlag & SYS_TICK_TASK_3)
    {
        g_ulSysTickTaskFlag -= SYS_TICK_TASK_3;
        //info battery full

        if (g_ucBatteryFullFlag > 0)       //      decide info or not
        {
            if (g_ucHT66ActionFlagMCU == 0)
            {
                g_ucHT66ActionFlagMCU = 1;
                g_ucHT66CommandMcu[0] = HT66_ADDRESS;
                g_ucHT66CommandMcu[1] = HT66_CMD_0x5D;
                g_ucHT66CommandMcu[2] = HT66_ADDR_0x03;
                g_ucHT66CommandMcu[3] = 0x0F;
                g_ucHT66CommandMcu[4] = 1;
                g_ucBatteryFullFlag = 0;
                return;
            }
        }
        // info battery recharge
        else if (g_ucBatteryRechargeFlag > 0)       //      decide recharge or not
        {
            if (g_ucHT66ActionFlagMCU == 0)
            {
                g_ucHT66ActionFlagMCU = 1;
                g_ucHT66CommandMcu[0] = HT66_ADDRESS;
                g_ucHT66CommandMcu[1] = HT66_CMD_0x5D;
                g_ucHT66CommandMcu[2] = HT66_ADDR_0x03;
                g_ucHT66CommandMcu[3] = 0xF0;
                g_ucHT66CommandMcu[4] = 1;
                g_ucBatteryRechargeFlag = 0;
                return;
            }
        }

        if (TSK_BatteryHT66StateMachineCounter < 60)
        {
            TSK_BatteryHT66StateMachineCounter++;
            return;
        }

        switch (g_ucBatteryI2CTxBuffer[1])
        {  // if finish first round 55 
            case 1:
                {
                    if (g_ucBatteryI2CTxBuffer[0] != 0x55)
                    {
                        //vboost2 = g_ucBatteryI2CTxBuffer[0];
                        HAL_I2C_Master_Transmit(&g_I2c3Handle, (uint16_t)HT66_ADDRESS, (uint8_t*)g_ucBatteryI2CTxBuffer, 1, 3);
                    }
                    g_ucHT66ActionFlagShift = 1;
                    g_ucHT66CommandMcu[5] = HT66_ADDRESS;
                    g_ucHT66CommandMcu[6] = HT66_CMD_0x52;
                    g_ucHT66CommandMcu[7] = HT66_ADDR_0x7C;
                    g_ucHT66CommandMcu[8] = 0x0A;
                    break;
                }
            // if finish second round 17
            case 2:
                {
                    g_ucBatteryI2CTxBuffer[1] = 0;
                    if (g_ucBatteryI2CTxBuffer[0] != 0x18)//shift
                    {
                        //vboost2 = g_ucBatteryI2CTxBuffer[0];
                        HAL_I2C_Master_Transmit(&g_I2c3Handle, (uint16_t)HT66_ADDRESS, (uint8_t*)g_ucBatteryI2CTxBuffer, 1, 3);
                    }
                    else
                        TSK_BatteryHT66StateMachineCounter = 0;
                    break;
                }
            default:
                {
                    g_ucBatteryI2CTxBuffer[1] = 0;
                    g_ucHT66ActionFlagShift = 1;
                    g_ucHT66CommandMcu[5] = HT66_ADDRESS;
                    g_ucHT66CommandMcu[6] = HT66_CMD_0x52;
                    g_ucHT66CommandMcu[7] = HT66_ADDR_0x7B;
                    g_ucHT66CommandMcu[8] = 0x0A;
                }
        }
								
		}
}

//
void TSK_VerifyBattery(void)
{

    //if (BATTERY_INSERT_N1 > 0) // Battery exist or not 
    {
        if (g_ausBqBatteryState[0][1] < 15)
        {
            g_ucBatteryType = LI_PO_BATTERY;
            STR_TX_Buffer[12] |= BATTERY_STATUS2_OK_SMBUS;
        }
        else
        {
						STR_TX_Buffer[12] &= BATTERY_STATUS2_ERR_SMBUS;
            if (g_ucBatteryChargeState != CHARGE_FAST)
            {
							if(CHARGER_one_puls == 0 && AC_FAIL_N_MCU1 == 1)
								 {
									 g_ucBatteryType = LI_PO_BATTERY;
									 CHARGER_one_puls = 1;
									 g_ausBqBatteryState[0][1] = 0;
								 }
								 else if(CHARGER_one_puls > 34)
								 {
									 if(g_Battery_resmbus_counter > 200)
									 {
										 g_Battery_resmbus_counter = 0;
										 g_ucBatteryType = NO_BATTERY;
										 if (PSON_MCU1 == 0 && g_ausADCReg[8] > 2964) //info HT66 turn off charger  
										 {
												 if (g_ucBatteryFullRechargeFlagCount == 0)
												 {
														 g_ucBatteryFullFlag = 1;
														 g_ucBatteryFullRechargeFlagCount = 20;
												 }
												 else
														 g_ucBatteryFullRechargeFlagCount--;
										 }
									 }
						   	 }
            }

            g_Battery_resmbus++;
						g_Battery_resmbus_counter++;
            if (g_Battery_resmbus > 50)   //Re-SMBus Check Period 5 Sec //20200316
            {
                g_Battery_resmbus = 0;
                if (LIB_MemRead(0,
                   BQ78350_I2C_ADDRESS,
                   BQ78350_SBS_BATTERY_STATUS,
                   2,
                   g_ausBqBatteryStatus) == 0)

                {
                    g_ausBqBatteryState[0][1] = 0;
                    g_Battery_resmbus = 0;
										g_Battery_resmbus_counter = 0;
                }
                //else
                    //g_ucBatteryType = NO_BATTERY;
            }

        }
    }
		/*
    else
    {		g_ulBatteryStopChargerErrCount_2 = 0;
        g_ulBatteryStopChargerErrCount = 0;
			  g_ucBatteryChargerState = 3; // 0000 0011
        g_ucBatteryType = NO_BATTERY;
    }
		*/
}

//
void TSK_CalculateSystemTime(void)
{
    //  uint8_t ucI2CTxBuffer[5];

    if (g_ulSysTickTaskFlag & SYS_TICK_TASK_1)
    {
        //
        g_ulSysTickTaskFlag -= SYS_TICK_TASK_1;

        //
        g_ulSysTick1Sec++;
        LIB_CalculateSystemTime(g_ucSysTime);
    }
}

//
void TSK_FlashLed(uint8_t setFlashCount)
{
	  if (g_ulSysTickTaskFlag & SYS_TICK_TASK_2)
    {
        //
        g_ulSysTickTaskFlag -= SYS_TICK_TASK_2;
			
						/*RLED_INDICATOR_L;
						g_Led_delay = 2;
						if (g_Led_act < g_Led_delay)
						{
							g_Led_act++;
							LED_INDICATOR_H;
						}
						else
						{
							g_Led_act = 0;
							LED_INDICATOR_L;
						}*/
				/*switch(g_ucLedSysStatus)
				{
					case LED_SYS_NO_BATTERY:
						LED_INDICATOR_L;
						//RLED_INDICATOR_H;
						g_Led_delay = 2;
						if (g_Led_act < g_Led_delay)
						{
							g_Led_act++;
							RLED_INDICATOR_H;
						}
						else
						{
							g_Led_act = 0;
							RLED_INDICATOR_L;
						}
						break;
					
					case LED_BAT_NORMAL_CHARGE:
						RLED_INDICATOR_L;
						g_Led_delay = 2;
						if (g_Led_act < g_Led_delay)
						{
							g_Led_act++;
							LED_INDICATOR_H;
						}
						else
						{
							g_Led_act = 0;
							LED_INDICATOR_L;
						}
					break;
						
					default:
					LED_INDICATOR_L;
					break;
				}*/
				
				RLED_INDICATOR_L;
				g_Led_delay = 2;
				if (g_Led_act < g_Led_delay)
				{
					g_Led_act++;
					LED_INDICATOR_H;
				}
				else
				{
					g_Led_act = 0;
					LED_INDICATOR_L;
				}

		}
}

//
void TSK_FanFunction(void)
{
		
    if (g_ulSysTickTaskFlag & SYS_TICK_TASK_4)
    {
        //
        g_ulSysTickTaskFlag -= SYS_TICK_TASK_4;

        if (DeleayFlag == 0)
            return;

        if (g_ausADCReg[6] > SENSOR_TEMPERATURE_50)
        {
            if (AC_FAIL_N_MCU1 == 0)
                g_ucFanPwmControlValue = 3454.6 - 0.7115 * g_ausADCReg[4];
            else   //ACmode
                g_ucFanPwmControlValue = 3619 - 0.9268 * g_ausADCReg[10];
            // Rock fan limit
            if (g_ucFanPwmControlValue < 1575)
                g_ucFanPwmControlValue = 1575;
            if (g_ucFanPwmControlValue > 2950)
                g_ucFanPwmControlValue = 2950;
            //Rock fan set
        }
        else
        {
            g_ucFanPwmControlValue = 5.5682 * g_ausADCReg[6] - 4483.2;
            if (g_ausADCReg[6] < 868)
                g_ucFanPwmControlValue = 350;
        }
				if(PSON_MCU1 ==0 &&CHARGER_Voltage <23)
					g_ucFanPwmControlValue = 3500;
	       
				__HAL_TIM_SetCompare(&g_TimHandle_Beep, TIM_CHANNEL_2, g_ucFanPwmControlValue);
        //
        g_ucFanStopFlag++;
        if (g_ucFanStopFlag > 1)
        {
            g_ucFanStopFlag = 2;
            g_ulFanDutyCycle = 0;
            g_ulFanFrequency = 0;
        }
    }
}

//
void TSK_BatteryAlgorithm(void)
{
    if (g_ulSysTickTaskFlag & SYS_TICK_TASK_5)
    {
        //
        g_ulSysTickTaskFlag -= SYS_TICK_TASK_5;
        // 
        if (g_ucRootFSP400ChargeManual != 0)
            return;
        //
        if (AC_FAIL_N_MCU1 == 0)      // No AC Algorithm
        {
            if (g_ucAcFailFlag > 1)   //Rock AC mode to DC mode 
            {
                g_ucBatteryChargeState = CHARGE_STOP;
                g_ucBatteryChargeMode = BATTERY_STOP_CHARGE_MODE;
                switch (g_ucBatteryType)
                {
                    case NO_BATTERY:
                        //EN_FAST_CHARGER_H;
                        //EN_CHARGER_L; // Disable
                        //EN_CHARGER_POWER_L;
                        g_Rock_value[7] |= 1;
                        g_ulBatteryChargerDelay = ENABLE_CHARGER_DELAY_TIME;
                        g_ulBatteryStopChargerTimer = 0;
                        g_ulBatteryStopChargerErrCount = 0;
                        g_ulBatteryChargeTime = 0;
                        //g_ucBatteryPercentage = 0;
                        g_ucBatteryChargeStartLiPo = 0;
                        g_ucBatteryInsertEstimateTime = BATTERY_ALG_TIMER_15_SEC;
                        g_ucBatteryLowState = 0;
                        g_ucBatteryDischargeOcpFlag = 0;

                        g_ucBqStateMachineSemaphore = 0;
                        g_ucBq24745ChargerMode = BQ24745_MODE_L0;

                        g_ulBqChargingCurrent = 0;
                        g_ulBqChargingVoltage = 0;

                        g_ucBqBatteryMultiReadLagTime = 0;
                        g_ulBqBatteryDelayTimer = 0;
                        g_ucBqBatteryMultiChargerSemaphore = 0;

                        g_ucBeepPeriod = 0;

                        break;

                    case NIMH_BATTERY:
                        break;

                    case LEAD_ACID_BATTERY:
                        BAT_DischargeAlgorithmLeadAcid();
                        break;

                    case LI_PO_BATTERY:
                        BAT_DischargeAlgorithmLiPo();
                        break;

                    case LI_ION_BATTERY:
                        ////              g_ucBqStateMachine = BQ78350_SBS_FF;
                        BAT_DischargeAlgorithmLiIon_LiFePO4();
                        break;

                    case LI_FE_PO_BATTERY:
                        ////              g_ucBqStateMachine = BQ78350_SBS_FF;
                        BAT_DischargeAlgorithmLiIon_LiFePO4();
                        break;
                }
            }
            else
                g_ucAcFailFlag++;
        }
        else                                                        // Have AC Algorithm
        {
            g_ucAcFailFlag = 0;
            //
            switch (g_ucBatteryType)
            {
                case NO_BATTERY:
                    //EN_FAST_CHARGER_H;

                    //EN_CHARGER_L; // Disable
                    //EN_288V_H;
                    //Rock reset if ini battery i2c
                    g_ucBatteryChargeState = CHARGE_STOP;
                    g_Rock_value[7] |= 2;
                    //EN_CHARGER_POWER_L;
                    g_ucBqStateMachine = BQ78350_SBS_FF;
                    g_ulBatteryChargerDelay = ENABLE_CHARGER_DELAY_TIME;
                    g_ulBatteryStopChargerTimer = 0;
                    g_ulBatteryStopChargerErrCount = 0;
                    g_ulBatteryChargeTime = 0;
                    g_ucBatteryChargeMode = BATTERY_STOP_CHARGE_MODE;
                    //g_ucBatteryPercentage = 0;
                    g_ucBatteryChargeStartLiPo = 0;
                    g_ucBatteryInsertEstimateTime = BATTERY_ALG_TIMER_15_SEC;
                    g_ucBatteryLowState = 0;
                    g_ucBatteryDischargeOcpFlag = 0;

                    g_ucBqStateMachineSemaphore = 0;
                    g_ucBq24745ChargerMode = BQ24745_MODE_L0;

                    g_ulBqChargingCurrent = 0;
                    g_ulBqChargingVoltage = 0;

                    g_ucBqBatteryMultiReadLagTime = 0;
                    g_ulBqBatteryDelayTimer = 0;
                    g_ucBqBatteryMultiChargerSemaphore = 0;

                    g_ucBeepPeriod = 0;
                    break;

                case NIMH_BATTERY:
                    break;

                case LEAD_ACID_BATTERY:
                    if (g_ucBatteryCheckFlag == 0)
                        BAT_ChargeAlgorithmLeadAcid();
                    break;

                case LI_PO_BATTERY:
                    if (g_ucBatteryCheckFlag == 0)
                        BAT_ChargeAlgorithmLiPo();
                    break;

                case LI_ION_BATTERY:
                    if (g_ucBatteryCheckFlag == 0)
                        BAT_ChargeAlgorithmLiIon_LiFePO4();
                    break;

                case LI_FE_PO_BATTERY:
                    if (g_ucBatteryCheckFlag == 0)
                        BAT_ChargeAlgorithmLiIon_LiFePO4();
                    break;
            }
        }
    }
}

void TSK_ReadHt66Parameter(void)
{
    if (g_ulSysTickTaskFlag & SYS_TICK_TASK_7)
    {
        //
        g_ulSysTickTaskFlag -= SYS_TICK_TASK_7;

        //
        if (g_ucHT66Semaphore > 0)
            return;

        g_ucHT66CommandMcu[0] = HT66_ADDRESS;
        g_ucHT66CommandMcu[1] = HT66_CMD_0x52;
        g_ucHT66CommandMcu[3] = 0x0A;
        switch (g_ucHT66FlashReadParaFlag)
        {
            case 0:
                g_ucHT66ActionFlagMCU = 1;
                g_ucHT66CommandMcu[2] = HT66_ADDR_0x40;
                g_ucHT66FlashReadParaFlag++;
                break;

            case 1:
                g_ucHT66ActionFlagMCU = 1;
                g_ucHT66CommandMcu[2] = HT66_ADDR_0x50;
                g_ucHT66FlashReadParaFlag++;
                break;

            case 2:
                g_ucHT66ActionFlagMCU = 1;
                g_ucHT66CommandMcu[2] = HT66_ADDR_0x51;
                g_ucHT66FlashReadParaFlag++;
                break;

            case 3:
                g_ucHT66ActionFlagMCU = 1;
                g_ucHT66CommandMcu[2] = HT66_ADDR_0x52;
                g_ucHT66FlashReadParaFlag++;
                break;

            case 4:
                g_ucHT66ActionFlagMCU = 1;
                g_ucHT66CommandMcu[2] = HT66_ADDR_0x53;
                g_ucHT66FlashReadParaFlag++;
                break;

            case 5:
                g_ucHT66ActionFlagMCU = 1;
                g_ucHT66CommandMcu[2] = HT66_ADDR_0x54;
                g_ucHT66FlashReadParaFlag++;
                break;

            case 6:
                g_ucHT66ActionFlagMCU = 1;
                g_ucHT66CommandMcu[2] = HT66_ADDR_0x55;
                g_ucHT66FlashReadParaFlag++;
                break;

            default:
                g_ucHT66FlashReadParaFlag = 8;

                break;
        }

    }
}

//
void TSK_CheckBatteryDischargeFunction(void)
{
    if (g_ulSysTickTaskFlag & SYS_TICK_TASK_8)
    {
        //
        g_ulSysTickTaskFlag -= SYS_TICK_TASK_8;

        switch (g_ChkBatDischargeFun)
        {
            // Start pin
            case 1:
                g_ChkBatDischargeFun++;
                //BATTERY_CHECK_H;
                break;

            // 
            case 2:
                g_ChkBatDischargeFun++;
                if (g_ausADCValue[1][0] > 100)
                    g_ChkBatDischargeFunCount++;
                break;

            // 
            case 3:
                g_ChkBatDischargeFun++;
                if (g_ausADCValue[1][0] > 100)
                    g_ChkBatDischargeFunCount++;
                break;

            // 
            case 4:
                g_ChkBatDischargeFun++;
                if (g_ausADCValue[1][0] > 100)
                    g_ChkBatDischargeFunCount++;
                break;

            // 
            case 5:
                g_ChkBatDischargeFun++;
                if (g_ausADCValue[1][0] > 100)
                    g_ChkBatDischargeFunCount++;
                break;

            // 
            case 6:
                g_ChkBatDischargeFun = 100;
                if (g_ausADCValue[1][0] > 100)
                    g_ChkBatDischargeFunCount++;
                //BATTERY_CHECK_L;
                break;

            default:
                break;
        }
    }
}

// Gas Gauge
void TSK_Bq20z45Action(void)
{
	if (g_ulSysTickTaskFlag & SYS_TICK_TASK_11)
  {
    //
    g_ulSysTickTaskFlag -= SYS_TICK_TASK_11;

    uint8_t data[2] = {0};

    switch(g_ucBatteryChargeMode)
		{
			case BATTERY_STOP_CHARGE_MODE:
				if ((PSON_MCU1 == 1) && (AC_FAIL_N_MCU1 == 0))  //Battery Mode
					g_battery_delay = 0; // Period 1 second
				else																						//AC, Stop Charge Mode
					g_battery_delay = 4; // Period 5 second
				break;
			
			case BATTERY_NORMAL_CHARGE_MODE:									//Charge Mode
				g_battery_delay = 0; // Period 1 second					
				break;
			
			default:
				g_battery_delay = 4; // Period 5 second
				break;
		}
		
		if (g_battery_i2c < g_battery_delay)
    {
        g_battery_i2c++;
        return;
    }
		
		g_battery_i2c = 0;
		    
		if (g_ausBqBatteryState[0][1] > 14)
    {
        g_ucBqStateMachine = BQ78350_SBS_FF;
				return;
    }

    g_ucBqStateMachineBatteryChannel = 0;

		 
		// Check battery insert
    if (g_ucBatteryType == NO_BATTERY)
			return;

    //
    //if (g_ucBatteryHT66Semaphore > 0)
      //return;

    //
    //if (g_ucBqStateMachineSemaphore > 0)
        //return;

			//HWF_ReInitI2C1Pin();
				
			g_ucBqStateMachineBatteryChannel = 0;
		
			//Check no other MASTER on SMBus
			//if(I2C1_SDA == 1 && I2C1_SCL == 1)
			//{
		            // CHARGING_CURRENT
                if(g_ulBqChargingCurrent == 0)
								{
									if(LIB_MemRead(0,
									BQ20Z45_I2C_ADDRESS,
									BQ20Z45_SBS_CHARGING_CURRENT,
									2,
									&data[0]) == 0)
									g_ulBqChargingCurrent = data[0] + data[1]*0x100;
                }
								HAL_Delay(1);
                // CHARGING_VOLTAGE
                if(g_ulBqChargingVoltage == 0)
								{
									if(LIB_MemRead(0,
									BQ20Z45_I2C_ADDRESS,
									BQ20Z45_SBS_CHARGING_VOLTAGE,
									2,
									&data[0]) == 0)
									g_ulBqChargingVoltage = data[0] + data[1]*0x100;
								}
								HAL_Delay(1);
                // TEMP
                LIB_MemRead(0,
                BQ20Z45_I2C_ADDRESS,
                BQ20Z45_SBS_TEMPERATURE,
                2,
                &g_ausBqBatteryTemperature[g_ucBqStateMachineBatteryChannel][0]);
								g_ulBqBatteryTemperature = g_ausBqBatteryTemperature[0][0] +
								g_ausBqBatteryTemperature[0][1] * 0x100;
								HAL_Delay(1);
                //Average Current
                if (LIB_MemRead(0,
                     BQ20Z45_I2C_ADDRESS,
                     BQ20Z45_SBS_AVGER_CURRENT,
                     2,
                     g_ausBqBatteryAverageCurrent) == 0)
                    memcpy(&STR_TX_Buffer[2], g_ausBqBatteryAverageCurrent, 2);
								HAL_Delay(1);
								//CURRENT 
                LIB_MemRead(0,
                    BQ20Z45_I2C_ADDRESS,
                    BQ20Z45_SBS_CURRENT,
                    2,
                    &g_ausBqBatteryCurrent[g_ucBqStateMachineBatteryChannel][0]);
								g_ucBatteryCurrent = g_ausBqBatteryCurrent[0][0] + g_ausBqBatteryCurrent[0][1] * 0x100;
								HAL_Delay(1);
								//VOLTAGE 
                LIB_MemRead(0,
                    BQ20Z45_I2C_ADDRESS,
                    BQ20Z45_SBS_VOLTAGE,
                    2,
                    &g_ausBqBatteryVoltage[g_ucBqStateMachineBatteryChannel][0]);
								g_ucBatteryVoltage = g_ausBqBatteryVoltage[0][0] + g_ausBqBatteryVoltage[0][1] * 0x100;
								HAL_Delay(1);
								//AverageCurrent 
                LIB_MemRead(0,
                    BQ20Z45_I2C_ADDRESS,
                    BQ20Z45_SBS_AVGER_CURRENT,
                    2,
                    &g_ausBqBatteryAverageCurrent[0]);
								g_ucBatteryAverageCurrent = g_ausBqBatteryAverageCurrent[0] + g_ausBqBatteryAverageCurrent[1] * 0x100;
								HAL_Delay(1);
								//REMAINING_DISCHARGE_TIME
                if (LIB_MemRead(0,
                    BQ20Z45_I2C_ADDRESS,
                    BQ20Z45_SBS_REMAINING_DISCHARGE_TIME,
                    2,
                    g_ausBqBatteryRemainingDischargingTime) == 0)
                    memcpy(&STR_TX_Buffer[8], g_ausBqBatteryRemainingDischargingTime, 2);
								
								g_ucBatteryREMAINING_DISCHARGE_TIME = g_ausBqBatteryRemainingDischargingTime[0] + g_ausBqBatteryRemainingDischargingTime[1] * 0x100;
								HAL_Delay(1);
								//RELATIVE_STATE_OF_CHARGE
                if (LIB_MemRead(0,
                    BQ20Z45_I2C_ADDRESS,
                    BQ20Z45_SBS_RELATIVE_STATE_OF_CHARGE,
                    2,
                    &g_ausBqBatteryRelativeStateOfCharge[g_ucBqStateMachineBatteryChannel][0]) == 0)
                    memcpy(&STR_TX_Buffer[4], &g_ausBqBatteryRelativeStateOfCharge[g_ucBqStateMachineBatteryChannel][0], 2);
										{
											g_ucBatteryPercentage = g_ausBqBatteryRelativeStateOfCharge[g_ucBqStateMachineBatteryChannel][0];
										}
                HAL_Delay(1);
								//Remaining Discharging Time 0x12
                if (LIB_MemRead(0,
                     BQ20Z45_I2C_ADDRESS,
                     BQ20Z45_SBS_ABSOLUTE_STATE_OF_CHARGE,
                     2,
                     &g_ausBqBatteryAbsoluteStateOfCharge[g_ucBqStateMachineBatteryChannel][0]) == 0)
                    memcpy(&STR_TX_Buffer[6], &g_ausBqBatteryAbsoluteStateOfCharge[g_ucBqStateMachineBatteryChannel][0], 2);
								g_ucBatteryAbsoluteStateOfCharge = g_ausBqBatteryAbsoluteStateOfCharge[0][0];
								HAL_Delay(1);
								//REMAINING_CAPACITY
                LIB_MemRead(0,
									BQ20Z45_I2C_ADDRESS,
									BQ20Z45_SBS_REMAINING_CAPACITY,
									2,
									&g_ausBqBatteryRemainingCapacity[g_ucBqStateMachineBatteryChannel][0]);
								g_ucBatteryRemainingCapacity = g_ausBqBatteryRemainingCapacity[0][0] + g_ausBqBatteryRemainingCapacity[0][1] * 0x100;
								HAL_Delay(1);
								//FULL_CHARGE_CAPACITY
                LIB_MemRead(0,
                    BQ20Z45_I2C_ADDRESS,
                    BQ20Z45_SBS_FULL_CHARGE_CAPACITY,
                    2,
                    &g_ausBqBatteryFullChargeCapacity[g_ucBqStateMachineBatteryChannel][0]);
                memcpy(&g_ausBqBatteryUpdateSysTime[g_ucBqStateMachineBatteryChannel][0], &g_ucSysTime[2], 5);
								g_ucBatteryFullChargeCapacity = g_ausBqBatteryFullChargeCapacity[0][0] + g_ausBqBatteryFullChargeCapacity[0][1] * 0x100;
                HAL_Delay(1);
								//CELL_VOLTAGE_5
								LIB_MemRead(0,
                    BQ20Z45_I2C_ADDRESS,
                    BQ20Z45_SBS_CELL_VOLTAGE_5,
                    2,
                    &g_ausBqBatteryCellVoltage5[0]);
										g_ucBatteryCELL_VOLTAGE_5 = g_ausBqBatteryCellVoltage5[0] + g_ausBqBatteryCellVoltage5[1] * 0x100;
                HAL_Delay(1);
								//CELL_VOLTAGE_4
								LIB_MemRead(0,
                    BQ20Z45_I2C_ADDRESS,
                    BQ20Z45_SBS_CELL_VOLTAGE_4,
                    2,
                    &g_ausBqBatteryCellVoltage4[g_ucBqStateMachineBatteryChannel][0]);
										g_ucBatteryCELL_VOLTAGE_4 = g_ausBqBatteryCellVoltage4[g_ucBqStateMachineBatteryChannel][0] + g_ausBqBatteryCellVoltage4[g_ucBqStateMachineBatteryChannel][1] * 0x100;
                HAL_Delay(1);
								//CELL_VOLTAGE_3
								LIB_MemRead(0,
                    BQ20Z45_I2C_ADDRESS,
                    BQ20Z45_SBS_CELL_VOLTAGE_3,
                    2,
                    &g_ausBqBatteryCellVoltage3[g_ucBqStateMachineBatteryChannel][0]);
										g_ucBatteryCELL_VOLTAGE_3 = g_ausBqBatteryCellVoltage3[g_ucBqStateMachineBatteryChannel][0] + g_ausBqBatteryCellVoltage3[g_ucBqStateMachineBatteryChannel][1] * 0x100;
								HAL_Delay(1);
								//CELL_VOLTAGE_2
								LIB_MemRead(0,
                    BQ20Z45_I2C_ADDRESS,
                    BQ20Z45_SBS_CELL_VOLTAGE_2,
                    2,
                    &g_ausBqBatteryCellVoltage2[g_ucBqStateMachineBatteryChannel][0]);
										g_ucBatteryCELL_VOLTAGE_2 = g_ausBqBatteryCellVoltage2[g_ucBqStateMachineBatteryChannel][0] + g_ausBqBatteryCellVoltage2[g_ucBqStateMachineBatteryChannel][1] * 0x100;
								HAL_Delay(1);
								//CELL_VOLTAGE_1
								LIB_MemRead(0,
                    BQ20Z45_I2C_ADDRESS,
                    BQ20Z45_SBS_CELL_VOLTAGE_1,
                    2,
                    &g_ausBqBatteryCellVoltage1[g_ucBqStateMachineBatteryChannel][0]);
										g_ucBatteryCELL_VOLTAGE_1 = g_ausBqBatteryCellVoltage1[g_ucBqStateMachineBatteryChannel][0] + g_ausBqBatteryCellVoltage1[g_ucBqStateMachineBatteryChannel][1] * 0x100;
								HAL_Delay(1);
								//BATTERY_STATUS 0x16
                if (LIB_MemRead(0,
                    BQ20Z45_I2C_ADDRESS,
                    BQ20Z45_SBS_BATTERY_STATUS,
                    2,
                    g_ausBqBatteryStatus) == 0) // bbu
                    memcpy(&STR_TX_Buffer[10], g_ausBqBatteryStatus, 2);
								HAL_Delay(1);

			//}
	}
}

// Charger
void SET_WATCHDOG(uint8_t p){
		uint8_t ucI2CTxBuffer[5] = { 0x00 };
		uint8_t ucp=p;
		
		// BQ25756E I2C Address 0X6A
				ucI2CTxBuffer[0] = 0x6A;	// DevAddress

		
		ucI2CTxBuffer[1] = 2;		// Size
		ucI2CTxBuffer[2] = 0x15;	// Addr 							// ChargeOptions() 

	HAL_I2C_Master_Receive(&g_I2c3Handle, 
				(uint16_t)ucI2CTxBuffer[0], 	// Device 
				(uint8_t*)&ucI2CTxBuffer[2],   // Addr  
                          ucI2CTxBuffer[1],            // Length Addr + Data
                                          2);
	ucI2CTxBuffer[3] &= (~0x30);
	
	ucp &= (~0x03);
	ucp <<= 4;

	ucI2CTxBuffer[3] |= ucp;


	
		HAL_I2C_Master_Transmit(&g_I2c3Handle,
				(uint16_t)ucI2CTxBuffer[0], 	// Device 
				(uint8_t*)&ucI2CTxBuffer[2],   // Addr + Data 
				ucI2CTxBuffer[1],				// Length
				2);

	
}

void SET_MPPT(void){
	uint8_t DATA;
	uint8_t Device;
	DATA = 0x07;
	
		Device= 0x6A;	// DevAddress
		Device <<= 1;
		// REG0x1A_MPPT_Control Register					// ChargeOptions() 
	HWF_ReInitI2C3Pin();
	HAL_I2C_Mem_Write(&g_I2c3Handle,
			(uint16_t)Device, 	// Device 
			(uint8_t)0x1A,   // Addr + Data 
	                     1, 
			(uint8_t*)&DATA,   // Addr + Data 
	                     1, 
			2);
		HAL_Delay(2);
}

void BQ25756EInit(void){
	//uint8_t DATA;
		uint8_t ucI2CTxBuffer[5] = { 0x00 };
		//uint8_t ucChargeOptions[2] = { 0x00 };
		
		HAL_StatusTypeDef I2C_S = 0;
		uint16_t VFB_V = 1536; // mV
		uint16_t ucCurrent = 8000; // 20000; // mA
		ucCurrent /= 50;
		ucCurrent <<= 2;
	/*
	FB Voltage Regulation Target (VFB_REG) 1.536 V
	Battery Low Voltage (VBAT_LOWV ) 66.7% x VFB_REG = 1.0245 V
	Recharge Voltage (VRECHG) 97.6% x VFB_REG =1.4991 V
	
	Charging Current HW Limit (ICHG pin) ICHG = KICHG / RICHG
	Pre-Charge Current HW Limit (ICHG pin) 20% x ICHG
	Termination Current HW Limit (ICHG pin) 10% x ICHG
	
	CV Timer Disabled
	NTC Temperature Profile JEITA
	Safety Timer 12 hours
	*/
	ENABLE_CHARGER_H;
		// BQ25756E	I2C Address 0X6A
		ucI2CTxBuffer[0] = 0x6A;	// DevAddress
		ucI2CTxBuffer[0] <<= 1;
	
		ucI2CTxBuffer[2] = 0x17;	
		ucI2CTxBuffer[3] = 0xD0;	// pData		
		
	HWF_ReInitI2C3Pin();
	HAL_I2C_Mem_Write(&g_I2c3Handle,
			(uint16_t)ucI2CTxBuffer[0], 	// Device 
			(uint8_t)(ucI2CTxBuffer[2]),   // Addr + Data 
	                     1, 
			(uint8_t*)&ucI2CTxBuffer[3],   // Addr + Data 
	                     1, 
			2);
	HAL_Delay(2);
	///////////////////
	/*
	ucI2CTxBuffer[3] = 0x1F;	// pData		
							ucI2CTxBuffer[2] = 0x00; 
HWF_ReInitI2C3Pin();							
HAL_I2C_Mem_Read(&g_I2c3Handle,
                       (uint16_t)ucI2CTxBuffer[0], 
	                     (uint8_t)ucI2CTxBuffer[2],
	                     1, 
	                     (uint8_t *)&ucI2CTxBuffer[3], 
	                     1,
	                     2);
	HAL_Delay(2);
	*/
	///////////////////
							ucI2CTxBuffer[1] = 3;
							ucI2CTxBuffer[2] = 0x02;                                // ChargeCurrent()
							ucI2CTxBuffer[3] = ucCurrent/0x100;
							ucI2CTxBuffer[4] = ucCurrent%0x100;
	HWF_ReInitI2C3Pin();
	HAL_I2C_Mem_Write(&g_I2c3Handle,
			(uint16_t)ucI2CTxBuffer[0], 	// Device 
			(uint8_t)ucI2CTxBuffer[2],   // Addr + Data 
	                     1, 
			(uint8_t*)&ucI2CTxBuffer[4],   // Addr + Data 
	                     1, 
			2);
	HAL_Delay(2);
	HWF_ReInitI2C3Pin();
	HAL_I2C_Mem_Write(&g_I2c3Handle,
			(uint16_t)ucI2CTxBuffer[0], 	// Device 
			(uint8_t)(ucI2CTxBuffer[2]+1),   // Addr + Data 
	                     1, 
			(uint8_t*)&ucI2CTxBuffer[3],   // Addr + Data 
	                     1, 
			2);
	HAL_Delay(2);
			
		/// REG0x15_Timer_Control Register (Address = 0x15) [Reset = 0x1D]	
	ucI2CTxBuffer[3] = 0X0D;	// pData		
							ucI2CTxBuffer[2] = 0x15; 
	HWF_ReInitI2C3Pin();
	HAL_I2C_Mem_Write(&g_I2c3Handle,
			(uint16_t)ucI2CTxBuffer[0], 	// Device 
			(uint8_t)(ucI2CTxBuffer[2]),   // Addr + Data 
	                     1, 
			(uint8_t*)&ucI2CTxBuffer[3],   // Addr + Data 
	                     1, 
			2);
	HAL_Delay(2);
	//ucI2CTxBuffer[4] = BQ25756E_Read(0x17);
	//ucI2CTxBuffer[4] = DATA;
	
	
	ucI2CTxBuffer[3] = 0;	// pData		
							ucI2CTxBuffer[1] = 2;
							ucI2CTxBuffer[2] = 0x17; 
HWF_ReInitI2C3Pin();							
HAL_I2C_Mem_Read(&g_I2c3Handle,
                       (uint16_t)ucI2CTxBuffer[0], 
	                     (uint8_t)ucI2CTxBuffer[2],
	                     1, 
	                     (uint8_t *)&ucI2CTxBuffer[3], 
	                     1,
	                     2);
	HAL_Delay(2);
	///////////////////
	/*
	// EN_CHG Charge enable control
	ucI2CTxBuffer[1] = 2;		// Size
	ucI2CTxBuffer[2] = 0x17;	// Addr 							// ChargeOptions() 
	ucI2CTxBuffer[3] = 0xC8;	// pData			
	HWF_ReInitI2C3Pin();
	HAL_I2C_Mem_Write(&g_I2c3Handle,
			(uint16_t)ucI2CTxBuffer[0], 	// Device 
			(uint8_t)ucI2CTxBuffer[2],   // Addr + Data 
	                     1, 
			(uint8_t*)&ucI2CTxBuffer[3],   // Addr + Data 
	                     1, 
			2);
	HAL_Delay(2);
	///////////////////
	
	ucI2CTxBuffer[3] = 0;	// pData		
							ucI2CTxBuffer[1] = 2;
							ucI2CTxBuffer[2] = 0x17; 
HWF_ReInitI2C3Pin();							
HAL_I2C_Mem_Read(&g_I2c3Handle,
                       (uint16_t)ucI2CTxBuffer[0], 
	                     (uint8_t)ucI2CTxBuffer[2],
	                     1, 
	                     (uint8_t *)&ucI2CTxBuffer[3], 
	                     1,
	                     2);
	HAL_Delay(2);
	*/
	///////////////////

	ucI2CTxBuffer[1] = 2;		// Size
	ucI2CTxBuffer[2] = 0x18;	// Addr 							// ChargeOptions() 
	ucI2CTxBuffer[3] = 0xF0;	// pData			
	HWF_ReInitI2C3Pin();
	HAL_I2C_Mem_Write(&g_I2c3Handle,
			(uint16_t)ucI2CTxBuffer[0], 	// Device 
			(uint8_t)ucI2CTxBuffer[2],   // Addr + Data 
	                     1, 
			(uint8_t*)&ucI2CTxBuffer[3],   // Addr + Data 
	                     1, 
			2);
	HAL_Delay(2);
	
		// VFB_REG
		ucI2CTxBuffer[1] = 2;		// Size
		ucI2CTxBuffer[2] = 0x00;	// Addr 	

// Range: 1504mV-1566mV
		if(1503 < VFB_V && VFB_V < 1567){
		ucI2CTxBuffer[3] = (VFB_V-1504)/2;	// pData	
	HWF_ReInitI2C3Pin();
	HAL_I2C_Mem_Write(&g_I2c3Handle,
			(uint16_t)ucI2CTxBuffer[0], 	// Device 
			(uint8_t)ucI2CTxBuffer[2],   // Addr + Data 
	                     1, 
			(uint8_t*)&ucI2CTxBuffer[3],   // Addr + Data 
	                     1, 
			2);
			}

		
		HAL_Delay(2);
		// VBAT_LOWV PRECHG to FASTCHG 
		ucI2CTxBuffer[1] = 2;		// Size
		ucI2CTxBuffer[2] = 0x14;	// Addr 							// ChargeOptions() 
		ucI2CTxBuffer[3] = 0x09;	// pData								// ChargeOptions() 
		ucI2CTxBuffer[3] |= 0x06; // //////////VBAT_LOWV
	HWF_ReInitI2C3Pin();
	HAL_I2C_Mem_Write(&g_I2c3Handle,
			(uint16_t)ucI2CTxBuffer[0], 	// Device 
			(uint8_t)ucI2CTxBuffer[2],   // Addr + Data 
	                     1, 
			(uint8_t*)&ucI2CTxBuffer[3],   // Addr + Data 
	                     1, 
			2);
		HAL_Delay(2);
			
#if			1 ////////////////////// MPPT
			// 4200mV-36000mV
			VFB_V = 12500; // mV
			VFB_V /= 20;
			VFB_V <<= 2;
			
							ucI2CTxBuffer[3] = VFB_V/0x100;
							ucI2CTxBuffer[4] = VFB_V%0x100;
			// REG0x08_Input_Voltage_DPM_Limit Register
		ucI2CTxBuffer[2] = 0x08;	// Addr 							// ChargeOptions() 
	HWF_ReInitI2C3Pin();
	HAL_I2C_Mem_Write(&g_I2c3Handle,
			(uint16_t)ucI2CTxBuffer[0], 	// Device 
			(uint8_t)ucI2CTxBuffer[2],   // Addr + Data 
	                     1, 
			(uint8_t*)&ucI2CTxBuffer[4],   // Addr + Data 
	                     1, 
			2);
		HAL_Delay(2);
		
		ucI2CTxBuffer[2] = 0x09;	// Addr 							// ChargeOptions() 
	HWF_ReInitI2C3Pin();
	HAL_I2C_Mem_Write(&g_I2c3Handle,
			(uint16_t)ucI2CTxBuffer[0], 	// Device 
			(uint8_t)ucI2CTxBuffer[2],   // Addr + Data 
	                     1, 
			(uint8_t*)&ucI2CTxBuffer[3],   // Addr + Data 
	                     1, 
			2);
		HAL_Delay(2);
			
		// REG0x1A_MPPT_Control Register
		ucI2CTxBuffer[1] = 2;		// Size
		ucI2CTxBuffer[2] = 0x1A;	// Addr 							// ChargeOptions() 
		ucI2CTxBuffer[3] = 0x07;	// pData								// ChargeOptions() 
	HWF_ReInitI2C3Pin();
	HAL_I2C_Mem_Write(&g_I2c3Handle,
			(uint16_t)ucI2CTxBuffer[0], 	// Device 
			(uint8_t)ucI2CTxBuffer[2],   // Addr + Data 
	                     1, 
			(uint8_t*)&ucI2CTxBuffer[3],   // Addr + Data 
	                     1, 
			2);
		HAL_Delay(2);
		
#endif
		/*
		HAL_Delay(2);		
		// CV_TMR
		ucI2CTxBuffer[1] = 3;		// Size
		ucI2CTxBuffer[2] = 0x16;	// Addr 							// ChargeOptions() 
		ucI2CTxBuffer[3] = ucChargeOptions[0];	// pData	
	HWF_ReInitI2C3Pin();
	HAL_I2C_Mem_Write(&g_I2c3Handle,
			(uint16_t)ucI2CTxBuffer[0], 	// Device 
			(uint8_t)ucI2CTxBuffer[2],   // Addr + Data 
	                     1, 
			(uint8_t*)&ucI2CTxBuffer[3],   // Addr + Data 
	                     1, 
			2);
		
		
		HAL_Delay(2);
		// Safety Timer
		ucI2CTxBuffer[1] = 2;		// Size
		ucI2CTxBuffer[2] = 0x15;	// Addr 							// ChargeOptions() 
		ucI2CTxBuffer[3] = ucChargeOptions[0];	// pData	
	HWF_ReInitI2C3Pin();
	HAL_I2C_Mem_Write(&g_I2c3Handle,
			(uint16_t)ucI2CTxBuffer[0], 	// Device 
			(uint8_t)ucI2CTxBuffer[2],   // Addr + Data 
	                     1, 
			(uint8_t*)&ucI2CTxBuffer[3],   // Addr + Data 
	                     1, 
			2);
		*/
			
		// VRECHG auto-recharge
		ucI2CTxBuffer[1] = 2;		// Size
		ucI2CTxBuffer[2] = 0x17;	// Addr 							// ChargeOptions() 
		ucI2CTxBuffer[3] = 0x08;	// pData						// ChargeOptions() 
		ucI2CTxBuffer[3] |= 0xC0; /////////// VRECHG
		/*
	HWF_ReInitI2C3Pin();
	HAL_I2C_Mem_Write(&g_I2c3Handle,
			(uint16_t)ucI2CTxBuffer[0], 	// Device 
			(uint8_t)ucI2CTxBuffer[2],   // Addr + Data 
	                     1, 
			(uint8_t*)&ucI2CTxBuffer[3],   // Addr + Data 
	                     1, 
			2);
		HAL_Delay(2);
			// EN_CHG Charge enable control
			ucI2CTxBuffer[1] = 2;		// Size
			ucI2CTxBuffer[2] = 0x17;	// Addr 							// ChargeOptions() 
			ucI2CTxBuffer[3] = 0xC8;	// pData								// ChargeOptions() 
			*/
			ucI2CTxBuffer[3] |= 0x01; // ////////// EN_CHG
			ucI2CTxBuffer[3] |= 0x10; // ////////// DIS_CE_PIN
	HWF_ReInitI2C3Pin();
	HAL_I2C_Mem_Write(&g_I2c3Handle,
			(uint16_t)ucI2CTxBuffer[0], 	// Device 
			(uint8_t)ucI2CTxBuffer[2],   // Addr + Data 
	                     1, 
			(uint8_t*)&ucI2CTxBuffer[3],   // Addr + Data 
	                     1, 
			2);
			HAL_Delay(2);
			
			ENABLE_CHARGER_L;

}


// BQ25756
void TSK_BQ25756Action(void)
{
		uint8_t ucI2CTxBuffer[5] = { 0x00 };
		//uint8_t ucVoltage[2] = { 0x00 };0
		uint16_t ucCurrent;
		uint8_t ucChargeOptions[2] = { 0x00 };
    if (g_ulSysTickTaskFlag & SYS_TICK_TASK_9)
    {
        //
        g_ulSysTickTaskFlag -= SYS_TICK_TASK_9;

		// BQ25756E	I2C Address 0X6A
		ucI2CTxBuffer[0] = 0x6A;	// DevAddress
		ucI2CTxBuffer[0] <<= 1;
	

/*		
	ucI2CTxBuffer[3] = 0;	// pData		
							ucI2CTxBuffer[1] = 2;
							ucI2CTxBuffer[2] = 0x24; //REG0x24_Fault_Status
HWF_ReInitI2C3Pin();							
HAL_I2C_Mem_Read(&g_I2c3Handle,
                       (uint16_t)ucI2CTxBuffer[0], 
	                     (uint8_t)ucI2CTxBuffer[2],
	                     1, 
	                     (uint8_t *)&ucI2CTxBuffer[3], 
	                     1,
	                     2);
	HAL_Delay(2);
			
		if((ucI2CTxBuffer[3] & 0xC0) > 0){
				set_led_mode(Error_Solar_panel_LED);
		}
		else if(ucI2CTxBuffer[3]  > 0){
				set_led_mode(Error_Battery_LED);
		}
		//HWF_ReInitI2C3Pin();
		HAL_Delay(2);
		//
		ucI2CTxBuffer[1] = 3;		// Size
		ucI2CTxBuffer[2] = 0x12;	// Addr 							// ChargeOptions() 
		ucI2CTxBuffer[3] = ucChargeOptions[0];	// pData					// default 0x7902
		ucI2CTxBuffer[4] = ucChargeOptions[1];	// pData
	HWF_ReInitI2C3Pin();
	HAL_I2C_Mem_Write(&g_I2c3Handle,
			(uint16_t)ucI2CTxBuffer[0], 	// Device 
			(uint8_t)ucI2CTxBuffer[2],   // Addr + Data 
	                     1, 
			(uint8_t*)&ucI2CTxBuffer[3],   // Addr + Data 
	                     1, 
			2);
			*/
	
/*		
	ucI2CTxBuffer[3] = 0;	// pData		
							ucI2CTxBuffer[1] = 2;
							ucI2CTxBuffer[2] = 0x24; //0x15; 
HWF_ReInitI2C3Pin();							
HAL_I2C_Mem_Read(&g_I2c3Handle,
                       (uint16_t)ucI2CTxBuffer[0], 
	                     (uint8_t)ucI2CTxBuffer[2],
	                     1, 
	                     (uint8_t *)&ucI2CTxBuffer[3], 
	                     1,
	                     2);
	HAL_Delay(2);
		if ((g_ulBqChargingVoltage != g_ulOLDChargingVoltage) ){
			g_ulOLDChargingVoltage = g_ulBqChargingVoltage;	///////////////
    // 
    double RTOP = 249000.0;    // 
    double VBATREG = g_ulOLDChargingVoltage/1000; // 16.0;     // 
    double RFBG = 33.0;        // 
    double RBOT = 26700.0;// 23700.0;     /////////// 

    // 
    double numerator = (RBOT - RFBG) * VBATREG / RTOP;
    double denominator = 1 + (RBOT - RFBG) / RTOP;

    //  VFB
    double VFB = numerator / denominator;
			VFB *= 1000;
			VFB -= 1504;
			
		//VFB=1548;
			VFB /= 2;
							ucI2CTxBuffer[1] = 3;
							ucI2CTxBuffer[2] = 0x00;       
							ucI2CTxBuffer[3] = (uint8_t)VFB/0x100;
							ucI2CTxBuffer[4] = (uint8_t)VFB%0x100;
			
	HWF_ReInitI2C3Pin();
	HAL_I2C_Mem_Write(&g_I2c3Handle,
			(uint16_t)ucI2CTxBuffer[0], 	// Device 
			(uint8_t)ucI2CTxBuffer[2],   // Addr + Data 
	                     1, 
			(uint8_t*)&ucI2CTxBuffer[4],   // Addr + Data 
	                     1, 
			2);
	HAL_Delay(2);
	HWF_ReInitI2C3Pin();
	HAL_I2C_Mem_Write(&g_I2c3Handle,
			(uint16_t)ucI2CTxBuffer[0], 	// Device 
			(uint8_t)(ucI2CTxBuffer[2]+1),   // Addr + Data 
	                     1, 
			(uint8_t*)&ucI2CTxBuffer[3],   // Addr + Data 
	                     1, 
			2);
	HAL_Delay(2);
		}
		*/
		
		//////////////////////// TEST B
/*		
							ucI2CTxBuffer[2] = 0x33; //REG0x33_VBAT_ADC
HWF_ReInitI2C3Pin();							
HAL_I2C_Mem_Read(&g_I2c3Handle,
                       (uint16_t)ucI2CTxBuffer[0], 
	                     (uint8_t)ucI2CTxBuffer[2],
	                     1, 
	                     (uint8_t *)&ucI2CTxBuffer[3], 
	                     2,
	                     2);
	HAL_Delay(2);
	g_ulBqChargingVoltage = ucI2CTxBuffer[3]*0x100+ucI2CTxBuffer[4];
	
	if(Precharge_MODE == 1){
		
		Precharge_MODE = 0;
	g_ucBatteryVoltage = 16000;
	}
	
	
	g_ulBqChargingCurrent = 8000;
	if( g_ucBatteryVoltage == 16000)
		g_ucBatteryPercentage = 100;
	else if( g_ucBatteryVoltage == 15800)
		g_ucBatteryPercentage = 95;
	else if( g_ucBatteryVoltage == 15500)
		g_ucBatteryPercentage = 20;
	else if( g_ucBatteryVoltage == 15000)
		g_ucBatteryPercentage = 10;
		
		//////////////////////// TEST E
		
	*/
		//if ((g_ulBqChargingCurrent != 0) && (g_ulBqChargingVoltage != 0))
		if ((g_ulBqChargingCurrent != 0) ) // TSK_Bq24745Action
        {
					if(g_ulBqChargingCurrent > 8000) // max Current
						
					ucCurrent =  160; // 20000mA
					else if(g_ulBqChargingCurrent <= 400){						
					ucCurrent =  8; // 20000mA
					}
					else
					ucCurrent = g_ulBqChargingCurrent/50;
					
					//if(g_ucBatteryVoltage < (g_ulBqChargingVoltage*70/100)){		
					if(g_ucBatteryVoltage < 14800){ ////// test					
						ucCurrent = 48; //2400/50;
						Precharge_MODE = 1;
					}
					
					ucCurrent <<= 2;
							//
							//HWF_ReInitI2C3Pin();
					
							ENABLE_CHARGER_H;
					
	
							ucI2CTxBuffer[2] = 0x02;                                // ChargeCurrent()
							ucI2CTxBuffer[3] = ucCurrent/0x100;
							ucI2CTxBuffer[4] = ucCurrent%0x100;
	HWF_ReInitI2C3Pin();
	HAL_I2C_Mem_Write(&g_I2c3Handle,
			(uint16_t)ucI2CTxBuffer[0], 	// Device 
			(uint8_t)ucI2CTxBuffer[2],   // Addr + Data 
	                     1, 
			(uint8_t*)&ucI2CTxBuffer[4],   // Addr + Data 
	                     1, 
			2);
	HAL_Delay(2);
	HWF_ReInitI2C3Pin();
	HAL_I2C_Mem_Write(&g_I2c3Handle,
			(uint16_t)ucI2CTxBuffer[0], 	// Device 
			(uint8_t)(ucI2CTxBuffer[2]+1),   // Addr + Data 
	                     1, 
			(uint8_t*)&ucI2CTxBuffer[3],   // Addr + Data 
	                     1, 
			2);
	HAL_Delay(2);
									
							ENABLE_CHARGER_L;
						
        }
				
		/*		
				if(CHARGER_MODE == 3){
					if(g_ucBatteryVoltage < (g_ulBqChargingVoltage*95/100)){
						EN_charging(1);
					}
				}
				*/
				
				
				
				
		}
		
}
void TSK_Bq24745Action(void)
{
		uint8_t ucI2CTxBuffer[5] = { 0x00 };
		uint8_t ucVoltage[2] = { 0x00 };
		uint8_t ucCurrent[2] = { 0x00 };
		uint8_t ucChargeOptions[2] = { 0x00 };
		    
    if (g_ulSysTickTaskFlag & SYS_TICK_TASK_9)
    {
        //
        g_ulSysTickTaskFlag -= SYS_TICK_TASK_9;

        // Battery check
        if (g_ucBatteryCheckFlag == 1)
            return;

        // Select mode
        switch (g_ucBq24745ChargerMode)
        {
            // Battery too low charge
            case BQ24745_MODE_L1:
                {
                    ucVoltage[0] = 0x00; // 12800 mV
                    ucVoltage[1] = 0x32;
                    ucCurrent[0] = 0x00; // 256 mA
                    ucCurrent[1] = 0x01;
                    break;
                }

            // Pre-charge
            case BQ24745_MODE_L2:
                {
                    if (g_ulBqChargingVoltage == 16800)
                    {
                        ucVoltage[0] = 0xA0; // 16800 mV
                        ucVoltage[1] = 0x41;
                    }
                    else
                    {
                        ucVoltage[0] = 0x40; // 14400 mV
                        ucVoltage[1] = 0x38;
                    }
                    ucCurrent[0] = 0x00; // 512 mA
                    ucCurrent[1] = 0x02;
                    break;
                }

            // LT
            case BQ24745_MODE_L3:
                {
                    if (g_ulBqChargingVoltage == 16800)
                    {
                        ucVoltage[0] = 0xA0; // 16800 mV
                        ucVoltage[1] = 0x41;
                    }
                    else
                    {
                        ucVoltage[0] = 0x40; // 14400 mV
                        ucVoltage[1] = 0x38;
                    }
                    ucCurrent[0] = 0x00; // 1536 mA
                    ucCurrent[1] = 0x06;
                    break;
                }

            // ST
            case BQ24745_MODE_L4:
                {
                    if (g_ulBqChargingVoltage == 16800)
                    {
                        ucVoltage[0] = 0xA0; // 16800 mV
                        ucVoltage[1] = 0x41;
                    }
                    else
                    {
                        ucVoltage[0] = 0x40; // 14400 mV
                        ucVoltage[1] = 0x38;
                    }
                    ucCurrent[0] = 0x00; // 2560 mA
                    ucCurrent[1] = 0x0A;
                    break;
                }

            // HT
            case BQ24745_MODE_L5:
                {
                    if (g_ulBqChargingVoltage == 16800)
                    {
                        ucVoltage[0] = 0xA0; // 16800 mV
                        ucVoltage[1] = 0x41;
                    }
                    else
                    {
                        ucVoltage[0] = 0x40; // 14400 mV
                        ucVoltage[1] = 0x38;
                    }
                    ucCurrent[0] = 0x00; // 512 mA
                    ucCurrent[1] = 0x02;
                    break;
                }
						 // Reset to stop charge
            case BQ24745_MODE_L6:
                {
                    ucVoltage[0] = 0x00; // 0 mV
                    ucVoltage[1] = 0x00;
                    ucCurrent[0] = 0x00; // 0 mA
                    ucCurrent[1] = 0x00;
                    break;
                }

        }
				
				// Set BQ24725A ChargerOptions
				switch (g_ucBQ24725AChargerOptions)
        {
					
            // default mode 0x7902
            case BQ24725A_DEFAULT_CHARGE:
                {
                    ucChargeOptions[0] = 0x02; // Normal Mode
                    ucChargeOptions[1] = 0x7B;
                    break;
                }
						case BQ24725A_STOP_CHARGE:
                {
                    ucChargeOptions[0] = 0x03; // Stop Charge Mode
                    ucChargeOptions[1] = 0x7B;
                    break;
                }

				}

				
        if ((g_ulBqChargingCurrent != 0) && (g_ulBqChargingVoltage != 0))
        {
							//
							HWF_ReInitI2C3Pin();
							HAL_Delay(2);
							//
							ucI2CTxBuffer[0] = 0x12;
							ucI2CTxBuffer[1] = 3;
							ucI2CTxBuffer[2] = 0x12;                                // ChargeOptions() 
							ucI2CTxBuffer[3] = ucChargeOptions[0];                  // default 0x7902
							ucI2CTxBuffer[4] = ucChargeOptions[1];
							HAL_I2C_Master_Transmit(&g_I2c3Handle,
									(uint16_t)ucI2CTxBuffer[0],     // Device 
									(uint8_t*)&ucI2CTxBuffer[2],   // Addr + Data 
									ucI2CTxBuffer[1],               // Length
									2);
							HAL_Delay(2);
					
							//
							ucI2CTxBuffer[0] = 0x12;
							ucI2CTxBuffer[1] = 3;
							ucI2CTxBuffer[2] = 0x3F;                                // InputCurrent() 
							ucI2CTxBuffer[3] = 0xAC;                                // Rsense Using 4 mOhm, 200W/21.3V = 9.39A  9390 mA * 0.4 = 3756 = 0x0EACh
							ucI2CTxBuffer[4] = 0x0E;																// 
							HAL_I2C_Master_Transmit(&g_I2c3Handle,
									(uint16_t)ucI2CTxBuffer[0],     // Device 
									(uint8_t*)&ucI2CTxBuffer[2],   // Addr + Data 
									ucI2CTxBuffer[1],               // Length
									2);
							HAL_Delay(2);

							//
							if (g_ucBq24745ChargerMode == BQ24745_MODE_L0)
							return;
							
							ucI2CTxBuffer[0] = 0x12;
							ucI2CTxBuffer[1] = 3;
							ucI2CTxBuffer[2] = 0x15;                                // ChargeVoltage()
							ucI2CTxBuffer[3] = ucVoltage[0];
							ucI2CTxBuffer[4] = ucVoltage[1];
							HAL_I2C_Master_Transmit(&g_I2c3Handle,
									(uint16_t)ucI2CTxBuffer[0],     // Device 
									(uint8_t*)&ucI2CTxBuffer[2],   // Addr + Data 
									ucI2CTxBuffer[1],               // Length
									2);
							HAL_Delay(2);

							//
							ucI2CTxBuffer[0] = 0x12;
							ucI2CTxBuffer[1] = 3;
							ucI2CTxBuffer[2] = 0x14;                                // ChargeCurrent()
							ucI2CTxBuffer[3] = ucCurrent[0];
							ucI2CTxBuffer[4] = ucCurrent[1];
							HAL_I2C_Master_Transmit(&g_I2c3Handle,
									(uint16_t)ucI2CTxBuffer[0],     // Device 
									(uint8_t*)&ucI2CTxBuffer[2],   // Addr + Data 
									ucI2CTxBuffer[1],               // Length
									2);
						
            //EN_CHARGER_L; // LTC4100 
        }

        //
        g_ucBq24745ChargerMode = BQ24745_MODE_L0;
    }
}

// FSP 650
void TSK_FSP650ReplayAll(void)
{
    uint8_t loop;

    switch (g_ucFSP650Cmd)
    {
        // (0x01) AC input state
        case CMD_0x01:
            g_USBD_HID_InStream[0] = HS1;
            g_USBD_HID_InStream[1] = HS2;
            g_USBD_HID_InStream[2] = 3;
            g_USBD_HID_InStream[3] = CMD_0x01;
            if (AC_FAIL_N_MCU1 > 0)
                g_USBD_HID_InStream[4] = 0x01; // Line active
            else
                g_USBD_HID_InStream[4] = 0x02; // Line off
            g_USBD_HID_InStream[5] = g_USBD_HID_InStream[0] + g_USBD_HID_InStream[1] +
                g_USBD_HID_InStream[2] + g_USBD_HID_InStream[3] +
                g_USBD_HID_InStream[4];
            g_USBD_HID_InFlag = 1;
            break;

        // (0x02) Battery state
        case CMD_0x02:
            g_USBD_HID_InStream[0] = HS1;
            g_USBD_HID_InStream[1] = HS2;
            g_USBD_HID_InStream[2] = 3;
            g_USBD_HID_InStream[3] = CMD_0x02;
            if (AC_FAIL_N_MCU1 == 0) // No AC 
            {
                if (g_ucBatteryPercentage > 20)
                    g_USBD_HID_InStream[4] = 0x05; // Battery mode
                else if (g_ucBatteryPercentage > 10)
                    g_USBD_HID_InStream[4] = 0x03; // Battery low
                else
                    g_USBD_HID_InStream[4] = 0x04; // Battery critical low
            }
            else // Have AC 
            {
                switch (g_ucBatteryType)
                {
                    case NO_BATTERY:
                        g_USBD_HID_InStream[4] = 0x02; // No Battery
                        break;

                    default:
                        g_USBD_HID_InStream[4] = 0x01; // Battery OK
                        break;
                }
            }
            g_USBD_HID_InStream[5] = g_USBD_HID_InStream[0] + g_USBD_HID_InStream[1] +
                g_USBD_HID_InStream[2] + g_USBD_HID_InStream[3] +
                g_USBD_HID_InStream[4];
            g_USBD_HID_InFlag = 1;
            break;

        // (0x03) Fan state
        case CMD_0x03:
            g_USBD_HID_InStream[0] = HS1;
            g_USBD_HID_InStream[1] = HS2;
            g_USBD_HID_InStream[2] = 3;
            g_USBD_HID_InStream[3] = CMD_0x03;
            if (g_ulFanFrequency > 30)
                g_USBD_HID_InStream[4] = 0x01; // OK
            else if (g_ulFanFrequency > 0)
                g_USBD_HID_InStream[4] = 0x03; // Fan warning
            else
                g_USBD_HID_InStream[4] = 0x02; // Fan error
            g_USBD_HID_InStream[5] = g_USBD_HID_InStream[0] + g_USBD_HID_InStream[1] +
                g_USBD_HID_InStream[2] + g_USBD_HID_InStream[3] +
                g_USBD_HID_InStream[4];
            g_USBD_HID_InFlag = 1;
            break;

        // (0x04) Temperature state	
        case CMD_0x04:
            g_USBD_HID_InStream[0] = HS1;
            g_USBD_HID_InStream[1] = HS2;
            g_USBD_HID_InStream[2] = 3;
            g_USBD_HID_InStream[3] = CMD_0x04;
            //if(SENSOR_TEMPERATURE_LIMIT_OFF > g_ulTemperature)
            //  g_USBD_HID_InStream[4] = 0x03; // Temperature limit off (Not support)
            //else if(SENSOR_TEMPERATURE_CRITICAL > g_ulTemperature)
            if (SENSOR_TEMPERATURE_CRITICAL > g_ulTemperature[0])
                g_USBD_HID_InStream[4] = 0x02; // Temperature critical
            else
                g_USBD_HID_InStream[4] = 0x01; // Temperature OK
            g_USBD_HID_InStream[5] = g_USBD_HID_InStream[0] + g_USBD_HID_InStream[1] +
                g_USBD_HID_InStream[2] + g_USBD_HID_InStream[3] +
                g_USBD_HID_InStream[4];
            g_USBD_HID_InFlag = 1;
            break;

        // (0x05) Mouse click, return battery
        case CMD_0x05:
            if (g_ChkBatDischargeFun == 0) // Start check
            {
                g_ChkBatDischargeFun = 1;
                g_ChkBatDischargeFunCount = 0;
                return;
            }
            else if (g_ChkBatDischargeFun > 99) // Finish check
            {
                g_ChkBatDischargeFun = 0;

                g_USBD_HID_InStream[0] = HS1;
                g_USBD_HID_InStream[1] = HS2;
                g_USBD_HID_InStream[2] = 3;
                g_USBD_HID_InStream[3] = CMD_0x05;
                if (g_ChkBatDischargeFunCount == 5)
                    g_USBD_HID_InStream[4] = 0x01; // Battery OK
                else
                    g_USBD_HID_InStream[4] = 0x02; // No Battery
                g_USBD_HID_InStream[5] = g_USBD_HID_InStream[0] + g_USBD_HID_InStream[1] +
                    g_USBD_HID_InStream[2] + g_USBD_HID_InStream[3] +
                    g_USBD_HID_InStream[4];
                g_USBD_HID_InFlag = 1;
            }
            else // Wait check
                return;
            break;

        // (0x06) Reply all
        case CMD_0x06:
            g_USBD_HID_InStream[0] = HS1;
            g_USBD_HID_InStream[1] = HS2;
            g_USBD_HID_InStream[2] = 7;
            g_USBD_HID_InStream[3] = CMD_0x06;
            // *** AC input state *** //
            if (AC_FAIL_N_MCU1 > 0)
                g_USBD_HID_InStream[4] = 0x01; // Line active
            else
                g_USBD_HID_InStream[4] = 0x02; // Line off
                                               // *** Battery state *** //
            if (AC_FAIL_N_MCU1 == 0) // No AC 
            {
                if (g_ucBatteryPercentage > 20)
                    g_USBD_HID_InStream[5] = 0x05; // Battery mode
                else if (g_ucBatteryPercentage > 10)
                    g_USBD_HID_InStream[5] = 0x03; // Battery low
                else
                    g_USBD_HID_InStream[5] = 0x04; // Battery critical low (Countdown)
            }
            else // Have AC 
            {
                switch (g_ucBatteryType)
                {
                    case NO_BATTERY:
                        g_USBD_HID_InStream[5] = 0x02; // No Battery
                        break;

                    default:
                        g_USBD_HID_InStream[5] = 0x01; // Battery OK
                        break;
                }
            }
            // *** Fan state *** //
            if (g_ulFanFrequency > 30)
                g_USBD_HID_InStream[6] = 0x01; // OK
            else if (g_ulFanFrequency > 0)
                g_USBD_HID_InStream[6] = 0x03; // Fan warning
            else
                g_USBD_HID_InStream[6] = 0x02; // Fan error (Countdown)
                                               // *** Temperature state *** //	
            if (SENSOR_TEMPERATURE_CRITICAL > g_ulTemperature[0])
                g_USBD_HID_InStream[7] = 0x02; // Temperature critical (Countdown)
            else
                g_USBD_HID_InStream[7] = 0x01; // Temperature OK
                                               // *** Countdown time	*** //
            if ((g_USBD_HID_InStream[5] == 0x04) ||
                (g_USBD_HID_InStream[6] == 0x02) ||
                (g_USBD_HID_InStream[7] == 0x02))
            {
                if (g_ucFSP650Countdown > 0)
                    g_ucFSP650Countdown--;
                g_USBD_HID_InStream[8] = g_ucFSP650Countdown;
            }
            else
            {
                g_ucFSP650Countdown = 60;
                g_USBD_HID_InStream[8] = g_ucFSP650Countdown;
            }
            // *** Checksum *** //
            g_USBD_HID_InStream[9] = g_USBD_HID_InStream[0] + g_USBD_HID_InStream[1] +
                g_USBD_HID_InStream[2] + g_USBD_HID_InStream[3] +
                g_USBD_HID_InStream[4] + g_USBD_HID_InStream[5] +
                g_USBD_HID_InStream[6] + g_USBD_HID_InStream[7] +
                g_USBD_HID_InStream[8];
            g_USBD_HID_InFlag = 1;
            break;

        // (0x08) FW ID
        case CMD_0x08:
            g_USBD_HID_InStream[0] = HS1;
            g_USBD_HID_InStream[1] = HS2;
            g_USBD_HID_InStream[2] = 18;
            g_USBD_HID_InStream[3] = CMD_0x08;
            memcpy(&g_USBD_HID_InStream[4], FSP650_FW_ID, 16);
            g_USBD_HID_InStream[20] = 0;
            for (loop = 0; loop < 20; loop++)
                g_USBD_HID_InStream[20] = g_USBD_HID_InStream[20] + g_USBD_HID_InStream[loop];
            g_USBD_HID_InFlag = 1;
            break;

        // (0x09) FW Release date
        case CMD_0x09:
            g_USBD_HID_InStream[0] = HS1;
            g_USBD_HID_InStream[1] = HS2;
            g_USBD_HID_InStream[2] = 12;
            g_USBD_HID_InStream[3] = CMD_0x09;
            memcpy(&g_USBD_HID_InStream[4], FSP650_RELEASE_DATE, 10);
            g_USBD_HID_InStream[14] = 0;
            for (loop = 0; loop < 14; loop++)
                g_USBD_HID_InStream[14] = g_USBD_HID_InStream[14] + g_USBD_HID_InStream[loop];
            g_USBD_HID_InFlag = 1;
            break;

        default:
            break;
    }
    g_ucFSP650Cmd = 0;
}

// Root Function
void TSK_RootFunction(void)
{
    uint8_t ucI2CTxBuffer[5] = { 0x00 };
    uint8_t ucVoltage[2] = { 0x00 };
    uint8_t ucCurrent[2] = { 0x00 };

    if (g_ulSysTickTaskFlag & SYS_TICK_TASK_32)
    {
        //
        g_ulSysTickTaskFlag -= SYS_TICK_TASK_32;

        //
        switch (g_ucRootFlag)
        {
            //
            case ROOT_CMD_0x10:
                g_ucRootFSP400ChargeManual = 1;
                //EN_CHARGER_L; // BQ24745

                //
                ucVoltage[0] = 0xA0; // 16800 mV
                ucVoltage[1] = 0x41;
                ucCurrent[0] = 0x00; // 512 mA
                ucCurrent[1] = 0x02;

                //
                HAL_Delay(2);
                HWF_ReInitI2C1Pin();

                //
                ucI2CTxBuffer[0] = 0x12;
                ucI2CTxBuffer[1] = 3;
                ucI2CTxBuffer[2] = 0x3F;                                // InputCurrent() 
                ucI2CTxBuffer[3] = 0x00;
                ucI2CTxBuffer[4] = 0x10;
                HAL_I2C_Master_Transmit(&g_I2c1Handle,
                    (uint16_t)ucI2CTxBuffer[0],     // Device 
                    (uint8_t*)&ucI2CTxBuffer[2],   // Addr + Data 
                    ucI2CTxBuffer[1],               // Length
                    2);
                HAL_Delay(2);

                //
                ucI2CTxBuffer[0] = 0x12;
                ucI2CTxBuffer[1] = 3;
                ucI2CTxBuffer[2] = 0x15;                                // ChargeVoltage()
                ucI2CTxBuffer[3] = ucVoltage[0];
                ucI2CTxBuffer[4] = ucVoltage[1];
                HAL_I2C_Master_Transmit(&g_I2c1Handle,
                    (uint16_t)ucI2CTxBuffer[0],     // Device 
                    (uint8_t*)&ucI2CTxBuffer[2],   // Addr + Data 
                    ucI2CTxBuffer[1],               // Length
                    2);
                HAL_Delay(2);

                //
                ucI2CTxBuffer[0] = 0x12;
                ucI2CTxBuffer[1] = 3;
                ucI2CTxBuffer[2] = 0x14;                                // ChargeCurrent()
                ucI2CTxBuffer[3] = ucCurrent[0];
                ucI2CTxBuffer[4] = ucCurrent[1];
                HAL_I2C_Master_Transmit(&g_I2c1Handle,
                    (uint16_t)ucI2CTxBuffer[0],     // Device 
                    (uint8_t*)&ucI2CTxBuffer[2],   // Addr + Data 
                    ucI2CTxBuffer[1],               // Length
                    2);
                //HAL_Delay(2);
                //EN_CHARGER_L; // LTC4100
                break;

            //
            case ROOT_CMD_0x11:
                g_ucRootFSP400ChargeManual = 0;
                //EN_CHARGER_H;
                break;

            //
            case ROOT_CMD_0x12:
                g_ucRootFSP400FanManual = 1;
                break;

            //
            case ROOT_CMD_0x13:
                g_ucRootFSP400FanManual = 2;
                break;

            //
            case ROOT_CMD_0x14:
                g_ucRootFSP400FanManual = 0;
                break;
        }
        g_ucRootFlag = 0;
    }
}
// Gas Gauge
void TSK_Bq78350Action(void)
{

    if (g_ausBqBatteryState[0][1] > 14)
    {
        g_ucBqStateMachine = BQ78350_SBS_FF;
    }
    if (g_battery_act < 30)
    {
        g_battery_act++;
        return;
    }
    //

    g_ucBqStateMachineBatteryChannel = 0;

    switch (g_ucBqStateMachine)
    {
        case BQ78350_SBS_TEMPERATURE:
            {
                // TEMP
                LIB_MemRead(0,
                BQ78350_I2C_ADDRESS,
                BQ78350_SBS_TEMPERATURE,
                2,
                &g_ausBqBatteryTemperature[g_ucBqStateMachineBatteryChannel][0]);
                g_ucBqStateMachine = BQ78350_SBS_AVGER_CURRENT;
                break;
            }

        case BQ78350_SBS_AVGER_CURRENT:
            {
                //SIR  Average Current     0x0B 
                if (LIB_MemRead(0,
                     BQ78350_I2C_ADDRESS,
                     BQ78350_SBS_AVGER_CURRENT,
                     2,
                     g_ausBqBatteryAverageCurrent) == 0)
                    memcpy(&STR_TX_Buffer[2], g_ausBqBatteryAverageCurrent, 2);

                g_ucBqStateMachine = BQ78350_SBS_CURRENT;
                break;
            }
        case BQ78350_SBS_CURRENT:
            {
                LIB_MemRead(0,
                    BQ78350_I2C_ADDRESS,
                    BQ78350_SBS_CURRENT,
                    2,
                    &g_ausBqBatteryCurrent[g_ucBqStateMachineBatteryChannel][0]);
                g_ucBqStateMachine = BQ78350_SBS_VOLTAGE;
                break;
            }
        case BQ78350_SBS_VOLTAGE:
            {
                LIB_MemRead(0,
                    BQ78350_I2C_ADDRESS,
                    BQ78350_SBS_VOLTAGE,
                    2,
                    &g_ausBqBatteryVoltage[g_ucBqStateMachineBatteryChannel][0]);
                g_ucBqStateMachine = BQ78350_SBS_REMAINING_DISCHARGE_TIME;
                break;
            }
        case BQ78350_SBS_REMAINING_DISCHARGE_TIME:
            {
                if (LIB_MemRead(0,
                    BQ78350_I2C_ADDRESS,
                    BQ78350_SBS_REMAINING_DISCHARGE_TIME,
                    2,
                    g_ausBqBatteryRemainingDischargingTime) == 0)
                    memcpy(&STR_TX_Buffer[8], g_ausBqBatteryRemainingDischargingTime, 2);
                g_ucBqStateMachine = BQ78350_SBS_RELATIVE_STATE_OF_CHARGE;
                break;
            }

        case BQ78350_SBS_RELATIVE_STATE_OF_CHARGE:
            {
                if (LIB_MemRead(0,
                    BQ78350_I2C_ADDRESS,
                    BQ78350_SBS_RELATIVE_STATE_OF_CHARGE,
                    2,
                    &g_ausBqBatteryRelativeStateOfCharge[g_ucBqStateMachineBatteryChannel][0]) == 0)
                    memcpy(&STR_TX_Buffer[4], &g_ausBqBatteryRelativeStateOfCharge[g_ucBqStateMachineBatteryChannel][0], 2);
                {
                    g_ucBatteryPercentage = g_ausBqBatteryRelativeStateOfCharge[g_ucBqStateMachineBatteryChannel][0];
                    g_ucBqStateMachine = BQ78350_SBS_ABSOLUTE_STATE_OF_CHARGE;
                }
                break;
            }
        //absolute state
        case BQ78350_SBS_ABSOLUTE_STATE_OF_CHARGE:
            {
                // sir Remaining discharging time 0x12
                if (LIB_MemRead(0,
                     BQ78350_I2C_ADDRESS,
                     BQ78350_SBS_ABSOLUTE_STATE_OF_CHARGE,
                     2,
                     &g_ausBqBatteryAbsoluteStateOfCharge[g_ucBqStateMachineBatteryChannel][0]) == 0)
                    memcpy(&STR_TX_Buffer[6], &g_ausBqBatteryAbsoluteStateOfCharge[g_ucBqStateMachineBatteryChannel][0], 2);
                g_ucBqStateMachine = BQ78350_SBS_REMAINING_CAPACITY;
                break;
            }
        //
        case BQ78350_SBS_REMAINING_CAPACITY:
            {
                LIB_MemRead(0,
        BQ78350_I2C_ADDRESS,
        BQ78350_SBS_REMAINING_CAPACITY,
        2,
        &g_ausBqBatteryRemainingCapacity[g_ucBqStateMachineBatteryChannel][0]);
                g_ucBqStateMachine = BQ78350_SBS_FULL_CHARGE_CAPACITY;
                break;
            }
        //
        case BQ78350_SBS_FULL_CHARGE_CAPACITY:
            {
                LIB_MemRead(0,
                    BQ78350_I2C_ADDRESS,
                    BQ78350_SBS_FULL_CHARGE_CAPACITY,
                    2,
                    &g_ausBqBatteryFullChargeCapacity[g_ucBqStateMachineBatteryChannel][0]);
                memcpy(&g_ausBqBatteryUpdateSysTime[g_ucBqStateMachineBatteryChannel][0], &g_ucSysTime[2], 5);
                g_ucBqStateMachine = BQ78350_SBS_BATTERY_STATUS;
                break;
            }
        // battery status 0x16
        case BQ78350_SBS_BATTERY_STATUS:
            {
                if (LIB_MemRead(0,
                    BQ78350_I2C_ADDRESS,
                    BQ78350_SBS_BATTERY_STATUS,
                    2,
                    g_ausBqBatteryStatus) == 0)
                    memcpy(&STR_TX_Buffer[10], g_ausBqBatteryStatus, 2);

                g_ucBqStateMachine = BQ78350_SBS_TEMPERATURE;
                break;
            }
        default:
            //if (BATTERY_INSERT_N1 > 0)
                g_ucBqStateMachine = BQ78350_SBS_REMAINING_CAPACITY;
    }
}

//
void I2C3_To_SIR_Write_Block(void)
{ 
	  //delay
    if (g_battery_act < 30)
        return;
   
		//reset ADC
	  int AND = g_ausADCReg[0] ^ g_ausADCReg[2] ^ g_ausADCReg[4] ^ g_ausADCReg[6] ^ g_ausADCReg[8] ^ g_ausADCReg[10];
		
		if(g_AdcAndValue == AND)
		{
		  g_AdcAndValueCounter++;
			if(g_AdcAndValueCounter > 10)
			{
				g_AdcAndValueCounter = 0;
				g_Rock_value[3]++;
				HWF_InitADCMulCH();
			}
		}
		else
		{
		  g_AdcAndValue = AND;
			g_AdcAndValueCounter = 0;
		}
		
		// TO SIR DS BOARD
 		if (SIR_counter < 9)
    {
        SIR_counter++;
        return;
    }
    SIR_counter = 0;

    //
    STR_TX_Buffer[0] = SIR_CMD_Battery_Info;  //[0] 0xA0
    STR_TX_Buffer[1] = 12;                    //[1] byte count    12
                                              //STR_TX_Buffer[2] = 10;//Average Current  g_ausBqBatteryCurrent[g_ucBqStateMachineBatteryChannel][0]
                                              //STR_TX_Buffer[3] = 210;
                                              //STR_TX_Buffer[4] = 10;//Relative Charging state  g_ausBqBatteryRemainingCapacity[g_ucBqStateMachineBatteryChannel][0]
                                              //STR_TX_Buffer[5] = 150;
                                              //STR_TX_Buffer[6] = 240;//Absolute Charging state
                                              //STR_TX_Buffer[7] = 40;
                                              //STR_TX_Buffer[8] = 8;//Remaining discharging time (average)
                                              //STR_TX_Buffer[9] = 5;
                                              //STR_TX_Buffer[10] = 4;//Battery status
                                              //STR_TX_Buffer[11] = 9;
                                              //STR_TX_Buffer[12] = 0;//Battery status2
                                              //STR_TX_Buffer[13] = 1;
    if (HAL_I2C_Master_Transmit(
        &g_I2c3Handle,
        SIR_DEVICE_ADDRESS,
        (uint8_t*)STR_TX_Buffer,
        14,
        2) == 0)
    {
        // SIR Tx ok
    }
    memset(STR_TX_Buffer, 0xFF, 15);
    /*
	transmit process
	s Slave address Wr -------command----------byte count ---------- DATA ----p
	0x17        0          0xA0    				     10
	STR_TX_Buffer[0]		STR_TX_Buffer[1]		 [2]~[11]

	STR_TX_Buffer
	[0]   command       0xA0
	[1]   byte count    10
	[2]   Average Current
	[3]
	[4]   Relative Charging state
	[5]
	[6]   Absolute Charging state
	[7]
	[8]   Remaining discharging time (average)
	[9]
	[10]  Battery status
	[11]
	[12]  Battery status2
	[13]
	*/
}
// CAN2USB
void TSK_TMS320F28032Action(void)
{
	if (g_ulSysTickTaskFlag & SYS_TICK_TASK_11)
  {
    //
    g_ulSysTickTaskFlag -= SYS_TICK_TASK_11;

    //uint8_t data[2] = {0};

    /*switch(g_ucBatteryChargeMode)
		{
			case BATTERY_STOP_CHARGE_MODE:
				if ((PSON_MCU1 == 1) && (AC_FAIL_N_MCU1 == 0))  //Battery Mode
					g_battery_delay = 0; // Period 1 second
				else																						//AC, Stop Charge Mode
					g_battery_delay = 4; // Period 5 second
				break;
			
			case BATTERY_NORMAL_CHARGE_MODE:									//Charge Mode
				g_battery_delay = 0; // Period 1 second					
				break;
			
			default:
				g_battery_delay = 4; // Period 5 second
				break;
		}
		
		if (g_battery_i2c < g_battery_delay)
    {
        g_battery_i2c++;
        //return;
    }
		
		g_battery_i2c = 0;*/
		    
		/*if (g_ausBqBatteryState[0][1] > 14)
    {
        g_ucBqStateMachine = BQ78350_SBS_FF;
				return;
    }

    g_ucBqStateMachineBatteryChannel = 0;*/

		 
		// Check battery insert
    //if (g_ucBatteryType == NO_BATTERY)
			//return;

    //
    //if (g_ucBatteryHT66Semaphore > 0)
      //return;

    //
    //if (g_ucBqStateMachineSemaphore > 0)
        //return;

			//HWF_ReInitI2C1Pin();
				
			g_ucBqStateMachineBatteryChannel = 0;
			if(Read_CAN_Flag == 1)
			{
      // CANID
      LIB_MemRead(0,
      TMS320F28032_I2C_ADDRESS,
      TMS320F28032_CANID,
      2,
      &g_ausCANID[0]);
			//g_ulCANID = g_ausCANID[0] +
			//g_ausCANID[1] * 0x100;
			HAL_Delay(1);
			// CAN DATA
			LIB_MemRead(0,
      TMS320F28032_I2C_ADDRESS,
      TMS320F28032_CANDATA,
      6,
      &g_ausCANDATA[0]);
			HAL_Delay(1);
			}

	}
}
// CAN2USB_SPI
uCAN_MSG txMessage;
uCAN_MSG rxMessage;
uCAN2_MSG txMessage2;
uCAN2_MSG rxMessage2;
uint8_t CAN1DATA1_interval = 0;
uint8_t CAN2DATA1_interval = 0;
uint8_t CAN2DATA2_interval = 0;
uint8_t CAN1DATA1_counter = 0;
uint8_t CAN2DATA1_counter = 0;
uint8_t CAN2DATA2_counter = 0;
void TSK_MCP2515Action(void)
{
		/*uint8_t ucI2CTxBuffer[12] = { 0x00 };
		memset(EEPROM_RX_Buffer, 0, sizeof EEPROM_RX_Buffer);
		
		//
		ucI2CTxBuffer[0] = 0x00;                  // Data Address H
		ucI2CTxBuffer[1] = 0x00;                  // Data Address L
		ucI2CTxBuffer[2] = 11;											// Length
		//ucI2CTxBuffer[3] = 0xAA;									// Data 1
		//ucI2CTxBuffer[4] = 0xBB;									// Data 2
		//ucI2CTxBuffer[5] = 0xCC;									// Data 3
		//ucI2CTxBuffer[6] = 0xDD;									// Data 4

	if (g_ulSysTickTaskFlag & SYS_TICK_TASK_11)
	{
					//
		g_ulSysTickTaskFlag -= SYS_TICK_TASK_11;
		
		LIB_MemByteReadWrite(0 , EEPROM_ADDRESS, &ucI2CTxBuffer[0], ucI2CTxBuffer[2] , &EEPROM_RX_Buffer[0]);
		
		g_Auto_CAN_Broadcast = EEPROM_RX_Buffer[0];
		
		if(g_Auto_CAN_Broadcast == 1)
		{
			txMessage.frame.idType = 1;		// Standard Frame Type
			txMessage.frame.id = EEPROM_RX_Buffer[1] + (EEPROM_RX_Buffer[2] << 8);		//CAN ID LSB + MSB (Little Eddien)
			txMessage.frame.dlc = 8;			//CAN DATA Length
			txMessage.frame.data0 = EEPROM_RX_Buffer[3];		// 58.8V 5A
			txMessage.frame.data1 = EEPROM_RX_Buffer[4];
			txMessage.frame.data2 = EEPROM_RX_Buffer[5];
			txMessage.frame.data3 = EEPROM_RX_Buffer[6];
			txMessage.frame.data4 = EEPROM_RX_Buffer[7];
			txMessage.frame.data5 = EEPROM_RX_Buffer[8];
			txMessage.frame.data6 = EEPROM_RX_Buffer[9];
			txMessage.frame.data7 = EEPROM_RX_Buffer[10];
			CANSPI2_Transmit(&txMessage2);
		}
		if(g_CANFlash == 0)
		{
			if(CANSPI1_Receive(&rxMessage))
				g_CAN1_receive = 1;
			if(CANSPI2_Receive(&rxMessage))
				g_CAN2_receive = 1;
		}*/


			//g_CAN_channel = 0  None
			//g_CAN_channel = 1  CAN1
			//g_CAN_channel = 2  CAN2
			//g_CAN_channel = 3  Both
			
			//CAN1DATA1  //VCU
			txMessage.frame.idType = g_ausCAN1_DATA1[0];
			txMessage.frame.id = g_ausCAN1_DATA1[1] + (g_ausCAN1_DATA1[2] << 8);
			txMessage.frame.dlc = g_ausCAN1_DATA1[3];
			txMessage.frame.data0 = g_ausCAN1_DATA1[4];
			txMessage.frame.data1 = g_ausCAN1_DATA1[5];
			txMessage.frame.data2 = g_ausCAN1_DATA1[6];
			txMessage.frame.data3 = g_ausCAN1_DATA1[7];
			txMessage.frame.data4 = g_ausCAN1_DATA1[8];
			txMessage.frame.data5 = g_ausCAN1_DATA1[9];
			txMessage.frame.data6 = g_ausCAN1_DATA1[10];
			txMessage.frame.data7 = g_ausCAN1_DATA1[11];
			CAN1DATA1_interval = g_ausCAN1_DATA1[12];				//50ms per 1
			if((g_CAN_channel == 1 || g_CAN_channel == 3) && g_ausCAN1_DATA1[3] != 0)
			{
				CAN1DATA1_counter++;
				if(CAN1DATA1_counter == CAN1DATA1_interval)
				{
					CANSPI2_Transmit(&txMessage2);
					CAN1DATA1_counter = 0;
				}
			}
			//CAN2DATA1 //BMS 0x151
			txMessage.frame.idType = g_ausCAN2_DATA1[0];
			txMessage.frame.id = g_ausCAN2_DATA1[1] + (g_ausCAN2_DATA1[2] << 8);
			txMessage.frame.dlc = g_ausCAN2_DATA1[3];
			txMessage.frame.data0 = g_ausCAN2_DATA1[4];
			txMessage.frame.data1 = g_ausCAN2_DATA1[5];
			txMessage.frame.data2 = g_ausCAN2_DATA1[6];
			txMessage.frame.data3 = g_ausCAN2_DATA1[7];
			txMessage.frame.data4 = g_ausCAN2_DATA1[8];
			txMessage.frame.data5 = g_ausCAN2_DATA1[9];
			txMessage.frame.data6 = g_ausCAN2_DATA1[10];
			txMessage.frame.data7 = g_ausCAN2_DATA1[11];
			CAN2DATA1_interval = g_ausCAN2_DATA1[12];
			if((g_CAN_channel == 2 || g_CAN_channel == 3) && g_ausCAN2_DATA1[3] != 0)
			{
				CAN2DATA1_counter++;
				if(CAN2DATA1_counter == CAN2DATA1_interval)
				{
					CANSPI2_Transmit(&txMessage2);
					CAN2DATA1_counter = 0;
				}
			}
			//CAN2DATA2 //BMS 0x152
			txMessage.frame.idType = g_ausCAN2_DATA2[0];
			txMessage.frame.id = g_ausCAN2_DATA2[1] + (g_ausCAN2_DATA2[2] << 8);
			txMessage.frame.dlc = g_ausCAN2_DATA2[3];
			txMessage.frame.data0 = g_ausCAN2_DATA2[4];
			txMessage.frame.data1 = g_ausCAN2_DATA2[5];
			txMessage.frame.data2 = g_ausCAN2_DATA2[6];
			txMessage.frame.data3 = g_ausCAN2_DATA2[7];
			txMessage.frame.data4 = g_ausCAN2_DATA2[8];
			txMessage.frame.data5 = g_ausCAN2_DATA2[9];
			txMessage.frame.data6 = g_ausCAN2_DATA2[10];
			txMessage.frame.data7 = g_ausCAN2_DATA2[11];
			CAN2DATA2_interval = g_ausCAN2_DATA2[12];
			if((g_CAN_channel == 2 || g_CAN_channel == 3) && g_ausCAN2_DATA2[3] != 0)
			{
				CAN2DATA2_counter++;
				if(CAN2DATA2_counter == CAN2DATA2_interval)
				{
					CANSPI2_Transmit(&txMessage2);
					CAN2DATA2_counter = 0;
				}
			}
}
void EEPROM_Action(void)
{
		//uint8_t ucI2CTxBuffer[16] = { 0x00 };
		//uint8_t EEPROM_buffer = 0;
		//memset(EEPROM_RX_Buffer, 0, sizeof EEPROM_RX_Buffer);
		//
						/*ucI2CTxBuffer[0] = 0x00;                									  // EEPROM Data Address H
						ucI2CTxBuffer[1] = 0x00;             										    // EEPROM Data Address L
						ucI2CTxBuffer[2] = 8;									// Length 8
						ucI2CTxBuffer[3] = 1;                 											// OfflineAuto Flag 0
						ucI2CTxBuffer[4] = 0x40;                 // CAN ID L	1
						ucI2CTxBuffer[5] = 0x01;                 // CAN ID H	2
						ucI2CTxBuffer[6] = 0x0A;									// Data 1	
						ucI2CTxBuffer[7] = 0x4C;									// Data 2
						ucI2CTxBuffer[8] = 0x02;									// Data 3
						ucI2CTxBuffer[9] = 0x0F;								// Data 4
						ucI2CTxBuffer[10] = 0x0F;								// Data 5
						ucI2CTxBuffer[11] = 0x01;								// Data 6
						ucI2CTxBuffer[12] = 0x01;								// Data 7
						ucI2CTxBuffer[13] = 0x0F;								// Data 8*/
						
						//Write EEPROM
						//LIB_MemByteReadWrite(1, EEPROM_ADDRESS, &ucI2CTxBuffer[0], ucI2CTxBuffer[2] + 3 , &ucI2CTxBuffer[3]);
	
		//uint8_t ausRecevie[20] = {0};
		//uint8_t data = ucI2CTxBuffer[2];
		//uint8_t length = ucI2CTxBuffer[1];
		//Write EEPROM
		//LIB_MemByteReadWrite(1 , EEPROM_ADDRESS, &ucI2CTxBuffer[0], ucI2CTxBuffer[2] , &ucI2CTxBuffer[3]);
		HAL_Delay(5); // Wait Write Time 5ms
		//LIB_MemByteReadWrite(0 , EEPROM_ADDRESS, &ucI2CTxBuffer[0], ucI2CTxBuffer[2] , &EEPROM_RX_Buffer[0]);
						memset(EEPROM_RX_Buffer, 0, sizeof EEPROM_RX_Buffer);
						//LIB_MemByteReadWrite(0 , EEPROM_ADDRESS, &g_ReadEEPROMAddress[0], ucI2CTxBuffer[2] + 3 , &EEPROM_RX_Buffer[0]);

    // [SELECT REGISTER] + [WRITE] 
    /*ausRecevie[0] = ucI2CTxBuffer[2];
		ausRecevie[1] = ucI2CTxBuffer[3];
    memcpy(&ausRecevie[2], &ucI2CTxBuffer[4], 2 + length);   
    if(HAL_I2C_Master_Transmit(&g_I2c3Handle,   
                               (uint16_t)EEPROM_ADDRESS,              // Device 
                               (uint8_t *)&ausRecevie[0],             // AddrH + AddrL + Data
                               2 + length,                            // Length
                               2) != HAL_OK)                          // Wait time = 2ms
		;

		HAL_Delay(4); // Wait Write Time
		
		uint8_t ucI2CAddrBuffer[2] = { 0x00 };
		ucI2CAddrBuffer[0] = 0x00;
		ucI2CAddrBuffer[1] = 0x00;
		
		HWF_ReInitI2C3Pin();
		// [SELECT REGISTER]
		if(HAL_I2C_Master_Transmit(&g_I2c3Handle,
                               (uint16_t)0xA0,                      // Device
                               (uint8_t *)&ucI2CTxBuffer[2],    	  // AddrH + AddrL
                               2,                           		    // Addr Length
                               2) != HAL_OK)                        // Wait time = 2ms
    ;
    // [READ]
    if(HAL_I2C_Master_Receive(&g_I2c3Handle,
                              (uint16_t)0xA0,                      // Device
                              (uint8_t *)&EEPROM_RX_Buffer[0],     // Read Data
                              1,                                	 // Length
                              2) != HAL_OK)                        // Wait time = 2ms*/
		;
		
}


void EN_charging(uint8_t EN_CHG){
	
		if(EN_CHG == 0){
			ENABLE_CHARGER_H;
		}
		else{
			ENABLE_CHARGER_L;
		}
	
}

void EN_chargingx(uint8_t EN_CHG){
		uint8_t ucI2CTxBuffer[5] = { 0x00 };
		// BQ25756E	I2C Address 0X6A
		ucI2CTxBuffer[0] = 0x6A;	// DevAddress
		ucI2CTxBuffer[0] <<= 1;
		
		
		
	ucI2CTxBuffer[1] = 2;		// Size
		if(EN_CHG == 0){
		ucI2CTxBuffer[3] = 0xD0;	// pData			
		}
		else {
		ucI2CTxBuffer[3] = 0xD9;	// pData		
		}
		
	ucI2CTxBuffer[2] = 0x17;	// Addr 							// ChargeOptions() 
	HWF_ReInitI2C3Pin();
	HAL_I2C_Mem_Write(&g_I2c3Handle,
			(uint16_t)ucI2CTxBuffer[0], 	// Device 
			(uint8_t)ucI2CTxBuffer[2],   // Addr + Data 
	                     1, 
			(uint8_t*)&ucI2CTxBuffer[3],   // Addr + Data 
	                     1, 
			2);
	HAL_Delay(2);
	
}

void get_sys_statusx(void)
{
	
			
		if(CHARGER_MODE == 8)
			{
				EN_charging(1);
			CHARGER_MODE = 1;
			}
			else  if(CHARGER_MODE == 9)
			{
				EN_charging(0);
			CHARGER_MODE = 1;
			}
}

void get_sys_status(void){
	
		uint8_t ucI2CTxBuffer[5] = { 0x00 };
		uint8_t Buffer = 0;
		// BQ25756E	I2C Address 0X6A
		ucI2CTxBuffer[0] = 0x6A;	// DevAddress
		ucI2CTxBuffer[0] <<= 1;
	
	ucI2CTxBuffer[3] = 0;	// pData		
							ucI2CTxBuffer[1] = 2;
							ucI2CTxBuffer[2] = 0x21; //REG0x21_Charger_Status_1 
HWF_ReInitI2C3Pin();							
HAL_I2C_Mem_Read(&g_I2c3Handle,
                       (uint16_t)ucI2CTxBuffer[0], 
	                     (uint8_t)ucI2CTxBuffer[2],
	                     1, 
	                     (uint8_t *)&ucI2CTxBuffer[3], 
	                     1,
	                     2);
	HAL_Delay(2);
		
// 000b = Not charging
// 001b = Trickle Charge (VBAT < VBAT_SHORT)
// 010b = Pre-Charge (VBAT < VBAT_LOWV)
// 011b = Fast Charge (CC mode)
// 100b = Taper Charge (CV mode)
// 101b = Reserved
// 110b = Top-off Timer Charge
// 111b = Charge Termination Done
	ucI2CTxBuffer[3] &= 0x07;
		if(ucI2CTxBuffer[3] > 0 && ucI2CTxBuffer[3] < 5){
			
			if(OLD_CHARGER_MODE == 0){
				EN_charging(1);
			current_mode = Charging_LED;
			CHARGER_MODE = 1;
			}
		}
		
							ucI2CTxBuffer[2] = 0x31; //REG0x31_VAC_ADC
HWF_ReInitI2C3Pin();							
HAL_I2C_Mem_Read(&g_I2c3Handle,
                       (uint16_t)ucI2CTxBuffer[0], 
	                     (uint8_t)ucI2CTxBuffer[2],
	                     1, 
	                     (uint8_t *)&ucI2CTxBuffer[3], 
	                     2,
	                     2);
	HAL_Delay(2);
		
		
		if(CHARGER_MODE == 9 )
			{
		//if(CHARGER_MODE == 0 || CHARGER_MODE == 3){		
			if(g_ucBatteryVoltage < g_ulBqChargingVoltage){
				EN_charging(1);
			current_mode = Charging_LED;
			CHARGER_MODE = 1;
			}
		}

		
/*		
							ucI2CTxBuffer[2] = 0x37; //REG0x37_TS_ADC
HWF_ReInitI2C3Pin();							
HAL_I2C_Mem_Read(&g_I2c3Handle,
                       (uint16_t)ucI2CTxBuffer[0], 
	                     (uint8_t)ucI2CTxBuffer[2],
	                     2,//1, 
	                     (uint8_t *)&ucI2CTxBuffer[3], 
	                     1,
	                     2);
	HAL_Delay(2);
		if((ucI2CTxBuffer[3] == 0x17) && (ucI2CTxBuffer[4] == 0x00)){
			if(OLD_CHARGER_MODE != 0){
				EN_charging(0);
			}
				CHARGER_MODE = 0;
			current_mode = Standby_LED;
		}
		*/
		
	ucI2CTxBuffer[3] = 0;	// pData		
							ucI2CTxBuffer[1] = 2;
							ucI2CTxBuffer[2] = 0x24; //REG0x24_Fault_Status
HWF_ReInitI2C3Pin();							
HAL_I2C_Mem_Read(&g_I2c3Handle,
                       (uint16_t)ucI2CTxBuffer[0], 
	                     (uint8_t)ucI2CTxBuffer[2],
	                     1,//1, 
	                     (uint8_t *)&ucI2CTxBuffer[3], 
	                     1,
	                     2);
	HAL_Delay(2);
		/*	
		if((ucI2CTxBuffer[3] & 0x80) > 0){
			if(OLD_CHARGER_MODE != 0){
				EN_charging(0);
			}
				CHARGER_MODE = 0;
			current_mode = Standby_LED;
		}
		else 
		*/
	
		/*	
		if(g_ulBqChargingVoltage == 0){
			if(OLD_CHARGER_MODE != 0){
				EN_charging(0);
			}
				CHARGER_MODE = 0;
			current_mode = Standby_LED;
		}
		else 
		*/
/*	
			//Buffer = ucI2CTxBuffer[3] & 0xC0;
			Buffer = ucI2CTxBuffer[3] & 0x40;
		if(Buffer > 0){
			if(OLD_CHARGER_MODE != 4){
				EN_charging(0);
			}
				CHARGER_MODE = Buffer;
				CHARGER_MODE = 4;
			current_mode = Error_Solar_panel_LED;
		}
		else if(ucI2CTxBuffer[3]  > 0){
				//set_led_mode(Error_Battery_LED);
			if(OLD_CHARGER_MODE != 4){
				EN_charging(0);
			}
				CHARGER_MODE = 4;
			current_mode = Error_Solar_panel_LED;
		}
		*/
		
		// g_ausBqBatteryStatus[0] = 0;  ///  test
		// g_ausBqBatteryStatus[1] = 0;  ///  test
		
		/*
		if(g_ausBqBatteryStatus[1] > 0x00) // Alarm Bits
			{
			if(OLD_CHARGER_MODE != 4){
				EN_charging(0);
			}
				CHARGER_MODE = 4;
			current_mode = Error_Battery_LED;
			}
		
		
		if((g_ausBqBatteryStatus[0] & 0x0F) > 0) // Error Code
			{				
			if(OLD_CHARGER_MODE != 4){
				EN_charging(0);
			}
			current_mode = Error_Battery_LED;
				CHARGER_MODE = 4;
			}
		*/
		
		
		
		if(g_ulBqChargingVoltage == 0){
			if(OLD_CHARGER_MODE != 0){
				EN_charging(0);
			}
				CHARGER_MODE = 0;
			current_mode = Standby_LED;
		}
		else
		{
			if(CHARGER_MODE == 0){
				EN_charging(1);
				CHARGER_MODE = 1;
			}
			
		}
		
		if((g_ausBqBatteryStatus[1] & 0xA0) > 0) // 
			{
			if(OLD_CHARGER_MODE == 1){
				EN_charging(0);
			}
			current_mode = Error_Battery_LED;
				CHARGER_MODE = 4;
			}
		
		else if((g_ausBqBatteryStatus[0] & 0x20) > 0){
			if(OLD_CHARGER_MODE == 1){
				EN_charging(0);
			}
			current_mode = Charging_finished_LED;
				CHARGER_MODE = 2;
		}
		else if((g_ausBqBatteryStatus[0] & 0x40) > 0){
			/*
			//////
	if( g_ucBatteryVoltage == 15900 )
		g_ucBatteryPercentage = 100;
	else if( g_ucBatteryVoltage < 15300)
		g_ucBatteryPercentage = 10;
	else if( g_ucBatteryVoltage < 15500)
		g_ucBatteryPercentage = 20;
	else if( g_ucBatteryVoltage < 15700)
		g_ucBatteryPercentage = 95;
	//////
			*/
	
			if(g_ucBatteryPercentage == 100){
				if(OLD_CHARGER_MODE == 1){
				EN_charging(0);
			}
			current_mode = Charging_finished_LED;
				CHARGER_MODE = 2;
			}
			else{				
				CHARGER_MODE = 3;		
				current_mode = Discharge_LED;	
				if (g_ucBatteryPercentage <= 10)
					current_mode = Battery_critical_low_LED;
				else if (g_ucBatteryPercentage <= 20)
					current_mode = Battery_low_LED;
			}	
		}
		else{			
			if(g_ulBqChargingVoltage > 0){
				if(OLD_CHARGER_MODE == 0 || OLD_CHARGER_MODE == 3){
				EN_charging(1);
				current_mode = Charging_LED;
				CHARGER_MODE = 1;
				}
			}
				
			
		}
		
		
			if(current_mode != old_LED_MODE){
				OLD_CHARGER_MODE = CHARGER_MODE;
				set_led_mode(current_mode);
			}
		
}

void get_adc(void){
	
	ADC_ChannelConfTypeDef sConfig;
	
				
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = 4;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	sConfig.Offset = 0;
        if (HAL_ADC_ConfigChannel(&g_AdcHandle, &sConfig) != HAL_OK) {
            //Error_Handler();
        }

        HAL_ADC_Start(&g_AdcHandle);  // ?? ADC ??
        HAL_ADC_PollForConversion(&g_AdcHandle, HAL_MAX_DELAY);  // ??????
        adc_value_pa3 = HAL_ADC_GetValue(&g_AdcHandle);  // ?? ADC ??

}