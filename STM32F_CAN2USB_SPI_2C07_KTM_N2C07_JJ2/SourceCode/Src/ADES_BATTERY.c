
#include "main.h"

//
void BAT_DischargeAlgorithmLeadAcid(void)
{
    uint8_t batteryPercentage;
    uint16_t base;
    uint16_t calValue;

    //
    //EN_FAST_CHARGER_H;
    //EN_288V_H;
    //EN_CHARGER_L;
    //EN_CHARGER_POWER_L;
    g_ulBatteryChargerDelay = 0;
    g_ulBatteryStopChargerTimer = 0;
    g_ulBatteryStopChargerErrCount = 0;
    g_ulBatteryChargeTime = 0;
    g_ucLedSysStatus = LED_BAT_DISCHARGE;
    g_ucBatteryChargeState = CHARGE_STOP;
    g_ulBatteryTime++;
    //Rock 20170626 cancel pwm stop
    //HAL_TIM_PWM_Stop(&g_TimHandle_Beep, TIM_CHANNEL_2);

    // Battery low signal ( Pin and Beep )
    if (g_ausADCValue[0][0] < BATTERY_VOLTAGE_21_7)
        BATTERY_LOW_PIN_H;
    if (g_ucBatteryLowState == 0)
    {
        if (g_ausADCValue[0][0] < BATTERY_VOLTAGE_22_1)
            g_ucBatteryLowState = 1;
    }
    else if (g_ucBatteryLowState < 7) // 3 Sec Beep
    {
        g_ucBatteryLowState++;
        if (g_ucBatteryLowState == 7)
        {
            g_ucBatteryLowState = 1;

            // Beep
            //Rock 20170626 cancel pwm start
            //HAL_TIM_PWM_Start(&g_TimHandle_Beep, TIM_CHANNEL_2);
        }

        if (g_ausADCValue[0][0] < BATTERY_VOLTAGE_21_7)
            g_ucBatteryLowState = 8;
    }
    else if (g_ucBatteryLowState > 7) // 1 Sec Beep
    {
        g_ucBatteryLowState++;
        if (g_ucBatteryLowState == 10)
        {
            g_ucBatteryLowState = 8;

            // Beep
            //Rock 20170626 cancel pwm start
            //HAL_TIM_PWM_Start(&g_TimHandle_Beep, TIM_CHANNEL_2);
        }
    }

    // 
    calValue = g_ausADCValue[0][0];

    //
    if (calValue > BATTERY_DISCHARGE_PERCENTAGE_V100)
        calValue = BATTERY_DISCHARGE_PERCENTAGE_V100;
    if (calValue < BATTERY_DISCHARGE_PERCENTAGE_V0)
        calValue = BATTERY_DISCHARGE_PERCENTAGE_V0;
    if (calValue > BATTERY_DISCHARGE_PERCENTAGE_V20)
    {
        base = (BATTERY_DISCHARGE_PERCENTAGE_V100 - BATTERY_DISCHARGE_PERCENTAGE_V20);
        batteryPercentage = ((calValue - BATTERY_DISCHARGE_PERCENTAGE_V20) * 100) / base;
        batteryPercentage = batteryPercentage * 80 / 100 + 20;
        if (batteryPercentage < g_ucBatteryPercentage)
        {
            if (g_ucBatteryPercentage > 20)
                g_ucBatteryPercentage--;
        }
    }
    else if (calValue > BATTERY_DISCHARGE_PERCENTAGE_V10)
    {
        base = (BATTERY_DISCHARGE_PERCENTAGE_V20 - BATTERY_DISCHARGE_PERCENTAGE_V10);
        batteryPercentage = ((calValue - BATTERY_DISCHARGE_PERCENTAGE_V10) * 100) / base;
        batteryPercentage = batteryPercentage * 10 / 100 + 10;
        if (batteryPercentage < g_ucBatteryPercentage)
        {
            if (g_ucBatteryPercentage > 10)
                g_ucBatteryPercentage--;
        }
    }
    else
    {
        base = (BATTERY_DISCHARGE_PERCENTAGE_V10 - BATTERY_DISCHARGE_PERCENTAGE_V0);
        batteryPercentage = ((calValue - BATTERY_DISCHARGE_PERCENTAGE_V0) * 100) / base;
        batteryPercentage = batteryPercentage * 10 / 100;
        if (batteryPercentage < g_ucBatteryPercentage)
        {
            if (g_ucBatteryPercentage > 0)
                g_ucBatteryPercentage--;
        }
    }
    //batteryPercentage = ((calValue - BATTERY_DISCHARGE_PERCENTAGE_V0) * 100) / base;
    //if(calValue < BATTERY_DISCHARGE_PERCENTAGE_V20)
    //{
    //  if(g_ucBatteryPercentage > 20)
    //    g_ucBatteryPercentage = 20;
    //}
    //if(batteryPercentage < g_ucBatteryPercentage) 
    //{ 
    //  if(g_ucBatteryPercentage != 0)
    //    g_ucBatteryPercentage--;
    //}

    //// Calculate battery percentage (0% ~ 100%) 
    //if(calValue > BATTERY_DISCHARGE_PERCENTAGE_V100) 
    //  calValue = BATTERY_DISCHARGE_PERCENTAGE_V100;
    //if(calValue < BATTERY_DISCHARGE_PERCENTAGE_V0)
    //  calValue = BATTERY_DISCHARGE_PERCENTAGE_V0;
    //batteryPercentage = ((calValue - BATTERY_DISCHARGE_PERCENTAGE_V0) * 100) / base;
    //if(batteryPercentage < g_ucBatteryPercentage)
    //{
    //  g_ucBatteryChargeCount = 0;
    //  if(g_ucBatteryPercentage != 0)
    //      g_ucBatteryPercentage--;
    //}
    ////else if((g_ucBatteryPercentage + 15) < batteryPercentage) // Compensation
    ////{
    ////  g_ucBatteryChargeCount++;
    ////  if(g_ucBatteryChargeCount > 4) // > 2 Sec 
    ////  {
    ////    g_ucBatteryChargeCount = 0;
    ////    g_ucBatteryPercentage++;
    ////  }
    ////}
    //////g_ucBatteryPercentage = batteryPercentage;

    return;
}

//
void BAT_ChargeAlgorithmLeadAcid(void)
{
    //uint32_t value_x, value_y;
    uint8_t batteryPercentage;
    uint16_t calValue;
    //
    g_ucBatteryChargeState = CHARGE_STOP;
    g_ulBatteryTime = 0;
    g_ucBatteryLowState = 0;
    BATTERY_LOW_PIN_L;
    //Rock 20170626 cancel pwm stop
    //HAL_TIM_PWM_Stop(&g_TimHandle_Beep, TIM_CHANNEL_2);

    // Check charge voltage
    if (g_ausADCValue[0][0] > BATTERY_VOLTAGE_30_0)
    {
        g_ulBatteryChargeTime = 0;
        g_ulBatteryStopChargerErrCount = 0;
        g_ucBatteryChargeMode = BATTERY_STOP_CHARGE_MODE;
        g_ucBatteryChargeState = CHARGE_STOP;

        // Info HT66 
        g_ucHT66ActionFlagMCU = 1;
        g_ucHT66CommandMcu[0] = HT66_ADDRESS;
        g_ucHT66CommandMcu[1] = HT66_CMD_0x54;
        g_ucHT66CommandMcu[2] = HT66_ADDR_0x07;
        g_ucHT66CommandMcu[3] = 0x01;
    }

    //
    switch (g_ucBatteryChargeMode)
    {
        // Normal charge
        case BATTERY_NORMAL_CHARGE_MODE:
            {
                // Charge flow
                g_ucBatterySystemOnCharge = 0;
                g_ucLedSysStatus = LED_BAT_NORMAL_CHARGE;
                //  
                if (g_ucPwStateMachine == PW_STEP_4)
                {
                    // Action
                    g_ucBatteryChargeState = CHARGE_NORMAL;
                    if (g_ausADCValue[1][0] < BATTERY_CHARGE_CURRENT_2_00)
                        g_ulBatteryChargeTime++;
                    else
                        g_ulBatteryChargeTime = 0;
                    if (g_ulBatteryChargeTime > BATTERY_ALG_TIMER_10_SEC) // I < 2 A and 10 Sec 
                    {
                        g_ulBatteryStopChargerTimer = 0;
                        g_ulBatteryChargeTime = 0;
                        g_ucBatteryChargeCC2CV = 0;
                        g_ulBatteryStopChargerErrCount = 0;
                        g_ucBatteryChargeMode = BATTERY_FAST_CHARGE_MODE;
                        g_ucBatteryChargeState = CHARGE_FAST;

                        // Charge flow
                        g_ucLedSysStatus = LED_BAT_FAST_CC_CHARGE;
                    }

                    // Calculate battery percentage (0% ~ 60%)
                    calValue = g_ausADCValue[0][0];
                    if (calValue > BATTERY_CHARGE_PERCENTAGE_VM)
                        calValue = BATTERY_CHARGE_PERCENTAGE_VM;
                    else if (calValue < BATTERY_CHARGE_PERCENTAGE_V0)
                        calValue = BATTERY_CHARGE_PERCENTAGE_V0;
                    calValue = calValue - BATTERY_CHARGE_PERCENTAGE_V0;
                    batteryPercentage = (calValue * 100) /
                                        (BATTERY_CHARGE_PERCENTAGE_VM - BATTERY_CHARGE_PERCENTAGE_V0);
                    batteryPercentage = (batteryPercentage * BATTERY_CHARGE_LM) / 100; // Convert to 60% range
                    if (g_ucBatteryInsertEstimateTime == 0) // For normal estimate
                    {
                        if (batteryPercentage > g_ucBatteryPercentage)
                            g_ucBatteryPercentage++;
                    }
                    else // For fast estimate
                    {
                        g_ucBatteryInsertEstimateTime--;
                        if (batteryPercentage > g_ucBatteryPercentage)
                            g_ucBatteryPercentage = batteryPercentage;
                    }
                }
                else
                {
                    //EN_FAST_CHARGER_L;
                    //EN_288V_H;
                    //EN_CHARGER_L;
                    //EN_CHARGER_POWER_L;
                    g_ulBatteryStopChargerTimer = 0;
                    g_ulBatteryChargeTime = 0;
                }
                break;
            }

        // Fast charge
        case BATTERY_FAST_CHARGE_MODE:
            {
                // 
                if (g_ucPwStateMachine == PW_STEP_4)
                {
                    // Action
                    g_ucBatteryChargeState = CHARGE_FAST;
                    if (g_ausADCValue[1][0] < BATTERY_CHARGE_CURRENT_0_20)
                        g_ulBatteryChargeTime++;
                    else
                        g_ulBatteryChargeTime = 0;
                    if (g_ulBatteryChargeTime > BATTERY_ALG_TIMER_3_SEC) // I < 0.2 A and 3 Sec 
                    {
                        //EN_FAST_CHARGER_L;
                        //EN_288V_L;
                        //g_ulBatteryStopChargerTimer = BATTERY_ALG_TIMER_30_SEC;//BATTERY_ALG_TIMER_2_HR;
                        g_ulBatteryStopChargerTimer = BATTERY_ALG_TIMER_2_HR;
                        g_ulBatteryChargeTime = 0;
                        g_ulBatteryStopChargerErrCount = 0;
                        g_ucBatteryChargeMode = BATTERY_RE_NORMAL_CHARGE_MODE;
                        g_ucBatteryChargeState = CHARGE_RE_NORMAL;
                    }

                    // Calculate battery percentage (0% ~ 90%)
                    if (g_ucBatteryChargeCC2CV == 0) // CC mode  (0% ~ 60%)
                    {
                        // Charge flow
                        g_ucLedSysStatus = LED_BAT_FAST_CC_CHARGE;

                        calValue = g_ausADCValue[0][0];
                        if (calValue > BATTERY_CHARGE_PERCENTAGE_VM)
                        {
                            calValue = BATTERY_CHARGE_PERCENTAGE_VM;
                            g_ucBatteryChargeCC2CV = 1; // Translate to CV
                        }
                        else if (calValue < BATTERY_CHARGE_PERCENTAGE_V0)
                            calValue = BATTERY_CHARGE_PERCENTAGE_V0;
                        calValue = calValue - BATTERY_CHARGE_PERCENTAGE_V0;
                        batteryPercentage = (calValue * 100) /
                                            (BATTERY_CHARGE_PERCENTAGE_VM - BATTERY_CHARGE_PERCENTAGE_V0);
                        batteryPercentage = (batteryPercentage * BATTERY_CHARGE_LM) / 100;
                        if (g_ucBatteryInsertEstimateTime == 0) // For normal estimate
                        {
                            if (batteryPercentage > g_ucBatteryPercentage)
                                g_ucBatteryPercentage++;
                        }
                        else // For fast estimate
                        {
                            g_ucBatteryInsertEstimateTime--;
                            if (batteryPercentage > g_ucBatteryPercentage)
                                g_ucBatteryPercentage = batteryPercentage;
                        }
                    }
                    else // CV mode  (61% ~ 90%)
                    {
                        // Charge flow
                        g_ucLedSysStatus = LED_BAT_FAST_CV_CHARGE;

                        // 61% ~ 90%
                        calValue = g_ausADCValue[1][0];
                        if (calValue > BATTERY_CHARGE_CURRENT_CM)
                            calValue = BATTERY_CHARGE_CURRENT_CM;
                        else if (calValue < BATTERY_CHARGE_CURRENT_C90)
                            calValue = BATTERY_CHARGE_CURRENT_C90;
                        calValue = BATTERY_CHARGE_CURRENT_CM - calValue;
                        batteryPercentage = (calValue) * 100 /
                                            (BATTERY_CHARGE_CURRENT_CM - BATTERY_CHARGE_CURRENT_C90);
                        batteryPercentage = (batteryPercentage * (BATTERY_CHARGE_L90 - BATTERY_CHARGE_LM)) / 100 + BATTERY_CHARGE_LM;
                        if (g_ucBatteryInsertEstimateTime == 0) // For normal estimate
                        {
                            if (batteryPercentage > g_ucBatteryPercentage)
                                g_ucBatteryPercentage++;
                        }
                        else // For fast estimate
                        {
                            g_ucBatteryInsertEstimateTime--;
                            if (batteryPercentage > g_ucBatteryPercentage)
                                g_ucBatteryPercentage = batteryPercentage;
                        }
                    }

                    // (90%)
                    if (g_ucBatteryChargeMode == BATTERY_RE_NORMAL_CHARGE_MODE)
                    {
                        g_ucBatteryInsertEstimateTime = 0;
                        batteryPercentage = 90;
                        if (batteryPercentage > g_ucBatteryPercentage)
                            g_ucBatteryPercentage = batteryPercentage;
                    }
                }
                break;
            }

        // Re normal charge
        case BATTERY_RE_NORMAL_CHARGE_MODE:
            {
                // Charge flow
                g_ucLedSysStatus = LED_BAT_RE_NORMAL_CHARGE;

                //
                if (g_ucPwStateMachine == PW_STEP_4)
                {
                    g_ucBatteryChargeState = CHARGE_RE_NORMAL;

                    // Countdown stop charger
                    if (g_ulBatteryStopChargerTimer > 0)
                    {
                        g_ulBatteryStopChargerTimer--;

                        batteryPercentage = 90 + ((BATTERY_ALG_TIMER_2_HR - g_ulBatteryStopChargerTimer) * 10 / BATTERY_ALG_TIMER_2_HR);
                        if (batteryPercentage > g_ucBatteryPercentage)
                            g_ucBatteryPercentage = batteryPercentage;
                    }
                    else
                    {
                        g_ucBatteryPercentage = 100;

                        g_ulBatteryChargeTime = 0;
                        g_ulBatteryStopChargerErrCount = 0;
                        g_ucBatteryChargeMode = BATTERY_STOP_CHARGE_MODE;
                        g_ucBatteryChargeState = CHARGE_STOP;

                        // Info HT66 
                        g_ucHT66ActionFlagMCU = 1;
                        g_ucHT66CommandMcu[0] = HT66_ADDRESS;
                        g_ucHT66CommandMcu[1] = HT66_CMD_0x54;
                        g_ucHT66CommandMcu[2] = HT66_ADDR_0x07;
                        g_ucHT66CommandMcu[3] = 0x01;
                    }
                }

                break;
            }

        // Stop charge
        case BATTERY_STOP_CHARGE_MODE:
            {
                // Charge flow
                g_ucBatteryChargeState = CHARGE_STOP;

                // Action
                //if(g_ucHT66Flash0x40 & BATTERY_FLOAT_CHARGE_FUNCTION)
                //{
                //  EN_CHARGER_POWER_H;
                //  EN_CHARGER_H; 
                //  g_ucLedSysStatus = LED_BAT_FLOAT_CHARGE;
                //}
                //else
                {
                    //EN_CHARGER_L;
                    //EN_CHARGER_POWER_L;
                    g_ucLedSysStatus = LED_BAT_FULL;
                }

                // Re-action after system restart
                switch (g_ucBatterySystemOnCharge)
                {
                    //
                    case 0:
                        if (PWRGOOD_N_MCU1 > 0) // System off
                            g_ucBatterySystemOnCharge = 1;
                        break;

                    //
                    case 1:
                        if (PWRGOOD_N_MCU1 == 0) // System on
                        {
                            g_ucBatterySystemOnCharge = 0;
                            g_ucBatteryChargeMode = BATTERY_NORMAL_CHARGE_MODE;
                        }
                        break;
                }
                break;
            }
    }

    return;
}

//
uint8_t BAT_BatteryPercentageLiPo(void)
{
    //uint32_t batteryPercentage;
    //uint8_t cellCount;
    uint8_t cellExist;

    cellExist = 0;
/*
    if ((g_ausBqBatteryState[1][0] == BQ20Z45_CHECK_TCA9546A) && // No switch
       (g_ausBqBatteryState[2][0] == BQ20Z45_CHECK_TCA9546A) &&
       (g_ausBqBatteryState[3][0] == BQ20Z45_CHECK_TCA9546A))
    {
        if (g_ausBqBatteryState[0][1] < BQ20Z45_TX_RX_ERROR_THRESHOLD)
        {
            g_ucBatteryPercentage = g_ausBqBatteryRelativeStateOfCharge[0][0];
            cellExist |= 0x01;
        }
        else
            g_ucBatteryPercentage = 0;
    }
    else // Switch
    {
        cellCount = 0;
        batteryPercentage = 0;
        
        if((g_ausBqBatteryState[0][0] == 0) && (g_ausBqBatteryState[0][1] < BQ20Z45_TX_RX_ERROR_THRESHOLD))
        {
          g_ucBatteryPercentage += g_ausBqBatteryRelativeStateOfCharge[0][0];
          cellCount++;
          cellExist |= 0x01; 
        }
        if((g_ausBqBatteryState[1][0] == 0) && (g_ausBqBatteryState[1][1] < BQ20Z45_TX_RX_ERROR_THRESHOLD))
        {
          g_ucBatteryPercentage += g_ausBqBatteryRelativeStateOfCharge[1][0];
          cellCount++;
          cellExist |= 0x02; 
        }
        if((g_ausBqBatteryState[2][0] == 0) && (g_ausBqBatteryState[2][1] < BQ20Z45_TX_RX_ERROR_THRESHOLD))
        {
          g_ucBatteryPercentage += g_ausBqBatteryRelativeStateOfCharge[2][0];
          cellCount++;
          cellExist |= 0x04; 
        }
        if((g_ausBqBatteryState[3][0] == 0) && (g_ausBqBatteryState[3][1] < BQ20Z45_TX_RX_ERROR_THRESHOLD))
        {
          g_ucBatteryPercentage += g_ausBqBatteryRelativeStateOfCharge[3][0];
          cellCount++;
          cellExist |= 0x08; 
        }
        
        if (g_ausBqBatteryState[0][1] < BQ20Z45_TX_RX_ERROR_THRESHOLD)
        {
            batteryPercentage += g_ausBqBatteryRelativeStateOfCharge[0][0];
            cellCount++;
            cellExist |= 0x01;
        }
        if (g_ausBqBatteryState[1][1] < BQ20Z45_TX_RX_ERROR_THRESHOLD)
        {
            batteryPercentage += g_ausBqBatteryRelativeStateOfCharge[1][0];
            cellCount++;
            cellExist |= 0x02;
        }
        if (g_ausBqBatteryState[2][1] < BQ20Z45_TX_RX_ERROR_THRESHOLD)
        {
            batteryPercentage += g_ausBqBatteryRelativeStateOfCharge[2][0];
            cellCount++;
            cellExist |= 0x04;
        }
        if (g_ausBqBatteryState[3][1] < BQ20Z45_TX_RX_ERROR_THRESHOLD)
        {
            batteryPercentage += g_ausBqBatteryRelativeStateOfCharge[3][0];
            cellCount++;
            cellExist |= 0x08;
        }
        if (cellCount != 0)
            batteryPercentage = batteryPercentage / cellCount;
        g_ucBatteryPercentage = batteryPercentage;
    }
*/
    return cellExist;
}

//
void BAT_DischargeAlgorithmLiPo(void)
{
		if (PSON_MCU1 == 1 && AC_FAIL_N_MCU1 == 0)
    {
			g_ucBatteryError = 0;
      if (g_ucBatteryPercentage < 15 && g_ucBatteryVoltage != 0)  // < 15%
				g_ucLedSysStatus = LED_BAT_LOW_VOLTAGE; // LED
      else
				g_ucLedSysStatus = LED_BAT_DISCHARGE; // LED

			if (g_ausBqBatteryState[0][1] < 15)
			{
				if (g_ucBatteryVoltage < 11200)
        {
            g_PowerUVP1++;
            if (g_PowerUVP1 > 10)                             // 10 ~ 20 ms
            {
							g_PowerUVP1 =0;
              g_Battery_Low = 1;
              g_ausBqBatteryCellVoltage8[0] |= 1;
            }
						else
						  g_Battery_Low = 0;
        }
        else
            g_PowerUVP1 = 0;
			
			}
			else
			{
				if (g_ausADCReg[0] < BATTERY_VOLTAGE_11_5)
        {
            g_PowerUVP1++;
            if (g_PowerUVP1 > 10)                             // 10 ~ 20 ms
            {
							g_PowerUVP1 =0;
              g_Battery_Low = 1;
              g_ausBqBatteryCellVoltage8[0] |= 1;
            }
						else
						  g_Battery_Low = 0;
        }
        else
            g_PowerUVP1 = 0;
			}
		    //
			//BI_ACT_L;
			//EN_FAST_CHARGER_H;
			//EN_288V_H;
			//EN_CHARGER_L;
			//EN_CHARGER_POWER_L;
			g_ulBatteryChargerDelay = ENABLE_CHARGER_DELAY_TIME;
			g_Rock_value[7] |= 8;
			g_ulBatteryStopChargerErrCount = 0;
			g_ulBatteryChargeTime = 0;
			//g_ucLedSysStatus = LED_BAT_DISCHARGE;
			g_ucBatteryChargeState = CHARGE_STOP;
			g_ulBatteryTime++;		
    }
		else if(AC_FAIL_N_MCU1 == 0)
			g_ucLedSysStatus = LED_BAT_FULL; // LED

}

//
void BAT_ChargeAlgorithmLiPo(void)
{
    uint32_t temperature;//, value_x, value_y, value_z; 
    uint32_t batteryPercentage;
    uint8_t cellExist;
		//temperature = g_ausBqBatteryTemperature[g_ucBqStateMachineBatteryChannel][0] +
		//g_ausBqBatteryTemperature[g_ucBqStateMachineBatteryChannel][1] * 0x100;

		// Battery check
    if (g_ucBatteryCheckFlag == 1)
			return;

    //
    g_ucBq24745ChargerMode = BQ24745_MODE_L0;
    //g_ucBq24745ChargerModeFlag = g_ucBq24745ChargerMode;
    g_ulBatteryTime = 0;
    g_ucBatteryDischargeOcpFlag = 0;

    //
    g_ulBatteryTime = 0;
    g_ucBatteryLowState = 0;
    //BATTERY_LOW_PIN_L;
    g_ucBeepPeriod = 0;
    //BATTERY_LOW_BEEP_PIN_L;
    //HAL_TIM_PWM_Stop(&g_TimHandle_Beep, TIM_CHANNEL_2);

    // Battery percentage 
    cellExist = BAT_BatteryPercentageLiPo();
		// Battery Status
		if ((g_ausBqBatteryStatus[1] >> 7) || (g_ausBqBatteryStatus[0] >> 7 == 0))//Battery status stop  (over charger alarm) (Battery OK alarm)
        g_ucBatteryChargerState |= BATTERY_OVER_BIT15;
    else
        g_ucBatteryChargerState &= BATTERY_UNDER_BIT15;
		
		//Battery Voltage Check
		/*if (g_ausADCValue[0][0] > BATTERY_VOLTAGE_11_5) // over 11.5V
    {
        g_ulBatteryChargeTime = 0;
        g_ulBatteryStopChargerErrCount = 0;
        g_ucBatteryChargeMode = BATTERY_STOP_CHARGE_MODE;
        g_ucBatteryChargeState = CHARGE_STOP;

        // Info HT66 
        g_ucHT66ActionFlagMCU = 1;
        g_ucHT66CommandMcu[0] = HT66_ADDRESS;
        g_ucHT66CommandMcu[1] = HT66_CMD_0x54;
        g_ucHT66CommandMcu[2] = HT66_ADDR_0x07;
        g_ucHT66CommandMcu[3] = 0x03;

        g_ausBqBatteryCellVoltage8[0] |= 32;
        g_ucBatteryChargerState |= BATTERY_OVER_VOLTAGE;
    }
    else
        g_ucBatteryChargerState &= BATTERY_UNDER_VOLTAGE;*/

    if (g_ucBatteryChargerState >> 2)
    {
        g_ucBatteryChargerStateCounter++;
        if (g_ucBatteryChargerStateCounter > 4)
        {
            g_ucBatteryChargerStateCounter = 0;
            //g_ucBatteryChargeMode = BATTERY_STOP_CHARGE_MODE;
        }
    }
    else
        g_ucBatteryChargerStateCounter = 0;

    // One battery pack
    if (g_ucBqBatteryMultiMode == 0)
    {
        g_ucBqBatteryMultiReadLagTime = 0;

        // Open charger IC power (Delay 2 Second)
        if (g_ucPwStateMachine == PW_STEP_4)
        {
            // Check system watt
            if (g_ucBatteryHaltChargeFlag == 1)
            {
                if (g_dSysWatt < CHARGE_RELEASE_WATT)
                    g_ucBatteryHaltChargeFlag = 0;

                g_ucBatteryChargeStartLiPo = 0; // Single	
                g_ulBqBatteryDelayTimer = 0; // Multi
            }
            else
            {
                if (g_dSysWatt > CHARGE_HALT_WATT)
                {
                    g_ucBatteryHaltChargeFlag = 1;
                    //EN_CHARGER_H; // Disable charger IC power
										//g_ucBq24745ChargerMode = BQ24745_MODE_L6;
										//g_ucBQ24725AChargerOptions = BQ24725A_STOP_CHARGE;

                    // Single
                    g_ucBatteryChargeStartLiPo = 0;

                    // Multi
                    g_ulBqBatteryDelayTimer = 0;
                    g_ucBqBatteryMultiChargerSemaphore = 0;
                    g_ucBatteryHT66ActionFlagMCU = 0;
                }
            }

            if (g_ucBatteryChargeStartLiPo > 3)
            {
                if (g_ucBatteryCheckFlag == 0)
								{
										//g_ucBQ24725AChargerOptions = BQ24725A_DEFAULT_CHARGE;
                    //EN_CHARGER_L; // Enable (BQ24745)
                                  ; // (LTC4100)
								}
                else
                    //g_ucBq24745ChargerMode = BQ24745_MODE_L6;
										//g_ucBQ24725AChargerOptions = BQ24725A_STOP_CHARGE;
										;//EN_CHARGER_H; // Disable charger IC power
            }
            else
            {
                g_ucBatteryChargeStartLiPo++;
                //g_ulBq24745PrechargeTimer = 0;
            }
        }
        else
        {
            //EN_CHARGER_H; // Disable charger IC power
						//g_ucBq24745ChargerMode = BQ24745_MODE_L6;
						//g_ucBQ24725AChargerOptions = BQ24725A_STOP_CHARGE;
            g_ucBatteryChargeStartLiPo = 0;
            //g_ulBq24745PrechargeTimer = 0;
        }

        // Wait semaphore (After TSK_Bq20z45Action())
        //if (g_ucBqStateMachineSemaphore == 0)
            //return;
        g_ucBqStateMachineSemaphore = 0; // Release to TSK_Bq20z45Action()

        //
        if ((g_ausBqBatteryState[g_ucBqStateMachineBatteryChannel][0] == 0) || // Normal
           (g_ausBqBatteryState[g_ucBqStateMachineBatteryChannel][1] < BQ20Z45_TX_RX_ERROR_THRESHOLD)) // Add fuzzy time 
        {
            // Battery percentage
            //batteryPercentage = g_ausBqBatteryRelativeStateOfCharge[g_ucBqStateMachineBatteryChannel][0];
            // Delay 5 sec
            if((3 < g_ucBatteryChargeStartLiPo) && (g_ucBatteryChargeStartLiPo < 14)) 
              g_ucBatteryChargeStartLiPo++;

            // Charger action
            //if(g_ucBatteryChargeStartLiPo > 6)
            //if (g_ucBatteryChargeStartLiPo > 3)
            //{
                //
                switch (g_ucBatteryChargeMode)
                {
                    // Normal charge
                    case BATTERY_NORMAL_CHARGE_MODE:
                        {
                            //
                            g_ucBatterySystemOnCharge = 0;
                            g_ucLedSysStatus = LED_BAT_NORMAL_CHARGE; // LED													
														g_ucBatteryChargerState &= BATTERY_UNDER_TEMPERATURE;
                            // Delay for bq24745 cmd
                            if (g_ulBq24745DelayCmdTimer > g_ulSysTick1Sec)
                                break;
                            g_ulBq24745DelayCmdTimer = g_ulSysTick1Sec + 5; // Period 5 second
														//g_ucBQ24725AChargerOptions = BQ24725A_DEFAULT_CHARGE;
														if (g_ucBatteryVoltage < 10400 && g_ucBatteryVoltage != 0)
															g_ucBq24745ChargerMode = BQ24745_MODE_L1;
                            // Pre-charge action
                            if (g_ucBatteryVoltage >= 10400 && g_ulBq24745PrechargeTimer == 0)
                            {
                                g_ulBq24745PrechargeTimer = g_ulSysTick1Sec + 30; // Pre-charge 30 second
                                g_ucBq24745ChargerMode = BQ24745_MODE_L2;					// 512mA
                            }
                            else if (g_ulBq24745PrechargeTimer > g_ulSysTick1Sec)
                            {
                                g_ucBq24745ChargerMode = BQ24745_MODE_L2;					// 512mA
																if(g_ucBatteryCurrent < 10)
																{
																	g_ucBatteryError = 1;
																	g_ucBatteryChargeMode = BATTERY_STOP_CHARGE_MODE;			//Battery Error: fail to charge
																	break;
																}
                            }
                            else
                            {
                                // 
                                if (g_ulBqBatteryTemperature < BQ20Z45_CK_TEMPERATURE_L2)       // LT ,  12DegC
                                    g_ucBq24745ChargerMode = BQ24745_MODE_L3;     // 1535mA
                                else if (BQ20Z45_CK_TEMPERATURE_L4 < g_ulBqBatteryTemperature)  // HT ,  55DegC
                                    g_ucBq24745ChargerMode = BQ24745_MODE_L5;     // 512mA
                                else                                              							// ST1 and ST2
                                    g_ucBq24745ChargerMode = BQ24745_MODE_L4;     // 2560mA
																if(g_ucBatteryPercentage < 99 && g_ucBatteryCurrent < 10)
																{
																	g_ucBatteryError = 1;
																	g_ucBatteryChargeMode = BATTERY_STOP_CHARGE_MODE;			//Battery Error: fail to charge or suspend
																	break;
																}																
                            }
														
                            //Battery Full = Percentage >= 98%, Battery Voltage > 16.6V, Battery Current <= 300mA
                            if (g_ucBatteryPercentage >= 98 && g_ucBatteryVoltage > 16600 && g_ucBatteryCurrent <= 300)
                            {
                                if (g_ucBatteryChargeStartLiPo > 3) // Wait 1 Sec
                                {
                                    g_ucBatteryChargeMode = BATTERY_STOP_CHARGE_MODE;
																		g_ucBatteryFullFlag = 1;
                                }
                                else
                                    g_ucBatteryChargeStartLiPo++;
                            }
                            break;
                        }

                    // Stop charge
                    case BATTERY_STOP_CHARGE_MODE:
                        {
                            // Charge flow 
														g_ulBq24745PrechargeTimer = 0;
                            g_ucLedSysStatus = LED_BAT_FULL;
                            //EN_CHARGER_H; // Disable charger IC power
														//g_ucBQ24725AChargerOptions = BQ24725A_STOP_CHARGE;
                            g_ucBatteryChargeStartLiPo = 0;
                            g_ucBq24745ChargerMode = BQ24745_MODE_L6;  // Reset 0 remainng value to charger after stopping charge	
														if(g_ucBatteryError == 1)  //Battery Error
                            {
																g_ucBatteryChargerState |= BATTERY_OVER_ocp;
                                g_ucBatteryChargeStartLiPo = 0;
                                g_ucBatteryChargeMode = BATTERY_STOP_CHARGE_MODE;
                                break;
                            }
														else
															g_ucBatteryChargerState &= BATTERY_UNDER_ocp;
														// Check temperature 0C < battery temperature < 47C
														if ((g_ulBqBatteryTemperature < BQ20Z45_CK_TEMPERATURE_L1) || (BQ20Z45_CK_TEMPERATURE_L4 < g_ulBqBatteryTemperature))
                            {
                                //EN_CHARGER_H; // Disable charger IC power
																//g_ucBQ24725AChargerOptions = BQ24725A_STOP_CHARGE;
                                g_ucBatteryChargeStartLiPo = 0;
																g_ucBatteryChargerState |= BATTERY_OVER_TEMPERATURE;
                                g_ucBatteryChargeMode = BATTERY_STOP_CHARGE_MODE;
                                break;
                            }
														else
															g_ucBatteryChargerState &= BATTERY_UNDER_TEMPERATURE;
														// Check Percentage after full charge if battery <=95 start charge
														if (g_ucBatteryPercentage <= 95 && g_ucBatteryVoltage != 0)
														{
															g_ucBatteryChargerState &= BATTERY_UNDER_PERCENTAGE;
															// Check AC and LLC Power OK
															if (AC_FAIL_N_MCU1 == 1 && ACINOK_N_MCU1 == 0) //  AC On and AC2DC Power OK
															{
																g_ucBatteryChargeMode = BATTERY_NORMAL_CHARGE_MODE;
															}
														}
														else
														{
																g_ucBatteryChargeMode = BATTERY_STOP_CHARGE_MODE;
																g_ucBatteryChargerState |= BATTERY_OVER_PERCENTAGE;
																if (g_ucBatteryChargerShutdownVCC > 3) // Wait 1 Sec
																		g_ucBatteryFullFlag = 1;
																else
                                    g_ucBatteryChargerShutdownVCC++;
														}												
													break;
												}
                }
        }
        else // Abnormal
        {
            //EN_CHARGER_H; // Disable charger IC power
						//g_ucBQ24725AChargerOptions = BQ24725A_STOP_CHARGE;
            g_ucBatteryChargeStartLiPo = 0; // Enable charger IC timer
        }

        g_ucBq24745ChargerModeFlag[0] = g_ucBq24745ChargerMode;
    }

    // Multi Battery pack
    if (g_ucBqBatteryMultiMode > 0)
    {
        // Wait semaphore (After TSK_Bq20z45Action())
        if (g_ucBqStateMachineSemaphore == 0)
            return;

        // Check system watt
        if (g_ucBatteryHaltChargeFlag == 1)
        {
            if (g_dSysWatt < CHARGE_RELEASE_WATT)
                g_ucBatteryHaltChargeFlag = 0;

            g_ucBatteryChargeStartLiPo = 0; // Single	
            g_ulBqBatteryDelayTimer = 0; // Multi
        }
        else
        {
            if (g_dSysWatt > CHARGE_HALT_WATT)
            {
                g_ucBatteryHaltChargeFlag = 1;
                //EN_CHARGER_H; // Disable charger IC power

                // Single
                g_ucBatteryChargeStartLiPo = 0;

                // Multi
                g_ulBqBatteryDelayTimer = 0;
                g_ucBqBatteryMultiChargerSemaphore = 0;
                g_ucBatteryHT66ActionFlagMCU = 0;
            }
        }

        // Check charger Semaphore
        if (g_ucBqBatteryMultiChargerSemaphore == 0)
        {
            g_ucLedSysStatus = LED_BAT_SELECT_CHARGE_CH;

            // Delay for scan
            if (g_ulBqBatteryDelayTimer == 0)
            {
                g_ucBqStateMachineSemaphore = 0; // Release to TSK_Bq20z45Action()
                g_ulBqBatteryDelayTimer = g_ulSysTick1Sec + 6; // Period 6 second
                return;
            }
            else if (g_ulBqBatteryDelayTimer > g_ulSysTick1Sec)
            {
                g_ucBqStateMachineSemaphore = 0; // Release to TSK_Bq20z45Action()
                return;
            }

            // Select charge channel
            if (g_ucBatteryHT66ActionFlagMCU == 0)      // Channel select
            {
                //
                g_ucBqBatteryMultiSelectChannel = 0;
                g_ucBatteryHT66CommandMcu[2] = 0x21;
                g_ucBatteryHT66CommandMcu[3] = 0xF0;
                batteryPercentage = 100;
                if ((g_ausBqBatteryState[0][0] == 0) && (g_ausBqBatteryState[0][1] < BQ20Z45_TX_RX_ERROR_THRESHOLD))
                {
                    if (g_ausBqBatteryRelativeStateOfCharge[0][0] < batteryPercentage)
                    {
                        batteryPercentage = g_ausBqBatteryRelativeStateOfCharge[0][0];
                        g_ucBatteryHT66CommandMcu[2] = 0x21;
                        g_ucBatteryHT66CommandMcu[3] = 0x0F;
                    }
                }
                if ((g_ausBqBatteryState[1][0] == 0) && (g_ausBqBatteryState[1][1] < BQ20Z45_TX_RX_ERROR_THRESHOLD))
                {
                    if (g_ausBqBatteryRelativeStateOfCharge[1][0] < batteryPercentage)
                    {
                        g_ucBqBatteryMultiSelectChannel = 1;
                        batteryPercentage = g_ausBqBatteryRelativeStateOfCharge[1][0];
                        g_ucBatteryHT66CommandMcu[2] = 0x22;
                        g_ucBatteryHT66CommandMcu[3] = 0x0F;
                    }
                }
                if ((g_ausBqBatteryState[2][0] == 0) && (g_ausBqBatteryState[2][1] < BQ20Z45_TX_RX_ERROR_THRESHOLD))
                {
                    if (g_ausBqBatteryRelativeStateOfCharge[2][0] < batteryPercentage)
                    {
                        g_ucBqBatteryMultiSelectChannel = 2;
                        batteryPercentage = g_ausBqBatteryRelativeStateOfCharge[2][0];
                        g_ucBatteryHT66CommandMcu[2] = 0x23;
                        g_ucBatteryHT66CommandMcu[3] = 0x0F;
                    }
                }
                if ((g_ausBqBatteryState[3][0] == 0) && (g_ausBqBatteryState[3][1] < BQ20Z45_TX_RX_ERROR_THRESHOLD))
                {
                    if (g_ausBqBatteryRelativeStateOfCharge[3][0] < batteryPercentage)
                    {
                        g_ucBqBatteryMultiSelectChannel = 3;
                        batteryPercentage = g_ausBqBatteryRelativeStateOfCharge[3][0];
                        g_ucBatteryHT66CommandMcu[2] = 0x24;
                        g_ucBatteryHT66CommandMcu[3] = 0x0F;
                    }
                }

                //
                g_ucBatteryHT66ActionFlagMCU = 1;
                g_ucBatteryHT66CommandMcu[0] = BATTERY_HT66_ADDRESS;
                g_ucBatteryHT66CommandMcu[1] = HT66_CMD_0x5D;
                g_ucBatteryHT66CmdDoubleFlag = 1;
                return;
            }
            else if (g_ucBatteryHT66ActionFlagMCU == 1) // Channel process
            {
                return;
            }
            else if (g_ucBatteryHT66ActionFlagMCU == 2) // Channel fail
            {
                g_ucBatteryHT66ActionFlagMCU = 1;
                g_ucBqStateMachineSemaphore = 0; // Release to TSK_Bq20z45Action()
                g_ucBqBatteryMultiReadLagTime = 0;
                return;
            }
            else if (g_ucBatteryHT66ActionFlagMCU == 3) // Channel success
            {
                g_ucBatteryHT66ActionFlagMCU = 0;
                g_ucBqStateMachineSemaphore = 0; // Release to TSK_Bq20z45Action()
                if (g_ucBatteryHT66CommandMcu[3] == 0x0F) // Battery charge 
                {
                    g_ucBqBatteryMultiChargerSemaphore = 1;

                    //
                    g_ucBqBatteryMultiReadLagTime = 0;
                    g_ucBatteryChargeStartLiPo = 0;

                    // Set charge goal
                    batteryPercentage = g_ausBqBatteryRelativeStateOfCharge[g_ucBqBatteryMultiSelectChannel][0];
                    batteryPercentage = (10 - batteryPercentage % 10) + batteryPercentage;
                    if (batteryPercentage > 100)
                        batteryPercentage = 100;
                    g_ucBqBatteryMultiChargeGoal = batteryPercentage;
                }
                else // Battery full
                {
                    //
                    g_ucBqBatteryMultiReadLagTime++;
                    if (g_ucBqBatteryMultiReadLagTime > 10)
                    {
                        g_ucBqBatteryMultiChargerSemaphore = 2;

                        //
                        g_ucBqBatteryMultiReadLagTime = 0;
                        g_ucBqBatteryMultiCellExist = cellExist;
                        g_ucBatterySystemOnCharge = 0;

                        //if(batteryPercentage == 100)
                        //{
                        //  if(g_ucBatteryChargeStartLiPo > 14) // Wait 5 Sec
                        //  {
                        //    g_ucBatteryChargeMode = BATTERY_STOP_CHARGE_MODE;
                        //    
                        //    // Info HT66 
                        //    g_ucHT66ActionFlagMCU = 1;
                        //    g_ucHT66CommandMcu[0] = HT66_ADDRESS;
                        //    g_ucHT66CommandMcu[1] = HT66_CMD_0x5D;
                        //    g_ucHT66CommandMcu[2] = HT66_ADDR_0x03;
                        //    g_ucHT66CommandMcu[3] = 0x0F;
                        //    g_ucHT66CmdDoubleFlag = 1;
                        //  }
                        //  else
                        //    g_ucBatteryChargeStartLiPo++;
                        //}
                    }
                }
            }
        }

        // Charge flow
        if (g_ucBqBatteryMultiChargerSemaphore == 1)
        {
            g_ucBqStateMachineSemaphore = 0; // Release to TSK_Bq20z45Action()

            //
            g_ucDebug[0] = g_ucBqBatteryMultiSelectChannel; // TEST
            switch (g_ucBqBatteryMultiSelectChannel)
            {
                case 0:
                    g_ucLedSysStatus = LED_BAT_MULTI_CHARGE_CH0; // LED
                    break;

                case 1:
                    g_ucLedSysStatus = LED_BAT_MULTI_CHARGE_CH1; // LED
                    break;

                case 2:
                    g_ucLedSysStatus = LED_BAT_MULTI_CHARGE_CH2; // LED
                    break;

                case 3:
                    g_ucLedSysStatus = LED_BAT_MULTI_CHARGE_CH3; // LED
                    break;
            }

            // Check battery exist 
            if (g_ausBqBatteryState[g_ucBqBatteryMultiSelectChannel][1] >= BQ20Z45_TX_RX_ERROR_THRESHOLD)
            {
                //
                //EN_CHARGER_H; // Disable charger IC power
                g_ucBqBatteryMultiChargerSemaphore = 0;
                g_ucBatteryHT66ActionFlagMCU = 0;
                g_ucBq24745ChargerMode = BQ24745_MODE_L0;
                g_ucBatteryChargeStartLiPo = 0;
                g_ulBqBatteryDelayTimer = 0;
                return;
            }

            //////////////////

            // Open charger IC power (Delay 2 Second)
            if (g_ucPwStateMachine == PW_STEP_4)
            {
                if (g_ucBatteryChargeStartLiPo > 3)
                {
                    if (g_ucBatteryCheckFlag == 0)
                        ;//EN_CHARGER_L; // Enable (BQ24745)
                                      //; // (LTC4100)
                    else
                        ;//EN_CHARGER_H; // Disable charger IC power
                }
                else
                {
                    g_ucBatteryChargeStartLiPo++;
                    g_ulBq24745PrechargeTimer = 0;
                }
            }
            else
            {
                //EN_CHARGER_H; // Disable charger IC power
                g_ucBatteryChargeStartLiPo = 0;
                g_ulBq24745PrechargeTimer = 0;
            }

            // Check battery percentage
            if (g_ausBqBatteryRelativeStateOfCharge[g_ucBqBatteryMultiSelectChannel][0] >= g_ucBqBatteryMultiChargeGoal)
            {
                //EN_CHARGER_H; // Disable charger IC power
                g_ucBqBatteryMultiChargerSemaphore = 0;
                g_ucBatteryHT66ActionFlagMCU = 0;
                g_ucBq24745ChargerMode = BQ24745_MODE_L0;
                g_ucBatteryChargeStartLiPo = 0;
                g_ulBqBatteryDelayTimer = 0;
                return;
            }

            //// Delay 1 sec
            //if((3 < g_ucBatteryChargeStartLiPo) && (g_ucBatteryChargeStartLiPo < 7)) 
            //  g_ucBatteryChargeStartLiPo++;
            // Charger action
            //if(g_ucBatteryChargeStartLiPo > 6)
            if (g_ucBatteryChargeStartLiPo > 3)
            {
                // Check temperature
                temperature = g_ausBqBatteryTemperature[g_ucBqBatteryMultiSelectChannel][0] +
                              g_ausBqBatteryTemperature[g_ucBqBatteryMultiSelectChannel][1] * 0x100;
                if ((temperature < BQ20Z45_CK_TEMPERATURE_L1) || (BQ20Z45_CK_TEMPERATURE_L5 < temperature))
                {
                    //EN_CHARGER_H; // Disable charger IC power
                    g_ucBatteryChargeStartLiPo = 0;
                    return;
                }

                // Delay for bq24745 cmd
                if (g_ulBq24745DelayCmdTimer > g_ulSysTick1Sec)
                    return;
                g_ulBq24745DelayCmdTimer = g_ulSysTick1Sec + 5; // Period 5 second

                // Pre-charge action 
                if (g_ulBq24745PrechargeTimer == 0)
                {
                    g_ulBq24745PrechargeTimer = g_ulSysTick1Sec + 30; // Pre-charge 30 second
                    g_ucBq24745ChargerMode = BQ24745_MODE_L2;
                }
                else if (g_ulBq24745PrechargeTimer > g_ulSysTick1Sec)
                {
                    g_ucBq24745ChargerMode = BQ24745_MODE_L2;
                }
                else
                {
                    // 
                    if (temperature < BQ20Z45_CK_TEMPERATURE_L2)       // LT
                        g_ucBq24745ChargerMode = BQ24745_MODE_L3;
                    else if (BQ20Z45_CK_TEMPERATURE_L4 < temperature)  // HT
                        g_ucBq24745ChargerMode = BQ24745_MODE_L5;
                    else                                              // ST1 and ST2
                        g_ucBq24745ChargerMode = BQ24745_MODE_L4;
                }
            }

            g_ucBq24745ChargerModeFlag[g_ucBqBatteryMultiSelectChannel] = g_ucBq24745ChargerMode;
            //////////////////
        }

        // Battery full
        if (g_ucBqBatteryMultiChargerSemaphore == 2)
        {
            g_ucBqStateMachineSemaphore = 0; // Release to TSK_Bq20z45Action()

            g_ucLedSysStatus = LED_BAT_FULL;

            // Charge flow 
            //g_ucLedSysStatus = LED_BAT_FULL;
            //EN_CHARGER_H; // Disable charger IC power
            g_ucBatteryChargeStartLiPo = 0;
            g_ucBq24745ChargerMode = BQ24745_MODE_L0;

            // Re-action after system restart
            switch (g_ucBatterySystemOnCharge)
            {
                //
                case 0:
                    if (PWRGOOD_N_MCU1 > 0) // System off
                        g_ucBatterySystemOnCharge = 1;
                    break;

                //
                case 1:
                    if (PWRGOOD_N_MCU1 == 0) // System on
                    {
                        g_ucBatteryHT66ActionFlagMCU = 0;
                        g_ucBqBatteryMultiReadLagTime = 0;
                        g_ucBqBatteryMultiChargerSemaphore = 0;
                        g_ulBqBatteryDelayTimer = 0;
                    }
                    break;
            }

            //
            if (g_ucBqBatteryMultiCellExist != cellExist)
            {
                g_ucBatteryHT66ActionFlagMCU = 0;
                g_ucBqBatteryMultiReadLagTime = 0;
                g_ucBqBatteryMultiChargerSemaphore = 0;
                g_ulBqBatteryDelayTimer = 0;
            }
        }
    }
}

//
void BAT_DischargeAlgorithmLiIon_LiFePO4(void)
{

	if ((PSON_MCU1 == 1) && (AC_FAIL_N_MCU1 == 0))
    {
			
			if (g_ausBqBatteryState[0][1] < 15)
			{
				if (g_ausBqBatteryVoltage[0][0] + g_ausBqBatteryVoltage[0][1] * 0x100 < 20900)
        {
            g_PowerUVP1++;
            if (g_PowerUVP1 > 10)                             // 10 ~ 20 ms
            {
							g_PowerUVP1 =0;
              g_Battery_Low = 1;
              g_ausBqBatteryCellVoltage8[0] |= 1;
            }
						else
						  g_Battery_Low = 0;
        }
        else
            g_PowerUVP1 = 0;
			
			}
			else
			{
				if (g_ausADCReg[0] < BATTERY_VOLTAGE_20_9)
        {
            g_PowerUVP1++;
            if (g_PowerUVP1 > 10)                             // 10 ~ 20 ms
            {
							g_PowerUVP1 =0;
              g_Battery_Low = 1;
              g_ausBqBatteryCellVoltage8[0] |= 1;
            }
						else
						  g_Battery_Low = 0;
        }
        else
            g_PowerUVP1 = 0;
			}
			
    }

    //
    //BI_ACT_L;
    //EN_FAST_CHARGER_H;
    //EN_288V_H;
    //EN_CHARGER_L;
    //EN_CHARGER_POWER_L;
    g_ulBatteryChargerDelay = ENABLE_CHARGER_DELAY_TIME;
    g_Rock_value[7] |= 8;
    g_ulBatteryStopChargerErrCount = 0;
    g_ulBatteryChargeTime = 0;
    g_ucLedSysStatus = LED_BAT_DISCHARGE;
    g_ucBatteryChargeState = CHARGE_STOP;
    g_ulBatteryTime++;
}

//
void BAT_ChargeAlgorithmLiIon_LiFePO4(void)
{
    // check battery / charge status
    uint32_t Temperature;
    Temperature = g_ausBqBatteryTemperature[g_ucBqStateMachineBatteryChannel][0] +
                g_ausBqBatteryTemperature[g_ucBqStateMachineBatteryChannel][1] * 0x100;
    if (Temperature > 2732)
        Temperature -= 2732;

    g_ucBatteryChargeState = CHARGE_STOP;
    g_ulBatteryTime = 0;

    if ((g_ausBqBatteryStatus[1] >> 7) || (g_ausBqBatteryStatus[0] >> 7 == 0))//Battery status stop  (over charger alarm) (Battery OK alarm)
        g_ucBatteryChargerState |= BATTERY_OVER_BIT15;
    else
        g_ucBatteryChargerState &= BATTERY_UNDER_BIT15;

    if (g_ausADCValue[0][0] > BATTERY_VOLTAGE_30_0) // over 30V
    {
        g_ulBatteryChargeTime = 0;
        g_ulBatteryStopChargerErrCount = 0;
        g_ucBatteryChargeMode = BATTERY_STOP_CHARGE_MODE;
        g_ucBatteryChargeState = CHARGE_STOP;

        // Info HT66 
        g_ucHT66ActionFlagMCU = 1;
        g_ucHT66CommandMcu[0] = HT66_ADDRESS;
        g_ucHT66CommandMcu[1] = HT66_CMD_0x54;
        g_ucHT66CommandMcu[2] = HT66_ADDR_0x07;
        g_ucHT66CommandMcu[3] = 0x03;

        g_ausBqBatteryCellVoltage8[0] |= 32;
        g_ucBatteryChargerState |= BATTERY_OVER_VOLTAGE;
    }
    else
        g_ucBatteryChargerState &= BATTERY_UNDER_VOLTAGE;

    if ((g_ucBatteryChargerState >> 2) || (g_ausBqBatteryCellVoltage8[0] != 0))
    {
        g_ucBatteryChargerStateCounter++;
        if (g_ucBatteryChargerStateCounter > 4)
        {
            g_ucBatteryChargerStateCounter = 0;
            g_ucBatteryChargeMode = BATTERY_STOP_CHARGE_MODE;
        }
    }
    else
        g_ucBatteryChargerStateCounter = 0;
   
    // 
    switch (g_ucBatteryChargeMode)
    {
        // Normal charge
        case BATTERY_NORMAL_CHARGE_MODE:
            {
                g_ucBatterySystemOnCharge = 0;
                g_ucLedSysStatus = LED_BAT_NORMAL_CHARGE;
                //"PW_STEP_4" means ready to charge
                if (g_ucPwStateMachine == PW_STEP_4)
                {
                    g_ulBatteryChargeTime++;
                    // Action 
                    g_ucBatteryChargeState = CHARGE_NORMAL;
                    //pre normal(pluse) charge 
                    switch (CHARGER_Voltage)
                    {               // Period(T) /  pluse width
											  case 19:    // 15ms      /  600us   3min(<17.5 only)  BI_ACT_L
												if(g_ulPreBatteryChargeTime<120)
												{
												  g_ulPreBatteryChargeTime++;
													return;
												}
												case 20:    // 16ms       /  650us                    BI_ACT_L
												case 21:    // 10ms       /  850us                    BI_ACT_L
												case 22:    // 10ms       /  1ms                      BI_ACT_H
												case 23:    // EN_CHARGER_POWER                       BI_ACT_H
                            {
  
															  if((g_ausBqBatteryCurrent[0][0] + g_ausBqBatteryCurrent[0][1] * 0x100) == 0)
															    CHARGER_0Current_counter++;
															  else
																	CHARGER_0Current_counter = 0;
																
																if (((CHARGER_Voltage!=23)&&(135 > (g_ausBqBatteryCurrent[0][0] + g_ausBqBatteryCurrent[0][1] * 0x100)))||
																	  (75 > (g_ausBqBatteryCurrent[0][0] + g_ausBqBatteryCurrent[0][1] * 0x100)))
                                {  
                                    if (CHARGER_Voltage_counter >10) // delay
                                    {
                                      CHARGER_Voltage_counter = 0;
                                      if(CHARGER_Voltage == 23)
																			{
                                        g_BI_ACT_Delay = 10;																				
																				CHARGER_Voltage = 26;																				
																			}
																		  else if(CHARGER_Voltage == 19 && g_ulPreBatteryChargeTime == 120 &&
																				     (g_ausBqBatteryVoltage[0][0] + g_ausBqBatteryVoltage[0][1] * 0x100) < 17500)
																		  {																			
																				g_ucBatteryChargerState |= BATTERY_OVER_ocp;
                                        g_ulBatteryChargeTime = 0;
                                        g_ucBatteryChargeMode = BATTERY_STOP_CHARGE_MODE;
                                        g_ucBatteryChargeState = CHARGE_STOP;
                                        // Charge flow
                                        g_ucLedSysStatus = LED_BAT_FULL;
																				return;																				
																			}
																			else
																			{																		
																				g_ulBatteryChargerDelay_offset++;
																				if(g_ulBatteryChargerDelay_offset>3 || CHARGER_0Current_counter>15)
																				{
																					CHARGER_0Current_counter = 0;
																					g_ulBatteryChargerDelay_offset = 0;
																					CHARGER_Voltage++;
																				}																				
																			}
																		}
                                    else
                                        CHARGER_Voltage_counter++;
                                }
																else
																	CHARGER_Voltage_counter = 0;
                                return;															
                            }                                          														 
										}
                    
										g_ulNormalBatteryChargeTime++;
                    if (g_ulNormalBatteryChargeTime > BATTERY_ALG_TIMER_1_MIN&&((g_ausBqBatteryVoltage[0][0] + g_ausBqBatteryVoltage[0][1] * 0x100) > 21800))
                    {
                        g_ulNormalBatteryChargeTime = 0;
                        g_ucBatteryChargeCC2CV = 0;
                        g_ulBatteryStopChargerErrCount = 0;
                        g_ucBatteryChargeMode = BATTERY_FAST_CHARGE_MODE;
                        g_ucBatteryChargeState = CHARGE_FAST;

                        g_ucLedSysStatus = LED_BAT_FAST_CC_CHARGE;
                    }
                    else// decrise 0.5A charge time for test   
                    {
                        LIB_MemRead(0,
                BQ78350_I2C_ADDRESS,
                BQ78350_SBS_DEVICE_CHEMISTRY,
                3,
                &g_ausBqBatteryDeviceChemistry[0]);

                        if ((g_ausBqBatteryDeviceChemistry[1] == 0x46) ||   // 'F' or 'f' 
                   (g_ausBqBatteryDeviceChemistry[1] == 0x66))
                            g_ulNormalBatteryChargeTime += 5;
                    }
                }
                else
                {
                    //BI_ACT_L;
                    //EN_288V_H;
                    //EN_CHARGER_L;
                    //EN_CHARGER_POWER_L;
                    g_ulBatteryChargeTime = 0;
									  g_ulNormalBatteryChargeTime =0;
                    g_Rock_value[7] |= 16;
                }
                break;
            }

        // Fast charge
        case BATTERY_FAST_CHARGE_MODE:
             {
                // 
                if (g_ucPwStateMachine == PW_STEP_4)
                {
                    // Action 
                    g_ucBatteryChargeState = CHARGE_FAST;
                    g_ulBatteryChargeTime++;
									  //BI ACT delay
									  if(g_BI_ACT_Delay > 0)
										  g_BI_ACT_Delay--;
										
                    if (g_ulBatteryChargeTime > BATTERY_ALG_TIMER_5_HR)
                    {
                        g_ucBatteryChargerState |= BATTERY_OVER_5HR;
                        g_ulBatteryChargeTime = 0;
                        g_ucBatteryChargeCC2CV = 0;
                        g_ulBatteryStopChargerErrCount = 0;
                        g_ucBatteryChargeMode = BATTERY_STOP_CHARGE_MODE;
                        g_ucBatteryChargeState = CHARGE_STOP;

                        // Charge flow
                        g_ucLedSysStatus = LED_BAT_FULL;
                    }
                    //26 to 29
                    if ((CHARGER_Voltage == 26) && (g_ucBatteryPercentage > 15))
                    {
                        if (CHARGER_Voltage_counter > 10) // 0000 1100
                        {
                            CHARGER_Voltage_counter = 0;
                            CHARGER_Voltage = 29;
                        }
                        else
                            CHARGER_Voltage_counter++;
                    }
										else
											CHARGER_Voltage_counter = 0;
                    //full
                    if (300 > g_ausBqBatteryCurrent[0][0] + g_ausBqBatteryCurrent[0][1] * 0x100)  // Rock set 150 mA stop charger          			                                                             
                      if((g_ausBqBatteryVoltage[0][0] + g_ausBqBatteryVoltage[0][1] * 0x100) < 28800 && 400>Temperature)    
										    g_ucBatteryLowCurrentTimer+= 10;
											else
											  g_ucBatteryLowCurrentTimer++;
                    else
                        g_ucBatteryLowCurrentTimer = 0;									               
								}
                break;
            }

        // Stop charge
        case BATTERY_STOP_CHARGE_MODE:
            {							
                g_Rock_value[7] |= 32;
                // Charge flow
                g_ucBatteryChargeState = CHARGE_STOP;
                //EN_CHARGER_L;
                //EN_CHARGER_POWER_L;
                g_ucLedSysStatus = LED_BAT_FULL;

                if (g_ucBatteryPercentage > 90)
                    g_ucBatteryChargerState |= BATTERY_OVER_PERCENTAGE;
                else
                    g_ucBatteryChargerState &= BATTERY_UNDER_PERCENTAGE;

                if (Temperature > 399)
                    g_ucBatteryChargerState |= BATTERY_OVER_TEMPERATURE;
                else
                    g_ucBatteryChargerState &= BATTERY_UNDER_TEMPERATURE;

                //set 18 26 29V
                g_Rock_value[3] = g_ausBqBatteryVoltage[0][0] + g_ausBqBatteryVoltage[0][1] * 0x100;
										
						  	if ((g_ausBqBatteryVoltage[0][0] + g_ausBqBatteryVoltage[0][1] * 0x100) < 17500)        // 18.5V PEAK
								{
									CHARGER_Voltage = 19;
									g_ulPreBatteryChargeTime = 0;
								}
								else if((g_ausBqBatteryVoltage[0][0] + g_ausBqBatteryVoltage[0][1] * 0x100) < 21000)
								{
									CHARGER_Voltage = 19;
								  g_ulPreBatteryChargeTime = 1000;
								}
								else if((g_ausBqBatteryVoltage[0][0] + g_ausBqBatteryVoltage[0][1] * 0x100) < 22000)
								  CHARGER_Voltage = 20;
								else if((g_ausBqBatteryVoltage[0][0] + g_ausBqBatteryVoltage[0][1] * 0x100) < 22400)
						      CHARGER_Voltage = 21;
                else if (g_ucBatteryPercentage < 10)              // 26V
                  CHARGER_Voltage = 26;
                else                                              // 29V
                  CHARGER_Voltage = 29;

                // Re-action for test
                if (g_ucBatteryChargerState >> 2 == 0 && g_ausBqBatteryCellVoltage8[0] == 0)
                {
									//charge 
                    if (g_ausADCReg[8] > 2000)
                    { 
											//15V3 on					  	
                        if (g_ucBatteryChargerState == 0)
                        {
                            g_ucBatteryFullRechargeFlagCount = 20;
                            g_ucBatteryChargeMode = BATTERY_NORMAL_CHARGE_MODE;
                        }
                        else
                            g_ucBatteryChargerState--;
                    }
                    else
                    {
                        if (g_ucBatteryFullRechargeFlagCount == 0)
                        {
                            g_ucBatteryFullRechargeFlagCount = 20;
                            g_ucBatteryRechargeFlag = 1;
                        }
                        else
                            g_ucBatteryFullRechargeFlagCount--;
                    }
                }
                else                                              //Full charge
                {									  
                    if (PSON_MCU1 == 0 && g_ausADCReg[8] > 2000)
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
                // Re-action after system restart
                switch (g_ucBatterySystemOnCharge)
                {
                    //
                    case 0:
                        //if(PWRGOOD_N_MCU1 > 0) // System off
                        if (AC_FAIL_N_MCU1 == 0) // No AC
                            g_ucBatterySystemOnCharge = 1;
                        break;

                    //
                    case 1:
                        //if(PWRGOOD_N_MCU1 == 0) // System on
                        if (AC_FAIL_N_MCU1 > 0) //  AC On
                        {
                            g_ucBatterySystemOnCharge = 0;
                            g_ucBatteryChargeMode = BATTERY_NORMAL_CHARGE_MODE;
                        }
                        break;
                }
                break;
            }
    }
}
