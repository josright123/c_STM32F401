
#include "main.h"

//
#ifdef USB_MODULE 

uint32_t USB_Config(void)
{
  /* Init Device Library */
  USBD_Init(&g_hUSBDDevice, &HID_Desc, 0);
  
  /* Add Supported Class */
  USBD_RegisterClass(&g_hUSBDDevice, USBD_HID_CLASS);

  /* Start Device Process */
  USBD_Start(&g_hUSBDDevice);
  
  return 0;
}

//
uint32_t USB_UnConfig(void)
{
  /* Disconnect the USB device */
  USBD_Stop(&g_hUSBDDevice);
  USBD_DeInit(&g_hUSBDDevice);
  
  return 0;
}
#endif

//
void USB_ProcessRX(void)
{
#ifdef USB_MODULE

  uint32_t value;
  uint32_t addr; 
  uint8_t  data;  
 
  if(g_USBD_HID_OutFlag > 0)
  {

    g_USBD_HID_OutFlag = 0;
    memset(g_USBD_HID_InStream, 0x00, 64);

    // Parse packet
    switch(g_USBD_HID_OutStream[0])
    {
      //
      case USB_CMD_POLL:
      {
        // Battery check
        if(g_USBD_HID_OutStream[7] == 1)                             // Start
        {
//ROCk    EN_288V_L;
          //EN_CHARGER_L;
          //EN_CHARGER_POWER_L;
//  			EN_FAST_CHARGER_H;
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);       // Battery check pin
          g_ucBatteryCheckFlag = 1;
        }
        else if(g_USBD_HID_OutStream[7] == 2)                        // Stop
        {
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);     // Battery check pin
          g_ucBatteryCheckFlag = 0;
        }
  
        // MCU -> PC
        g_USBD_HID_InStream[0] = USB_CMD_POLL_RSP;
        memcpy(&g_USBD_HID_InStream[2], g_ucSysTime, 6);             // HH:MM:SS
        value = g_ausADCValue[0][0];                                 // ADC0
        g_USBD_HID_InStream[8] = value & 0xFF;                      
        g_USBD_HID_InStream[9] = (value>>8) & 0xFF;
        value = g_ausADCValue[1][0];                                 // ADC1
        g_USBD_HID_InStream[10] = value & 0xFF;                     
        g_USBD_HID_InStream[11] = (value>>8) & 0xFF;
        value = g_ausADCValue[2][0];                                 // ADC2
        g_USBD_HID_InStream[12] = value & 0xFF;                     
        g_USBD_HID_InStream[13] = (value>>8) & 0xFF;
        value = g_ausADCValue[3][0];
				g_USBD_HID_InStream[14] = value & 0xFF;                      
        g_USBD_HID_InStream[15] = (value>>8) & 0xFF;
        value = g_ausADCValue[4][0];                                 // ADC4
        g_USBD_HID_InStream[16] = value & 0xFF;                    
        g_USBD_HID_InStream[17] = (value>>8) & 0xFF;
        value = g_ausADCValue[5][0];                                 // ADC5
        g_USBD_HID_InStream[18] = value & 0xFF;                     
        g_USBD_HID_InStream[19] = (value>>8) & 0xFF;
        value = voutin;//g_ausADCValue[8][0];                                 // Temperature  FOR 5V AUX SELF TEST
        g_USBD_HID_InStream[20] = value & 0xFF;                     
        g_USBD_HID_InStream[21] = (value>>8) & 0xFF;
        value = g_ulFanFrequency;                                    // FAN Frequency
        g_USBD_HID_InStream[22] = value & 0xFF;               
        g_USBD_HID_InStream[23] = (value>>8) & 0xFF;
        g_USBD_HID_InStream[24] = g_ucLedSysStatus;
        //if(AC_FAIL_N_MCU1 > 0)
          g_USBD_HID_InStream[25] |= 0x80; 
        //if(BATTERY_INSERT_N1 > 0)
          g_USBD_HID_InStream[25] |= 0x40; 
        //if(BATTERY_CHECK_PIN > 0)
          g_USBD_HID_InStream[25] |= 0x20;
        g_USBD_HID_InStream[25] = (g_USBD_HID_InStream[25] & 0xF0) + g_ucBatteryType;
        value = g_ausADCValue[7][0];                                 // ADC6
        g_USBD_HID_InStream[26] = value & 0xFF;                     
        g_USBD_HID_InStream[27] = (value>>8) & 0xFF;
        value = vboost2;                                             // ADC7
        g_USBD_HID_InStream[28] = value & 0xFF;                     
        g_USBD_HID_InStream[29] = (value>>8) & 0xFF;

        if((g_ucBatteryType == LI_ION_BATTERY) || (g_ucBatteryType == LI_FE_PO_BATTERY))
          g_USBD_HID_InStream[36] = g_ucBatteryPercentage;             // Battery Percentage   
        else
        {
          if(g_ucBatteryInsertEstimateTime == 0)
            g_USBD_HID_InStream[36] = g_ucBatteryPercentage;           // Battery Percentage
          else
            g_USBD_HID_InStream[36] = 0;
        }
	
        g_USBD_HID_InStream[40] = (g_ulBatteryTime) & 0xFF;          // Battery time
        g_USBD_HID_InStream[41] = (g_ulBatteryTime>>8) & 0xFF;
				g_USBD_HID_InStream[42] = g_ucBatteryChargerState;
        g_USBD_HID_InStream[48] = g_USBD_HID_OutStream[48] + 1;      // SN
        g_USBD_HID_InStream[52] = g_ucDebug[0];
        g_USBD_HID_InStream[53] = g_ucDebug[1];
        g_USBD_HID_InStream[54] = g_ucDebug[2];
        g_USBD_HID_InStream[55] = g_ucDebug[3];
        g_USBD_HID_InStream[56] = main_program_version[0];	         // Version
        g_USBD_HID_InStream[57] = main_program_version[1];
        g_USBD_HID_InStream[58] = main_program_version[2];
        g_USBD_HID_InStream[59] = main_program_version[3];
        g_USBD_HID_InStream[60] = main_program_version[4];
        g_USBD_HID_InStream[61] = main_program_version[5];
        g_USBD_HID_InStream[62] = main_program_version[6];
        g_USBD_HID_InStream[63] = main_program_version[7];	
#ifdef FSP400
        if(g_ucVoltageMode == 0) // 18 Voltage Mode
          g_USBD_HID_InStream[63] = 0x41;
        else if(g_ucVoltageMode == 1) // 24 Voltage Mode 
          g_USBD_HID_InStream[63] = 0x42;
        else if(g_ucVoltageMode == 2) // 15 Voltage Mode 
          g_USBD_HID_InStream[63] = 0x43;
#else
        if(g_ucSysMode == 1) // fsp550-70mua
          g_USBD_HID_InStream[63] = 0x41;
        else // fsp450-60mua
          g_USBD_HID_InStream[63] = 0x42;
#endif

        //
        g_USBD_HID_InFlag = 1;
        break;
      }

      //
      case USB_CMD_SYN:
      {	 
        // PC -> MCU
        memcpy(g_ucSysTime, &g_USBD_HID_OutStream[2], 6);
  
        // MCU -> PC
        g_USBD_HID_InStream[0] = USB_CMD_SYN_RSP;
        g_USBD_HID_InStream[48] = g_USBD_HID_OutStream[48] + 1;      // SN

        //
        g_USBD_HID_InFlag = 1;
        break;
      }

      //
      case USB_CMD_SMBUS3_HT66:
      {
        if((g_USBD_HID_OutStream[2] == SMBUS_READ) || // SMBus
           (g_USBD_HID_OutStream[2] == SMBUS_WRITE))
        {
          // PC -> MCU      
          // 
          if(g_USBD_HID_OutStream[2] == SMBUS_READ) // Read
          {
            if(g_USBD_HID_OutStream[5] == 0) // Normal Mode
            {
              // [SELECT REGISTER]
              value = 1;
              if(HAL_I2C_Master_Transmit(&g_I2c1Handle,                         
                                         (uint16_t)g_USBD_HID_OutStream[1],     // Connect 
                                         (uint8_t *)&g_USBD_HID_OutStream[3],   // Data (Addr)
                                         1,                                     // Length = 1 
                                         2) == HAL_OK)                          // Wait time = 2ms
              {
                value = 0;
              }
              
              // [READ]
              if(value == 0)
              {
                value = 2;
                if(HAL_I2C_Master_Receive(&g_I2c1Handle, 
                                          (uint16_t)g_USBD_HID_OutStream[1],  // Device   
                                          (uint8_t *)&g_USBD_HID_InStream[8], // Data     
                                          g_USBD_HID_OutStream[4],            // Length
                                          2) == HAL_OK)                       // Wait time = 2ms
                {
                  value = 0;
                }
              }
            }
            else // Repeated Start Mode
            {				
              // [SELECT REGISTER]
              value = 1;
              if(HAL_I2C_Master_Transmit_Repeated_Start(&g_I2c1Handle,                         
                                         (uint16_t)g_USBD_HID_OutStream[1],     // Connect 
                                         (uint8_t *)&g_USBD_HID_OutStream[3],   // Data (Addr)
                                         1,                                     // Length = 1 
                                         2) == HAL_OK)                          // Wait time = 2ms
              {
                value = 0;
              }
              
              // [READ]
              if(value == 0)
              {
                value = 2;
                if(HAL_I2C_Master_Receive_Repeated_Start(&g_I2c1Handle, 
                                          (uint16_t)g_USBD_HID_OutStream[1],  // Device   
                                          (uint8_t *)&g_USBD_HID_InStream[8], // Data     
                                          g_USBD_HID_OutStream[4],            // Length
                                          2) == HAL_OK)                       // Wait time = 2ms
                {
                  value = 0;
                }
              }
            }
          }
          else // Write
          {
            // [SELECT REGISTER][WRITE]
            value = 1;
            g_USBD_HID_OutStream[4] = g_USBD_HID_OutStream[4] + 1;            // Length = Length + 1(Addr)
            g_USBD_HID_OutStream[7] = g_USBD_HID_OutStream[3];                // Pack Addr 
            if(HAL_I2C_Master_Transmit(&g_I2c1Handle,   
                                       (uint16_t)g_USBD_HID_OutStream[1],     // Device 
                                       (uint8_t *)&g_USBD_HID_OutStream[7],   // Addr + Data 
                                       g_USBD_HID_OutStream[4],               // Length
                                       2) == HAL_OK)                          // Wait time = 2ms
              value = 0;
          }
          
          // MCU -> PC
          g_USBD_HID_InStream[0] = USB_CMD_SMBUS3_HT66_RSP;	
          g_USBD_HID_InStream[1] = value;
          g_USBD_HID_InStream[48] = g_USBD_HID_OutStream[48] + 1;      // SN
          
          //
          g_USBD_HID_InFlag = 1;
        }
        else // HT66
        {
           // PC -> MCU 
           g_ucHT66Command[0] = g_USBD_HID_OutStream[1];
           g_ucHT66Command[1] = g_USBD_HID_OutStream[2];
           g_ucHT66Command[2] = g_USBD_HID_OutStream[3];
           g_ucHT66Command[3] = g_USBD_HID_OutStream[4];
       
           // MCU -> PC
           g_USBD_HID_InStream[0] = USB_CMD_SMBUS3_HT66_RSP;
           g_USBD_HID_InStream[48] = g_USBD_HID_OutStream[48] + 1;      // SN
       
           //
           g_ucHT66ActionFlagApp = 1;
           g_USBD_HID_InFlag = 0;
        }
        break;
      }

      //
      case USB_CMD_BATTERY_SYSTEM: 
      {
        // PC -> MCU
        data = g_USBD_HID_OutStream[2];                                      // Select channel
  
        // MCU -> PC
        g_USBD_HID_InStream[0] = USB_CMD_BATTERY_SYSTEM_RSP;
        g_USBD_HID_InStream[6] = g_ausBqBatteryState[data][0];               // Connect
        g_USBD_HID_InStream[7] = g_ausBqBatteryState[data][1];               // Connect-Err
        g_USBD_HID_InStream[9] = g_ucBq24745ChargerModeFlag[data];
        g_USBD_HID_InStream[10] = g_ausBqBatteryTemperature[data][0];        // Temperature
        g_USBD_HID_InStream[11] = g_ausBqBatteryTemperature[data][1];
        g_USBD_HID_InStream[12] = g_ausBqBatteryVoltage[data][0];            // Voltage
        g_USBD_HID_InStream[13] = g_ausBqBatteryVoltage[data][1];
        g_USBD_HID_InStream[14] = g_ausBqBatteryCurrent[data][0];            // Current
        g_USBD_HID_InStream[15] = g_ausBqBatteryCurrent[data][1];
        g_USBD_HID_InStream[16] = g_ausBqBatteryRemainingCapacity[data][0];  // RemainingCapacity
        g_USBD_HID_InStream[17] = g_ausBqBatteryRemainingCapacity[data][1];
        g_USBD_HID_InStream[18] = g_ausBqBatteryFullChargeCapacity[data][0]; // FullChargeCapacity
        g_USBD_HID_InStream[19] = g_ausBqBatteryFullChargeCapacity[data][1];
				g_USBD_HID_InStream[20] = g_ausBqBatteryAverageCurrent[0];
 				g_USBD_HID_InStream[21] = g_ausBqBatteryAverageCurrent[1];
				g_USBD_HID_InStream[22] = g_ausBqBatteryStatus[0];
				g_USBD_HID_InStream[23] = g_ausBqBatteryStatus[1];

				//uint32_t value;
				//
				//value  =  g_Rock_value[4];
				g_USBD_HID_InStream[24] = g_ausBqBatteryCellVoltage4[data][0];       // CellVoltage4
        g_USBD_HID_InStream[25] = g_ausBqBatteryCellVoltage4[data][1];
        //
				//value  =  g_Rock_value[3];
				g_USBD_HID_InStream[26] = g_ausBqBatteryCellVoltage3[data][0];       // CellVoltage3
        g_USBD_HID_InStream[27] = g_ausBqBatteryCellVoltage3[data][1];
        //
				//value  =  g_Rock_value[2];
				g_USBD_HID_InStream[28] = g_ausBqBatteryCellVoltage2[data][0];       // CellVoltage2
        g_USBD_HID_InStream[29] = g_ausBqBatteryCellVoltage2[data][1];
        //
				//value  =  g_Rock_value[1];
				g_USBD_HID_InStream[30] = g_ausBqBatteryCellVoltage1[data][0];       // CellVoltage1
        g_USBD_HID_InStream[31] = g_ausBqBatteryCellVoltage1[data][1];
				//
        g_USBD_HID_InStream[32] = g_ausBqBatteryCellVoltage8[0];             // CellVoltage8
        g_USBD_HID_InStream[33] = g_ausBqBatteryCellVoltage8[1];            
        //
				//value  =  g_Rock_value[7];
				g_USBD_HID_InStream[34] = g_ausBqBatteryCellVoltage7[0];             // CellVoltage7 ROCK   are the numbers right?
        g_USBD_HID_InStream[35] = g_ausBqBatteryCellVoltage7[1];            
        //
				//value  =  g_Rock_value[6];
				g_USBD_HID_InStream[36] = g_ausBqBatteryCellVoltage6[0];             // CellVoltage6 
        g_USBD_HID_InStream[37] = g_ausBqBatteryCellVoltage6[1];            
        //
				//value  =  g_Rock_value[5];
				g_USBD_HID_InStream[38] = g_ausBqBatteryCellVoltage5[0];             // CellVoltage5
        g_USBD_HID_InStream[39] = g_ausBqBatteryCellVoltage5[1];        
				//
				g_USBD_HID_InStream[40] = g_ausBqBatteryRelativeStateOfCharge[data][0];     // Relative State 
        g_USBD_HID_InStream[41] = g_ausBqBatteryAbsoluteStateOfCharge[data][0];     // Absolute State
        memcpy(&g_USBD_HID_InStream[44], &g_ausBqBatteryUpdateSysTime[data][0], 4); // HH:MM:SS
        g_USBD_HID_InStream[48] = g_USBD_HID_OutStream[48] + 1;              // SN
				
				g_USBD_HID_InStream[49] = g_ausBqBatteryRemainingDischargingTime[0]; 
				g_USBD_HID_InStream[50] = g_ausBqBatteryRemainingDischargingTime[1]; 
				
				//

        g_USBD_HID_InFlag = 1;
        break;                                // ADC1
      }
//for test
			/*case  USB_CMD_Rock:
			{
			  g_USBD_HID_InStream[0] = USB_CMD_Rock_RSP;
        g_USBD_HID_InStream[48] = g_USBD_HID_OutStream[48] + 1;      // SN
        g_USBD_HID_InFlag = 1;
				switch( g_USBD_HID_OutStream[1])
				{	
					case 5:
					{
						LIB_MemRead(0,
                        BQ78350_I2C_ADDRESS,
                        g_USBD_HID_OutStream[2],
                        g_USBD_HID_OutStream[3],
                        &g_USBD_HID_InStream[1]);
g_Rock_value[7] = 0;
g_Rock_value[6] = 0;
g_Rock_value[5] = 0;
g_Rock_value[4] = 0;
g_Rock_value[3] = 0;
g_Rock_value[2] = 0;
g_Rock_value[1] = 0;	
            break;
					}
					case 7 :
					{
						g_Rock_value[4] |= 2;
						HWF_InitADCMulCH();
           	break;			
					}
					case 8 :
					{
						uint32_t address;
						memcpy(&address,&g_USBD_HID_OutStream[2],4);
					  memcpy(&g_USBD_HID_InStream[1],(unsigned *)address,43);
					  break;
					}					
			  }
				break;
			}*/
      //
      case USB_CMD_FLASH:
      {
        //
        addr = g_USBD_HID_OutStream[8] + 
               g_USBD_HID_OutStream[9] * 0x100 + 
               g_USBD_HID_OutStream[10] * 0x10000 +
               g_USBD_HID_OutStream[11] * 0x1000000;
        value = g_USBD_HID_OutStream[12] + 
                g_USBD_HID_OutStream[13] * 0x100 + 
                g_USBD_HID_OutStream[14] * 0x10000 +
                g_USBD_HID_OutStream[15] * 0x1000000;
        if(g_USBD_HID_OutStream[1] == 0) // Write
        {  
          HAL_FLASH_Unlock();    
          HAL_FLASH_Program(TYPEPROGRAM_BYTE, addr, g_USBD_HID_OutStream[12]);	
          HAL_FLASH_Program(TYPEPROGRAM_BYTE, addr+1, g_USBD_HID_OutStream[13]);
          HAL_FLASH_Program(TYPEPROGRAM_BYTE, addr+2, g_USBD_HID_OutStream[14]);
          HAL_FLASH_Program(TYPEPROGRAM_BYTE, addr+3, g_USBD_HID_OutStream[15]); 
        }
        else // Read
          value = *((uint32_t *) addr);  

        // MCU -> PC
        g_USBD_HID_InStream[0] = USB_CMD_FLASH_RSP;	  
        g_USBD_HID_InStream[1] = 0;
        if(g_USBD_HID_OutStream[1] == 1) // Read
        {
          g_USBD_HID_InStream[12] = (value) & 0xFF;
          g_USBD_HID_InStream[13] = (value>>8) & 0xFF;
          g_USBD_HID_InStream[14] = (value>>16) & 0xFF;
          g_USBD_HID_InStream[15] = (value>>24) & 0xFF;
        }
        g_USBD_HID_InStream[48] = g_USBD_HID_OutStream[48] + 1;      // SN
  
        //
        g_USBD_HID_InFlag = 1;
        break;
      }

      //
      case USB_CMD_SWPIN:
      {
        //
        value = 1;

        //
        switch(g_USBD_HID_OutStream[2])
        {
          //
          case IO_PIN_PA9:
            if(g_USBD_HID_OutStream[4])
              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
            else
              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
            value = 0;
            break;

          //
          case IO_PIN_PA10:
            if(g_USBD_HID_OutStream[4])
              ;//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
            else
              ;//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
            value = 0;
            break;

          //
          case IO_PIN_PB0:
            if(g_USBD_HID_OutStream[4])
              ;//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
            else
              ;//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
//g_ucBatteryHT66ActionFlagMCU = 1;
//g_ucBatteryHT66CommandMcu[0] = BATTERY_HT66_ADDRESS;
//g_ucBatteryHT66CommandMcu[1] = HT66_CMD_0x5D;
//g_ucBatteryHT66CommandMcu[2] = 0x21;
//g_ucBatteryHT66CommandMcu[3] = 0x0F;
//g_ucBatteryHT66CmdDoubleFlag = 1;
            value = 0;
            break;

          //
          case IO_PIN_PB1:
            if(g_USBD_HID_OutStream[4])
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
            else
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
//g_ucBatteryHT66ActionFlagMCU = 1;
//g_ucBatteryHT66CommandMcu[0] = BATTERY_HT66_ADDRESS;
//g_ucBatteryHT66CommandMcu[1] = HT66_CMD_0x5D;
//g_ucBatteryHT66CommandMcu[2] = 0x22;
//g_ucBatteryHT66CommandMcu[3] = 0x0F;
//g_ucBatteryHT66CmdDoubleFlag = 1;
            value = 0;
            break;

          //
          case IO_PIN_PB3:
            if(g_USBD_HID_OutStream[4])
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
            else
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
            value = 0;
            break;

          //
          case IO_PIN_PB4:
            if(g_USBD_HID_OutStream[4])
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
            else
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
            value = 0;
            break;			

          //
          case IO_PIN_PB12:
            if(g_USBD_HID_OutStream[4])
              ;//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
            else
              ;//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
            value = 0;
            break;	

          //
          case IO_PIN_PB13:
            if(g_USBD_HID_OutStream[4])
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
            else
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
            value = 0;
            break;	

          //
          case IO_PIN_PB14:
            if(g_USBD_HID_OutStream[4])
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
            else
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            value = 0;
            break;	

          //
          case IO_PIN_PB15:
            if(g_USBD_HID_OutStream[4])
            {
              //EN_FAST_CHARGER_L;
              //EN_CHARGER_L;
              //EN_CHARGER_POWER_L;
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);       // Battery check pin
              g_ucBatteryCheckFlag = 1;
            }
            else
            {
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
              g_ucBatteryCheckFlag = 0;
            }
            value = 0;
            break;				

          default:
            break;
        }


        // MCU -> PC
        g_USBD_HID_InStream[0] = USB_CMD_SWPIN_RSP;
        g_USBD_HID_InStream[1] = value;
        g_USBD_HID_InStream[48] = g_USBD_HID_OutStream[48] + 1;      // SN
  
        //
        g_USBD_HID_InFlag = 1;
        break;
      }

      //
      case USB_CMD_ROOT:
      {
        // PC -> MCU
        g_ucRootFlag = g_USBD_HID_OutStream[1];

        // MCU -> PC
        g_USBD_HID_InStream[0] = USB_CMD_ROOT_RSP;
        g_USBD_HID_InStream[48] = g_USBD_HID_OutStream[48] + 1;      // SN
  
        //
        g_USBD_HID_InFlag = 1;
        break;
      }

      //
      case USB_CMD_UPGRADE:
      {
        if(g_USBD_HID_OutStream[1] == USB_CMD_UPGRADE_START)
        {
          // 
          g_ucResetFlag = 1;
  
          // MCU -> PC
          g_USBD_HID_InStream[0] = USB_CMD_UPGRADE; 
          g_USBD_HID_InStream[1] = USB_CMD_UPGRADE_START_RSP;		
          g_USBD_HID_InStream[2] = 0; 
          g_USBD_HID_InStream[3] = g_USBD_HID_OutStream[2] + 1;	  
          g_USBD_HID_InFlag = 1;    
        }
        break; 
      }

      // FSP 650
      case HS1:
      {
        data = g_USBD_HID_OutStream[0] + 
               g_USBD_HID_OutStream[1] + 
               g_USBD_HID_OutStream[2] +
               g_USBD_HID_OutStream[3];
        if((g_USBD_HID_OutStream[1] != HS2) || (data != g_USBD_HID_OutStream[4]))
        {
          g_USBD_HID_OutFlag = 0;   
          break;
        }

        //
        g_ucFSP650Cmd = g_USBD_HID_OutStream[3];
        g_USBD_HID_OutFlag = 0; 
      }
			
      //
      default:
        g_USBD_HID_OutFlag = 0;
        break;
    }				
		
  }
#endif
}

//
void USB_ProcessTX(void)
{
#ifdef USB_MODULE 
  //
  if(g_USBD_HID_InFlag > 0)
  {
    //    
    USBD_HID_SendReport (&g_hUSBDDevice, g_USBD_HID_InStream, 64);
    g_USBD_HID_InFlag = 0;
  }

  //
  if(g_ucResetFlag > 0)
  {
    if(g_ucResetFlag < 5) // Delay
    {
      g_ucResetFlag++;
      return;
    }
    USB_UnConfig();
    HAL_FLASH_Unlock(); 
    HAL_FLASH_Program(TYPEPROGRAM_WORD, UPDATE_FW_FLAG_ADDRESS, 0x1A2B);
    HAL_NVIC_SystemReset();
  }
#endif
}
