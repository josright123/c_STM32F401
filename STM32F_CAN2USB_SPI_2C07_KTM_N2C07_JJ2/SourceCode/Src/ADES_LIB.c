
#include "main.h"

void LIB_CalculateSystemTime(uint8_t *sysTime)
{
    uint8_t day = sysTime[2];

    // Second
    sysTime[5]++;
    if((sysTime[5] & 0x0f) == 0x0A)
        sysTime[5] += 0x06;                                   // 0x06 = 0x10 - 0x0A
    if(sysTime[5] == 0x60)
    {
        sysTime[5] = 0x00;
        sysTime[4]++;
    }

    // Minute
    if((sysTime[4] & 0x0f) == 0x0A)
        sysTime[4] += 0x06;                    
    if(sysTime[4] == 0x60)
    {
        sysTime[4] = 0x00;
        sysTime[3]++;
    }

    // Hour
    if((sysTime[3] & 0x0f) == 0x0A)
        sysTime[3] += 0x06;                    
    if(sysTime[3] == 0x24)
    {
        sysTime[3] = 0x00;
        sysTime[2]++;
    }

    // Day
    if(day != sysTime[2])
    {
        if((sysTime[1] == 0x01) || (sysTime[1] == 0x03) || 
           (sysTime[1] == 0x05) || (sysTime[1] == 0x07) ||
           (sysTime[1] == 0x08) || (sysTime[1] == 0x10) ||
           (sysTime[1] == 0x12))
        {
            if((sysTime[2] & 0x0f) == 0x0A)
                sysTime[2] += 0x06;
            if(sysTime[2] == 0x32)
            {
                sysTime[2] = 0x01;
                sysTime[1]++;
            }         
        }
        else if(sysTime[1] == 0x02)
        {
            if((sysTime[2] & 0x0f) == 0x0A)
                sysTime[2] += 0x06;  

            // Leap year
            if((sysTime[0] == 0x16) || (sysTime[0] == 0x20) ||
               (sysTime[0] == 0x24) || (sysTime[0] == 0x28) ||
               (sysTime[0] == 0x32) || (sysTime[0] == 0x36))
            {
                if(sysTime[2] == 0x30)
                {
                    sysTime[2] = 0x01;
                    sysTime[1]++;
                }  
            }
            else
            {
                if(sysTime[2] == 0x29)
                {
                    sysTime[2] = 0x01;
                    sysTime[1]++;
                } 
            }
        }
        else
        {
            if((sysTime[2] & 0x0f) == 0x0A)
                sysTime[2] += 0x06;                    
            if(sysTime[2] == 0x31)
            {
                sysTime[2] = 0x01;
                sysTime[1]++;
            }
        }
    }

    // Month
    if((sysTime[1] & 0x0f) == 0x0A)
        sysTime[1] += 0x06;
    if(sysTime[1] == 0x13)
    {
        sysTime[1] = 0x01;
        sysTime[0]++;
    }

    // Year
    if((sysTime[0] & 0x0f) == 0x0A)
        sysTime[0] += 0x06; 
}

/*
 * return : 0 = Success, 1 = Fail
 * rw : 0 = read, 1 = write 
 */
uint8_t LIB_TxRxBq20z45(uint8_t rw, uint8_t device, uint8_t cmd, uint8_t length, uint8_t *data)   
{
  uint8_t ausRecevie[20] = {0};
  HWF_ReInitI2C3Pin();  
  
  // 
  if(rw == 0) // Read
  {
    // [SELECT REGISTER]
    if(HAL_I2C_Master_Transmit(&g_I2c3Handle,                         
                               (uint16_t)device,                     // Device 
                               (uint8_t *)&cmd,                      // Addr
                               1,                                    // Length = 1 
                               2) != HAL_OK)                         // Wait time = 2ms
      return 1;
    
    // [READ]
    if(HAL_I2C_Master_Receive(&g_I2c3Handle, 
                              (uint16_t)device,                      // Device   
                              (uint8_t *)data,                       // Data     
                              length,                                // Length
                              2) != HAL_OK)                          // Wait time = 2ms
      return 1;
  }
  else // Write
  {
    // [SELECT REGISTER] + [WRITE] 
    ausRecevie[0] = cmd;
    memcpy(&ausRecevie[1], data, length);   
    if(HAL_I2C_Master_Transmit(&g_I2c3Handle,   
                               (uint16_t)device,                      // Device 
                               (uint8_t *)&ausRecevie[0],             // Addr + Data 
                               1 + length,                            // Length
                               2) != HAL_OK)                          // Wait time = 2ms
      return 1;
  }

  return 0;	
}
/*
 * return : 0 = Success, 1 = Fail
 * rw : 0 = read, 1 = write 
 */
uint8_t LIB_MemByteReadWrite(uint8_t rw, uint8_t device, uint8_t *cmd, uint8_t length, uint8_t *data)   
{
  uint8_t ausRecevie[20] = {0};
  HWF_ReInitI2C3Pin();  
  memcpy(&ausRecevie[0], cmd, 2);
  // 
  if(rw == 0) // Read
  {
		// [SELECT REGISTER]
		if(HAL_I2C_Master_Transmit(&g_I2c3Handle,
                               (uint16_t)0xA0,                      // Device
                               (uint8_t *)&ausRecevie[0],    	  		// AddrH + AddrL
                               2,                           		    // Addr Length
                               2) != HAL_OK)                        // Wait time = 2ms
    return 1;
    // [READ]
    if(HAL_I2C_Master_Receive(&g_I2c3Handle,
                              (uint16_t)0xA0,                      // Device
                              (uint8_t *)&EEPROM_RX_Buffer[0],     // Read Data
                              length,                              // Length
                              2) != HAL_OK)                        // Wait time = 2ms
    return 1;
  }
  else // Write
  {
    // [SELECT REGISTER] + [WRITE]
    memcpy(&ausRecevie[2], data, length); 
    if(HAL_I2C_Master_Transmit(&g_I2c3Handle,   
                               (uint16_t)EEPROM_ADDRESS,              // Device 
                               (uint8_t *)&ausRecevie[0],             // AddrH + AddrL + Data
                               2 + length,                            // Length
                               2) != HAL_OK)                          // Wait time = 2ms
      return 1;
  }

  return 0;	
}
// rock add i2c3 tx function for sir
uint8_t LIB_Tx_I2C3_SIR(uint8_t device, uint8_t cmd, uint8_t length, uint8_t *data)   
{

  HWF_ReInitI2C3Pin();  
 
    // [SELECT REGISTER]
    if(HAL_I2C_Master_Transmit(&g_I2c3Handle,                         
                               (uint16_t)device,                     // Device 
                               (uint8_t *)&cmd,                      // Addr
                               1,                                    // Length = 1 
                               2) != HAL_OK)                         // Wait time = 2ms
      return 1;
    else
			return 0;
}
//
uint8_t LIB_Tx_I2C3_SIR_write_block(uint8_t device, uint8_t cmd, uint8_t length, uint8_t *data)   
{
   HWF_ReInitI2C3Pin();  
	return 0;
}


uint8_t LIB_MemRead(uint8_t rw, uint8_t device, uint8_t cmd, uint8_t length, uint8_t *data)   
{
				HWF_ReInitI2C1Pin();	
	if (HAL_I2C_Mem_Read(&g_I2c1Handle,
                       (uint16_t)device, 
	                     (uint8_t)cmd,
	                     1, 
	                     (uint8_t *)CRC_buffer, 
	                     length + 1,
	                     2) == 0)
                {                                      //success
									CRC_ALL_Data[1] = device;            // battery address
									CRC_ALL_Data[2] = cmd;
									CRC_ALL_Data[3] = device|1;
									CRC_ALL_Data[4] = CRC_buffer[0];
									CRC_ALL_Data[5] = CRC_buffer[1];
									CRC_ALL_Data[6] = CRC_buffer[2];
									
									if(length != 2 || CRC_Check(CRC_ALL_Data,6) == 0)
									{                                    // CRC success
										CHARGER_one_puls = 65535;
										memcpy(data,CRC_buffer,length);
										if (g_ausBqBatteryState[g_ucBqStateMachineBatteryChannel][1] > 0)
                        g_ausBqBatteryState[g_ucBqStateMachineBatteryChannel][1]--;
										return	0;
									}
                }
                                                                                   // fail		
                g_ausBqBatteryState[g_ucBqStateMachineBatteryChannel][0] = cmd;
                if (g_ausBqBatteryState[g_ucBqStateMachineBatteryChannel][1] < BQ20Z45_TX_RX_ERROR_THRESHOLD)
                    g_ausBqBatteryState[g_ucBqStateMachineBatteryChannel][1]++;
								return	1;	
								
}
uint8_t CRC_Check(uint8_t *crc_all, uint8_t length)
{
  CRC_Data = 0;
  for (int i = 1 ; i <= length ; i++) 
            {
              CRC_Data = (CRC_Data<<8)|(crc_all[i]);
              CRC_Data ^= (CRC8[CRC_Data>>12]<<4);
              CRC_Data ^= (CRC8[CRC_Data>>8]);
            }
						return(CRC_Data);
}
