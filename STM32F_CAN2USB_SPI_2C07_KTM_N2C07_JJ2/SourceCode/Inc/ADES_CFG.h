
#ifndef __ADES_CFG_H_
#define __ADES_CFG_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FSP400

/* Define the flash memory start address */
#define USER_FLASH_STARTADDRESS    ((uint32_t)0x08000000) /* Flash Start Address */

/* Define the address from where user application will be loaded.
   Note: the 1st sector 0x08000000-0x08007FFF is reserved for the Firmware upgrade code */
#define APPLICATION_ADDRESS        (uint32_t)0x08004000 // Bootloader 32 KBytes

/* End of the Flash address */
#define USER_FLASH_END_ADDRESS     ((uint32_t)0x0803FFFF)

/* Define the user application size */
#define USER_FLASH_SIZE   (USER_FLASH_END_ADDRESS - USER_FLASH_STARTADDRESS + 1)

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbyte */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbyte */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbyte */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbyte */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbyte */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbyte */

/* Update FW flag address */
#define UPDATE_FW_FLAG_ADDRESS  ((uint32_t)0x0801FFF0)

// DEFINE CONFIGURE
// USB_MODULE 
#define USB_CMD_POLL                      0x50    
#define USB_CMD_POLL_RSP                  0x51     
#define USB_CMD_SYN                       0x52    
#define USB_CMD_SYN_RSP                   0x53  
#define USB_CMD_SMBUS3_HT66               0x54    
#define USB_CMD_SMBUS3_HT66_RSP           0x55
#define USB_CMD_BATTERY_SYSTEM            0x5C
#define USB_CMD_BATTERY_SYSTEM_RSP        0x5D	
#define USB_CMD_FLASH                     0x56
#define USB_CMD_FLASH_RSP                 0x57
//BEN add pc con control
#define USB_CMD_CAN2                      0x5E
#define USB_CMD_CAN2_RSP                  0x5F


#define USB_CMD_SWPIN                     0x5A
#define USB_CMD_SWPIN_RSP                 0x5B
#define USB_CMD_CAN2_TX_RX			          0x5C
#define USB_CMD_CAN2_TX_RX_RSP   		      0x5D
#define USB_CMD_SWPIN_RSP                 0x5B
#define USB_CMD_UPGRADE                   0x60    
#define USB_CMD_UPGRADE_RSP               0x61
#define USB_CMD_ROOT                      0x70    
#define USB_CMD_ROOT_RSP                  0x71
#define USB_CMD_UPGRADE_START             0xA0
#define USB_CMD_UPGRADE_START_RSP         0xA1
#define USB_CMD_UPGRADE_ERASE             0xA2
#define USB_CMD_UPGRADE_ERASE_RSP         0xA3
#define USB_CMD_UPGRADE_WRITE             0xA4
#define USB_CMD_UPGRADE_WRITE_RSP         0xA5
#define USB_CMD_UPGRADE_RESET             0xA6
#define USB_CMD_UPGRADE_RESET_RSP         0xA7

#define ROOT_CMD_0x10                     0x10
#define ROOT_CMD_0x11                     0x11
#define ROOT_CMD_0x12                     0x12
#define ROOT_CMD_0x13                     0x13
#define ROOT_CMD_0x14                     0x14

// SYSTEM
#define TICK_TASK_QUEUE_QUANTITY            20 

#define SYSTEM_EVERY_100MS_OFFSET_00MS       0
#define SYSTEM_EVERY_100MS_OFFSET_10MS      10
#define SYSTEM_EVERY_100MS_OFFSET_20MS      20
#define SYSTEM_EVERY_100MS_OFFSET_30MS      30
#define SYSTEM_EVERY_100MS_OFFSET_40MS      40
#define SYSTEM_EVERY_100MS_OFFSET_50MS      50
#define SYSTEM_EVERY_100MS_OFFSET_60MS      60
#define SYSTEM_EVERY_100MS_OFFSET_70MS      70
#define SYSTEM_EVERY_100MS_OFFSET_80MS      80
#define SYSTEM_EVERY_100MS_OFFSET_90MS      90

#define SYSTEM_000MS                         0 
#define SYSTEM_100MS                       100
#define SYSTEM_200MS                       200
#define SYSTEM_300MS                       300
#define SYSTEM_400MS                       400
#define SYSTEM_500MS                       500
#define SYSTEM_600MS                       600
#define SYSTEM_700MS                       700
#define SYSTEM_800MS                       800
#define SYSTEM_900MS                       900
#define SYSTEM_1000MS                     1000

// SYSTEM FLAG   
#define SYS_TICK_TASK_1                 0x0001   // Calculate system timer
#define SYS_TICK_TASK_2                 0x0002   // LED
#define SYS_TICK_TASK_3                 0x0004   // HT66 shift and error process
#define SYS_TICK_TASK_4                 0x0008   // FAN PWM identify
#define SYS_TICK_TASK_5                 0x0010   // Battery algorithm
#define SYS_TICK_TASK_6                 0x0020   // Power process
#define SYS_TICK_TASK_7                 0x0040   // Read HT66 Flash Parameter
#define SYS_TICK_TASK_8                 0x0080   // Check battery discharge function
#define SYS_TICK_TASK_9                 0x0100   // Bq24745 Action
#define SYS_TICK_TASK_10                0x0200   // Adjust battery charge current
#define SYS_TICK_TASK_11                0x0400   // Bq78350
#define SYS_TICK_TASK_12                0x0800   // deleay init for en288

#define SYS_TICK_TASK_32            0x80000000   // Root function

// POWER & BATTERY
#define PW_STEP_0                         0x00
#define PW_STEP_1                         0x01
#define PW_STEP_2                         0x02
#define PW_STEP_3                         0x03
#define PW_STEP_3_1                       0x04
#define PW_STEP_3_2                       0x05
#define PW_STEP_3_3                       0x06
#define PW_STEP_4                         0x07

#define NO_BATTERY                        0x00
#define NIMH_BATTERY                      0x01
#define LEAD_ACID_BATTERY                 0x02
#define LI_PO_BATTERY                     0x03
#define LI_ION_BATTERY                    0x04
#define LI_FE_PO_BATTERY                  0x05

#define CHARGE_OFF                           0
#define CHARGE_NORMAL                        1
#define CHARGE_FAST                          2
#define CHARGE_RE_NORMAL                     3
#define CHARGE_STOP                          4
//rock add define charge_check
#define CHARGE_CHECK                         5

#define CHARGE_DELAY_TIME                  200
#define ENABLE_CHARGER_DELAY_TIME         5010

#define BATTERY_ALG_TIMER_5_HR           36000 
#define BATTERY_ALG_TIMER_2_HR           14400 
//Rock add 2min define
#define BATTERY_ALG_TIMER_2_MIN            240

#define BATTERY_ALG_TIMER_1_MIN            130//
#define BATTERY_ALG_TIMER_30_SEC            60 
#define BATTERY_ALG_TIMER_15_SEC            30  
#define BATTERY_ALG_TIMER_10_SEC            20 
#define BATTERY_ALG_TIMER_3_SEC              6
#define BATTERY_ALG_TIMER_1_SEC              2

// USBD_HID
#define USBD_HID_DATA_INBUF_SIZE          0x40
#define USBD_HID_DATA_OUT_BUF_SIZE        0x40

// ERROR CODE
#define ZX_OK                             0x00
#define ZX_ERR_01                         0x01

// LED
#define LED_IDLE_SEC                         7   // 1.6 Sec = (7 + 1)*0.2
// LED - Err state
#define LED_SYS_NO_BATTERY                   2
#define LED_SYS_AC_FAIL                      3 
#define LED_SYS_ACINOK                       4
#define LED_SYS_ADC_CH1_OVER                 5
#define LED_BAT_OVER_TEMPERATURE             6
#define LED_BAT_OVER_VOLTAGE                 7
#define LED_BAT_OVER_CHARGE_CURRENT          8
#define LED_BAT_LOW_VOLTAGE                  9
#define LED_BAT_LOW_CRITICAL_VOLTAGE        10
// LED - State
#define LED_BAT_FULL                         1
#define LED_BAT_DISCHARGE                   11
#define LED_BAT_NORMAL_CHARGE               12
#define LED_BAT_FAST_CC_CHARGE              13
#define LED_BAT_FAST_CV_CHARGE              14
#define LED_BAT_RE_NORMAL_CHARGE            15
#define LED_BAT_FLOAT_CHARGE                16
#define LED_BAT_SELECT_CHARGE_CH            17
#define LED_BAT_MULTI_CHARGE_CH0            20
#define LED_BAT_MULTI_CHARGE_CH1            21
#define LED_BAT_MULTI_CHARGE_CH2            22
#define LED_BAT_MULTI_CHARGE_CH3            23

// I2C
#define SIR_DEVICE_ADDRESS                0x2E
#define SIR_CMD_Battery_Info              0xA0
#define EEPROM_ADDRESS										0xA0
//#define I2C_ADDRESS                       0x3C  // **** ***X  ([EX] 0xA2 = 1010 0010) 
#define I2C_ADDRESS                       0x3C  // **** ***X  ([EX] 0xA2 = 1010 0010) 
#define HT66_ADDRESS                      0x00  
#define BATTERY_HT66_ADDRESS              0x30 
#define HT66_STEP_0                       0x00
#define HT66_STEP_1                       0x01
#define HT66_STEP_2                       0x02
#define HT66_STEP_3                       0x03
#define HT66_CMD_0x52                     0x52
#define HT66_CMD_0x54                     0x54
#define HT66_CMD_0x5D                     0x5D
#define HT66_ADDR_0x03                    0x03
#define HT66_ADDR_0x05                    0x05
#define HT66_ADDR_0x07                    0x07
#define HT66_ADDR_0x17                    0x17
#define HT66_ADDR_0x40                    0x40
#define HT66_ADDR_0x41                    0x41
#define HT66_ADDR_0x42                    0x42
#define HT66_ADDR_0x43                    0x43
#define HT66_ADDR_0x44                    0x44
#define HT66_ADDR_0x45                    0x45
#define HT66_ADDR_0x46                    0x46
#define HT66_ADDR_0x47                    0x47
#define HT66_ADDR_0x48                    0x48
#define HT66_ADDR_0x49                    0x49
#define HT66_ADDR_0x4A                    0x4A
#define HT66_ADDR_0x4B                    0x4B
#define HT66_ADDR_0x4C                    0x4C
#define HT66_ADDR_0x4D                    0x4D
#define HT66_ADDR_0x4E                    0x4E
#define HT66_ADDR_0x4F                    0x4F
#define HT66_ADDR_0x50                    0x50
#define HT66_ADDR_0x51                    0x51
#define HT66_ADDR_0x52                    0x52
#define HT66_ADDR_0x53                    0x53
#define HT66_ADDR_0x54                    0x54
#define HT66_ADDR_0x55                    0x55
#define HT66_ADDR_0x56                    0x56
#define HT66_ADDR_0x7B                    0x7B
#define HT66_ADDR_0x7C                    0x7C
// SMBUS
#define SMBUS_READ                        0x70
#define SMBUS_WRITE                       0x71 

// SWITCH PIN
#define IO_PIN_PA9                        0xA9
#define IO_PIN_PA10                       0xAA
#define IO_PIN_PB0                        0xB0
#define IO_PIN_PB1                        0xB1
#define IO_PIN_PB2                        0xB2
#define IO_PIN_PB3                        0xB3
#define IO_PIN_PB4                        0xB4
#define IO_PIN_PB9                        0xB9
#define IO_PIN_PB10                       0xBA
#define IO_PIN_PB12                       0xBC
#define IO_PIN_PB13                       0xBD
#define IO_PIN_PB14                       0xBE
#define IO_PIN_PB15                       0xBF

// BATTERY_PARAMETER
/*  
 *  FSP550
 *  Battery voltage = 11.984 * 0.00080048 * ADC - 0.6374
 */
#define BATTERY_DISCHARGE_PERCENTAGE_V100 2488 // 24.5 V, Voltage 100%  
#define BATTERY_DISCHARGE_PERCENTAGE_V20  2238 // 22.1 V, Voltage 20% 
#define BATTERY_DISCHARGE_PERCENTAGE_V10  2196 // 21.7 V, Voltage 10% 
#define BATTERY_DISCHARGE_PERCENTAGE_V0   2154 // 21.3 V, Voltage 0% 
 
#define BATTERY_CHARGE_PERCENTAGE_VM      2957 // 29.0 V, CC => CV
#define BATTERY_CHARGE_PERCENTAGE_V0      2561 // 25.2 V

#define BATTERY_VOLTAGE_30_0              3061 // 30.0 V
#define BATTERY_VOLTAGE_24_5              2488 // 24.5 V
#define BATTERY_VOLTAGE_24_0              2436 // 24.0 V
#define BATTERY_VOLTAGE_23_0              2332 // 23.0 V
#define BATTERY_VOLTAGE_22_1              2238 // 22.1 V
#define BATTERY_VOLTAGE_21_7              2160 //2000 21.7 V  for test
#define BATTERY_VOLTAGE_20_9              2096 //20.89 V  
//FLEX300-U12 
//Battery voltage = (Vset * (33.2/(33.2+374))) / (3.3/4096)
#define BATTERY_VOLTAGE_17_2              1714 //17.2 V
#define BATTERY_VOLTAGE_12_2              1235 //12.2 V
// rock set battery uvp
#define BATTERY_VOLTAGE_20_7              2000 // 20.7 V

#define BATTERY_VOLTAGE_19_8              1998 // 19.8 V
#define BATTERY_VOLTAGE_19_3              1946 // 19.3 V

#define BATTERY_CHARGE_L90                  90
#define BATTERY_CHARGE_LM                   60 // CC => CV
#define BATTERY_CHARGE_L0                    0

// Ben set battery uvp
#define BATTERY_VOLTAGE_11_5              1163 // 11.5 V
#define BATTERY_VOLTAGE_9_0								911 // 9.0 V
#define BATTERY_VOLTAGE_8_0								810 // 9.0 V
/* 
 *  FSP550
 *  Discharge current = (4.8862 * 0.00080048 * ADC1 + 0.2642)*2
 */
 
 //Rock 650 fortest
#define BATTERY_DISCHARGE_CURRENT_20      2239//2617  // rock
#define BATTERY_DISCHARGE_CURRENT_24      3000

/* 
 *  FSP550
 *  Charge current = 1.3076 * 0.00080048 * ADC2 + 0.0887
 */
#define BATTERY_CHARGE_CURRENT_0_20        106            // Rock for Lead Acid
#define BATTERY_CHARGE_CURRENT_2_00       1826            // Rock for Lead Acid

//disabble for test
//STM = 306 *discharge current - 171                        // rock for SIR 0.5 constant current
#define BATTERY_CHARGE_CURRENT_2_01       9999//2621//750
#define BATTERY_CHARGE_CURRENT_2_50       1800//2621//1200  3.9A
#define BATTERY_CHARGE_CURRENT_3_00       2300//2600//1500

#define BATTERY_CHARGE_CURRENT_6_00       2652//1664       5.6A
#define BATTERY_CHARGE_CURRENT_6_50       3021//1817
#define BATTERY_CHARGE_CURRENT_7_00       9999//1970



#define BATTERY_CHARGE_CURRENT_C90         202 // 0.3 A, Current 90%
#define BATTERY_CHARGE_CURRENT_CM         2113 // 2.3 A, Current middle% 

/* 
 *  Charge mode
 */
#define BATTERY_NORMAL_CHARGE_MODE           0  
#define BATTERY_FAST_CHARGE_MODE             1
#define BATTERY_RE_NORMAL_CHARGE_MODE        2
#define BATTERY_STOP_CHARGE_MODE             3
#define BATTERY_FLOAT_CHARGE_FUNCTION     0x01
#define BATTERY_TIMER_PARAMETER             20 // 10 Sec

//Charger Flag
#define BATTERY_OVER_VOLTAGE                131 //1000 0011
#define BATTERY_OVER_TEMPERATURE             67 //0100 0011
#define BATTERY_OVER_PERCENTAGE              35 //0010 0011
#define BATTERY_OVER_BIT15                   19 //0001 0011
#define BATTERY_OVER_5HR                     11 //0000 1011
#define BATTERY_OVER_ocp                      7 //0000 0111


#define BATTERY_UNDER_VOLTAGE               127 //0111 1111
#define BATTERY_UNDER_TEMPERATURE           191 //1011 1111
#define BATTERY_UNDER_PERCENTAGE            223 //1101 1111
#define BATTERY_UNDER_BIT15                 239 //1110 1111
#define BATTERY_UNDER_5HR                   247 //1111 0111
#define BATTERY_UNDER_ocp                   251 //1111 1011


// CHARGE HALT WATT
#define CHARGE_HALT_WATT                      350
#define CHARGE_RELEASE_WATT                   330

// BQ20Z45 ACTION
#define TCA9546A_I2C_ADDRESS                  0xE0
#define BQ20Z45_I2C_ADDRESS                   0x16

#define BQ20Z45_SBS_TEMPERATURE               0x08
#define BQ20Z45_SBS_VOLTAGE                   0x09
#define BQ20Z45_SBS_CURRENT                   0x0A
#define BQ20Z45_SBS_AVGER_CURRENT             0x0B
#define BQ20Z45_SBS_RELATIVE_STATE_OF_CHARGE  0x0D
#define BQ20Z45_SBS_ABSOLUTE_STATE_OF_CHARGE  0x0E
#define BQ20Z45_SBS_REMAINING_CAPACITY        0x0F
#define BQ20Z45_SBS_FULL_CHARGE_CAPACITY      0x10
#define BQ20Z45_SBS_REMAINING_DISCHARGE_TIME  0x12
#define BQ20Z45_SBS_CHARGING_CURRENT          0x14
#define BQ20Z45_SBS_CHARGING_VOLTAGE          0x15
#define BQ20Z45_SBS_BATTERY_STATUS						0x16
#define BQ20Z45_SBS_CELL_VOLTAGE_5            0x3B
#define BQ20Z45_SBS_CELL_VOLTAGE_4            0x3C
#define BQ20Z45_SBS_CELL_VOLTAGE_3            0x3d
#define BQ20Z45_SBS_CELL_VOLTAGE_2            0x3e
#define BQ20Z45_SBS_CELL_VOLTAGE_1            0x3f
#define BQ20Z45_SBS_FF                        0xFF
#define BQ20Z45_TX_RX_ERROR_THRESHOLD           20 // > 10 Sec
#define BQ20Z45_CHECK_TCA9546A                   1
#define BQ20Z45_BATTERY_TEMPERATURE              2
#define BQ20Z45_BATTERY_VOLTAGE                  3
#define BQ20Z45_BATTERY_CURRENT                  4
#define BQ20Z45_BATTERY_REMAINING_CAPACITY       5
#define BQ20Z45_BATTERY_FULL_CHARGE_CAPACITY     6
#define BQ20Z45_BATTERY_CHARGING_CURRENT         7
#define BQ20Z45_BATTERY_CHARGING_VOLTAGE         8
#define BQ20Z45_BATTERY_CELL_VOLTAGE_4           9
#define BQ20Z45_BATTERY_CELL_VOLTAGE_3          10
#define BQ20Z45_BATTERY_CELL_VOLTAGE_2          11
#define BQ20Z45_BATTERY_CELL_VOLTAGE_1          12
#define BQ20Z45_BATTERY_NO_INSERT               20
#define BQ20Z45_CK_TEMPERATURE_L1             2711 // -2 DegC, Too low
#define BQ20Z45_CK_TEMPERATURE_L2             2851 // 12 DegC
#define BQ20Z45_CK_TEMPERATURE_L3             3031 // 30 DegC
#define BQ20Z45_CK_TEMPERATURE_L4             3201 // 47 DegC
#define BQ20Z45_CK_TEMPERATURE_L5             3281 // 55 DegC, Too high
#define BQ20Z45_CK_CELL_VOLTAGE_L1            3000 // 3000 mV, Too low
#define BQ20Z45_CK_CELL_VOLTAGE_L2            3100 // 3100 mV
#define BQ20Z45_CK_CELL_VOLTAGE_L3            4100 // 4100 mV
#define BQ20Z45_CK_CELL_VOLTAGE_L4            4200 // 4200 mV

// BQ24745 ACTION 
#define BQ24745_MODE_L0                      0
#define BQ24745_MODE_L1                      1
#define BQ24745_MODE_L2                      2
#define BQ24745_MODE_L3                      3
#define BQ24745_MODE_L4                      4
#define BQ24745_MODE_L5                      5
#define BQ24745_MODE_L6                      6
#define BQ24745_MODE_L7                      7
#define BQ24745_MODE_L8                      8

// BQ24725A ACTION
#define BQ24725A_DEFAULT_CHARGE              0
#define BQ24725A_STOP_CHARGE                 1

// BQ78350 ACTION
#define BQ78350_I2C_ADDRESS                   0x16 // 0010 1100    16  2c
#define BQ78350_SBS_TEMPERATURE               0x08
#define BQ78350_SBS_VOLTAGE                   0x09
#define BQ78350_SBS_CURRENT                   0x0A
#define BQ78350_SBS_AVGER_CURRENT             0x0B
#define BQ78350_SBS_RELATIVE_STATE_OF_CHARGE  0x0D
#define BQ78350_SBS_ABSOLUTE_STATE_OF_CHARGE  0x0E
#define BQ78350_SBS_REMAINING_CAPACITY        0x0F
#define BQ78350_SBS_FULL_CHARGE_CAPACITY      0x10
#define BQ78350_SBS_REMAINING_DISCHARGE_TIME  0x12
#define BQ78350_SBS_CHARGING_CURRENT          0x14
#define BQ78350_SBS_CHARGING_VOLTAGE          0x15
#define BQ78350_SBS_BATTERY_STATUS            0X16
#define BQ78350_SBS_DEVICE_CHEMISTRY          0x22
#define BQ78350_SBS_CELL_VOLTAGE_8            0x38
#define BQ78350_SBS_CELL_VOLTAGE_7            0x39
#define BQ78350_SBS_CELL_VOLTAGE_6            0x3a
#define BQ78350_SBS_CELL_VOLTAGE_5            0x3b
#define BQ78350_SBS_CELL_VOLTAGE_4            0x3c
#define BQ78350_SBS_CELL_VOLTAGE_3            0x3d
#define BQ78350_SBS_CELL_VOLTAGE_2            0x3e
#define BQ78350_SBS_CELL_VOLTAGE_1            0x3f
#define BQ78350_SBS_FF                        0xFF

//TMS320F28032 Action
#define TMS320F28032_I2C_ADDRESS              0xEE
#define TMS320F28032_CANID		                0xA1
#define TMS320F28032_CANDATA		              0xB1
#define TMS320F28032_WriteCANID		            0xA0
#define TMS320F28032_WriteCANDATA		          0xB0

//MCP2515 Action
//#define SPI_CAN                 &hspi1
//#define SPI_TIMEOUT             10

// SENSOR TEMPERATURE
/*  
 *  NTSE0103FZB12
 *  B=3435, R25=10k, RS=10k
 *  T=(298.15*B)/(B+298.15*LN(((3.3*ADC/4096)*10)/(3.3-3.3*ADC/4096)/RS))-273.15
 *  
 */
#define SENSOR_TEMPERATURE_105             329 // 105 C
#define SENSOR_TEMPERATURE_100             368 // 100 C
#define SENSOR_TEMPERATURE_95              412 // 95 C
#define SENSOR_TEMPERATURE_90              462 // 90 C
#define SENSOR_TEMPERATURE_85              519 // 85 C
#define SENSOR_TEMPERATURE_80              584 // 80 C
#define SENSOR_TEMPERATURE_LIMIT_OFF       657 // 75 C 
#define SENSOR_TEMPERATURE_CRITICAL        741 // 70 C
#define SENSOR_TEMPERATURE_65              835 // 65 C
#define SENSOR_TEMPERATURE_60              941 // 60 C
#define SENSOR_TEMPERATURE_55              1059 // 55 C
#define SENSOR_TEMPERATURE_50              1191 // 50 C

// SYSTEM WATT
/* 
 *  value = ADC5 * 0.00080048
 *  System current = value * 15.228 
 *  System watt = System current * 15.228
 */
 // Rock 20170628 add syswat def  (almost...)

#define SYSTEM_WATT_650                   3000 // Rock for sir650 protect 1973
#define SYSTEM_WATT_450                   1920 // 450 W
#define SYSTEM_WATT_250                   920 // 250 W
// ADC FACTOR
#define FACTOR_X1                            9
#define FACTOR_X2                            1

// FSP 650
#define HS1                               0xA5
#define HS2                               0x5A
#define CMD_0x01                          0x01
#define CMD_0x02                          0x02
#define CMD_0x03                          0x03
#define CMD_0x04                          0x04
#define CMD_0x05                          0x05
#define CMD_0x06                          0x06
#define CMD_0x08                          0x08
#define CMD_0x09                          0x09

// Rock Fan Speed , Min 3500 MAX 0
/*
#define FAN_SPEED_10                      3150
#define FAN_SPEED_20                      2800
#define FAN_SPEED_30                      2480
#define FAN_SPEED_40                      2100
#define FAN_SPEED_50                      1750
#define FAN_SPEED_60                      1400
#define FAN_SPEED_70                      1050
#define FAN_SPEED_80                      700
*/

#endif  // __ADES_CFG_H_

#define BATTERY_STATUS2_OK_SMBUS           1//0000 0001
#define BATTERY_STATUS2_ERR_SMBUS        254//1111 1110

#define N_PSU_OTP50C                     240//1100 0000
#define N_PSU_OTP85C                      63//0011 1111

//

//#define SPI_CAN                 &hspi3
//#define SPI_TIMEOUT             10
