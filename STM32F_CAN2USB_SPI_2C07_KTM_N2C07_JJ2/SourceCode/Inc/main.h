/**
  ******************************************************************************
  * @file    main.h 
  * @author  
  * @version 
  * @date    
  * @brief   Header for main.c module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h" 
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_spi.h"

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_hid.h" 

#include "ADES_BATTERY.h"
#include "ADES_CFG.h"
#include "ADES_GLOBAL_VAR.h"
#include "ADES_HARDWARE_FUNCTION.h"
#include "ADES_LIB.h"
#include "ADES_TICK_TASK.h"
#include "ADES_USB_PROCESS.h"
#include "ADES_EXE_FUNCTION.h"

#include "CANSPI1.h"
#include "MCP25151.h"
#include "CANSPI2.h"
#include "MCP25152.h"

#include "DM9051.h"
#include "uip.h"
#include "uip_arp.h"
#include "tapdev.h"
#include "timer.h"
#include "uip-conf.h"

#include "stdbool.h"

#include "led.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
extern const uint8_t main_program_version[64];

extern const uint8_t FSP650_FW_ID[17];
extern const uint8_t FSP650_RELEASE_DATE[11];

//extern PCD_HandleTypeDef hpcd;
/* Exported macro ------------------------------------------------------------*/
// GPIO OUT
#define I2C_CLK_CHARGER_H      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define I2C_CLK_CHARGER_L      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)

#define I2C1_SCL				       HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)
#define I2C1_SCL_H				       HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
#define I2C1_SCL_L				       HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)
#define I2C1_SDA 					     HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)
#define I2C1_SDA_H 					     HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET)
#define I2C1_SDA_L 					     HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET)


#define BATTERY_LOW_PIN_H      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET)
#define BATTERY_LOW_PIN_L      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET)
// NORMAL H   FAST CHARGE L  5A  0.5A
#define EN_288V_H              //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_SET) ///////////////
#define EN_288V_L              //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_RESET)

#define CAN_CS1_Pin GPIO_PIN_4
#define CAN_CS1_GPIO_Port GPIOA
#define CAN_CS2_Pin GPIO_PIN_12
#define CAN_CS2_GPIO_Port GPIOB

#define MCP2515_CS1_HIGH()   HAL_GPIO_WritePin(CAN_CS1_GPIO_Port, CAN_CS1_Pin, GPIO_PIN_SET)
#define MCP2515_CS1_LOW()    HAL_GPIO_WritePin(CAN_CS1_GPIO_Port, CAN_CS1_Pin, GPIO_PIN_RESET)
#define MCP2515_CS2_HIGH()   HAL_GPIO_WritePin(CAN_CS2_GPIO_Port, CAN_CS2_Pin, GPIO_PIN_SET)
#define MCP2515_CS2_LOW()    HAL_GPIO_WritePin(CAN_CS2_GPIO_Port, CAN_CS2_Pin, GPIO_PIN_RESET)

#define SPI_DM9051_CS_HIGH()   HAL_GPIO_WritePin(CAN_CS2_GPIO_Port, CAN_CS2_Pin, GPIO_PIN_SET)
#define SPI_DM9051_CS_LOW()    HAL_GPIO_WritePin(CAN_CS2_GPIO_Port, CAN_CS2_Pin, GPIO_PIN_RESET)
//#define EN_CHARGER_POWER_H     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
//#define EN_CHARGER_POWER_L     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
//#define TG_CHARGER_POWER_X     HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12)
//#define EN_CHARGER_POWER       HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)

//#define BI_ACT_H               HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET)
//#define BI_ACT_L               HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET)
#define RLED_INDICATOR_H         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)
#define RLED_INDICATOR_L         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)



//#define MCU1_OCP_H             HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)
//#define MCU1_OCP_L             HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)

//#define INTERLOCK_OUT_MCU_H    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)
//#define INTERLOCK_OUT_MCU_L    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)


#define LED_INDICATOR_H        //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)  ///////////////
#define LED_INDICATOR_L        //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)

#define ENABLE_54V_H        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)
#define ENABLE_54V_L        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)
#define LED2_BAT_H        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET)
#define LED2_BAT_L        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET)
#define LED1_STATUS_H        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
#define LED1_STATUS_L        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define ENABLE_CHARGER_L              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET)	// BQ25756E_CE PA6 ENABLE_CHARGER //ENABLE_CHARGER_H
#define ENABLE_CHARGER_H              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET)	// BQ25756E_CE PA6 ENABLE_CHARGER  //ENABLE_CHARGER_L
#define ENABLE_LANPOWER_H              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)
#define ENABLE_LANPOWER_L              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)

#define DM9051_Reset_H              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET)
#define DM9051_Reset_Active_L              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET)


#define DM9051_INT_IN         HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_10)
/*#define EN_CHARGER_H           HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)
#define EN_CHARGER_L           HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)
#define EN_CHARGER             HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)

#define EN_FAST_CHARGER_H      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)// normal   H 29V  ,  L 26.2
#define EN_FAST_CHARGER_L      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)
#define EN_FAST_CHARGER        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)

#define TG_FAST_CHARGER        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14)
#define FAST_CHARGER           HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)

#define BATTERY_CHECK_H        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)
#define BATTERY_CHECK_L        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)
#define BATTERY_CHECK_PIN      HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_15)  */

//#define ENATXPOWER_N_AUX_H     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
//#define ENATXPOWER_N_AUX_L     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
//#define ENATXPOWER_N_AUX       HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_12)  // INT

#define ENATXPOWER_MCU1        HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_1)  // INT

#define AC_FAIL_N_MCU1         1//HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_2)  // INT ///////////////  ????

#define ACINOK_N_MCU1          HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_15)  // INT ///////////////

#define PWRGOOD_N_MCU1         HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_10)  // INT ///////////////  ????

#define BATTERY_INSERT_N1      //HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_9)  // INT ///////////////

#define PSON_MCU1              1//HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_13)	/////////// ????


#define LowPowerKey            HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_0)


//#define INTERLOCK_OUT_MCU      HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_3)

#define DATA_TEMP2      			 HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_3)

/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
