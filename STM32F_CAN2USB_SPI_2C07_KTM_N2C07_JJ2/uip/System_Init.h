#ifndef _SYSTEM_INIT_H_
#define _SYSTEM_INIT_H_

//#include "includes.h"
#include "def_generation.h"
#include "DM9051.h"
/****************************************************************************************/
/* Configure DM9051 SPI channel */
#define DM9051_SPI                                      ((SPI_TypeDef*)SPI2_BASE)// ((SPI_TypeDef*)SPI1_BASE) // SPI2
/* DM9051 SPI Chip Select Group and PIN*/
#define DM9051_SPI_CS_GROUP                             GPIOA
#define DM9051_SPI_CS_PIN                               GPIO_Pin_4
/* DM9051 SPI SCK & MISO & MOSI Group and PIN */
#define DM9051_SPI_PIN_GROUP                            GPIOA
#define DM9051_SPI_SCK_PIN                              GPIO_Pin_5
#define DM9051_SPI_MISO_PIN                             GPIO_Pin_6
#define DM9051_SPI_MOSI_PIN                             GPIO_Pin_7
/* DM9051 Interrupt  */
#define DM9051_RX_INT_GROUP                             GPIOC
#define DM9051_RX_INT_PIN                               GPIO_Pin_7
#define DM9051_RX_INT_PortSource                        GPIO_PortSourceGPIOC
#define DM9051_RX_INT_PINSource                         GPIO_PinSource7
#define DM9051_RX_INT_EXTI                              EXTI_Line7
#define DM9051_IRQHandler                               EXTI9_5_IRQHandler

#define DM9051_RX_Final 																GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)

/* DM9051 DMA Configure */
#ifdef SPI_DMA 
/* Define SPI USE DMA Channel */
#define DM9051_SPI_DMA                                  DMA1
#define DM9051_SPI_DMA_CLK                              RCC_AHBPeriph_DMA1
/* Define DMA RX Channel */
#define DM9051_SPI_DMA_RX_Channel                       DMA1_Channel2
#define DM9051_SPI_DMA_RX_TC_FLAG                       DMA1_FLAG_TC2
#define DM9051_SPI_DMA_RX_GL_FLAG                       DMA1_FLAG_GL2
/* Define DMA TX Channel */
#define DM9051_SPI_DMA_TX_Channel                       DMA1_Channel3
#define DM9051_SPI_DMA_TX_TC_FLAG                       DMA1_FLAG_TC3 
#define DM9051_SPI_DMA_TX_GL_FLAG                       DMA1_FLAG_GL3 
/* Define SPI Data Register Address */
#define DM9051_SPI_DMA_DR_BaseAddr                      SPI1_BASE + 0xC  // DMA Channel + SPI_DR Address = DMA_Channel + 0x0c
#endif //SPI_DMA
/****************************************************************************************/

void DM9051_SPI_Configuration(void);
/* Configure DM9051 R/W Register function prototypes */
uint8_t DM9051_Read_Reg(uint8_t Reg_Off);
void DM9051_Write_Reg(uint8_t Reg_Off, uint8_t spi_data);
/* Configure DM9051 R/W Memory function prototypes */
void DM9051_Read_Mem(uint8_t* pu8data, uint16_t datalen);
void DM9051_Write_Mem(uint8_t* pu8data, uint16_t datalen);
void DM9051_Free_Mem(uint16_t datalen);

/* STM32 Control all Interrupt function prototypes */
void INT_ALL_DISABLE(void);
void INT_ALL_ENABLE(void);

void DMA1_Channel3_IRQHandler(void);
void DMA1_Channel2_IRQHandler(void);

/* MCU Initional */
void delay_us(unsigned int us);
void delay_ms(unsigned int ms);
void SPI_Configuration(void);  //system_init.c
void DM9051_SPI_DMA_Configuration(void);
void DM9051_INT_GPIO_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void System_Initialization(void);
#endif /* _SYSTEM_INIT_H_*/
