/******************** (C) COPYRIGHT 2014 91mcu **************************
* File Name         : system_init.c
* Description       : System Init       
* Platform          : STM32F103ZE
* Libraries version : ST3.5.0
**********************************************************************************/	
//#include "includes.h"
#include "uip.h"
#include "System_Init.h"
#include "DM9051.h"
#include "stm32f4xx_hal.h"

#include "MCP25152.h"
#include "main.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* DM9051 SPI DMA Interrupt Flag, no use uCOS/-II*/
#ifndef uCOS_EN 
uint8_t RxComplete = 0;
uint8_t TxComplete = 0;
#endif //uCOS_EN

extern SPI_HandleTypeDef        hspi1;
#define SPI_CAN                 &hspi1
#define SPI_TIMEOUT             10
/**
* @brief  This function handles disble all interrupt.
* @param  None
* @retval None
*/
void INT_ALL_DISABLE(void)
{		  
///	__ASM volatile("cpsid i");
}

/**
* @brief  This function handles Enable ALL Interrupt.
* @param  None
* @retval None
*/
void INT_ALL_ENABLE(void)
{
///	__ASM volatile("cpsie i");		  
}

/**
* @brief  This function handles DMA1 Channel 2 interrupt request.
* @param  None
* @retval None
*/
void DMA1_Channel2_IRQHandler(void)
{
}

/**
* @brief  This function handles Tx DMA1 Channel 3 interrupt request.
* @param  None
* @retval None
*/
void DMA1_Channel3_IRQHandler(void)
{
}

void delay_us(unsigned int us)
{
	uint16_t i = 0;
	
	while(us--)
	{
		i = 10;
		while(i--);
	}
}

void delay_ms(unsigned int ms)
{
	unsigned char i=100,j;
	
	for(;ms;ms--)
	{
		while(--i)
		{
			j=10;
			while(--j);
		}
	}
}

/* SPI Tx Wrapper function */
static void SPI_Tx(uint8_t data)
{
  HAL_SPI_Transmit(SPI_CAN, &data, 1, SPI_TIMEOUT);    
}

/* SPI Tx Wrapper function */
static void SPI_TxBuffer(uint8_t *buffer, uint8_t length)
{
  HAL_SPI_Transmit(SPI_CAN, buffer, length, SPI_TIMEOUT);    
}

/*********************************************************************************************************
* @brief  Sends a byte through the SPI interface and return the byte received from the SPI bus.
* @param  byte : byte to send.
* @retval The value of the received byte.
**********************************************************************************************************/
static __inline uint8_t SPI_DM9051_SendByte(uint8_t byte){
//static __inline uint8_t SPI_DM9051_SendByte	(uint8_t startAddress, uint8_t endAddress, uint8_t *data){
  //SPI_DM9051_CS_LOW();
  uint8_t byte1;
  HAL_SPI_Transmit(&hspi2, &byte, 1, SPI_TIMEOUT);
  HAL_SPI_Receive(&hspi2, &byte1, 1, SPI_TIMEOUT);
	return byte1;
  //SPI_Tx(MCP2515_WRITE);
  ///SPI_Tx(startAddress);
  //SPI_TxBuffer(data, (endAddress - startAddress + 1));
  
  //SPI_DM9051_CS_HIGH();
}



/* -----------------------------------------------------*/
/* DM9051_Read_Mem()																		*/
/* DM9051 burst read command: SPI_RD_BURST = 0x72				*/
/*------------------------------------------------------*/
void DM9051_Read_Mem(uint8_t* pu8data, uint16_t datalen)
//void hal_spi_mem_read(uint8_t *buf, uint16_t len)
{
	SPI_DM9051_CS_LOW();
        //uint8_t reg = DM9051_MRCMD | OPC_REG_R;
        uint8_t reg = DM9051_MRCMD;
        HAL_SPI_Transmit(&hspi2, &reg, 1, SPI_TIMEOUT);
        HAL_SPI_Receive(&hspi2, pu8data, datalen, SPI_TIMEOUT*5);
	SPI_DM9051_CS_HIGH();	
}

/* -----------------------------------------------------*/
/* DM9051_Write_Mem()																		*/
/* DM9051 burst write command: SPI_WR_BURST = 0xF8			*/
/*------------------------------------------------------*/


void DM9051_Write_Mem(uint8_t* pu8data, uint16_t datalen)
{    
	
	SPI_DM9051_CS_LOW();
	//uint8_t reg = DM9051_MWCMD | OPC_REG_W;
	uint8_t reg = DM9051_MWCMD | 0x80;
	HAL_SPI_Transmit(&hspi2, &reg, 1, SPI_TIMEOUT);
	HAL_SPI_Transmit(&hspi2, pu8data, datalen, SPI_TIMEOUT*5);
	SPI_DM9051_CS_HIGH();	
}

void DM9051_Free_Mem(uint16_t datalen)
{
	uint32_t i;
	uint8_t burstcmd = SPI_RD_BURST; // Read SPI_Data_Array back from the slave
	
#ifdef DM9051_INT	
	INT_ALL_DISABLE();
#endif //DM9051_INT

	SPI_DM9051_CS_LOW();
	
	SPI_DM9051_SendByte(burstcmd);
	
	for(i=0; i<datalen; i++) {
		SPI_DM9051_SendByte(0x0);
	}
	SPI_DM9051_CS_HIGH();
	
#ifdef DM9051_INT
	INT_ALL_ENABLE();
#endif //DM9051_INT
}	

/* -----------------------------------------------------*/
/* DM9051_Read_Reg()																		*/
/* SPI read command: bit7 = 0,													*/
/*                   bit[6:0] = Reg. offset addr				*/
/*------------------------------------------------------*/

/* 1 byte read */
uint8_t DM9051_Read_Reg (uint8_t Reg_Off)
{
  uint8_t retVal;
  
  SPI_DM9051_CS_LOW();
  
  //SPI_Tx(MCP2515_READ);
  //SPI_Tx(Reg_Off);
  //retVal = SPI_Rx();
  
  HAL_SPI_Transmit(&hspi2, &Reg_Off, 1, SPI_TIMEOUT);
  HAL_SPI_Receive(&hspi2, &retVal, 1, SPI_TIMEOUT);
      
  SPI_DM9051_CS_HIGH();
  
  return retVal;
}


/* -----------------------------------------------------*/
/* DM9051_Write_Reg()																		*/
/* SPI write command: bit7 = 1, 												*/
/*                   bit[6:0] = Reg. offset addr				*/
/*------------------------------------------------------*/
void DM9051_Write_Reg(uint8_t Reg_Off, uint8_t spi_data)
{

	uint8_t cmdaddr;
	uint8_t data;
	cmdaddr = (Reg_Off | 0x80);
	data = spi_data;
	SPI_DM9051_CS_LOW();
  
  HAL_SPI_Transmit(&hspi2, &cmdaddr, 1, SPI_TIMEOUT);
  HAL_SPI_Transmit(&hspi2, &data, 1, SPI_TIMEOUT);
      
	
	SPI_DM9051_CS_HIGH();

	return;
}

/**
* @brief  Configure the nested vectored interrupt controller.
* @param  None
* @retval None
*/
void NVIC_Configuration(void)
{
#ifdef DM9051_INT	
	
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_SystemLPConfig(NVIC_PriorityGroup_2);	//??NVIC????2:2??????,2??????
	

	/* Configure one bit for preemption priority */
	NVIC_SystemLPConfig(NVIC_PriorityGroup_1);
	
	/* Enable the EXTI7 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif //DM9051_INT
	
#ifdef DMA_INT	
	/* Enable Rx DMA1 channel2 IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable Tx DMA1 channel3 IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif //DMA_INT
}

/**
* @brief  Configure the DM9051 SPI Initialization.
* @param  None
* @retval None
*/
#if 0
void DM9051_SPI_Configuration(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	
  /* SPI pin mappings */
  /*GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_0);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_0);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_0);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_0);*/
	
	/* Configure DM9051_SPI pins: SCK, MISO and MOSI */
	GPIO_InitStructure.GPIO_Pin = DM9051_SPI_SCK_PIN | DM9051_SPI_MISO_PIN | DM9051_SPI_MOSI_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DM9051_SPI_PIN_GROUP, &GPIO_InitStructure);

	/* Configure I/O for DM9051 Chip select */
	GPIO_InitStructure.GPIO_Pin = DM9051_SPI_CS_PIN;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(DM9051_SPI_CS_GROUP, &GPIO_InitStructure);

	//GPIO_SetBits(DM9051_SPI_PIN_GROUP, DM9051_SPI_SCK_PIN | DM9051_SPI_MISO_PIN | DM9051_SPI_MOSI_PIN);

	/* Deselect the FLASH: Chip Select high */
	//SPI_DM9051_CS_HIGH();

	/* DM9051_SPI configuration */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	//SPI_RxFIFOThresholdConfig(SPI1,SPI_RxFIFOThreshold_QF);
	SPI_Init(DM9051_SPI, &SPI_InitStructure);

	/* Enable DM9051_SPI  */
	SPI_Cmd(DM9051_SPI, ENABLE);

#ifdef SPI_DMA
	RCC_AHBPeriphClockCmd(DM9051_SPI_DMA_CLK, ENABLE);
	
	/* SPI_MASTER_Rx_DMA_Channel configuration ---------------------------------*/
	DMA_DeInit(DM9051_SPI_DMA_RX_Channel);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)DM9051_SPI_DMA_DR_BaseAddr;
#ifndef LwIP_EN
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uip_buf;
#endif  
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	//DMA_InitStructure.DMA_BufferSize = BufferSize;		//RX length
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DM9051_SPI_DMA_RX_Channel, &DMA_InitStructure);

	/* SPI_MASTER_Tx_DMA_Channel configuration ---------------------------------*/
	DMA_DeInit(DM9051_SPI_DMA_TX_Channel);  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)DM9051_SPI_DMA_DR_BaseAddr;
#ifndef LwIP_EN
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uip_buf;
#endif  
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DM9051_SPI_DMA_TX_Channel, &DMA_InitStructure);

#ifdef DMA_INT
	/* Enable DMA1 Channel2,3 Transfer Complete interrupt */
	DMA_ITConfig(SPI_MASTER_Rx_DMA_Channel, DMA_IT_TC, ENABLE);
	DMA_ITConfig(SPI_MASTER_Tx_DMA_Channel, DMA_IT_TC, ENABLE);  
#endif	//DMA_INT  

#endif //SPI_DMA

#ifdef DM9051_INT
	/* configure PC7 as external interrupt */
	GPIO_InitStructure.GPIO_Pin = DM9051_RX_INT_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(DM9051_RX_INT_GROUP, &GPIO_InitStructure);

	/* Configure DM9051 EXTI Line to generate an interrupt on falling edge */
	EXTI_InitStructure.EXTI_Line = EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Connect DM9051 EXTI Line to GPIOC Pin 7 */
	GPIO_EXTILineConfig(DM9051_RX_INT_PortSource, DM9051_RX_INT_PINSource);
	
	/* Clear DM9051 EXTI line pending bit */
	EXTI_ClearITPendingBit(DM9051_RX_INT_EXTI);	
#endif //DM9051_INT
}
#endif
/*******************************************************************************
* Function Name  : USART_Configuration
* Description    : Configure USART2
*******************************************************************************/
/*
void USART2_Configuration(void)
{
	#if 1
		// APB1 periph_USART2 & 
		// APB2 periph_GPIOA 
		#if 1
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // <-- USART2 'while-op' avoid hang
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // <--- USART2 GPIO_Pin_2 & GPIO_Pin_3 ebable it OK.
		#endif
	
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
		
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    USART_InitStructure.USART_BaudRate = 115200; //9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure); 
		USART_Cmd(USART2, ENABLE); // ?? USART1
	//main_test_atcmd_usart_print();
	#endif
}

/*******************************************************************************
* Function Name  : USART_Configuration
* Description    : Configure USART1 
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
/*
void USART1_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure; 

	//GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
	//GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
	
	// USART1_TX -> PA9 , USART1_RX ->	PA10 				
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);		   

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure); 
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	USART_ClearFlag(USART1,USART_FLAG_TC);
	USART_Cmd(USART1, ENABLE);
}
*/

/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{
#if 0
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();
	
	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if (HSEStartUpStatus == SUCCESS)
	{
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		/* Flash 2 wait state */
		FLASH_SetLatency(FLASH_Latency_2);

		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);

		/* PCLK2 = HCLK/2 */
		RCC_PCLK2Config(RCC_HCLK_Div2);

		/* PCLK1 = HCLK/2 */
		RCC_PCLK1Config(RCC_HCLK_Div2);

		/* PLLCLK = 8MHz * 9 = 72 MHz */
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

		/* Enable PLL */
		RCC_PLLCmd(ENABLE);

		/* Wait till PLL is ready */
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{}

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* Wait till PLL is used as system clock source */
		while (RCC_GetSYSCLKSource() != 0x08)
		{}
	}
#endif //0
#if 0
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();
	RCC_HSICmd(ENABLE); //Enable HSI 
	while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET){} //Wait HSI Enable
	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	RCC_PCLKConfig(RCC_HCLK_Div2);
	RCC_PCLKConfig(RCC_HCLK_Div1);	
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_2);
	RCC_PLLCmd(ENABLE);
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){}
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	while(RCC_GetSYSCLKSource() != 0x08){}
#endif //0
		
#if 0
	/* Enable DM9051_SPI and GPIO clocks */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_USART1, ENABLE); // | RCC_APB2Periph_SYSCFG
#endif //0
}

/**
* @brief  This function handles system IO Configuration.
* @param  None
* @retval None
*/
void System_Initialization(void)
{	
	/* System clocks configuration */
	RCC_Configuration();
	
	/* UART2 Configuration */
	//USART2_Configuration();
	/* UART1 Configuration */
	//USART1_Configuration();

	/* DM9051 SPI & DMA Configure */
	//DM9051_SPI_Configuration();
	
	/* NVIC configuration */
	NVIC_Configuration();
}

ErrorStatus HSEStartUpStatus;

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{	
	#if 0
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);
  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	#endif
	#if 0
	USART_SendData(USART2, (uint8_t) ch);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
  return ch;
	#endif
  return 0;
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

#if 1

#if 0
char at_tmpstr[120];
//#define _PR2(format, args...) do { snprintf(at_tmpstr, sizeof(at_tmpstr), format, ##args); atcmd_resp_cmd(at_tmpstr); } while (0) 

void u_writeuart1(char uartchar)
{
	USART_SendData(USART2, (uint8_t)uartchar);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}

/**
 * atcmd_resp_cmd(char *uartTXstr) : at uart response message.
 * AT command: kaconfig 10 5 5
 */
void atcmd_resp_cmd(char *uartTXstr)
{
		uint8_t txlen = strlen(uartTXstr);
		uint8_t i;
		
		u_writeuart1('+');
		for(i=0; i<txlen; i++) {
			u_writeuart1(uartTXstr[i]);	
		}
		u_writeuart1('\r');
		u_writeuart1('\n');
}
#endif

/*     =  test in main ()           */
#if 0
void main_test_atcmd_usart_print(void)
{
	u_writeuart1('*'); // This 'char', Not OUT to the stream
	u_writeuart1('\r');
	u_writeuart1('\n');
	
	u_writeuart1('*');
	u_writeuart1('\r');
	u_writeuart1('\n');
	
	u_writeuart1('*');
	u_writeuart1('\r');
	u_writeuart1('\n');
	
	//(1)
	_PR2("JJ:%d (DEBUG_USART USART1) NOT equip in my board", 0);
	_PR2("JJ:%d (ATCMD_USART USART2) Used for all of Dbg,ATCMD,cvt-dat", 1);
	
	//(2)
	u_writeuart1('+');

	u_writeuart1('O');
  #if 1	
	USART_SendData(USART2, (uint8_t)'O');
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
	#endif	
	
	u_writeuart1('\r');
	u_writeuart1('\n');
	
	//(3)
	atcmd_resp_cmd("X");
	
	//(5)
		uint8_t txlen = 1;
		uint8_t i;
		char ch= 'Y'; 
		
		u_writeuart1('+');
		for(i=0; i<txlen; i++) {
			u_writeuart1(ch);	
		}
		u_writeuart1('\r');
		u_writeuart1('\n');
	
	u_writeuart1('+');
	for(i=0; i<txlen; i++) {
		u_writeuart1('O');
	}
	u_writeuart1('\r');
	u_writeuart1('\n');
	
	u_writeuart1('\r');
	u_writeuart1('\n');
}
#endif

#if 0
void arp_test_broadcast_print(struct arp_hdr *BUF)
{
	printf("+DestMac************************ %02x %02x %02x %02x %02x %02x\r\n",
		BUF->ethhdr.dest.addr[0],
		BUF->ethhdr.dest.addr[1],
		BUF->ethhdr.dest.addr[2],
		BUF->ethhdr.dest.addr[3],
		BUF->ethhdr.dest.addr[4],
		BUF->ethhdr.dest.addr[5]);
	printf("+SrcIP************************** %d.%d.%d.%d\r\n",
		uip_ipaddr1(BUF->sipaddr),
		uip_ipaddr2(BUF->sipaddr),
		uip_ipaddr3(BUF->sipaddr),
		uip_ipaddr4(BUF->sipaddr));
}
#endif
#endif

