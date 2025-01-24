/*********************************************************************************************************//**
* @file    DM9051.c
* @version V1.02
* @date    2014/6/10
* @brief   The DM9051 driver for uCOSII+LwIP & uIP on STM32F103.
*************************************************************************************************************
*
* <h2><center>Copyright (C) 2014 DAVICOM Semiconductor Inc. All rights reserved</center></h2>
*
***********************************************************************************************************
* History:
*	V1.02:  2014/8/8
*					1. Add bus driving to 8mA(DM9051_PBCR). For SPI clock upgrade to 36Mhz.
*
*/

/* Includes ------------------------------------------------------------------------------------------------*/

#include "DM9051.h"

#include "System_Init.h"
#include "uip.h"
#include "uip-conf.h"

/* Private define ------------------------------------------------------------*/

enum DM9051_PHY_mode
{
	DM9051_10MHD   = 0,
	DM9051_100MHD  = 1,
	DM9051_10MFD   = 4,
	DM9051_100MFD  = 5,
	DM9051_10M     = 6,
	DM9051_AUTO    = 8,
	DM9051_1M_HPNA = 0x10
};

enum DM9051_TYPE
{
	TYPE_DM9051
};

struct DM9051_eth
{
	enum DM9051_TYPE type;
	enum DM9051_PHY_mode mode;
	uint8_t  imr_all;
	uint8_t  packet_cnt; // packet I or II 
	uint16_t queue_packet_len; // queued packet (packet II) 
	/* interface address info. */
	uint8_t  dev_addr[6]; // hw address 
	/* Byte counter */
	//uint32_t_t txb_count;	
	//uint32_t_t rxb_count;  
};

static struct DM9051_eth DM9051_device;

/* DM9051 SPI DMA Interrupt Flag, no use uCOS/-II*/
/*#ifndef uCOS_EN 
uint8_t RxComplete = 0;
uint8_t TxComplete = 0;
#endif //uCOS_EN*/

/* PHY function prototypes */
static uint16_t phy_read(uint32_t uReg);
static void phy_write(uint16_t uReg, uint16_t uValue);
static void phy_mode_set(uint32_t uMediaMode);
/* Delay function prototypes */
static void _DM9051_Delay(uint32_t uDelay);
static void _DM9051__DM9051_Delay_ms(uint32_t uDelay);

uint8_t DM9051_RX_INT_TRIGGER = 0;

extern struct uip_eth_addr uip_ethaddr;
#ifdef SPI_DMA															
#define SPI_MASTER_RX_DMA_CH 0
#define SPI_MASTER_TX_DMA_CH 1
#endif //SPI_DMA

/*********************************************************************************************************//**
  * @brief  uS level delay function.
  * @param  uDelay: Delay time
  * @retval None
  ***********************************************************************************************************/
static void _DM9051_Delay(uint32_t uDelay)
{
  while (uDelay--);
}

/*********************************************************************************************************//**
  * @brief  mS lebel delay function.
  * @param  uDelay: delay time based on ms.
  * @retval None
  ***********************************************************************************************************/
static void _DM9051_Delay_ms(uint32_t uDelay)
{
  uint32_t i, j;

  for (i = 0; i < uDelay; i++ )
  {
    for (j = 0; j < 1141; j++);
  }
}

/***************************************************
* @brief   This function Set DM9051 Fiber mode.
* @retval  None
***************************************************/
#ifdef DM9051_Fiber
void DM9051_Set_Fiber(void)
{
		phy_write(MII_BMCR,0x2100);
		//100M Full
		phy_write(MII_PHY_SPEC_CFG, 0x0010);
		//Disable AutoMdix
		phy_write(MII_PHY_DSCR, 0x4014); //Bypass Scrambler/Descrambler & 100BASE-FX mode 
}
#endif //DM9051_Fiber

#ifdef DM9051_INT
/***************************************************
* @brief   This function handles EXTI interrupt.
* @retval  None
***************************************************/
void DM9051_IRQHandler(void)
{
#ifdef uCOS_EN
	unsigned  int cpu_sr;
	/* Tell uC/OS-II that we are starting an ISR */
	OS_ENTER_CRITICAL();                        
	OSIntNesting++;
	OS_EXIT_CRITICAL();
#if 0
	if (OSIntNesting > 0){
		OSIntNesting--;
	}
#endif	//0
#endif //uCOS_EN
	
	//printf("DM9051_IRQHandler \r\n");
	
	if (EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
		/* DM9051 RX Interrupt Trigger Flag */
		//DM9051_RX_INT_TRIGGER = 1;
		/* Clear the EXTI line 6 pending bit */  
		EXTI_ClearITPendingBit(EXTI_Line7);		
		
		DM9051_isr();
	}
	
#ifdef uCOS_EN	
	/* Tell uC/OS-II that we are leaving the ISR */
	OSIntExit();
#endif //0uCOS_EN
}


/****************************************
* @brief  Interrupt service routine.
* @retval None
****************************************/
void DM9051_isr(void)
{
	uint16_t int_status;
	uint8_t temp;
	uint32_t value = 0;
	uint16_t rx_status;
	uint8_t ReceiveData[4];
	uint16_t rx_len = 0;
	
	/* Disable RX Interrupt */
	//DM9051_Write_Reg(DM9051_IMR, DM9051_IMR_OFF);
	
	/* Got DM9051 interrupt status  */
	int_status = DM9051_Read_Reg(DM9051_ISR);    

	/* Clear ISR status */
	DM9051_Write_Reg(DM9051_ISR, (uint8_t)int_status);

	//printf("DM9051 isr: int status %04x \r\n", int_status); 
	
	/* receive overflow */
	if ((int_status & ISR_ROS) || (int_status & ISR_ROOS)){
		printf("overflow or overflow counter overflow\r\n");
		DM9051_RX_INT_TRIGGER = 1;
	}
	
	/* Received the coming packet */
	if (int_status & ISR_PRS)
	{
#ifdef uCOS_EN
		extern OS_EVENT* ethernetinput;
		/* a frame has been received */
		OSSemPost(ethernetinput);		
#endif //uCOS_EN
		
#ifndef LwIP_EN
		DM9051_RX_INT_TRIGGER = 1;
		//uip_len = (uint16_t)etherdev_read();
#endif //LwIP_EN
		
	}
	
#if 0
	/* Transmit Interrupt check                                                                               */
	if (int_status & ISR_PTS)
	{
		/* transmit done                                                                                        */
		int tx_status = DM9051_Read_Reg(DM9051_NSR);    /* Got TX status                                        */

		if (tx_status & (NSR_TX2END | NSR_TX1END))
		{
#if 0    	
			DM9051_device.packet_cnt --;
			if (DM9051_device.packet_cnt > 0)
			{
				printf("DM9051 isr: tx second packet \r\n");

				/* transmit packet II                                                                               */
				/* Set TX length to DM9051                                                                          */
				DM9051_Write_Reg(DM9051_TXPLL, DM9051_device.queue_packet_len & 0xff);
				DM9051_Write_Reg(DM9051_TXPLH, (DM9051_device.queue_packet_len >> 8) & 0xff);

				/* Issue TX polling command                                                                         */
				DM9051_Write_Reg(DM9051_TCR, TCR_TXREQ);    /* Cleared after TX complete                            */
			}
#endif	//0
			/* unlock DM9051 device                                                                               */
			//OSSemPost(DM9051_sem_ack);	//For TX interrupt
			
		}
	}
#endif	//0
	
	/* Enable all interrupt */
  //DM9051_Write_Reg(DM9051_IMR, IMR_PAR | IMR_PRM);
}
#endif	//DM9051_INT

/************************************************************************************************************
* @brief  Read function of PHY.
* @param  uReg: PHY register
* @retval uData: Data of register
***********************************************************************************************************/
static uint16_t phy_read(uint32_t uReg)
{
	uint16_t uData;

	/* Fill the phyxcer register into REG_0C */
	DM9051_Write_Reg(DM9051_EPAR, DM9051_PHY | uReg);
	/* Issue phyxcer read command */
	DM9051_Write_Reg(DM9051_EPCR, 0xc);           

	/* Wait read complete */
	//_DM9051_Delay_ms(100);                        
	while(DM9051_Read_Reg(DM9051_EPCR) & 0x1) {_DM9051_Delay(1);}; //Wait complete

	/* Clear phyxcer read command */
	DM9051_Write_Reg(DM9051_EPCR, 0x0); 
	uData = (DM9051_Read_Reg(DM9051_EPDRH) << 8) | DM9051_Read_Reg(DM9051_EPDRL);

	return uData;
}

/*******************************************************************************
* Function Name  : phy_write
* Description    : Write a word to phyxcer
* Input          : - reg: reg
*                  - value: data
* Output         : None
* Return         : None
* Attention         : None
*******************************************************************************/
static void phy_write(uint16_t reg, uint16_t value)
{
	/* Fill the phyxcer register into REG_0C                                                                */
	DM9051_Write_Reg(DM9051_EPAR, DM9051_PHY | reg);

	/* Fill the written data into REG_0D & REG_0E */
	DM9051_Write_Reg(DM9051_EPDRL, (value & 0xff));
	DM9051_Write_Reg(DM9051_EPDRH, ((value >> 8) & 0xff));
	/* Issue phyxcer write command */
	DM9051_Write_Reg(DM9051_EPCR, 0xa);    

	/* Wait write complete */
	//_DM9051_Delay_ms(500);                       
	while(DM9051_Read_Reg(DM9051_EPCR) & 0x1){_DM9051_Delay(1);}; //Wait complete
	
	/* Clear phyxcer write command */
	DM9051_Write_Reg(DM9051_EPCR, 0x0);    
}

/*********************************************************************************************************//**
* @brief  Set PHY mode.
* @param  uMediaMode:
*         @DM9051_AUTO Auto negotiation
*         @DM9051_10MHD 10MHz, Half duplex
*         @DM9051_10MFD 10MHz, Full duplex
*         @DM9051_100MHD 100MHz, Half duplex
*         @DM9051_100MFD 100MHz, Full duplex
* @retval None
***********************************************************************************************************/
static void phy_mode_set(uint32_t uMediaMode)
{
	uint16_t phy_reg4 = 0x01e1, phy_reg0 = 0x1000;

	if (!(uMediaMode & DM9051_AUTO) )
	{
		switch (uMediaMode)
		{
		case DM9051_10MHD:
			{
				phy_reg4 = 0x21;
				phy_reg0 = 0x0000;
				break;
			}
		case DM9051_10MFD:
			{
				phy_reg4 = 0x41;
				phy_reg0 = 0x1100;
				break;
			}
		case DM9051_100MHD:
			{
				phy_reg4 = 0x81;
				phy_reg0 = 0x2000;
				break;
			}
		case DM9051_100MFD:
			{
				phy_reg4 = 0x101;
				phy_reg0 = 0x3100;
				break;
			}
		case DM9051_10M:
			{
				phy_reg4 = 0x61;
				phy_reg0 = 0x1200;
				break;
			}
		}
		
		/* Set PHY media mode */
		phy_write(4, phy_reg4);
		/* Write rphy_reg0 to Tmp */
		phy_write(0, phy_reg0);
		_DM9051_Delay(10);
	}
#if 0
	DM9051_Write_Reg(DM9051_GPCR, 0x01);  /* Let GPIO0 output                                                 */
	DM9051_Write_Reg(DM9051_GPR,  0x00);  /* Enable PHY                                                       */
#endif	//0
}


#ifndef LwIP_EN
/**************************************************/
/*uIP                                             */
/**************************************************/
void etherdev_init(void)
{
	DM9051_Init();
}

void etherdev_send(void)
{  
	DM9051_TX();
}

uint16_t etherdev_read(void)
{	
	return DM9051_RX();
}
#endif	//LwIP_EN

/*********************************************************************************************************//**
* @brief  Tx function.
* @param  pbuf: buffer link list
* @retval Always 0
***********************************************************************************************************/
#ifdef LwIP_EN
uint32_t DM9051_TX(struct netif *netif, struct pbuf* p)
#else
uint32_t DM9051_TX(void)
#endif	//LwIP_EN
{
#ifdef FifoPointCheck
	uint16_t calc_MWR;
#endif // FifoPointCheck
	
	uint16_t SendLength;
	
#ifdef LwIP_EN	
	uint8_t err;
	struct pbuf *q;
	/* Count TX bytes. */
	//netif->txb_count += p->tot_len;
	SendLength = p->tot_len;
#else
	SendLength = uip_len;
#endif	//LwIP_EN

#ifdef uCOS_EN
	/* lock DM9051 */
	OSSemPend(DM9051_sem_lock, 0,&err);
#endif //uCOS_EN

#ifdef DMA_INT
	/* Disable DM9051a interrupt */
	DM9051_Write_Reg(DM9051_IMR, IMR_PAR);
#endif //DMA_INT
	if(DM9051_Read_Reg(DM9051_TCR)& DM9051_TCR_SET){
		_DM9051_Delay(5);
	}
#ifdef FifoPointCheck
	/* 計算下一個傳送的指針位 , 若接收長度為奇數，需加一對齊偶字節。*/
	/* 若是超過 0x0bff ，則需回歸繞到 0x0000 起始位置 */
	calc_MWR = (DM9051_Read_Reg(DM9051_MWRH) << 8) + DM9051_Read_Reg(DM9051_MWRL);
	calc_MWR += SendLength;
	//printf("calc_MWR = 0x%X\r\n", calc_MWR);
	//	if(SendLength & 0x01) calc_MWR++;
	if(calc_MWR > 0x0bff) 
		calc_MWR -= 0x0c00;
#endif //FifoPointCheck

	/* Move data to DM9051 TX RAM */
	//_DM9051_WriteRegAddr(DM9051_MWCMD);

#ifdef LwIP_EN
	/* q traverses through linked list of pbuf's. 
			This list MUST consist of a single packet ONLY */
	q = p;
	//Write data to FIFO
	DM9051_Write_Reg(DM9051_TXPLL, q->tot_len & 0xff);
	DM9051_Write_Reg(DM9051_TXPLH, (q->tot_len >> 8) & 0xff);
	DM9051_Write_Mem((uint8_t*)q->payload, q->tot_len);
#else  
	
	/* Write data to FIFO */
	DM9051_Write_Reg(DM9051_TXPLL, uip_len & 0xff);
	DM9051_Write_Reg(DM9051_TXPLH, (uip_len >> 8) & 0xff);
	DM9051_Write_Mem(&uip_buf[0], SendLength);
#endif	//LwIP_EN

#if 0
	printf("Tx Read Pointer H = 0x%02X.\r\n", DM9051_Read_Reg(DM9051_TRPAH));
	printf("Tx Read Pointer L = 0x%02X.\r\n", DM9051_Read_Reg(DM9051_TRPAL));
	printf("DM9051_MWRH H = 0x%02X.\r\n", DM9051_Read_Reg(DM9051_MWRH));
	printf("DM9051_MWRL L = 0x%02X.\r\n", DM9051_Read_Reg(DM9051_MWRL));
#endif  //0

	
	/* Issue TX polling command */
	DM9051_Write_Reg(DM9051_TCR, TCR_TXREQ); /* Cleared after TX complete */
#if 0
	if (DM9051_device.packet_cnt == 0){
		printf("DM9051 tx: first packet \r\n");

		DM9051_device.packet_cnt ++;
		/* Set TX length to DM9051 */
		DM9051_Write_Reg(DM9051_TXPLL, p->tot_len & 0xff);
		DM9051_Write_Reg(DM9051_TXPLH, (p->tot_len >> 8) & 0xff);

		/* Issue TX polling command */
		DM9051_Write_Reg(DM9051_TCR, TCR_TXREQ);/* Cleared after TX complete */
	}else{
		printf("DM9051 tx: second packet \r\n");
		DM9051_device.packet_cnt ++;
		DM9051_device.queue_packet_len = p->tot_len;
	}
#endif //0

#if 0
	printf("Tx control Reg = 0x%02X.\r\n", DM9051_Read_Reg(DM9051_TCR));
	printf("Tx Read Pointer H = 0x%02X.\r\n", DM9051_Read_Reg(DM9051_TRPAH));
	printf("Tx Read Pointer L = 0x%02X.\r\n", DM9051_Read_Reg(DM9051_TRPAL));
	printf("DM9051_MWRH H = 0x%02X.\r\n", DM9051_Read_Reg(DM9051_MWRH));
	printf("DM9051_MWRL L = 0x%02X.\r\n", DM9051_Read_Reg(DM9051_MWRL));
	printf("Tx control Reg = 0x%02X.\r\n", DM9051_Read_Reg(DM9051_TCR));
	printf("ISR Reg = 0x%02X.\r\n", DM9051_Read_Reg(DM9051_ISR));
	printf("NSR Reg = 0x%02X.\r\n", DM9051_Read_Reg(DM9051_NSR));
#endif  //0


#ifdef FifoPointCheck	
	if(calc_MWR != ((DM9051_Read_Reg(DM9051_MWRH) << 8) + DM9051_Read_Reg(DM9051_MWRL)))
	{
#ifdef LwIP_EN
		netif->tx_reset++;
#endif	//LwIP_EN
		
#if 0 //Point_Error_Reset
		/* 若是指針出錯，等待此一封包送完 , 之後進行重置 */
		while(ior(DM9051_REG_TCR) & DM9051_TCR_SET) udelay (5);
		dm9k_reset();
		/* unlock DM9051 device                                                                                   */
		OSSemPost(DM9051_sem_lock);
		return;
#endif
		
		printf("DM9K MWR Error!! calc_MWR = 0x%X, SendLength = 0x%x\r\n", calc_MWR, SendLength);
		printf("MWRH = 0x%X, MWRL = 0x%X\r\n", DM9051_Read_Reg(DM9051_MWRH), DM9051_Read_Reg(DM9051_MWRL));
#if 0
		DM9051_Write_Reg(DM9051_ISR, 0x80);    /* DIsable INT                                               */
		DM9051_init(netif);
		DM9051_Write_Reg(DM9051_ISR, 0x81);    /* Enable INT */
#endif	//0

		/*若是指針出錯，將指針移到下一個傳送包的包頭位置  */
		DM9051_Write_Reg(DM9051_MWRH, (calc_MWR >> 8) & 0xff);
		DM9051_Write_Reg(DM9051_MWRL, calc_MWR & 0xff);
	}
#endif //FifoPointCheck

#ifdef DMA_INT
	/* Enable DM9051a interrupt */
	DM9051_Write_Reg(DM9051_IMR, IMR_PAR | IMR_PRM);
#endif //DMA_INT
	
#ifdef uCOS_EN
	/* unlock DM9051 device */
	OSSemPost(DM9051_sem_lock);
	/* wait ack */
	//OSSemPend(DM9051_sem_ack, 0,&err);	//For TX interrupt
#endif //uCOS_EN
	
	return 0;
}

/**
 * @brief  Read RX write pointer and memory data read address
 * 
 * @param  rwpa_wt  Pointer to store write pointer
 * @param  mdra_rd  Pointer to store read address
 */
void dm9051_read_rx_pointers(uint16_t *rwpa_wt, uint16_t *mdra_rd) 
{
  *rwpa_wt = (uint16_t)DM9051_Read_Reg(0x24) |      /* DM9051_RWPAL */
             (uint16_t)DM9051_Read_Reg(0x25) << 8;   /* DM9051_RWPAH */
             
  *mdra_rd = (uint16_t)DM9051_Read_Reg(0x74) |      /* DM9051_MRRL */
             (uint16_t)DM9051_Read_Reg(0x75) << 8;   /* DM9051_MRRH */
}

#define RX_BUFFER_START       0xC00
#define RX_BUFFER_END         0x4000
uint16_t wrpadiff(uint16_t rwpa_s, uint16_t rwpa_e)
{
    return (rwpa_e >= rwpa_s) ? 
           rwpa_e - rwpa_s : 
           (rwpa_e + RX_BUFFER_END - RX_BUFFER_START) - rwpa_s;
}

/*********************************************************************************************************//**
* @brief  Rx function.
* @retval pbuf
***********************************************************************************************************/
#ifdef LwIP_EN
/* LwIP TCP/IP stack */
struct pbuf *DM9051_RX(struct netif *netif)
#else
/* uIP TCP/IP stack  */
uint16_t DM9051_RX(void)
#endif	//uIP_NOOS
{
	uint8_t rxbyte;
	uint16_t rx_len = 0;
	//uint16_t i;
	uint16_t rx_status;
	uint8_t ReceiveData[4];
#ifdef FifoPointCheck
	uint16_t calc_MRR;
#endif  //FifoPointCheck
#ifdef LwIP_EN	
	uint8_t err;
	uint8_t* databyte = 0;
	struct pbuf* p;
	/* init p pointer */
	p = NULL;
#endif	//LwIP_EN

#ifdef uCOS_EN
	/* lock DM9051 */
	OSSemPend(DM9051_sem_lock, 0,&err);
#endif //uCOS_EN

#ifdef DMA_INT
	/* Disable DM9051a interrupt */
	DM9051_Write_Reg(DM9051_IMR, IMR_PAR);
#endif //DMA_INT

	/* Check packet ready or not */
	rxbyte = DM9051_Read_Reg(DM9051_MRCMDX);
	rxbyte = DM9051_Read_Reg(DM9051_MRCMDX);

	//printf("DM9051_rx process...1st byte = 0x%x\r\n",rxbyte);

	if((rxbyte != 1) && (rxbyte != 0)){

		/* Reset RX FIFO pointer */
		DM9051_Write_Reg(DM9051_RCR, RCR_DEFAULT);	//RX disable
		DM9051_Write_Reg(DM9051_MPCR, 0x01);		//Reset RX FIFO pointer
		_DM9051_Delay(2);
		DM9051_Write_Reg(DM9051_RCR, (RCR_DEFAULT | RCR_RXEN));		//RX Enable
		
		/* restore receive interrupt                                                                            */
		DM9051_device.imr_all |= IMR_PRM;
		//DM9051_device.imr_all = IMR_DEFAULT;
		DM9051_Write_Reg(DM9051_IMR, DM9051_device.imr_all);		
		
#ifdef LwIP_EN
		//netif->rx_reset++;
#endif	//LwIP_EN
#ifdef uCOS_EN
		/* unlock DM9051                                                                                          */
		OSSemPost(DM9051_sem_lock);
#endif //uCOS_EN
		return 0;
	}
	
/* debug section.s */	
		ReceiveData[0] = DM9051_Read_Reg(DM9051_MRCMDX);
		ReceiveData[1] = DM9051_Read_Reg(DM9051_PIDH);
		ReceiveData[2] = DM9051_Read_Reg(DM9051_PIDL);
/* debug section.e */	

	if (rxbyte){
#ifdef FifoPointCheck			
		calc_MRR = (DM9051_Read_Reg(DM9051_MRRH) << 8) + DM9051_Read_Reg(DM9051_MRRL);	//Save RX SRAM pointer
#endif //FifoPointCheck
		
		DM9051_Read_Reg(DM9051_MRCMDX);		//dummy read
		
		//DM9051_Read_Mem(ReceiveData, 4);
		ReceiveData[0] = DM9051_Read_Reg(SPI_RD_BURST);
		ReceiveData[1] = DM9051_Read_Reg(SPI_RD_BURST);
		ReceiveData[2] = DM9051_Read_Reg(SPI_RD_BURST);
		ReceiveData[3] = DM9051_Read_Reg(SPI_RD_BURST);
		
		rx_status = ReceiveData[0] + (ReceiveData[1] << 8);
		rx_len = ReceiveData[2] + (ReceiveData[3] << 8);
		
		//printf("DM9051 rx: status %04x len %d \r\n", rx_status, rx_len);
#ifdef LwIP_EN
		/* Count received bytes. */
		//netif->rxb_count += rx_len;
		/* allocate buffer */
		p = pbuf_alloc(PBUF_LINK, rx_len, PBUF_RAM);
		
		if(p != NULL){
			DM9051_Read_Mem((uint8_t *)p->payload, rx_len);
		}
		else{
			printf("DM9051 rx: no pbuf \r\n");
			/* no pbuf, discard data from DM9051  */
			DM9051_Read_Mem(databyte, rx_len);
		}		
#else
		// uip receive data over UIP_CONF_BUFFER_SIZE, return 0
		do {
			uint16_t dummy_rwpa;
			uint16_t mdra_rd_now;
			uint16_t diff;
			static uint16_t gkeep_mdra_rds = 0xc00;

			if(rx_len > UIP_CONF_BUFFER_SIZE){
			
				dm9051_read_rx_pointers(&dummy_rwpa, &mdra_rd_now);
				diff = wrpadiff(gkeep_mdra_rds, mdra_rd_now);
				printf("RX-Len Error : rxw %x, rx_pointer %x %x, diff %d\r\n", dummy_rwpa, gkeep_mdra_rds, mdra_rd_now, diff);
				
				DM9051_Free_Mem(rx_len);
				rx_len = 0;
				
				return rx_len;
			}else{
				//DM9051_Read_Mem(&uip_buf[0], rx_len);
				uint16_t i;
				for (i = 0; i< rx_len; i++)
					uip_buf[i] = DM9051_Read_Reg(SPI_RD_BURST);
				
				dm9051_read_rx_pointers(&dummy_rwpa, &mdra_rd_now);
				diff = wrpadiff(gkeep_mdra_rds, mdra_rd_now);
				printf("Got packet : rxw %x, rx_pointer %x %x, diff %d\r\n", dummy_rwpa, gkeep_mdra_rds, mdra_rd_now, diff);
			}
		} while(0);
#endif //LwIP_EN

#ifdef FifoPointCheck
		/* 若是超過 0x3fff ，則需回歸繞到 0x0c00 起始位置 */
		calc_MRR += (rx_len + 4);
		if(calc_MRR > 0x3fff) calc_MRR -= 0x3400;
		
		if(calc_MRR != ((DM9051_Read_Reg(DM9051_MRRH) << 8) + DM9051_Read_Reg(DM9051_MRRL)))
		{

			printf("DM9K MRR Error!!\r\n");
			printf("Predicut RX Read pointer = 0x%X, Current pointer = 0x%X\r\n", calc_MRR, ((DM9051_Read_Reg(DM9051_MRRH) << 8) + DM9051_Read_Reg(DM9051_MRRL)));
			
			/*若是指針出錯，將指針移到下一個包的包頭位置  */
			DM9051_Write_Reg(DM9051_MRRH, (calc_MRR >> 8) & 0xff);
			DM9051_Write_Reg(DM9051_MRRL, calc_MRR & 0xff);
		}
#endif //FifoPointCheck

		if ((rx_status & 0xbf00) || (rx_len < 0x40) || (rx_len > DM9051_PKT_MAX) ){
			//printf("rx error: status %04x, rx_len: %d \r\n", rx_status, rx_len);
			if (rx_status & 0x8000){
				////printf("rx length error \r\n");
			}
			
			if (rx_len > DM9051_PKT_MAX){
				////printf("rx length too big \r\n");
				/* RESET device */
				//_DM9051_WriteReg(DM9051_NCR, NCR_RST);
				//_DM9051_Delay_ms(5);
			}
#ifdef LwIP_EN
			pbuf_free(p);
			p = NULL;
#endif //LwIP_EN
		}
	}else{
		//printf("DM9051 rx: No packets received. \r\n");

		/* clear packet received latch status */
		/* restore receive interrupt */
		DM9051_device.imr_all |= IMR_PRM;
		//DM9051_device.imr_all = IMR_DEFAULT;
		DM9051_Write_Reg(DM9051_IMR, DM9051_device.imr_all);
		//rx_len = 0;
	}
	
#ifdef DMA_INT
	/* Enable DM9051 RX Interrupt Mask*/
	DM9051_Write_Reg(DM9051_IMR, IMR_PAR| IMR_PRM);
#endif //DMA_INT
	
#ifdef uCOS_EN
	/* unlock DM9051 */
	OSSemPost(DM9051_sem_lock);
#endif //uCOS_EN
	
#ifdef LwIP_EN	
	return p;
#else
	return rx_len;
#endif	//LwIP_EN	  
}

/*********************************************************************************************************
* @brief  DM9051 init function.
* @retval -1 or 0
**********************************************************************************************************/
#ifdef LwIP_EN
int32_t DM9051_Init(struct netif *netif)
#else
int32_t DM9051_Init()
#endif //LwIP_EN
{
	int i, oft, lnk;
	uint32_t value = 0;
	int retry = 0;
	uint16_t duplex, speed;
	
	/* Configure MCU SPI */
	//DM9051_SPI_Configuration();
	
	//_DM9051_Delay_ms(250);
	
	//DM9051_Write_Reg(DM9051_PBCR, PBCR_MAXDRIVE);
	/* Read DM9051 PID / VID, Check MCU SPI Setting correct */
		value |= (uint32_t)DM9051_Read_Reg(DM9051_VIDL);
	value |= (uint32_t)DM9051_Read_Reg(DM9051_VIDH) << 8;
	value |= (uint32_t)DM9051_Read_Reg(DM9051_PIDL) << 16;
	value |= (uint32_t)DM9051_Read_Reg(DM9051_PIDH) << 24;
	
	if(value != DM9051_ID){
		_DM9051_Delay_ms(250);
			value |= (uint32_t)DM9051_Read_Reg(DM9051_VIDL);
		value |= (uint32_t)DM9051_Read_Reg(DM9051_VIDH) << 8;
		value |= (uint32_t)DM9051_Read_Reg(DM9051_PIDL) << 16;
		value |= (uint32_t)DM9051_Read_Reg(DM9051_PIDH) << 24;
	}


	

	//value = (uint32_t)DM9051_Read_Reg(DM9051_NCR);
	//	DM9051_Write_Reg(DM9051_NCR, 0x08);
	//value = (uint32_t)DM9051_Read_Reg(DM9051_NCR);
	/* Show PIV / VID value */
	////printf("DM9051 id: 0x%x \r\n", value);

	/* Set DM9051 Network Type */
	DM9051_device.type  = TYPE_DM9051;
	
	/* Set Network Speed, Force 10M mode or Auto Mode */
#ifdef FORCE_10M
	/* Setting 10M mode */
	DM9051_device.mode = DM9051_10M;
#else
	/* Setting Auto Mode */
	DM9051_device.mode = DM9051_AUTO;
#endif //FORCE_10M
	
	//DM9051_device.packet_cnt = 0;//
	//DM9051_device.queue_packet_len = 0;

	/* SRAM Tx/Rx pointer automatically return to start address */
	/* Packet Transmitted, Packet Received                      */
	/* Enable DM9051 Interrupt Mask*/
	DM9051_device.imr_all = IMR_PAR | IMR_PRM;
	//DM9051_device.imr_all = IMR_DEFAULT;
	//DM9051_device.imr_all = 0x81;

#ifndef EEPROM_MAC_ADDR
#ifdef LwIP_EN
	DM9051_device.dev_addr[0] = netif->hwaddr[0];
	DM9051_device.dev_addr[1] = netif->hwaddr[1];
	DM9051_device.dev_addr[2] = netif->hwaddr[2];
	DM9051_device.dev_addr[3] = netif->hwaddr[3];
	DM9051_device.dev_addr[4] = netif->hwaddr[4];
	DM9051_device.dev_addr[5] = netif->hwaddr[5];
#else
	DM9051_device.dev_addr[0] = emacETHADDR0;
	DM9051_device.dev_addr[1] = emacETHADDR1;
	DM9051_device.dev_addr[2] = emacETHADDR2;
	DM9051_device.dev_addr[3] = emacETHADDR3;
	DM9051_device.dev_addr[4] = emacETHADDR4;
	DM9051_device.dev_addr[5] = emacETHADDR5;
#endif	//LwIP_EN
#endif //EEPROM_MAC_ADDR

#ifdef uCOS_EN
	//DM9051_sem_ack = OSSemCreate(0);	//For TX interrupt
	DM9051_sem_lock = OSSemCreate(1);
#endif //uCOS_EN
#ifdef DMA_INT  
	DM9051_txdma_flag = OSSemCreate(0);
	DM9051_rxdma_flag = OSSemCreate(0);
#endif	//DMA_INT

	/* RESET DM9051 device */
	for(i = 0; i <= 1; i++){
		DM9051_Write_Reg(DM9051_NCR, DM9051_REG_RESET);
		_DM9051_Delay(1); //wait 1 us
		DM9051_Write_Reg(DM9051_NCR, 0); // clear NCR
	}

	/* Set DM9051 SPI Type, This Part no standard, According to MCU SPI, Can reference DM9051 DataSheet and MCU SPI Set*/
	DM9051_Write_Reg(DM9051_GPCR, GPCR_GEP_CNTL);
	DM9051_Write_Reg(DM9051_GPR, 0x00);		//Power on PHY
	
	_DM9051_Delay_ms(100);

#ifdef DM9051_Fiber
	/* Set Fiber mode */
	DM9051_Set_Fiber();
#else
	/* Set PHY mode */
	phy_mode_set(DM9051_device.mode);
	//printf("PHY REG_0 = 0x%4x\r\n", phy_read(0));
	//printf("PHY REG_4 = 0x%4x\r\n", phy_read(4));
#endif //DM9051_Fiber
	
#ifndef EEPROM_MAC_ADDR
	/* Set mac address */
	for (i = 0, oft = DM9051_PAR; i < 6; i++, oft++){
		DM9051_Write_Reg(oft, DM9051_device.dev_addr[i]);
	}
#endif //EEPROM_MAC_ADDR
	
	/* set multicast address */
	for(i = 0; i < 8; i++){ /* Clear Multicast set */
		/* Set Broadcast */
		DM9051_Write_Reg(DM9051_MAR + i, (7 == i) ? 0x80 : 0x00); 
	}
	
	/* Show MAC Address */
	////printf("DM9051 MAC: ");
	for (i = 0, oft = DM9051_PAR; i < 6; i++, oft++){
		///printf("%02X:", DM9051_Read_Reg(oft));
		uip_ethaddr.addr[i] = DM9051_Read_Reg(oft);
	}
	////printf("\r\n");

	/************************************************
	*** Activate DM9051 and Setup DM9051 Registers **
	*************************************************/
	/* Clear DM9051 Set and Disable Wakeup function */
	DM9051_Write_Reg(DM9051_NCR, NCR_DEFAULT);
	
	/* Clear TCR Register set */
	DM9051_Write_Reg(DM9051_TCR, TCR_DEFAULT);
	//DM9051_Write_Reg(DM9051_TCR, 0x20); //Disable underrun retry.
	
	/* Discard long Packet and CRC error Packet */
	DM9051_Write_Reg(DM9051_RCR, RCR_DEFAULT);
	/*  Set 1.15 ms Jam Pattern Timer */
	DM9051_Write_Reg(DM9051_BPTR, BPTR_DEFAULT);
	
	/* Open / Close Flow Control */
#ifdef FLOW_CONTRORL
	DM9051_Write_Reg(DM9051_FCTR, FCTR_DEAFULT);
	DM9051_Write_Reg(DM9051_FCR, FCR_DEFAULT);
#else
	DM9051_Write_Reg(DM9051_FCTR, 0x3A);
	DM9051_Write_Reg(DM9051_FCR,  0x0); //Disable Flow_Control
#endif //FLOW_CONTRORL

	/* Set Memory Conttrol Register，TX = 3K，RX = 13K */
	DM9051_Write_Reg(DM9051_SMCR, SMCR_DEFAULT);
	/* Set Send one or two command Packet*/
	DM9051_Write_Reg(DM9051_TCR2, DM9051_TCR2_SET);
	
	//DM9051_Write_Reg(DM9051_TCR2, 0x80);
	DM9051_Write_Reg(DM9051_INTR, 0x1);

	/* Clear status */
	DM9051_Write_Reg(DM9051_NSR, NSR_CLR_STATUS);
	DM9051_Write_Reg(DM9051_ISR, ISR_CLR_STATUS);
	
	/* Judge DM9051 Link State Link OK */
	while((DM9051_Read_Reg(DM9051_NSR) & 0x40) != 0x40){
		retry++;
		if(retry >= 250000)
			break;
	}
		
	/* Judge DM9051 PHY speed mode */
	if ((DM9051_device.mode == DM9051_AUTO) || (DM9051_device.mode == DM9051_10M)){
		//printf("DM9051_REG_1fh = 0x%X.\r\n", DM9051_Read_Reg(0x1f));
		while (!(phy_read(1) & 0x20)){
			/* autonegation complete bit */
			_DM9051_Delay(2);
			//DBG_printf("PHY_REG00h = 0x%04X, PHY_REG01h = 0x%04X.\r\n", phy_read(0), phy_read(1));
			i++;
			if (i == 255){
				///printf("could not establish link \r\n");
				break;
			}
		}
	}

	/* see what we've got */
	duplex = DM9051_Read_Reg(DM9051_NCR) & 0x08;
	speed = DM9051_Read_Reg(DM9051_NSR) & 0x80;
	/*
	printf("operating at ");
		if((duplex == 0) && (speed  == 0)){
			printf("100M half duplex ");
		}else if((duplex == 0x08) && (speed == 0x80)){
			printf("10M full duplex ");
		}else if((duplex == 0) && (speed == 0x80)){
			printf("10M half duplex ");
		}else if((duplex == 0x08) && (speed == 0)){
			printf("100M full duplex ");
		}else{
			printf("unknown");
		}
	printf("mode \r\n");
	*/
		
	DM9051_Write_Reg(DM9051_IMR, DM9051_device.imr_all);    /* Enable RX interrupt mask */
	//DM9051_Write_Reg(DM9051_IMR, 0x82);
	DM9051_Write_Reg(DM9051_RCR, (RCR_DEFAULT | RCR_RXEN));  /* Enable RX */
	//DM9051_Write_Reg(DM9051_RCR, 0x03);		//promiscuous mode
		
	return 0;
}
