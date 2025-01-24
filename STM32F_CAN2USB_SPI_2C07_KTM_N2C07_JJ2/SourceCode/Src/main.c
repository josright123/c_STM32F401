/*
***************************************************************************************************
* SOFTWARE : APPLICATION
* AUTHOR   : Hao
* VERSION  : 0V0
* DATE     : 2014.05.08
***************************************************************************************************
* ========================================
* NVIC (Priority)
*   0 : EXTI Line[9:5] interrupts
*  16 : System Tick Timer
*  32 : DMA2 Stream0 global interrupt
*  48 : USB On The Go FS global interrupt
* ========================================
* [ Main loop ]
* 000 ms : LED flash
* 010 ms :
* 020 ms : (FSP400) Bq20z45 action (Gas Gauge)
(FSP500) Bq78350 action (Gas Gauge)
* 030 ms : Check battery existence and verify type
* 040 ms :
* 050 ms :
* 060 ms : Power process
* 070 ms :
* 080 ms : Read HT66 Flash Parameter
* 090 ms :
* 100 ms :
* 110 ms :
* 120 ms : (FSP400) Bq20z45 action (Gas Gauge)
(FSP500) Bq78350 action (Gas Gauge)
* 130 ms : Check battery existence and verify type
* 140 ms :
* 150 ms :
* 160 ms : Battery algorithm
* 170 ms :
* 180 ms :
* 190 ms :
* 200 ms : LED flash
* 210 ms :
* 220 ms : (FSP400) Bq20z45 action (Gas Gauge)
(FSP500) Bq78350 action (Gas Gauge)
* 230 ms : Check battery existence and verify type
* 240 ms :
* 250 ms :
* 260 ms : Check battery discharge function
* 270 ms :
* 280 ms :
* 290 ms :
* 300 ms :
* 310 ms : FAN PWM verify
* 320 ms : (FSP400) Bq20z45 action (Gas Gauge)
(FSP500) Bq78350 action (Gas Gauge)
* 330 ms : Check battery existence and verify type
* 340 ms :
* 350 ms :
* 360 ms :
* 370 ms :
* 380 ms : (FSP400) Bq24745 action (Charger)
* 390 ms :
* 400 ms : LED flash
* 410 ms :
* 420 ms : (FSP400) Bq20z45 action (Gas Gauge)
(FSP500) Bq78350 action (Gas Gauge)
* 430 ms : Check battery existence and verify type
* 440 ms :
* 450 ms :
* 460 ms :
* 470 ms :
* 480 ms :
* 490 ms :
* 500 ms :
* 510 ms :
* 520 ms : (FSP400) Bq20z45 action (Gas Gauge)
(FSP500) Bq78350 action (Gas Gauge)
* 530 ms : Check battery existence and verify type
* 540 ms :
* 550 ms :
* 560 ms : Power process
* 570 ms :
* 580 ms : (Root) Root function
* 590 ms :
* 600 ms : LED flash
* 610 ms :
* 620 ms : (FSP400) Bq20z45 action (Gas Gauge)
(FSP500) Bq78350 action (Gas Gauge)
* 630 ms : Check battery existence and verify type
* 640 ms :
* 650 ms :
* 660 ms : Battery algorithm
* 670 ms :
* 680 ms :
* 690 ms :
* 700 ms :
* 710 ms :
* 720 ms : (FSP400) Bq20z45 action (Gas Gauge)
(FSP500) Bq78350 action (Gas Gauge)
* 730 ms : Check battery existence and verify type
* 740 ms :
* 750 ms :
* 760 ms : Check battery discharge function
* 770 ms :
* 780 ms :
* 790 ms :
* 800 ms : LED flash
* 810 ms :
* 820 ms : (FSP400) Bq20z45 action (Gas Gauge)
(FSP500) Bq78350 action (Gas Gauge)
* 830 ms : Check battery existence and verify type
* 840 ms :
* 850 ms :
* 860 ms :
* 870 ms :
* 880 ms : (FSP400) Bq24745 action (Charger)
* 890 ms :
* 900 ms : Calculate system timer
* 910 ms :
* 920 ms : (FSP400) Bq20z45 action (Gas Gauge)
(FSP500) Bq78350 action (Gas Gauge)
* 930 ms : Check battery existence and verify type
* 940 ms :
* 950 ms :
* 960 ms :
* 970 ms :
* 980 ms :
* 990 ms :
***************************************************************************************************
*/

/**
******************************************************************************
* @file    main.c
* @author
* @version
* @date
* @brief
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "uip_init.h"

/* Private macro -------------------------------------------------------------*/
#define ABS(x)         (x < 0) ? (-x) : x
#define MAX_AB(a,b)       (a < b) ? (b) : a

extern void BQ25756EInit(void);
extern void ARP_TEST(void);
/* Private constant ---------------------------------------------------------*/
// __DATE__ ([EX] Aug 25 2014 (11bytes))
// APP : Application 
//const uint8_t main_program_version[64] = "A01 2V10"__DATE__" U-power technology Co., Ltd.";
const uint8_t main_program_version[64] = "A01 2C70"__DATE__" U-power technology Co., Ltd.";

// FSP 650
const uint8_t FSP650_FW_ID[17] = "2M.0001.0001.123";
const uint8_t FSP650_RELEASE_DATE[11] = "2015052601";

extern uint16_t LowPowerCnt;

extern uint8_t  RUNNING_PROGRAM;



extern void control_LED_all(void);
extern void update_mode(void );
extern void set_led_mode(uint8_t mode);
extern void get_sys_status(void);
extern void EN_charging(uint8_t EN_CHG);
extern uint8_t resetLED;
void uip_EventPoll(void); //??????
#define UIP_BUF ((struct uip_eth_hdr *)&uip_buf[0])
/* Private functions ---------------------------------------------------------*/
//UIP_APPCALL
void UIP_APPCALL1(void){
    
}
//UIP_APPCALL
void main_uiplog(char* msg) {
    
}

void Uip_init(void){
	
    tapdev_init();//???enc28j60
	
    //???UIP
    uip_init();
    uip_ipaddr_t ipaddr;
    uip_ipaddr(ipaddr, 192, 168, 1, 8);//MCU IP??
    uip_sethostaddr(ipaddr);
    uip_ipaddr(ipaddr, 192, 168, 1, 2);//??,???MCU????????
    uip_setdraddr(ipaddr);
    uip_ipaddr(ipaddr, 255, 255, 252, 0);//????
    uip_setnetmask(ipaddr);
}

#if 0
void tcpip_process1(void)
{
	int i;
#if XXXDHCPC_EN
	uint16_t status, linkch; 
#endif //XXXDHCPC_EN
	
    uip_len = tapdev_read();
	
    if(uip_len > 0) {
      /// if(BUF->type == htons(UIP_ETHTYPE_IP)) {
      if(UIP_BUF->type == htons(UIP_ETHTYPE_IP)) { /// UIP_BUF
				uip_arp_ipin();
				uip_input();
				/* If the above function invocation resulted in data that
					 should be sent out on the network, the global variable
					 uip_len is set to a value > 0. */
				if(uip_len > 0) {
					uip_arp_out();
					tapdev_send();
				}
      /// } else if(BUF->type == htons(UIP_ETHTYPE_ARP)) {
      } else if(UIP_BUF->type == htons(UIP_ETHTYPE_ARP)) {
				uip_arp_arpin();
				/* If the above function invocation resulted in data that
					 should be sent out on the network, the global variable
					 uip_len is set to a value > 0. */
				if(uip_len > 0) {
					tapdev_send();
				}
      }

    } 
		
#if 0 ///
		else if(timer_expired(&periodic_timer)) {
      timer_reset(&periodic_timer);
      for(i = 0; i < UIP_CONNS; i++) {
				uip_periodic(i);
				/* If the above function invocation resulted in data that
					 should be sent out on the network, the global variable
					 uip_len is set to a value > 0. */
				if(uip_len > 0) {
					uip_arp_out();
					tapdev_send();
				}
      }

#if UIP_UDP
      for(i = 0; i < UIP_UDP_CONNS; i++) {
				uip_udp_periodic(i);
				/* If the above function invocation resulted in data that
					 should be sent out on the network, the global variable
					 uip_len is set to a value > 0. */
				if(uip_len > 0) {
					uip_arp_out();
					tapdev_send();
				}
      }
#endif /* UIP_UDP */
      
      /* Call the ARP timer function every 10 seconds. */
      if(timer_expired(&arp_timer)) {
				timer_reset(&arp_timer);
				uip_arp_timer();
      }
#if XXXDHCPC_EN
			if(timer_expired(&dhcp_timer)) 
			{
				/* for now turn off the led when we start the dhcp process */
				status = DM9051_Read_Reg(DM9051_NSR);
				linkch = DM9051_Read_Reg(DM9051_ISR);
				DM9051_Write_Reg(DM9051_ISR, 0x20);
				
				if((status & 0x40) && (linkch & 0x20)){
					printf("DHCPC Renew...!!!\r\n");
					dhcpc_renew();
				}
				timer_reset(&dhcp_timer);
			}
#endif //XXXDHCPC_EN
    }
		
		#endif ///
		
		return ;
}
#endif


//void tcpip_process3(void){
//	   //????????
//        uip_len = tapdev_read();
//        if(uip_len == 0){
//            //??????
//            return;
//        }
//        //???ARP??
//        if(UIP_BUF->type == htons(UIP_ETHTYPE_ARP)) {
//            //??ARP????
//            uip_arp_arpin();
//            if(uip_len>0){
//                //??ARP??????
//                tapdev_send();
//            }
//        }
//        //???IPV4
//        if(UIP_BUF->type == htons(UIP_ETHTYPE_IP)) {
//       //ping???,??????
//            uip_arp_ipin();
//            //UIP????
//            uip_input();
//            if(uip_len > 0){
//                //????,?????ping????,??????,???????checksum bad ?????
//                uip_arp_out();
//                tapdev_send();
//            }
//        }
//}
//


void IntoStandBy(void)
{
//PA0

//

/* Enable Power Clock*/
//__HAL_RCC_PWR_CLK_ENABLE();
__PWR_CLK_ENABLE();


/* Allow access to Backup */
HAL_PWR_EnableBkUpAccess();

/* Reset RTC Domain */
__HAL_RCC_BACKUPRESET_FORCE();
__HAL_RCC_BACKUPRESET_RELEASE();

/* Disable all used wakeup sources: Pin1(PA.0) */
HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);

/* Clear all related wakeup flags */
__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

/* Re-enable all used wakeup sources: Pin1(PA.0) */
HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

/* Request to enter STANDBY mode  */
HAL_PWR_EnterSTANDBYMode();


}

void StandByKeyInit(void){
	GPIO_InitTypeDef  GPIO_InitStruct;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Pin = GPIO_PIN_1 ;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void MAIN_TickTask(void)
{
	//
	g_ucSysRingTaskPtr = g_ucSysRingPtr;
	// Main loop
	while (1)
	{
		tcpip_process();

		if(LowPowerKey == 0)
			{
			if(LowPowerCnt == 0 && RUNNING_PROGRAM == 1){
				StandByKeyInit();
				IntoStandBy();
				}
			}
		else{
			RUNNING_PROGRAM = 1;
			LowPowerCnt = 1000;
			}
		// Do action every 1ms ( or 1ms ~ 10ms)
		if (g_ulSysTick1MSFlag != g_ulSysTick1MS)
		{	
			g_ulSysTick1MSFlag = g_ulSysTick1MS; // Set flag
  		TSK_ADCProcess(); // ADC process
	  }
		
		// Check USB RX
		USB_ProcessRX();
		
		// Check tick task
		if (g_ucSysRingPtr == g_ucSysRingTaskPtr)     // No task
			continue;

		//**************************************************************************
		// 100 ms * X
		//**************************************************************************
		switch (g_ausSysTickTaskRing[g_ucSysRingTaskPtr])
		{
		case SYSTEM_000MS:
			g_ucSysRingTaskPtr++;
			//g_ulSysTickTaskFlag |= SYS_TICK_TASK_1;  // Calculate system timer
		  g_ulSysTickTaskFlag |= SYS_TICK_TASK_2;  // LED
			g_ulSysTickTaskFlag |= SYS_TICK_TASK_6;  // Power process
			g_ulSysTickTaskFlag |= SYS_TICK_TASK_11; // CAN2USB
			break;

		case SYSTEM_100MS:
			ARP_TEST();
			g_ucSysRingTaskPtr++;
			g_ulSysTickTaskFlag |= SYS_TICK_TASK_5;  // Battery algorithm
		  g_ulSysTickTaskFlag |= SYS_TICK_TASK_3;  // Rock shift ht66
			//g_ulSysTickTaskFlag |= SYS_TICK_TASK_11; // CAN2USB		
			break;

		case SYSTEM_200MS:
			g_ucSysRingTaskPtr++;
			//g_ulSysTickTaskFlag |= SYS_TICK_TASK_1;  // Calculate system timer
			//g_ulSysTickTaskFlag |= SYS_TICK_TASK_2;  // LED
			g_ulSysTickTaskFlag |= SYS_TICK_TASK_8;  // Check battery discharge function
			g_ulSysTickTaskFlag |= SYS_TICK_TASK_11; // CAN2USB		
			break;

		case SYSTEM_300MS:
			g_ucSysRingTaskPtr++;
			g_ulSysTickTaskFlag |= SYS_TICK_TASK_4;  // FAN PWM verify
			g_ulSysTickTaskFlag |= SYS_TICK_TASK_9;  // Bq24745 action
			//g_ulSysTickTaskFlag |= SYS_TICK_TASK_11; // CAN2USB		
			break;

		case SYSTEM_400MS:
			g_ucSysRingTaskPtr++;
			//g_ulSysTickTaskFlag |= SYS_TICK_TASK_1;  // Calculate system timer
			//g_ulSysTickTaskFlag |= SYS_TICK_TASK_10;
			g_ulSysTickTaskFlag |= SYS_TICK_TASK_11; // CAN2USB		
		
			break;

		case SYSTEM_500MS:
			g_ucSysRingTaskPtr++;
			g_ulSysTickTaskFlag |= SYS_TICK_TASK_2;  // LED
			g_ulSysTickTaskFlag |= SYS_TICK_TASK_6;  // Power process
			g_ulSysTickTaskFlag |= SYS_TICK_TASK_32; // (Root) Root function
		  //g_ulSysTickTaskFlag |= SYS_TICK_TASK_11; // CAN2USB
		  break;

		case SYSTEM_600MS:
			g_ucSysRingTaskPtr++;
			//g_ulSysTickTaskFlag |= SYS_TICK_TASK_1;  // Calculate system timer
			g_ulSysTickTaskFlag |= SYS_TICK_TASK_5;  // Battery algorithm
			//g_ulSysTickTaskFlag |= SYS_TICK_TASK_2;  // LED
			g_ulSysTickTaskFlag |= SYS_TICK_TASK_11; // CAN2USB		
			break;

		case SYSTEM_700MS:
			g_ucSysRingTaskPtr++;
			//g_ulSysTickTaskFlag |= SYS_TICK_TASK_8;  // Check battery discharge function
			//g_ulSysTickTaskFlag |= SYS_TICK_TASK_11; // CAN2USB		
		  break;

		case SYSTEM_800MS:
			g_ucSysRingTaskPtr++;
			//g_ulSysTickTaskFlag |= SYS_TICK_TASK_1;  // Calculate system timer
			//g_ulSysTickTaskFlag |= SYS_TICK_TASK_2;  // LED
			g_ulSysTickTaskFlag |= SYS_TICK_TASK_9;  // Bq24745 action
			g_ulSysTickTaskFlag |= SYS_TICK_TASK_11; // CAN2USB		
			break;

		case SYSTEM_900MS:
			g_ucSysRingTaskPtr++;
			g_ulSysTickTaskFlag |= SYS_TICK_TASK_1;  // Calculate system timer
			//g_ulSysTickTaskFlag |= SYS_TICK_TASK_11; // CAN2USB		
		  HWF_DeleayInit(); // deleay init
			break;

		default:
			break;
		}
		if (g_ucSysRingTaskPtr >= TICK_TASK_QUEUE_QUANTITY)
			g_ucSysRingTaskPtr = 0;

		// Check tick task
		if (g_ucSysRingPtr == g_ucSysRingTaskPtr)
			continue;                                  // No task

													   //**************************************************************************
													   // 10 ms * X
													   //**************************************************************************
		switch (g_ausSysTickTaskRing[g_ucSysRingTaskPtr])
		{
		case SYSTEM_EVERY_100MS_OFFSET_00MS:
			g_ucSysRingTaskPtr++;
			//TSK_HT66StateMachine();
			TSK_FlashLed(g_ucLedSysStatus);          // 300 & 800 ms
			TSK_CalculateSystemTime();               // 900 ms
			//TSK_FSP650ReplayAll();
			USB_ProcessTX();
			break;

		case SYSTEM_EVERY_100MS_OFFSET_10MS:
			g_ucSysRingTaskPtr++;			
			//TSK_FanFunction();                       // 310 ms
			//TSK_FSP650ReplayAll();
			USB_ProcessTX();
			break;

		case SYSTEM_EVERY_100MS_OFFSET_20MS:
			g_ucSysRingTaskPtr++;
			//TSK_MCP2515Action();											// *20 ms		
			//TSK_FSP650ReplayAll();
			USB_ProcessTX();
			TSK_Bq20z45Action();  //////// test
			break;

		case SYSTEM_EVERY_100MS_OFFSET_30MS:
			g_ucSysRingTaskPtr++;
			TSK_VerifyBattery();                     // *30 ms			
		 // TSK_HT66StateMachine();	
		  //TSK_FSP650ReplayAll();
			USB_ProcessTX();
			break;

		case SYSTEM_EVERY_100MS_OFFSET_40MS:
			g_ucSysRingTaskPtr++;
			//TSK_FSP650ReplayAll();
			USB_ProcessTX();
			break;

		case SYSTEM_EVERY_100MS_OFFSET_50MS:
			g_ucSysRingTaskPtr++;
			//TSK_MCP2515Action();
		  //TSK_HT66StateMachine();
  		//TSK_FSP650ReplayAll();
			USB_ProcessTX();
			break;

		case SYSTEM_EVERY_100MS_OFFSET_60MS:
			g_ucSysRingTaskPtr++;
			//TSK_PowerProcess();                      // 060 & 560 ms
			TSK_BatteryAlgorithm();                  // 160 & 660 ms 
			//TSK_CheckBatteryDischargeFunction();     // 260 & 760 ms  
			//TSK_FSP650ReplayAll();
			USB_ProcessTX();
			break;

		case SYSTEM_EVERY_100MS_OFFSET_70MS:
			g_ucSysRingTaskPtr++;
			//I2C3_To_SIR_Write_Block();
			//TSK_FSP650ReplayAll();
			USB_ProcessTX();
			break;

		case SYSTEM_EVERY_100MS_OFFSET_80MS:
			g_ucSysRingTaskPtr++;
			//TSK_ReadHt66Parameter();                 // 080 ms
			TSK_BQ25756Action();                     // 380 & 880 ms 
			//TSK_Bq24745Action();                     // 380 & 880 ms 
		  //TSK_HT66StateMachine();
			//TSK_RootFunction();                      // 580 ms
			//TSK_FSP650ReplayAll();
			USB_ProcessTX();
			break;

		case SYSTEM_EVERY_100MS_OFFSET_90MS:
			g_ucSysRingTaskPtr++;
		  //TSK_BatteryHT66StateMachine();	
	  	//TSK_FSP650ReplayAll();
		//set_led_mode(Standby_LED); ///////
		if(CHARGER_MODE != 4){
			//get_sys_status();   //////// test
			
		}
			EN_charging(1);
			if(resetLED == 1)
        update_mode();
        control_LED_all();
			USB_ProcessTX();
			break;

		default:
			break;
		}
		if (g_ucSysRingTaskPtr >= TICK_TASK_QUEUE_QUANTITY)
			g_ucSysRingTaskPtr = 0;
	}
}
void InitPA0(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

#if 0
GPIO_InitTypeDef GPIO_InitStructure; 




UART_HandleTypeDef huart2;

void SystemClock_Config(void);  // ??????
static void MX_GPIO_Init(void); // GPIO ???
static void MX_USART2_UART_Init(void); // USART2 ???

extern void Error_Handler(void);

static void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2; // ?? USART2
    huart2.Init.BaudRate = 9600; // ?????
    huart2.Init.WordLength = UART_WORDLENGTH_8B; // ????????
    huart2.Init.StopBits = UART_STOPBITS_1; // ??????
    huart2.Init.Parity = UART_PARITY_NONE; // ?????
    huart2.Init.Mode = UART_MODE_TX_RX; // ?????????
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE; // ???????
    huart2.Init.OverSampling = UART_OVERSAMPLING_16; // 16????
	
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler(); // ???????
    }
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __GPIOA_CLK_ENABLE(); // ?? GPIOA ??

    // USART2 TX (PA2)
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;      // ??????
    GPIO_InitStruct.Pull = GPIO_NOPULL;          // ???/??
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST; // ??
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2; // GPIO AF7: USART2
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // USART2 RX (PA3)
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void SystemClock_Config(void) 
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
	__PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
 // RCC_OscInitStruct.PLL.PLLR = 2;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}
#endif
//
int main(void)
{
	
#if 0
    int x = -123;
    float y = 45.678;

    // ????????
    char a[20]; // ?????? x ???
    char b[20]; // ??????? y ???

    // ???? x ?????? a
    sprintf(a, "%d", x);

    // ????? y ?????? b
    sprintf(b, "%.3f", y); // ????????
	
	
        uint8_t rxData=0x55;
	
	HAL_Delay(5);
    HAL_Init();                 // ??? HAL ?
    SystemClock_Config(); // ??????
    MX_GPIO_Init(); // ??? GPIO
    MX_USART2_UART_Init(); // ??? USART2
__USART2_CLK_ENABLE();
	
	if (!(RCC->APB1ENR & RCC_APB1ENR_USART2EN)) {
    ///printf("USART2 clock not enabled.\n");
		rxData=0x01;
}
	
	USART2->CR1 |= USART_CR1_UE;
if (!(USART2->CR1 & USART_CR1_UE)) {
    ///printf("USART2 not enabled.\n");
	rxData=0x02;
}

	USART2->CR1 |= USART_CR1_TE;
	USART2->CR1 |= USART_CR1_RE;
if (!(USART2->CR1 & USART_CR1_TE)) {
    ///printf("USART2 transmitter not enabled.\n");
	rxData=0x03;
}


//GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1; // ??????
//GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFRL2_Pos) | (7 << GPIO_AFRL_AFRL3_Pos); // AF7


    // UART ????
    char msg[] = "Hello, USART2!\r\n";
    //HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg) - 1, 1000);
	
	 while (1) {
        //if (HAL_UART_Receive(&huart2, &rxData, 1, HAL_MAX_DELAY) == HAL_OK) {
            HAL_UART_Transmit(&huart2, &rxData, 1, 0xFFFF); // Echo ??
        //}
		 //while (!(USART2->SR & USART_SR_TXE)); // ????????
    //USART2->DR = rxData; // ????
}
#endif		
/*
		volatile uint16_t VFB_V = 1536; // mV
			VFB_V = 4200; // 12500; // mV
			VFB_V /= 20;
			VFB_V <<= 2;
	*/
	
	HAL_Delay(5);
	HAL_Init();

	// Configure the system clock to 84 Mhz 
	HWF_SystemClock_Config();

	// Initiate USB
#ifdef USB_MODULE 
	USB_Config();
	//HAL_PCD_IRQHandler(&hpcd);
#endif


	// Initate PORT
	HWF_InitPORT();
	ENABLE_LANPOWER_H;
	//ENABLE_LANPOWER_L;
	// Initate ADC & Start convert
	HWF_InitADCMulCH();
	HWF_ADCStartConvert();

	// Initate capture PWM
	//HWF_InitFanPWM();
	//HWF_InitBeepPWM();
  HWF_TIM2_Init(50);
	HWF_TIM4_Init(500);	//50ms
	// Initate I2C (84Mhz => 4us)
	HWF_InitI2C1();
	//HWF_ReInitI2C1Pin();
	HWF_InitI2C3();
	BQ25756EInit();
	// Initate CAN_SPI
	//HWF_InitSPI1();
	//CANSPI1_Initialize();
	HWF_InitSPI2();
	
	///DM9051_Init();
	///Uip_init();
	
	
	tcpip_init();
	httpd_init();
	//CANSPI2_Initialize();
	// Initiate variable
	HWF_InitVar();

	// TEST 
	
	
	/* Check and handle if the system was resumed from StandBy mode */
	if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != 0)
	{
	  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
	  
	  /* Turn LED4 On */
	  //BSP_LED_On(LED4);
	}
	InitPA0();
	// Execute task
	MAIN_TickTask();

}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

