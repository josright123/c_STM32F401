
#include "main.h"

/**
* @brief  SYSTICK callback.
* @param  None
* @retval None
*/
void HAL_SYSTICK_Callback(void)
{
	//
	g_ulSysTick1MS++;

	// 
	switch (g_ulSysTick1MS)
	{
	case SYSTEM_100MS:
		g_ausSysTickTaskRing[g_ucSysRingPtr++] = SYSTEM_100MS;
		break;

	case SYSTEM_200MS:
		g_ausSysTickTaskRing[g_ucSysRingPtr++] = SYSTEM_200MS;
		break;

	case SYSTEM_300MS:
		g_ausSysTickTaskRing[g_ucSysRingPtr++] = SYSTEM_300MS;
		break;

	case SYSTEM_400MS:
		g_ausSysTickTaskRing[g_ucSysRingPtr++] = SYSTEM_400MS;
		break;

	case SYSTEM_500MS:
		g_ausSysTickTaskRing[g_ucSysRingPtr++] = SYSTEM_500MS;
		break;

	case SYSTEM_600MS:
		g_ausSysTickTaskRing[g_ucSysRingPtr++] = SYSTEM_600MS;
		break;

	case SYSTEM_700MS:
		g_ausSysTickTaskRing[g_ucSysRingPtr++] = SYSTEM_700MS;
		break;

	case SYSTEM_800MS:
		g_ausSysTickTaskRing[g_ucSysRingPtr++] = SYSTEM_800MS;
		break;

	case SYSTEM_900MS:
		g_ausSysTickTaskRing[g_ucSysRingPtr++] = SYSTEM_900MS;
		break;

	case SYSTEM_1000MS:
		g_ausSysTickTaskRing[g_ucSysRingPtr++] = SYSTEM_000MS;
		g_ulSysTick1MS = 0;
		break;

	default:
		break;
	}
	if (g_ucSysRingPtr >= TICK_TASK_QUEUE_QUANTITY)
		g_ucSysRingPtr = 0;

	// 
	switch (g_ulSysTick1MS % 100)
	{
	case SYSTEM_EVERY_100MS_OFFSET_00MS:
		g_ausSysTickTaskRing[g_ucSysRingPtr++] = SYSTEM_EVERY_100MS_OFFSET_00MS;
		break;

	case SYSTEM_EVERY_100MS_OFFSET_10MS:
		g_ausSysTickTaskRing[g_ucSysRingPtr++] = SYSTEM_EVERY_100MS_OFFSET_10MS;
		break;

	case SYSTEM_EVERY_100MS_OFFSET_20MS:
		g_ausSysTickTaskRing[g_ucSysRingPtr++] = SYSTEM_EVERY_100MS_OFFSET_20MS;
		break;

	case SYSTEM_EVERY_100MS_OFFSET_30MS:
		g_ausSysTickTaskRing[g_ucSysRingPtr++] = SYSTEM_EVERY_100MS_OFFSET_30MS;
		break;

	case SYSTEM_EVERY_100MS_OFFSET_40MS:
		g_ausSysTickTaskRing[g_ucSysRingPtr++] = SYSTEM_EVERY_100MS_OFFSET_40MS;
		break;

	case SYSTEM_EVERY_100MS_OFFSET_50MS:
		g_ausSysTickTaskRing[g_ucSysRingPtr++] = SYSTEM_EVERY_100MS_OFFSET_50MS;
		break;

	case SYSTEM_EVERY_100MS_OFFSET_60MS:
		g_ausSysTickTaskRing[g_ucSysRingPtr++] = SYSTEM_EVERY_100MS_OFFSET_60MS;
		break;

	case SYSTEM_EVERY_100MS_OFFSET_70MS:
		g_ausSysTickTaskRing[g_ucSysRingPtr++] = SYSTEM_EVERY_100MS_OFFSET_70MS;
		break;

	case SYSTEM_EVERY_100MS_OFFSET_80MS:
		g_ausSysTickTaskRing[g_ucSysRingPtr++] = SYSTEM_EVERY_100MS_OFFSET_80MS;
		break;

	case SYSTEM_EVERY_100MS_OFFSET_90MS:
		g_ausSysTickTaskRing[g_ucSysRingPtr++] = SYSTEM_EVERY_100MS_OFFSET_90MS;
		break;

	default:
		break;
	}
	if (g_ucSysRingPtr >= TICK_TASK_QUEUE_QUANTITY)
		g_ucSysRingPtr = 0;
}

//
void HAL_GPIO_EXTI_Callback(__IO uint16_t GPIO_Pin)
{
	//
	switch (GPIO_Pin)
	{
	case GPIO_PIN_2:         // (2)AC_FAIL_N_MCU1
	{
		g_Rock_value[7] |= 4;
		//g_ucBatteryChargeMode = BATTERY_STOP_CHARGE_MODE;
		//g_ucBatteryChargeState = CHARGE_STOP;
		//EN_CHARGER_POWER_L;
		//EN_CHARGER_L;
		//BI_ACT_L;		
		break;
	}
	/*
	case GPIO_PIN_9:         // (1)BATTERY_INSERT_N1
	{
		g_ucBatteryType = NO_BATTERY;
		g_battery_act = 0;
		break;
	}
	case GPIO_PIN_15:        // (3)ACINOK_N_MCU1
		break;
	*/
	}
	//
	if (g_ucRootFSP400ChargeManual == 0)
	{
		//EN_CHARGER_POWER_L;
		//EN_CHARGER_L; 
		//EN_FAST_CHARGER_L;
	}
	g_ucPwStateMachine = PW_STEP_0;
}

/**
* @brief  Input Capture callback in non blocking mode
* @param  htim: TIM IC handle
* @retval None
*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	/*
	*The minimum frequency value to measure is (TIM3 counter clock / CCR MAX)
	*                                          = (84MHz)/ 65535
	*                                          = 1680 Hz
	*/

	__IO uint32_t uwIC2Value;
	uint32_t frequency;

	//
	uwIC2Value = 0;
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		/* Get the Input Capture value */
		uwIC2Value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

		if (uwIC2Value != 0)
		{
			/* Duty cycle computation */
			g_ulFanDutyCycle = ((HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1)) * 100) / uwIC2Value;

			/* uwFrequency computation
			TIM3 counter clock = (RCC_Clocks.HCLK_Frequency) */
			frequency = (HAL_RCC_GetHCLKFreq()) / uwIC2Value;
			frequency = frequency / 128;
			if (frequency < 0x10A) // Avoid noise; if fanspeed > 8000rpm do not display
			g_ulFanFrequency = frequency;
			g_ucFanStopFlag = 0;
		}
		else
		{
			g_ulFanDutyCycle = 0;
			g_ulFanFrequency = 0;
		}
	}
}

//TIM2 interupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  //EX : Charger_power_50us_TIME = 105;
	//means charger 50us * 5 
	/*if(Charger_power_50us_TIME == 255)    // undo
		return;
	else if(Charger_power_50us_TIME == 0) // off charger
	  {
	  	Charger_power_50us_TIME = 255;
	  	//EN_CHARGER_POWER_L;
	  }
	else if(Charger_power_50us_TIME < 101) // count charge time
		Charger_power_50us_TIME--;
	else                             // turn on charger
	{
	  Charger_power_50us_TIME -=101;
		//EN_CHARGER_POWER_H;
	}*/
	if (htim->Instance == htim4.Instance)
	{
		TSK_MCP2515Action();
	}
		
}	

/**
* @brief ADC MSP Initialization
*        This function configures the hardware resources used in this example:
*           - Peripheral's clock enable
*           - Peripheral's GPIO Configuration
* @param huart: UART handle pointer
* @retval None
*/
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
	
	static DMA_HandleTypeDef  hdma_adc;

	__ADC1_CLK_ENABLE();
	__DMA2_CLK_ENABLE();
	if(hdma_adc.Instance != 0)
	  memset(hdma_adc.Instance,0,0x20);
	
	hdma_adc.Instance = DMA2_Stream0;
  hdma_adc.Init.Channel = DMA_CHANNEL_0;
	hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
	hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	hdma_adc.Init.Mode = DMA_CIRCULAR;
	hdma_adc.Init.Priority = DMA_PRIORITY_HIGH;
	hdma_adc.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	hdma_adc.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
	hdma_adc.Init.MemBurst = DMA_MBURST_SINGLE;
	hdma_adc.Init.PeriphBurst = DMA_PBURST_SINGLE;

	HAL_DMA_Init(&hdma_adc);

	__HAL_LINKDMA(hadc, DMA_Handle, hdma_adc);

	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/**
* @brief TIM MSP Initialization
*        This function configures the hardware resources used in this example:
*           - Peripheral's clock enable
*           - Peripheral's GPIO Configuration
* @param htim: TIM handle pointer
* @retval None
*/
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
	//GPIO_InitTypeDef   GPIO_InitStruct;

	/*##-1- Enable peripherals and GPIO Clocks #################################*/
	/* TIMx Peripheral clock enable */
	__TIM3_CLK_ENABLE();

	/*##-2- Configure the NVIC for TIMx #########################################*/
	/* Sets the priority grouping field */
	HAL_NVIC_SetPriority(TIM3_IRQn, 4, 0);

	/* Enable the TIM4 global Interrupt */
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	
	__TIM2_CLK_ENABLE();
	HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
	
	//if(htim->Instance==TIM4)
  //{
    /* Peripheral clock enable */
    //__TIM4_CLK_ENABLE();
    /* TIM4 interrupt Init */
    //HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    //HAL_NVIC_EnableIRQ(TIM4_IRQn);
  //}
}

/**
* @brief TIM MSP Initialization
*        This function configures the hardware resources used in this example:
*           - Peripheral's clock enable
*           - Peripheral's GPIO Configuration
* @param htim: TIM handle pointer
* @retval None
*/
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{	
	__TIM1_CLK_ENABLE();
}

/**
* @brief I2C MSP Initialization
*        This function configures the hardware resources used in this example:
*           - Peripheral's clock enable
*           - Peripheral's GPIO Configuration
*           - DMA configuration for transmission request by peripheral
*           - NVIC configuration for DMA interrupt request enable
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{	
	__I2C1_CLK_ENABLE();//I2Cx_CLK_ENABLE();   					
	__I2C3_CLK_ENABLE();//I2Cx_CLK_ENABLE(); 

}

/**
* @brief UART MSP Initialization
*        This function configures the hardware resources used in this example:
*           - Peripheral's clock enable
*           - Peripheral's GPIO Configuration
*           - NVIC configuration for UART interrupt request enable
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	//GPIO_InitTypeDef  GPIO_InitStruct;

	/*##-1- Enable peripherals and GPIO Clocks #################################*/
	/* Enable GPIO TX/RX clock */
	//__GPIOA_CLK_ENABLE();
	/* Enable USART1 clock */
	__USART1_CLK_ENABLE();

	///*##-2- Configure peripheral GPIO ##########################################*/  
	///* UART TX GPIO pin configuration  */
	//GPIO_InitStruct.Pin       = GPIO_PIN_9;
	//GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	//GPIO_InitStruct.Pull      = GPIO_NOPULL;
	//GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
	//GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	//
	//HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	///* UART RX GPIO pin configuration  */
	//GPIO_InitStruct.Pin = GPIO_PIN_10;
	//GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	//  
	//HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

	///*##-3- Configure the NVIC for UART ########################################*/
	///* NVIC for USART1 */
	//HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
	//HAL_NVIC_EnableIRQ(USARTx_IRQn);
}

/**
* @brief  System Clock Configuration
*         The system Clock is configured as follow :
*            System Clock source            = PLL (HSE)
*            SYSCLK(Hz)                     = 84000000
*            HCLK(Hz)                       = 84000000
*            AHB Prescaler                  = 1
*            APB1 Prescaler                 = 2
*            APB2 Prescaler                 = 1
*            HSE Frequency(Hz)              = 8000000
*            PLL_M                          = 8
*            PLL_N                          = 336
*            PLL_P                          = 4
*            PLL_Q                          = 7
*            VDD(V)                         = 3.3
*            Main regulator output voltage  = Scale2 mode
*            Flash Latency(WS)              = 2
* @param  None
* @retval None
*/
void HWF_SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Enable Power Control clock */
	__PWR_CLK_ENABLE();

	/* The voltage scaling allows optimizing the power consumption when the device is
	clocked below the maximum system frequency, to update the voltage scaling value
	regarding system frequency refer to product datasheet.  */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

//
void HWF_InitVar(void)
{
	// USBD_HID
	g_USBD_HID_OutFlag = 0;

	//
	g_ausADCValueOffset[0][0] = 0;
	g_ausADCValueOffset[0][1] = 0;
	g_ausADCValueOffset[1][0] = 0;
	g_ausADCValueOffset[1][1] = 0;
	g_ausADCValueOffset[2][0] = 0;
	g_ausADCValueOffset[2][1] = 0;
	g_ausADCValueOffset[3][0] = 0;
	g_ausADCValueOffset[3][1] = 0;
	g_ausADCValueOffset[4][0] = 0;
	g_ausADCValueOffset[4][1] = 0;
	g_ausADCValueOffset[5][0] = 0;
	g_ausADCValueOffset[5][1] = 0;
	g_ausADCValueOffset[6][0] = 0;
	g_ausADCValueOffset[6][1] = 0;

	memset(STR_TX_Buffer, 0xFF, 12);
}

//
void HWF_InitPORT(void)
{
	//
	GPIO_InitTypeDef  GPIO_InitStruct;

	// PORT A
	__GPIOA_CLK_ENABLE();
	// FOR GPIO OUT
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Pin = GPIO_PIN_6;	
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	// FOR GPIO IN
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	// FOR GPIO IRQ
	/*
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	*/
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	// FOR ADC
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	// FOR PWM IN
	/*GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);*/
	// FOR I2C
	GPIO_InitStruct.Pin = GPIO_PIN_8;//I2Cx_SCL_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD; // Open Drain Mode
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;//I2Cx_SCL_AF;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	//  // FOR UART TX 
	//  GPIO_InitStruct.Pin       = GPIO_PIN_9; // TX
	//  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP; // Push Pull Mode
	//  GPIO_InitStruct.Pull      = GPIO_NOPULL;
	//  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
	//  GPIO_InitStruct.Alternate = GPIO_AF7_USART1; 
	//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  
	//  //GPIO_InitStruct.Pin       = GPIO_PIN_10; // RX
	//  //GPIO_InitStruct.Alternate = GPIO_AF7_USART1; 
	//  //HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	// FOR PWM OUTPUT
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// PORT B
	__GPIOB_CLK_ENABLE();
	// FOR GPIO OUT
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Pin =  GPIO_PIN_2 |GPIO_PIN_3 | GPIO_PIN_8 ;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	// FOR MCP2515_CS
	//GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	//GPIO_InitStruct.Pull = GPIO_NOPULL;
	//GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
	//GPIO_InitStruct.Pin = GPIO_PIN_12;
	//HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	// FOR GPIO IN
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_10;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	// FOR IRQ
	/*
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	*/
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	// FOR I2C 
	GPIO_InitStruct.Pin = GPIO_PIN_4;//I2Cx_SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = GPIO_AF9_I2C3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_6; // I2C1 - SCL 
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_7; // I2C1 - SDA 
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);	
	// FOR ADC
	//GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	//GPIO_InitStruct.Pull = GPIO_NOPULL;
	//GPIO_InitStruct.Pin = GPIO_PIN_0;
	//HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	// PB0 disable
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	// PORT C
	__GPIOC_CLK_ENABLE();
	/*
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	*/
	
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// PORT D
	__GPIOD_CLK_ENABLE();
	// FOR GPIO OUT
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	// Verify System Mode
	g_ucSysMode = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10); // For FSP500, 1:fsp550-70mua 0:fsp450-60mua

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	//
	RLED_INDICATOR_L;
	I2C_CLK_CHARGER_L;
	//EN_CHARGER_POWER_L;
	//EN_CHARGER_L;
	//EN_FAST_CHARGER_H;
	//EN_288V_H;
	//ENATXPOWER_N_AUX_L;
	//MCU1_OCP_L;
	//INTERLOCK_OUT_MCU_L;
	//BATTERY_LOW_PIN_L;
	//MCP2515_CS_HIGH();
}

//
uint32_t HWF_InitADCMulCH(void)
{
	//
	ADC_ChannelConfTypeDef sConfig;
  if(g_AdcHandle.Instance != 0)
	  memset((g_AdcHandle.Instance),0,0x20);

	g_AdcHandle.Instance = ADC1;

	// 
	g_AdcHandle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	g_AdcHandle.Init.Resolution = ADC_RESOLUTION12b;
	g_AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	g_AdcHandle.Init.ScanConvMode = ENABLE;           // ENABLE (multi channels) DISABLE (one channel) 
	g_AdcHandle.Init.EOCSelection = DISABLE;
	g_AdcHandle.Init.ContinuousConvMode = DISABLE;    // ENABLE (Continuous mode) DISABLE (Single mode)
	g_AdcHandle.Init.DMAContinuousRequests = ENABLE;  // ENABLE (Continuous) DISABLE (Single) 
	g_AdcHandle.Init.NbrOfConversion = 7;
	g_AdcHandle.Init.DiscontinuousConvMode = DISABLE;
	g_AdcHandle.Init.NbrOfDiscConversion = 0;
	g_AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	g_AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;        //Rock useless?

	if (HAL_ADC_Init(&g_AdcHandle) != HAL_OK)
		return ZX_ERR_01;
	
/*
	//
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	sConfig.Offset = 0;
	HAL_ADC_ConfigChannel(&g_AdcHandle, &sConfig);

	//
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 2;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	sConfig.Offset = 0;
	HAL_ADC_ConfigChannel(&g_AdcHandle, &sConfig);

	//
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = 3;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	sConfig.Offset = 0;
	HAL_ADC_ConfigChannel(&g_AdcHandle, &sConfig);
	*/

	//
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = 4;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	sConfig.Offset = 0;
	HAL_ADC_ConfigChannel(&g_AdcHandle, &sConfig);

	//
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 5;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	sConfig.Offset = 0;
	HAL_ADC_ConfigChannel(&g_AdcHandle, &sConfig);

	//
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = 6;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	sConfig.Offset = 0;
	HAL_ADC_ConfigChannel(&g_AdcHandle, &sConfig);
	
	//
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = 7;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	sConfig.Offset = 0;
	HAL_ADC_ConfigChannel(&g_AdcHandle, &sConfig);

/*	
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = 7;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	sConfig.Offset = 0;
	HAL_ADC_ConfigChannel(&g_AdcHandle, &sConfig);
*/
	return ZX_OK;

}

//
void HWF_InitFanPWM(void)
{
	// Timer Input Capture Configuration Structure declaration
	TIM_IC_InitTypeDef       sConfig;

	// Slave configuration structure 
	TIM_SlaveConfigTypeDef   sSlaveConfig;

	/* Set TIMx instance */
	g_TimHandle.Instance = TIM3;

	/* Initialize TIMx peripheral as follows:
	+ Period = 0xFFFF
	+ Prescaler = 0
	+ ClockDivision = 0
	+ Counter direction = Up
	*/
	g_TimHandle.Init.Period = 0xFFFF;
	g_TimHandle.Init.Prescaler = 128;
	g_TimHandle.Init.ClockDivision = 0;
	g_TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	if (HAL_TIM_IC_Init(&g_TimHandle) != HAL_OK)
	{
		// Initialization Error
	}

	/*##-2- Configure the Input Capture channels ###############################*/
	/* Common configuration */
	sConfig.ICPrescaler = TIM_ICPSC_DIV1;
	sConfig.ICFilter = 0;

	/* Configure the Input Capture of channel 4 */
	sConfig.ICPolarity = TIM_ICPOLARITY_FALLING;
	sConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
	if (HAL_TIM_IC_ConfigChannel(&g_TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK) //
	{
		// Initialization Error
	}

	/* Configure the Input Capture of channel 3 */
	sConfig.ICPolarity = TIM_ICPOLARITY_RISING;
	sConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
	if (HAL_TIM_IC_ConfigChannel(&g_TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK) //
	{
		// Initialization Error
	}
	/*##-3- Configure the slave mode ###########################################*/
	/* Select the slave Mode: Reset Mode */
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
	sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
	if (HAL_TIM_SlaveConfigSynchronization(&g_TimHandle, &sSlaveConfig) != HAL_OK)
	{
		// Initialization Error
	}

	//
	if (HAL_TIM_IC_Start_IT(&g_TimHandle, TIM_CHANNEL_2) != HAL_OK) //
	{
		
		//
	}

	//
	if (HAL_TIM_IC_Start_IT(&g_TimHandle, TIM_CHANNEL_1) != HAL_OK) //
	{
		//
	}
}

//
void HWF_InitBeepPWM(void)  // now for fan pwm control
{
	/* Counter Prescaler value */
	uint32_t uwPrescalerValue = 0;
	/* Timer Output Compare Configuration Structure declaration */
	TIM_OC_InitTypeDef sConfig;

	/* Compute the prescaler value to have TIM1 counter clock equal to 18 MHz */
	//Rock 20170627 change 18000000 to 84000000(sysclk = 84000000)
	uwPrescalerValue = ((SystemCoreClock) / 84000000) - 1;

	/*##-1- Configure the TIM peripheral #######################################*/
	/* Initialize TIM1 peripheral as follow:
	+ Prescaler = (SystemCoreClock/2)/18000000
	+ Period = 18000  (to have an output frequency equal to 1 KHz)
	+ ClockDivision = 0
	+ Counter direction = Up
	*/
	g_TimHandle_Beep2.Instance = TIM1;
	g_TimHandle_Beep.Instance = TIM1;
	
	g_TimHandle_Beep.Init.Prescaler = uwPrescalerValue;
	//Rock 20170626 Period = 18000 зя  3500  counter
	g_TimHandle_Beep.Init.Period = (3500 - 1);
	g_TimHandle_Beep.Init.ClockDivision = 0;
	g_TimHandle_Beep.Init.CounterMode = TIM_COUNTERMODE_UP;


	if (HAL_TIM_PWM_Init(&g_TimHandle_Beep) != HAL_OK)
	{
		// Initialization Error
	}

	/*##-2- Configure the PWM channels #########################################*/
	/* Common configuration for all channels */
	sConfig.OCMode = TIM_OCMODE_PWM1;
	sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig.OCFastMode = TIM_OCFAST_DISABLE;

	/* Set the pulse value for channel 2 */
	//Rock 20170627 change duty  (original 9000) duty% *100* 35 = pulse  50 * 35 = 1750
	sConfig.Pulse = 2000;
	if (HAL_TIM_PWM_ConfigChannel(&g_TimHandle_Beep, &sConfig, TIM_CHANNEL_2) != HAL_OK)
	{
		// Configuration Error
	}
	//Rock 20170626 add pwm start
	HAL_TIM_PWM_Start(&g_TimHandle_Beep, TIM_CHANNEL_2);
	/*##-3- Start PWM signals generation #######################################*/
	/* Start channel 2 */
	//if(HAL_TIM_PWM_Start(&g_TimHandle_Beep, TIM_CHANNEL_2) != HAL_OK)
	//{
	//  // Starting Error 
	//}
}

void HWF_ReInitI2C1Pin(void)
{

	GPIO_InitTypeDef  GPIO_InitStruct;
	
	HAL_I2C_DeInit(&g_I2c1Handle);
  //GPIO set
	GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7; 
  GPIO_InitStruct.Pull = GPIO_PULLUP;	
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;      
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  //pull up DA CL
	
	HAL_GPIO_WritePin(GPIOB, 6, GPIO_PIN_SET);       
	HAL_GPIO_WritePin(GPIOB, 7, GPIO_PIN_SET);       

  g_I2c1Handle.Instance->CR1 |= I2C_CR1_SWRST;      
  g_I2c1Handle.Instance->CR1 -= I2C_CR1_SWRST;              
	
	// DeInit Port
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

	// Init Port  
	GPIO_InitStruct.Pin = GPIO_PIN_6; // I2C1 - SCL 
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_7; // I2C1 - SDA 
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*##-1- Configure the I2C peripheral ######################################*/
	g_I2c1Handle.Instance = I2C1;

	g_I2c1Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	//g_I2c1Handle.Init.ClockSpeed      = 10000; // Frequency (10KHz : 1Byte => 1ms)
	//g_I2c1Handle.Init.ClockSpeed      = 20000; // Frequency (20KHz : 1Byte => 0.5ms)
	//g_I2c1Handle.Init.ClockSpeed      = 50000; // Frequency (50KHz : 1Byte => 200us)
	g_I2c1Handle.Init.ClockSpeed = 100000; // Frequency (10KHz : 1Byte => us)
	g_I2c1Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
	g_I2c1Handle.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
	g_I2c1Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
	g_I2c1Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
	g_I2c1Handle.Init.OwnAddress1 = I2C_ADDRESS;
	g_I2c1Handle.Init.OwnAddress2 = 0xFE;

	if (HAL_I2C_Init(&g_I2c1Handle) != HAL_OK)
	{
		// Initialization Error
	}

}

void HWF_ReInitI2C3Pin(void)
{
#include <stm32f4xx.h>
	GPIO_InitTypeDef  GPIO_InitStruct;
	
	HAL_I2C_DeInit(&g_I2c3Handle);
	
	//GPIO set
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;      
  HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);
	//
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Pin = GPIO_PIN_8; 
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);	
  
	//pull up DA CL
	HAL_GPIO_WritePin(GPIOB, 4, GPIO_PIN_SET);       
	HAL_GPIO_WritePin(GPIOA, 8, GPIO_PIN_SET);       

  g_I2c3Handle.Instance->CR1 |= I2C_CR1_SWRST;      
  g_I2c3Handle.Instance->CR1 -= I2C_CR1_SWRST;              
	
	// DeInit Port
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

	// Init Port
	GPIO_InitStruct.Pin = GPIO_PIN_8;//I2Cx_SCL_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;//I2Cx_SCL_AF;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_4;//I2Cx_SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = GPIO_AF9_I2C3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	///////////////////////////////////////////////////////

	/*##-1- Configure the I2C peripheral ######################################*/
	g_I2c3Handle.Instance = I2C3;

	g_I2c3Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	//g_I2c1Handle.Init.ClockSpeed      = 10000; // Frequency (10KHz : 1Byte => 1ms)
	//g_I2c1Handle.Init.ClockSpeed      = 20000; // Frequency (20KHz : 1Byte => 0.5ms)
	//g_I2c1Handle.Init.ClockSpeed      = 50000; // Frequency (50KHz : 1Byte => 200us)
	g_I2c3Handle.Init.ClockSpeed = 400000; // Frequency (10KHz : 1Byte => us)
	g_I2c3Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
	g_I2c3Handle.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
	g_I2c3Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
	g_I2c3Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
	g_I2c3Handle.Init.OwnAddress1 = I2C_ADDRESS;
	g_I2c3Handle.Init.OwnAddress2 = 0xFE;

	if (HAL_I2C_Init(&g_I2c3Handle) != HAL_OK)
	{

	}

}

//
void HWF_InitI2C1(void)  
{
	GPIO_InitTypeDef  GPIO_InitStruct;
//
	
	// DeInit Port
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

	// Init Port  
	GPIO_InitStruct.Pin = GPIO_PIN_6; // I2C1 - SCL 
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_7; // I2C1 - SDA 
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*##-1- Configure the I2C peripheral ######################################*/
	g_I2c1Handle.Instance = I2C1;

	g_I2c1Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	//g_I2c1Handle.Init.ClockSpeed      = 10000; // Frequency (10KHz : 1Byte => 1ms)
	//g_I2c1Handle.Init.ClockSpeed      = 20000; // Frequency (20KHz : 1Byte => 0.5ms)
	//g_I2c1Handle.Init.ClockSpeed      = 50000; // Frequency (50KHz : 1Byte => 200us)
	g_I2c1Handle.Init.ClockSpeed = 100000; // Frequency (10KHz : 1Byte => us)
	g_I2c1Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
	g_I2c1Handle.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
	g_I2c1Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
	g_I2c1Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
	g_I2c1Handle.Init.OwnAddress1 = I2C_ADDRESS;
	g_I2c1Handle.Init.OwnAddress2 = 0xFE;

	if (HAL_I2C_Init(&g_I2c1Handle) != HAL_OK)
	{
		// Initialization Error
	}

}

//
void HWF_InitI2C3(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	// DeInit Port
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

	// Init Port
	GPIO_InitStruct.Pin = GPIO_PIN_8;//I2Cx_SCL_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;//I2Cx_SCL_AF;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_4;//I2Cx_SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = GPIO_AF9_I2C3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*##-1- Configure the I2C peripheral ######################################*/
	g_I2c3Handle.Instance = I2C3;

	g_I2c3Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	g_I2c3Handle.Init.ClockSpeed = 400000; // Frequency (100KHz : 1Byte => 100us)
	//g_I2c3Handle.Init.ClockSpeed = 100000; 
										   //g_I2c3Handle.Init.ClockSpeed      = 400000; // Frequency (400KHz : 1Byte => 50us)
	g_I2c3Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
	g_I2c3Handle.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
	g_I2c3Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
	g_I2c3Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
	///g_I2c3Handle.Init.OwnAddress1 = BQ25756E_ADDRESS;
	g_I2c3Handle.Init.OwnAddress2 = 0xFE;
	g_I2c3Handle.Init.OwnAddress1 = 0;
	g_I2c3Handle.Init.OwnAddress2 = 0;

	if (HAL_I2C_Init(&g_I2c3Handle) != HAL_OK)
	{
		// Initialization Error
	}

}

//
void HWF_InitUart(void)
{
	/*##-1- Configure the UART peripheral ######################################*/
	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	/* UART1 configured as follow:
	- Word Length = 8 Bits
	- Stop Bit = One Stop bit
	- Parity = None
	- BaudRate = 9600 baud
	- Hardware flow control disabled (RTS and CTS signals) */
	g_UartHandle.Instance = USART1;
	//g_UartHandle.Init.BaudRate   = 9600;
	g_UartHandle.Init.BaudRate = 1200;
	g_UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	g_UartHandle.Init.StopBits = UART_STOPBITS_1;
	g_UartHandle.Init.Parity = UART_PARITY_NONE;
	g_UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	g_UartHandle.Init.Mode = UART_MODE_TX_RX;

	if (HAL_UART_Init(&g_UartHandle) != HAL_OK)
	{
		// Initialization Error
	}
}

//
void HWF_ADCStartConvert(void)
{
	HAL_ADC_Start_DMA(&g_AdcHandle, (uint32_t*)g_ausADCReg, 7);
}

void HWF_DeleayInit()
{
	if(DeleayFlag > 1)
	{ 

    	//80c                                                // OTP        00     00   0000  
		/*if(g_ausADCReg[6] < SENSOR_TEMPERATURE_80)   //      OVER  80C    50c(use like a counter) 
      g_PowerOTP += 64;      
	  else
			g_PowerOTP &= N_PSU_OTP85C;
		
		// for test
		if(g_TimHandle_Beep.Instance != TIM1)
	  {
	    g_TimHandle_Beep.Instance = TIM1;
	  }*/
		return;
	}
	
  if(0)
	{
  ;/*GPIO_InitTypeDef  GPIO_InitStruct;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Pin = GPIO_PIN_14;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);*/
	}
	DeleayFlag++;
}

// basic tim2 init 
void HWF_TIM2_Init(uint32_t period)	
{
	htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = period-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim2);
		/* Enable the TIM4 global Interrupt */
	__TIM2_CLK_ENABLE();
	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);	
	//HAL_TIM_Base_Start_IT(&htim2);
	g_ulPreBatteryChargeTime = 1000;
}
//TIM2 INT to produce pulse
void HWF_TIM2_Stop()// stop TIM IRQ AND TIMER
{
	__TIM2_CLK_DISABLE();
	HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
  HAL_NVIC_DisableIRQ(TIM2_IRQn);
}

// basic tim4 init 
void HWF_TIM4_Init(uint32_t period)	
{
	/* Enable the TIM4 global Interrupt */
	/* Peripheral clock enable */
	__TIM4_CLK_ENABLE();
	/* TIM4 interrupt Init */
	HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	//TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  //TIM_MasterConfigTypeDef sMasterConfig = {0};
	htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8400-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = period-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim4);
	//sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	//HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);
	//sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  //sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	//HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

	HAL_TIM_Base_Start_IT(&htim4);
}
//TIM2 INT to produce pulse
void HWF_TIM4_Stop()// stop TIM IRQ AND TIMER
{
	__TIM4_CLK_DISABLE();
	HAL_NVIC_SetPriority(TIM4_IRQn, 3, 0);
  HAL_NVIC_DisableIRQ(TIM4_IRQn);
}

void HWF_RN_CHARGE_POWER_50us(uint8_t times)// stop TIM IRQ AND TIMER
{
if(Charger_power_50us_TIME == 255)
	Charger_power_50us_TIME = times;

}

void Error_Handler()
{
  while(1) 
  {
  }
}

/* SPI2 init function */
void HWF_InitSPI2(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
  /* Peripheral clock enable */
  __SPI2_CLK_ENABLE();

  __GPIOB_CLK_ENABLE();
	/**SPI2 GPIO Configuration    
	PB13     ------> SPI2_SCK
	PB14     ------> SPI2_MISO
	PB15     ------> SPI2_MOSI 
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, CAN_CS2_Pin, GPIO_PIN_SET);
	/*Configure GPIO pin : CAN_CS_Pin */
	GPIO_InitStruct.Pin = CAN_CS2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(CAN_CS2_GPIO_Port, &GPIO_InitStruct);
		
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi2.Init.CRCPolynomial = 10;
	//SPI_Init(SPI2, &SPI_HandleTypeDef);
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    //Error_Handler();
  }		
}

/* SPI1 init function */
void HWF_InitSPI1(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
  /* Peripheral clock enable */
  __SPI1_CLK_ENABLE();

  __GPIOB_CLK_ENABLE();
	/**SPI1 GPIO Configuration    
	PB13     ------> SPI1_SCK
	PB14     ------> SPI1_MISO
	PB15     ------> SPI1_MOSI 
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, CAN_CS1_Pin, GPIO_PIN_SET);
	/*Configure GPIO pin : CAN_CS_Pin */
	GPIO_InitStruct.Pin = CAN_CS1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(CAN_CS1_GPIO_Port, &GPIO_InitStruct);
		
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial = 10;
	//SPI_Init(SPI2, &SPI_HandleTypeDef);
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    //Error_Handler();
  }		
}

