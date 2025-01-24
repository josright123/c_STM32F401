
#ifndef __ADES_HARDWARE_FUNCTION_H_
#define __ADES_HARDWARE_FUNCTION_H_

extern void HWF_SystemClock_Config(void);
extern void HWF_InitVar(void);
extern void HWF_InitPORT(void);
extern uint32_t HWF_InitADCMulCH(void);
extern void HWF_InitFanPWM(void);
extern void HWF_InitBeepPWM(void);
extern void HWF_ReInitI2C1Pin(void);
extern void HWF_ReInitI2C3Pin(void);
extern void HWF_InitI2C1(void);
extern void HWF_InitI2C3(void);
extern void HWF_ADCStartConvert(void);
extern void HWF_InitUart(void);
extern void HWF_DeleayInit(void);
extern void HWF_TIM2_Init(uint32_t);
extern void HWF_TIM2_Stop(void);
extern void HWF_TIM4_Init(uint32_t);
extern void HWF_TIM4_Stop(void);
extern void HWF_RN_CHARGE_POWER_50us(uint8_t);
extern void HWF_InitSPI1(void);
extern void HWF_InitSPI2(void);
#endif  // __ADES_HARDWARE_FUNCTION_H_
