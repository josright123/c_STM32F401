
#ifndef __ADES_BATTERY_H_
#define __ADES_BATTERY_H_

extern void BAT_DischargeAlgorithmLeadAcid(void);
extern void BAT_ChargeAlgorithmLeadAcid(void);
extern void BAT_DischargeAlgorithmLiPo(void);
extern void BAT_ChargeAlgorithmLiPo(void);
extern void BAT_DischargeAlgorithmLiIon_LiFePO4(void);
extern void BAT_ChargeAlgorithmLiIon_LiFePO4(void);

#endif  // __ADES_BATTERY_H_

