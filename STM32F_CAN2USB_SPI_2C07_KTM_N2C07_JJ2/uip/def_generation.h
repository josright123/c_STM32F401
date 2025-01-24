//def_generation.h
#ifndef _DEF_GENERATION_H_
#define _DEF_GENERATION_H_

#define DMIP3	((uip_hostaddr[1] >> 8)&0xff) //TEST +1 //(((uip_hostaddr[1] >> 8)&0xff) + 1)  

void icmptab_clr_arp_wbit(uint8_t adrr);
void icmptab_set_arp_wbit1org(uint8_t adrr);
void icmptab_init_arp_wbit(void);

//void uip_arp_drop_update1(void);

/*void uip_set_wbit(struct arp_hdr *pBUF); in "uip_arp.c" */
#endif
