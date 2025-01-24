//ipscan_generation.c

/* in "uip_arp.c" */
/*-----------------------------------------------------------------------------------*/
//void uip_set_wbit(struct arp_hdr *pBUF){}
/*-----------------------------------------------------------------------------------*/
#include "uip.h"
#include "def_generation.h"



static u16_t arp_wbit[16]; 

void icmptab_clr_arp_wbit(u8_t adrr)
{
	arp_wbit[adrr / 16] &= ~(1 << (adrr % 16)); 
}
void icmptab_set_arp_wbit1org(u8_t adrr)
{
	arp_wbit[adrr / 16] |= 1 << (adrr % 16); 
}
//for 'icmptab_get_arp_wbit'
void icmptab_init_arp_wbit(void)
{
	int i;
	for (i=0; i<16; i++)
		arp_wbit[i]= 0;
	icmptab_set_arp_wbit1org(DMIP3); // device-self-added
		
	#if 0
	//for (i=0; i< 256; i++)
	//	if (i&1)
	//		icmptab_set_arp_wbit1org( (u8_t)i);
	//icmptab_clr_arp_wbit(0);
	//icmptab_clr_arp_wbit(255);
	#endif
}
char /*bit*/ icmptab_get_arp_wbit(u8_t adrr)
{
	//return arp_wbit[adrr / 16] >> (adrr % 16); //return value type: bit
	//return (arp_wbit[adrr / 16] >> (adrr % 16)) & 0x1; //return value type: char
	  return (arp_wbit[adrr / 16] >> (adrr % 16)) & 0x1;
}
