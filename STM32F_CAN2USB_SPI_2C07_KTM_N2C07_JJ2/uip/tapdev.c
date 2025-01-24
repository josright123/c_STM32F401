#include "tapdev.h"
//#include "enc28j60.h"//ENC28J60??
#include "uip.h"
#include "DM9051.h"

//??????uip.c????uip_ethaddr
extern struct uip_eth_addr uip_ethaddr;
//????
//unsigned char my_mac[6] = {0x29, 0x7C, 0x07, 0x37, 0x24, 0x63};
/*---------------------------------------------------------------------------*/
void tapdev_init(void)
{
	DM9051_Init();
}

void tapdev_init1(void) {
    //enc28j60_init(my_mac);
	/*
    for (int i = 0; i < 6; i++)
    {
     uip_ethaddr.addr[i] = my_mac[i];
    }
	*/

	
	uip_ethaddr.addr[0] = emacETHADDR0;
	uip_ethaddr.addr[1] = emacETHADDR1;
	uip_ethaddr.addr[2] = emacETHADDR2;
	uip_ethaddr.addr[3] = emacETHADDR3;
	uip_ethaddr.addr[4] = emacETHADDR4;
	uip_ethaddr.addr[5] = emacETHADDR5;
}
//??????
unsigned int tapdev_read(void){
    //enc28j60_packet_receive(uip_buf, MAX_FRAMELEN);
	DM9051_RX();
}
//??????
void tapdev_send(void) {
    //enc28j60_packet_send(uip_buf, MAX_FRAMELEN);
    DM9051_TX();
	
}