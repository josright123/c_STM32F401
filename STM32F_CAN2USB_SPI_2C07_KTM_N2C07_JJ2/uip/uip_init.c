/*
 * Copyright (c) 2001, Adam Dunkels.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by Adam Dunkels.
 * 4. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the uIP TCP/IP stack.
 *
 * $Id: main.c,v 1.16 2006/06/11 21:55:03 adam Exp $
 *
 */


#include "uip.h"
#include "uip_arp.h"
#include "tapdev.h"

#include "timer.h"

#include "DM9051.h"

#define BUF ((struct uip_eth_hdr *)&uip_buf[0])

#ifndef NULL
#define NULL (void *)0
#endif /* NULL */

struct timer periodic_timer, arp_timer;
#if DHCPC_EN
static struct timer dhcp_timer;
#else
void show_netwaork_configure();
#endif //DHCPC_EN

/*---------------------------------------------------------------------------*/
void tcpip_init(void)
{
  timer_set(&periodic_timer, CLOCK_SECOND / 4);
  timer_set(&arp_timer, CLOCK_SECOND * 10);
  
	//DM9051 Driver Init
  tapdev_init();
  uip_init();
  uip_arp_init();	// Clear arp table.
	
	/*Initial and start system tick time = 1ms */
  /////  SysTick_Config(SystemCoreClock / 1000);  
	
#if DHCPC_EN	
	/* setup the dhcp renew timer the make the first request */
	timer_set(&dhcp_timer, CLOCK_SECOND * 600);
	dhcpc_init(&uip_ethaddr, 6);
	
	dhcpc_request();
#else
	show_netwaork_configure();
#endif //DHCPC_EN
}
void ARP_TEST(void){
	//uip_arp_arpin();
    //network_send();
	/// uip_arp_out();
					/// tapdev_send();	
}
int tcpip_process(void)
{
	int i;
#if DHCPC_EN
	uint16_t status, linkch; 
#endif //DHCPC_EN
	
    uip_len = tapdev_read();
	
    if(uip_len > 0) {
      if(BUF->type == htons(UIP_ETHTYPE_IP)) {
				uip_arp_ipin();
				uip_input();
				/* If the above function invocation resulted in data that
					 should be sent out on the network, the global variable
					 uip_len is set to a value > 0. */
				if(uip_len > 0) {
					uip_arp_out();
					tapdev_send();
				}
      } else if(BUF->type == htons(UIP_ETHTYPE_ARP)) {
				uip_arp_arpin();
				/* If the above function invocation resulted in data that
					 should be sent out on the network, the global variable
					 uip_len is set to a value > 0. */
				if(uip_len > 0) {
					tapdev_send();
				}
      }

    } else if(timer_expired(&periodic_timer)) {
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
#if DHCPC_EN
			if(timer_expired(&dhcp_timer)) 
			{
				/* for now turn off the led when we start the dhcp process */
				status = DM9051_Read_Reg(DM9051_NSR);
				linkch = DM9051_Read_Reg(DM9051_ISR);
				DM9051_Write_Reg(DM9051_ISR, 0x20);
				
				if((status & 0x40) && (linkch & 0x20)){
					///printf("DHCPC Renew...!!!\r\n");
					dhcpc_renew();
				}
				timer_reset(&dhcp_timer);
			}
#endif //DHCPC_EN
    }
		
		return 0;
}

/*---------------------------------------------------------------------------*/
