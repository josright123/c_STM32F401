#include "includes.h"
#include "uip.h"
#include "uip_init.h"
#include "DM9051.h"
#include "dhcpc.h"

uint8_t dhcpc_config_done = 0;

uip_ipaddr_t hostip = {(192) | (168 << 8), (1) | (37 << 8)}; //{(192) | (168 << 8), (140) | (100 << 8)};
uip_ipaddr_t netmask = {(255) | (255 << 8), (255) | (0 << 8)}; //{(255) | (255 << 8), (255) | (0 << 8)};
//uip_ipaddr_t getway = {(192) | (168 << 8), (1) | (254 << 8)}; //{(192) | (168 << 8), (140) | (1 << 8)};
uip_ipaddr_t getway = {(192) | (168 << 8), (0) | (1 << 8)}; //{(192) | (168 << 8), (140) | (1 << 8)};

void tcp_appcall(void)
{
	/* Local Port */
	switch(uip_conn->lport)
	{
		case HTONS(80):
       httpd_appcall();
     break;
		default:
			break;
	}
	/* Remote Port */
	switch(uip_conn->rport)
	{
		default:
			break;
	}
}

void udp_appcall(void)
{		
	/* UDP Remote Port	*/
  switch(uip_udp_conn->rport){
		 case HTONS(53): //Transmite UDP listen port
       resolv_appcall();
     break;
		default:
			break;
  }
	
	/* UDP Local Port	*/
	switch(uip_udp_conn->lport){
#if DHCPC_EN
    case HTONS(67): //Transmite UDP listen port
        dhcpc_appcall();
      break;
		case HTONS(68): //Received UDP listen port
				dhcpc_appcall();
			break;
#endif //DHCPC_EN
		default:
			break;
	}
}

void
uip_log(char *m)
{
  /// printf("uIP log message: %s\r\n", m);
}

void dns_client_init(void)
{
	uip_ipaddr_t ipaddr;

	resolv_init();
	
	/*uip_ipaddr(ipaddr, at_type.dns_saddr[0], (at_type.dns_saddr[0] >> 8), at_type.dns_saddr[1], (at_type.dns_saddr[1] >> 8));
	
	resolv_conf(ipaddr);
	resolv_query(at_type.dns_srvname);*/
}

void
resolv_found(char *name, uint16_t *ipaddr)
{
  //uint16_t *ipaddr2;
  
  if(ipaddr == NULL) {
    /// printf("Host '%s' not found.\r\n", name);
  } else {
/// printf("Found name '%s' = %d.%d.%d.%d \r\n", name,
			/// htons(ipaddr[0]) >> 8,
			/// htons(ipaddr[0]) & 0xff,
			/// htons(ipaddr[1]) >> 8,
			/// htons(ipaddr[1]) & 0xff);
		
    /*    webclient_get("www.sics.se", 80, "/~adam/uip");   */
  }
}

#if DHCPC_EN
void dhcpc_configured(const struct dhcpc_state *s)
#else
void show_netwaork_configure()
#endif //DHCPC_EN
{
	 volatile uip_ipaddr_t ipaddr;
	volatile int ipa=0;

#if DHCPC_EN
	if(s->state == STATE_FAIL) {
		uip_ipaddr(ipaddr, hostip[0], hostip[0] >> 8, hostip[1], hostip[1] >> 8);		//Host IP address
  	uip_sethostaddr(ipaddr);
  	uip_ipaddr(ipaddr, netmask[0], netmask[0] >> 8, netmask[1], netmask[1] >> 8);		//Default Gateway
  	uip_setnetmask(ipaddr);
  	uip_ipaddr(ipaddr, getway[0], getway[0] >> 8, getway[1], getway[1] >>8);	//Network Mask
		uip_setdraddr(ipaddr);
  	//printf("\n------------- Fixed IP address ------------\r\n");
	}else {
  	uip_sethostaddr(s->ipaddr);
  	uip_setnetmask(s->netmask);
  	uip_setdraddr(s->default_router);
		//resolv_conf(s->dnsaddr);			// Now don't need DNS
		//printf("\n------ IP address setting from DHCP -------\r\n");
	}
#else
		uip_ipaddr(ipaddr, hostip[0], hostip[0] >> 8, hostip[1], hostip[1] >> 8);		//Host IP address
  	uip_sethostaddr(ipaddr);
  	uip_ipaddr(ipaddr, netmask[0], netmask[0] >> 8, netmask[1], netmask[1] >> 8);		//Default Gateway
		uip_setnetmask(ipaddr);
  	
  	uip_ipaddr(ipaddr, getway[0], getway[0] >> 8, getway[1], getway[1] >>8);	//Network Mask
  	uip_setdraddr(ipaddr);
  	/// printf("\n------------- Fixed IP address -------------\r\n");
#endif //DHCPC_EN
	
	/* Display system information */
	///printf("MUCCPU @ %d Hz\r\n", SystemCoreClock);
	//printf("SPI0 Bus Freq. = %d\n", SPI_GetBusClock(SPI0));
	///printf("Network chip: DAVICOM DM9051 \r\n");
	///printf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X \r\n", uip_ethaddr.addr[0], uip_ethaddr.addr[1],
						///uip_ethaddr.addr[2], uip_ethaddr.addr[3], uip_ethaddr.addr[4], uip_ethaddr.addr[5]);
	uip_gethostaddr(ipaddr);
	ipa ++;
	///printf("Host IP Address: %d.%d.%d.%d \r\n", uip_ipaddr1(ipaddr), uip_ipaddr2(ipaddr), uip_ipaddr3(ipaddr), uip_ipaddr4(ipaddr));
	uip_getnetmask(ipaddr);
	ipa ++;
	///printf("Network Mask: %d.%d.%d.%d \r\n", uip_ipaddr1(ipaddr), uip_ipaddr2(ipaddr), uip_ipaddr3(ipaddr), uip_ipaddr4(ipaddr));
	uip_getdraddr(ipaddr);
	ipa ++;
	///printf("Gateway IP Address: %d.%d.%d.%d \r\n", uip_ipaddr1(ipaddr), uip_ipaddr2(ipaddr), uip_ipaddr3(ipaddr), uip_ipaddr4(ipaddr));
	///printf("--------------------------------------------- \r\n");
	DM9051_Write_Reg(DM9051_ISR, 0x20);
	
	dhcpc_config_done = 1;
}

#ifdef WEB_CLIENT
void
smtp_done(unsigned char code)
{
  printf("SMTP done with code %d\n", code);
}

void
webclient_closed(void)
{
  printf("Webclient: connection closed\n");
}

void
webclient_aborted(void)
{
  printf("Webclient: connection aborted\n");
}

void
webclient_timedout(void)
{
  printf("Webclient: connection timed out\n");
}

void
webclient_connected(void)
{
  printf("Webclient: connected, waiting for data...\n");
}

void
webclient_datahandler(char *data, uint16_t len)
{
  printf("Webclient: got %d bytes of data.\n", len);
}
#endif //WEB_CLIENT
