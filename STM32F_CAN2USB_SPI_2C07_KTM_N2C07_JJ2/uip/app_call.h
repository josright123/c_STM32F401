
#ifndef __APP_CALL__
#define __APP_CALL__

#include "stdint.h"

#define CFG_TCP_APP_ENABLE         //Enable TCP Application, if no any TCP App disable it
#define CFG_UDP_APP_ENABLE         //Eable UDP Application, if no any UDP App disable it

/*--------------------- TCP APP ---------------------------------------*/
#ifdef CFG_TCP_APP_ENABLE
#include "httpd.h"

	typedef union
	{
		struct httpd_state webserver_app;
	}Str_TCP_App_State;

	typedef Str_TCP_App_State uip_tcp_appstate_t;
#else
	typedef int uip_tcp_appstate_t; 
#endif //CFG_TCP_APP_ENABLE
	
void tcp_appcall(void);
	
#ifndef UIP_APPCALL
#define UIP_APPCALL tcp_appcall
#endif //UIP_APPCALL

/*--------------------- UDP APP ---------------------------------------*/	
#ifdef CFG_UDP_APP_ENABLE
	
#define DHCPC_EN        1 //0  //1

#include "dhcpc.h"
#include "resolv.h"	
	
	typedef union
	{
		struct dhcpc_state dhcpc_app;
	}Str_UDP_App_State;
	
	typedef Str_UDP_App_State uip_udp_appstate_t;
#else
	typedef int uip_udp_appstate_t; 
#endif //CFG_UDP_APP_ENABLE.
	
void udp_appcall(void);
#ifndef UIP_UDP_APPCALL
#define UIP_UDP_APPCALL udp_appcall
#endif //UIP_UDP_APPCALL

extern uint8_t dhcpc_config_done;
	
void dns_client_init(void);
#endif /* __APP_CALL__ */
