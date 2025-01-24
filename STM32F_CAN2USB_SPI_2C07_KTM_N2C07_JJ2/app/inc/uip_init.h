#ifndef __UIP_INIT_H_
#define __UIP_INIT_H_

extern struct timer periodic_timer, arp_timer;

void tcpip_init(void);
int tcpip_process(void);

#endif /* __UIP_INIT_ */
