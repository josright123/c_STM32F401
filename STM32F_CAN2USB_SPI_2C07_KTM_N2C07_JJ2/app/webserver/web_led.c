#include "uip.h"
#include "timer.h"
#include "web_led.h"

char sCurrLED[4]= {'o', 'n', 0}; 
char sCurrP05[6]= {'o', 'f', 'f', 0};

int CurrTmp = 0;
int CurrHum = 0;

uint32_t LED_timer_flag;

void Delay(uint32_t times)
{
	while(times--)
	{
		uint32_t i;
		for (i=0; i<0xffff; i++);
	}
}

void Set_LED_mode(char lkkcode)
{
	//int i;
	
	//GPIO_SetMode(P3, BIT6, GPIO_PMD_OUTPUT);
	
	if(lkkcode == ('0')) // LED off
	{
		//P36 = 1;
	}else if (lkkcode == '1'){ // LED on
		//P36 = 0;
	}else if(lkkcode == '2') // LED Flash
	{
		//for(i = 0 ; i< 30 ; ++i)
		{
			//P36 = 0;
			//Delay(25);
			//P36 = 1;
			//Delay(25);
		}
	}
}
