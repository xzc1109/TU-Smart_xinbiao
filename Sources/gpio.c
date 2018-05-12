/*
 * gpio.c
 *
 *  Created on: Nov 7, 2017
 *      Author: Sanqing Qu
 */
#include "gpio.h"
void GPIO__input__enable(uint8_t pad,bool weak_pull_enable, bool pullup, uint8_t alt_fun)
{

	SIU.PCR[pad].R = 0; 
//	SIU.PCR[pad].B.HYS = 1;
	SIU.PCR[pad].B.WPE = weak_pull_enable;
	SIU.PCR[pad].B.WPS = (pullup!=0);
	SIU.PCR[pad].B.IBE = 1;
	SIU.PCR[pad].B.PA = 0;
	
}

void GPIO__output__enable(uint8_t pad)
{

//	SIU.PCR[pad].R = 0; 
//	SIU.PCR[pad].B.ODE = 1;
//	SIU.PCR[pad].B.OBE = 1;
//	SIU.PCR[pad].B.PA = 0;
	SIU.PCR[pad].R = 0x0200;//X0220
	// 0x0220 	寮�紡杈撳嚭
	// 0x0200 	鎺ㄦ尳杈撳嚭 
	SIU.GPDO[pad].B.PDO=1;//kai碌脝

}
void GPIO__output_toggle(uint8_t pad)
{
	SIU.GPDO[pad].B.PDO = ~ SIU.GPDO[pad].B.PDO;
}
void GPIO__output_low(uint8_t pad)
{
	SIU.GPDO[pad].B.PDO = 0;
}

void GPIO__output_high(uint8_t pad)
{
	SIU.GPDO[pad].B.PDO = 1;
}
