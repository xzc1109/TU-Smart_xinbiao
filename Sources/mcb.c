/*
 * mcb.c
 *
 *  Created on: Nov 11, 2017
 *      Author: Sanqing Qu
 */
/*
 * 暂时没有通过测试！！！切勿使用
 * */
#include"mcb.h"
#include "MPC5604C.h"
void mcb__config( EMIOSn_CH mcb_ch,bool pullup)
{

	uint8_t temp = mcb_ch;
	if(temp<=11)
		SIU.PCR[temp].R = 0x503; //PA[0]--PA[11];
	else if(temp>11&&temp<=15)
		SIU.PCR[temp+32].R = 0x503;//PC[12]—-PC[15];
	else if(temp>15&&temp<=23)
		SIU.PCR[48+temp].R = 0x503;//PE[0]--PE[7];
	else 
		SIU.PCR[36+temp].R =0x903;//PD[12]--PD[15]
//	SIU.PCR[EMIOSCH[mcb_ch].pcrno].B.PA  = 1;
//	SIU.PCR[EMIOSCH[mcb_ch].pcrno].B.IBE = 1;
////	SIU.PCR[EMIOSCH[mcb_ch].pcrno].B.ODE = 1;	
//	SIU.PCR[EMIOSCH[mcb_ch].pcrno].B.WPE = 1;
//	SIU.PCR[EMIOSCH[mcb_ch].pcrno].B.WPS = 1;

//	EMIOSCH[mcb_ch].reg->CADR.R = 0xffffff;
//	EMIOSCH[mcb_ch].reg->CCR.B.MODE = 0x51;
//	EMIOSCH[mcb_ch].reg->CCR.B.EDSEL = 0;
//	EMIOSCH[mcb_ch].reg->CCR.B.EDPOL = pullup;
	EMIOS_0.CH[mcb_ch].CADR.R = 0xffff;
	EMIOS_0.CH[mcb_ch].CCR.B.MODE = 0x51;
	EMIOS_0.CH[mcb_ch].CCR.B.EDSEL = 0;
	EMIOS_0.CH[mcb_ch].CCR.B.EDPOL = pullup;
	EMIOS__init();
}
uint32_t mcb__read_counter(EMIOSn_CH mcb_ch)
{
//	return	(vuint32_t) EMIOSCH[mcb_ch].reg->CCNTR.R;
	return	(vuint32_t) EMIOS_0.CH[mcb_ch].CCNTR.R;
}
