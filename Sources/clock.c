/*
 * clock.c
 *
 *  Created on: Oct 30, 2017
 *      Author: Sanqing Qu
 */
#include"clock.h"
void clock__config()
{
	/*��ʼϵͳʱ��Ϊ64MHz*/
	EMIOS_0.MCR.B.GPRE= 63;   //GPRE+1=��Ƶϵ����/* Divide 80 MHz sysclk by 79+1 = 80 for 1MHz(0.1us) eMIOS clk*/
	EMIOS_0.MCR.B.GPREN = 1;	/* Enable eMIOS clock */
	EMIOS_0.MCR.B.GTBE = 1;   /* Enable global time base */
	EMIOS_0.MCR.B.FRZ = 1;    /* Enable stopping channels when in debug mode */
}
