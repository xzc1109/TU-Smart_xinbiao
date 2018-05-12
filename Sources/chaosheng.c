/*
 * chaosheng.c
 *
 *  Created on: May 5, 2018
 *      Author: yqye
 */
#include "MPC5604C.h"
#include "stdbool.h"
#include "IntcInterrupts.h"
#include "chaosheng.h"
#include "DeutschesSpiel.h"

uint8_t cs_time2;
uint8_t cs_time3;
uint8_t chaosheng_InputCapture(void)
{
	uint8_t time1;
	time1 = STM__get_current_time();
	return time1;
}

void chaosheng_InputCapture2(void)
{
	cs_time2 = STM__get_current_time();
}
//8 10 12 14
void initEMIOS_chaosheng(void) 
{
	
	EMIOS_0.CH[6].CCR.B.MODE = 0x02; // Mode is SAIC, continuous 
	EMIOS_0.CH[6].CCR.B.BSL = 0x11; /* 用internal时钟 */
	EMIOS_0.CH[6].CCR.B.EDSEL = 1;  //Both edges
	//	EMIOS_0.CH[5].CCR.B.EDPOL=0; //Edge Select falling edge
	//	EMIOS_0.CH[5].CCR.B.FEN=1;  //interupt enbale
	SIU.PCR[3].R = 0x0102;  // 配置口对应的模块
	SIU.PSMI[16].R=1;//E0UC[6]选择B14，配置模块对应的口
	INTC_InstallINTCInterruptHandler(chaosheng_InputCapture2,144,13);//查中断向量的表
}

//新配的口
//void initEMIOS_chaosheng(void) 
//{
//	//B14场中断捕捉下降沿
//	EMIOS_0.CH[3].CCR.B.MODE = 0x02; // Mode is SAIC, continuous 
//	EMIOS_0.CH[3].CCR.B.BSL	 = 0x11; /* 用internal时钟 */
//	EMIOS_0.CH[3].CCR.B.EDSEL = 1;  //Both edges
//	//	EMIOS_0.CH[5].CCR.B.EDPOL=0; //Edge Select falling edge
//	//	EMIOS_0.CH[5].CCR.B.FEN=1;  //interupt enbale
//	SIU.PCR[3].R = 0x0102;  // 配置口对应的模块
//	SIU.PSMI[13].R=0;//E0UC[6]选择B14，配置模块对应的口
//	INTC_InstallINTCInterruptHandler(chaosheng_InputCapture,142,13);//查中断向量的表
//}
//
//void initEMIOS_chaosheng(void) 
//{
//	//B14场中断捕捉下降沿
//	EMIOS_0.CH[9].CCR.B.MODE = 0x02; // Mode is SAIC, continuous 
//	EMIOS_0.CH[9].CCR.B.BSL	 = 0x11; /* 用internal时钟 */
//	EMIOS_0.CH[9].CCR.B.EDSEL = 0;  //Both edges
//	EMIOS_0.CH[9].CCR.B.EDPOL=0; //Edge Select falling edge
//	//	EMIOS_0.CH[5].CCR.B.FEN=1;  //interupt enbale
//	SIU.PCR[9].R = 0x0102;  // 配置口对应的模块
//	//SIU.PSMI[13].R=0;//E0UC[6]选择B14，配置模块对应的口
//	INTC_InstallINTCInterruptHandler(chaosheng_InputCapture,145,13);//查中断向量的表
//}





//void initEMIOS_chaosheng(void) 
//{
//	//eMIOS0 B通道0设置/* EMIOS 0 CH 0: Modulus Counter */
//	EMIOS_0.CH[0].CCR.B.UCPRE=0;	    /* Set channel prescaler to divide by 1 */
//	EMIOS_0.CH[0].CCR.B.UCPEN = 1;   /* Enable prescaler; uses default divide by 1 */
//	//EMIOS_0.CH[0].CCR.B.FREN = 1; 	/* Freeze channel counting when in debug mode */
//	EMIOS_0.CH[0].CADR.R = 50000;/********设置周期20ms  50HZ*******/
//	EMIOS_0.CH[0].CCR.B.MODE = 0x50; /* Modulus Counter Buffered (MCB) */
//	EMIOS_0.CH[0].CCR.B.BSL = 0x3;	/* Use internal counter */
//
//	//B14场中断捕捉下降沿
//	EMIOS_0.CH[6].CCR.B.MODE = 0x02; // Mode is SAIC, continuous 
//	EMIOS_0.CH[6].CCR.B.BSL = 0x01; /* Use counter bus B (default) */
//	EMIOS_0.CH[6].CCR.B.EDSEL = 1;  //Both edges
//	//	EMIOS_0.CH[5].CCR.B.EDPOL=0; //Edge Select falling edge
//	//	EMIOS_0.CH[5].CCR.B.FEN=1;  //interupt enbale
//	SIU.PCR[3].R = 0x0102;  // Initialize pad for eMIOS channel Initialize pad for input
//	SIU.PSMI[16].R=1;//E0UC[6]选择B14
//	INTC_InstallINTCInterruptHandler(chaosheng_InputCapture,144,13);//这个中断函数怎么弄的？这个向量数144怎么弄
//}

//void initEMIOS_0Image(void) 
//{
//	//eMIOS0 B通道0设置/* EMIOS 0 CH 0: Modulus Counter */
//	EMIOS_0.CH[0].CCR.B.UCPRE=0;	    /* Set channel prescaler to divide by 1 */
//	EMIOS_0.CH[0].CCR.B.UCPEN = 1;   /* Enable prescaler; uses default divide by 1 */
//	//EMIOS_0.CH[0].CCR.B.FREN = 1; 	/* Freeze channel counting when in debug mode */
//	EMIOS_0.CH[0].CADR.R = 50000;/********设置周期20ms  50HZ*******/
//	EMIOS_0.CH[0].CCR.B.MODE = 0x50; /* Modulus Counter Buffered (MCB) */
//	EMIOS_0.CH[0].CCR.B.BSL = 0x3;	/* Use internal counter */
//
//	//B14场中断捕捉下降沿
//	EMIOS_0.CH[6].CCR.B.MODE = 0x02; // Mode is SAIC, continuous 
//	EMIOS_0.CH[6].CCR.B.BSL = 0x01; /* Use counter bus B (default) */
//	EMIOS_0.CH[6].CCR.B.EDSEL = 1;  //Both edges
//	//	EMIOS_0.CH[5].CCR.B.EDPOL=0; //Edge Select falling edge
//	//	EMIOS_0.CH[5].CCR.B.FEN=1;  //interupt enbale
//	SIU.PCR[30].R = 0x0102;  // Initialize pad for eMIOS channel Initialize pad for input
//	SIU.PSMI[16].R=1;//E0UC[6]选择B14
//	INTC_InstallINTCInterruptHandler(FieldInputCapture,144,13);
//
//	//B13行中断捕捉上升沿
//	EMIOS_0.CH[5].CCR.B.MODE = 0x02; // Mode is SAIC, continuous 
//	EMIOS_0.CH[5].CCR.B.BSL = 0x01; /* Use counter bus B (default) */
//	EMIOS_0.CH[5].CCR.B.EDPOL=1; //Edge Select rising edge
//	//EMIOS_0.CH[7].CCR.B.FEN=1;  //interupt enbale
//	SIU.PCR[29].R = 0x0102;  // Initialize pad for eMIOS channel Initialize pad for input 
//	SIU.PSMI[15].R=1;//E0UC[5]选择B13
//	INTC_InstallINTCInterruptHandler(RowInputCapture,143,12); 
//}
//
////*****************************************************************************************************************
////****************************************中断初始化******************************************************    	  *
////*****************************************************************************************************************
//void enableIrq(void) 
//{
//	INTC.CPR.B.PRI = 0;          /* Single Core: Lower INTC's current priority */
//	asm(" wrteei 1");	    	   /* Enable external interrupts */
//}

