/*
 * knopf.c
 *
 *  Created on: Jan 26, 2018
 *      Author: SanqingQu
 */
#include "MPC5604C.h"
#include "knopf.h"
#include "stdbool.h"
#include"IntcInterrupts.h"
bool ParkingMode_flag = 0,ObstacleMode_flag = 0;
void Knopf_func()
{
	if(SIU.ISR.B.EIF5 == 1)
	{
		ObstacleMode_flag = 1;
		ParkingMode_flag = 0;
		SIU.ISR.B.EIF5 = 1;	//清除标志位
	}
	if(SIU.ISR.B.EIF6 == 1)
	{
		ObstacleMode_flag = 0;
		ParkingMode_flag = 1;
		SIU.ISR.B.EIF6 = 1; //清除标志位
	}
};
void Knopf_init()
{
	/*******************
此处的按键检测中断采用引脚如下:	
	 EIRQ[5]---PC[2]----PCR[34]
	 EIRQ[6]---PC[3]----PCR[35]
	********************/
/*********引脚GPIO_input 配置********/	
	SIU.PCR[34].R = 0;
	SIU.PCR[34].B.PA = 00;
	SIU.PCR[34].B.WPE = 1;
	SIU.PCR[34].B.WPS = 0; 	// 选择弱下拉，使得引脚初始为低电平
	SIU.PCR[34].B.IBE = 1;
	
	SIU.PCR[35].R = 0;
	SIU.PCR[35].B.PA = 00;
	SIU.PCR[35].B.WPE = 1;
	SIU.PCR[35].B.WPS = 0;	// 选择弱下拉，使得引脚初始为低电平
	SIU.PCR[35].B.IBE = 1; 
	
/**********EIRQ(外部中断)配置********/
	SIU.IREER.B.IREE5 = 1;	//  上升沿触发; 结合硬件，按键按下后接入高电平，所以选择上升沿触发
	SIU.IREER.B.IREE6 = 1; 	//  上升沿触发; 结合硬件，按键按下后接入高电平，所以选择上升沿触发
	SIU.IFEER.B.IFEE5 = 0;	// Disable 下降沿触发; 
	SIU.IFEER.B.IFEE6 = 0;	// Disable 下降沿触发; 
	SIU.IFER.B.IFE5 = 1;	// Enable 按键滤波
	SIU.IFER.B.IFE6 = 1;	// Enable 按键滤波
	
	SIU.IRER.B.EIRE5 = 1;	// Enable 按键检测

	
	SIU.IRER.B.EIRE6 = 1;	// Enable 按键检测
	SIU.IFCPR.B.IFCP = 0xF;	// Prescaled Filter Clock Period = T(FIRC) x (IFCP + 1) = 62.5ns x 16 = 1ms;
	SIU.IFMC[5].B.MAXCNT = 0xF;// Filter Period = T(CK)*MAXCNTx + n*T(CK) = 1x15 + n = 15~18ms n=1~3;
	SIU.IFMC[6].B.MAXCNT = 0xF;
/**********中断服务函数********/
	INTC_InstallINTCInterruptHandler(Knopf_func,41,13);
}

