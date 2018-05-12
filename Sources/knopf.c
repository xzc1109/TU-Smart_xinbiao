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
		SIU.ISR.B.EIF5 = 1;	//�����־λ
	}
	if(SIU.ISR.B.EIF6 == 1)
	{
		ObstacleMode_flag = 0;
		ParkingMode_flag = 1;
		SIU.ISR.B.EIF6 = 1; //�����־λ
	}
};
void Knopf_init()
{
	/*******************
�˴��İ�������жϲ�����������:	
	 EIRQ[5]---PC[2]----PCR[34]
	 EIRQ[6]---PC[3]----PCR[35]
	********************/
/*********����GPIO_input ����********/	
	SIU.PCR[34].R = 0;
	SIU.PCR[34].B.PA = 00;
	SIU.PCR[34].B.WPE = 1;
	SIU.PCR[34].B.WPS = 0; 	// ѡ����������ʹ�����ų�ʼΪ�͵�ƽ
	SIU.PCR[34].B.IBE = 1;
	
	SIU.PCR[35].R = 0;
	SIU.PCR[35].B.PA = 00;
	SIU.PCR[35].B.WPE = 1;
	SIU.PCR[35].B.WPS = 0;	// ѡ����������ʹ�����ų�ʼΪ�͵�ƽ
	SIU.PCR[35].B.IBE = 1; 
	
/**********EIRQ(�ⲿ�ж�)����********/
	SIU.IREER.B.IREE5 = 1;	//  �����ش���; ���Ӳ�����������º����ߵ�ƽ������ѡ�������ش���
	SIU.IREER.B.IREE6 = 1; 	//  �����ش���; ���Ӳ�����������º����ߵ�ƽ������ѡ�������ش���
	SIU.IFEER.B.IFEE5 = 0;	// Disable �½��ش���; 
	SIU.IFEER.B.IFEE6 = 0;	// Disable �½��ش���; 
	SIU.IFER.B.IFE5 = 1;	// Enable �����˲�
	SIU.IFER.B.IFE6 = 1;	// Enable �����˲�
	
	SIU.IRER.B.EIRE5 = 1;	// Enable �������

	
	SIU.IRER.B.EIRE6 = 1;	// Enable �������
	SIU.IFCPR.B.IFCP = 0xF;	// Prescaled Filter Clock Period = T(FIRC) x (IFCP + 1) = 62.5ns x 16 = 1ms;
	SIU.IFMC[5].B.MAXCNT = 0xF;// Filter Period = T(CK)*MAXCNTx + n*T(CK) = 1x15 + n = 15~18ms n=1~3;
	SIU.IFMC[6].B.MAXCNT = 0xF;
/**********�жϷ�����********/
	INTC_InstallINTCInterruptHandler(Knopf_func,41,13);
}

