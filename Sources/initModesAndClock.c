/*
 * initModesAndClock.c
 *
 *  Created on: Aug 25, 2016
 *      Author: dell-pc
 */

#include "MPC5604C.h"
void initModesAndClock(void)
{
	
	//ģʽ����: ʹ��DRUN, RUN0, SAFE, RESETģʽ
	  ME.MER.R = 0x0000001D;   

	//��ʼ�����໷��
	//�ⲿ����Ϊ8MHz������PLL0Ϊ64MHz  // 0x05400100: 0000 0101 0100 0000 0000 0001 0000 0000
	//����IDF=2,ODF=4,NDIV=64;
	//���໷���ʱ��phi=(clkin*NDIV)/(IDF*ODF)=(8*64)/(2*4)=64MHz
	  CGM.FMPLL_CR.R = 0x05400100;

	//RUN0����: ����ѹ�������򿪣�Data Flash��������ģʽ��Code Flash��������ģʽ��
	//ʹ�����໷�����໷���ʱ����Ϊϵͳʱ�ӡ�
	  ME.RUN[0].R   = 0x001F0074;           
	  
	//������������0: ����������ģʽ�¶�����
	  ME.RUNPC[0].R = 0x000000FE;           
	 
	  // SIUL: ѡ�� ME.RUNPC[0] ������  
	  ME.PCTL[68].R = 0x00;                 

	// ���ý���RUN0ģʽ
	  ME.MCTL.R = 0x40005AF0;               //д��ģʽ����Կ
	  ME.MCTL.R = 0x4000A50F;               //д��ģʽ�ͷ���Կ

	//�ȴ�ģʽת�����
	  while(ME.GS.B.S_MTRANS) {};           
	//��֤������RUN0ģʽ
	  while(ME.GS.B.S_CURRENTMODE != 4) {}  
	
}
