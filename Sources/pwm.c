/*
 * pwm.c
 *
 *  Created on: Oct 30, 2017
 *      Author: Sanqing Qu
 */
#include"MPC5604C.h"
#include"pwm.h"

void EMIOS__init(void)
{
	EMIOS_0.MCR.B.GPRE= 63;//GPRE+1=��Ƶϵ����/* Divide 64 MHz sysclk by 63+1 = 64 for 1MHz eMIOS clk*/
	EMIOS_0.MCR.B.GPREN = 1;
	EMIOS_0.MCR.B.GTBE = 1;
	EMIOS_0.MCR.B.FRZ = 1   ;
}
void pwm__config(EMIOSn_CH ch)
{
	//��ʼռ�ձ�Ϊ0
	uint8_t temp = ch;
	//����CH23Ϊʱ�ӻ�׼
	EMIOS_0.CH[23].CCR.B.BSL = 0x3;// ѡ��EMIOSģ���ʱ���׼
	EMIOS_0.CH[23].CCR.B.MODE = 0x50;//Mode is Modulus Counter Buffered, init clk
	EMIOS_0.CH[23].CADR.B.CADR = 1000;//	62.5Hz--16000	period will be CADR+1 eMIOS clocks   (EMIOS_slc/fre) 20000
	EMIOS_0.CH[23].CCR.B.UCPRE = 0;//set channel prescaler to divide by 1
	EMIOS_0.CH[23].CCR.B.UCPEN = 1; //prescaler disabled
	
	EMIOS_0.CH[0].CCR.B.BSL = 0x3;// ѡ��EMIOSģ���ʱ���׼
	EMIOS_0.CH[0].CCR.B.MODE = 0x50;//Mode is Modulus Counter Buffered, init clk
	EMIOS_0.CH[0].CADR.B.CADR = 16000;//	62.5Hz--16000	period will be CADR+1 eMIOS clocks   (EMIOS_slc/fre) 20000
	EMIOS_0.CH[0].CCR.B.UCPRE = 0;//set channel prescaler to divide by 1
	EMIOS_0.CH[0].CCR.B.UCPEN = 1; //prescaler disabled
	
	//����PWM���ͨ��
	EMIOS_0.CH[ch].CCR.B.UCPEN = 0;
	EMIOS_0.CH[ch].CCR.B.BSL = 00;
	EMIOS_0.CH[ch].CCR.B.MODE = 0x60;
	EMIOS_0.CH[ch].CCR.B.EDPOL = 1;	
	EMIOS_0.CH[ch].CADR.R = 1;	
	EMIOS_0.CH[ch].CBDR.R = 2;
	EMIOS_0.CH[ch].CCR.B.UCPRE = 00; // channel prescaler pass through
	EMIOS_0.CH[ch].CCR.B.UCPEN = 1;

	EMIOS_0.CH[4].CCR.B.UCPEN = 0;
	EMIOS_0.CH[4].CCR.B.BSL = 01;
	EMIOS_0.CH[4].CCR.B.MODE = 0x60;
	EMIOS_0.CH[4].CCR.B.EDPOL = 1;	
	EMIOS_0.CH[4].CADR.R = 1;	
	EMIOS_0.CH[4].CBDR.R = 2;
	EMIOS_0.CH[4].CCR.B.UCPRE = 00; // channel prescaler pass through
	EMIOS_0.CH[4].CCR.B.UCPEN = 1;

	EMIOS__init();
	//��ʼ���������
	if(temp<=11)
		SIU.PCR[temp].R = 0x0600; //PA[0]--PA[11];
	else if(temp>11&&temp<=15)
		SIU.PCR[temp+32].R = 0x0600;//PC[12]��-PC[15];
	else if(temp>15&&temp<=23)
		SIU.PCR[48+temp].R = 0x0600;//PE[0]--PE[7];
	else 
		SIU.PCR[36+temp].R =0x0A00;//PD[12]--PD[15]
}

void pwm__duty_update(EMIOSn_CH ch,float duty)
{
	if(ch != EMIOS_CH4)//�˴�Ϊ�����
		EMIOS_0.CH[ch].CBDR.B.CBDR = duty*1000; //*16000
	else
		EMIOS_0.CH[ch].CBDR.B.CBDR = duty*16000;
}
void pwm__input_measure_config(EMIOSn_CH ch)
{
	/*�˶δ�����ֲ�ڲ�����*/
	uint8_t temp = ch;
	
	/*ʱ�ӻ�׼����*/
    //eMIOS0 Bͨ��0����/* EMIOS 0 CH 0: Modulus Counter */
	/*����ֻд��EMIOS0--EMIOS7*/
	if(temp<=7&&temp>=0)
	{
	    EMIOS_0.CH[0].CCR.B.UCPRE=0;	    /* Set channel prescaler to divide by 1 */
		EMIOS_0.CH[0].CCR.B.UCPEN = 1;   /* Enable prescaler; uses default divide by 1 */
		EMIOS_0.CH[0].CCR.B.FREN = 1; 	/* Freeze channel counting when in debug mode */
		EMIOS_0.CH[0].CADR.R =16000;/********��������16ms  62.5HZ*******/
		EMIOS_0.CH[0].CCR.B.MODE = 0x50; /* Modulus Counter Buffered (MCB) */
		EMIOS_0.CH[0].CCR.B.BSL = 0x3;	/* Use internal counter */
	}
//	else if(8<temp&&temp<=15)
//	{
//	    EMIOS_0.CH[8].CCR.B.UCPRE=0;	    /* Set channel prescaler to divide by 1 */
//		EMIOS_0.CH[8].CCR.B.UCPEN = 1;   /* Enable prescaler; uses default divide by 1 */
//		EMIOS_0.CH[8].CCR.B.FREN = 1; 	/* Freeze channel counting when in debug mode */
//		EMIOS_0.CH[8].CADR.R =16000;/********��������20ms  50HZ*******/
//		EMIOS_0.CH[8].CCR.B.MODE = 0x50; /* Modulus Counter Buffered (MCB) */
//		EMIOS_0.CH[8].CCR.B.BSL = 0x3;	/* Use internal counter */
//	}
//	else if(16<temp&&temp<=22)
//	{
//	    EMIOS_0.CH[16].CCR.B.UCPRE=0;	    /* Set channel prescaler to divide by 1 */
//		EMIOS_0.CH[16].CCR.B.UCPEN = 1;   /* Enable prescaler; uses default divide by 1 */
//		EMIOS_0.CH[16].CCR.B.FREN = 1; 	/* Freeze channel counting when in debug mode */
//		EMIOS_0.CH[16].CADR.R =16000;/********��������20ms  50HZ*******/
//		EMIOS_0.CH[16].CCR.B.MODE = 0x50; /* Modulus Counter Buffered (MCB) */
//		EMIOS_0.CH[16].CCR.B.BSL = 0x3;	/* Use internal counter */
//	}
//	else if(25<temp&&temp<=27)
//	{
//	    EMIOS_0.CH[24].CCR.B.UCPRE=0;	    /* Set channel prescaler to divide by 1 */
//		EMIOS_0.CH[24].CCR.B.UCPEN = 1;   /* Enable prescaler; uses default divide by 1 */
//		EMIOS_0.CH[24].CCR.B.FREN = 1; 	/* Freeze channel counting when in debug mode */
//		EMIOS_0.CH[24].CADR.R =16000;/********��������20ms  50HZ*******/
//		EMIOS_0.CH[24].CCR.B.MODE = 0x50; /* Modulus Counter Buffered (MCB) */
//		EMIOS_0.CH[24].CCR.B.BSL = 0x3;	/* Use internal counter */
//	}
//	
	EMIOS_0.CH[temp].CCR.B.BSL = 0x01;				/* Use counter bus B,C,D,or E */
	EMIOS_0.CH[temp].CCR.B.EDPOL = 1;			/* Polarity-leading edge sets output/trailing clears*/
	EMIOS_0.CH[temp].CCR.B.EDSEL = 1;				/* Both edges triggering*/
	EMIOS_0.CH[temp].CCR.B.FCK = 1;	    			/* Input filter bit clock reference is the system clock*/
	EMIOS_0.CH[temp].CCR.B.IF = 1;				    /* Input filter bit of 2 clock cycles*/ 
	EMIOS_0.CH[temp].CCR.B.MODE = 0b0000100; 		/* Mode is IPWM Input Pulse Width Measurement */
	EMIOS_0.CH[temp].CCR.B.FREN = 1;	    			/* Freeze channel counting when in debug mode */
	EMIOS_0.CH[temp].CCR.B.DMA = 0;	  				/* Flag/overrun assigned to interrupt request instead of CTU*/
	EMIOS_0.CH[temp].CCR.B.FEN = 0;	    			/* Enables Unified Channel FLAG bit to generate an interrupt signal*/

	SIU.PCR[temp].B.PA = 1;            			    /* Selects eMIOS path as alternative mode ch for input signal */
	SIU.PCR[temp].B.IBE = 1;
	
}
