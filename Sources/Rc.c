/*
 * Rc.c
 *
 *  Created on: Nov 16, 2017
 *      Author: Sanqing Qu
 */

#include"MPC5604C.h"
#include"Rc.h"
#include"pwm.h"
#include"Information_lampe.h"
#include"pit.h"
Rem_con_t Rc_contorller;
extern Lampe Info_Lampe;
uint8_t RC__flag = 0;
void RC__config(Rem_con contro,EMIOSn_CH motor_ch,
		EMIOSn_CH steer_ch,EMIOSn_CH RC_motor_inch,EMIOSn_CH RC_steer_inch,EMIOSn_CH mode_switch)
{
	contro->motor_ch = motor_ch;
	contro->steer_ch = steer_ch;
	contro->RC_motor_inch = RC_motor_inch;
	contro->RC_steer_inch = RC_steer_inch;
	contro->mode_switch = mode_switch;
}
//void RC_check()
//{
//	uint32_t time1 = 0,time2 = 0;
//	PIT__clear_flag(PIT_Timer5);
//	if(EMIOS_0.CH[Rc_contorller.RC_motor_inch].CSR.B.FLAG==1)
//	{	
//		time2=EMIOS_0.CH[Rc_contorller.RC_motor_inch].CBDR.R;
//		time1=EMIOS_0.CH[Rc_contorller.RC_motor_inch].CADR.R;    
//		Rc_contorller.motor_duty = time1-time2;
//		if(time2>time1)
//			Rc_contorller.motor_duty = time1+EMIOS_0.CH[0].CADR.R-time2;
//		EMIOS_0.CH[Rc_contorller.RC_motor_inch].CSR.B.FLAG = 1;											/*Clear Flag*/
//	}  
//	if(EMIOS_0.CH[Rc_contorller.RC_steer_inch].CSR.B.FLAG==1)
//	{	
//		time2=EMIOS_0.CH[Rc_contorller.RC_steer_inch].CBDR.R;
//		time1=EMIOS_0.CH[Rc_contorller.RC_steer_inch].CADR.R;    
//		Rc_contorller.steer_duty = time1-time2;
//		if(time2>time1)
//			Rc_contorller.steer_duty=time1+EMIOS_0.CH[0].CADR.R-time2;
//		EMIOS_0.CH[Rc_contorller.RC_steer_inch].CSR.B.FLAG = 1;											/*Clear Flag*/
//	}  
//	if(EMIOS_0.CH[Rc_contorller.mode_switch].CSR.B.FLAG==1)
//	{	
//		time2=EMIOS_0.CH[Rc_contorller.mode_switch].CBDR.R;
//		time1=EMIOS_0.CH[Rc_contorller.mode_switch].CADR.R;    
//		Rc_contorller.mode_duty=time1-time2;
//		if(time2>time1)
//			Rc_contorller.mode_duty = time1+EMIOS_0.CH[0].CADR.R-time2;
//		EMIOS_0.CH[Rc_contorller.mode_switch].CSR.B.FLAG = 1;											/*Clear Flag*/
//	} 
//	if(Rc_contorller.steer_duty > 2000)
//		Rc_contorller.steer_duty = 2000;
//	else if(Rc_contorller.steer_duty < 1000)
//		Rc_contorller.steer_duty = 1000;
//	if(Rc_contorller.motor_duty > 1420)
//		Rc_contorller.motor_duty = 1420;
//	else if(Rc_contorller.motor_duty < 850)
//		Rc_contorller.motor_duty = 850;
//	if(Rc_contorller.mode_duty<1300&&Rc_contorller.mode_duty>700)
//	{
//		RC__flag =  1;
//		//pwm__duty_update(Rc_contorller.motor_ch,Rc_contorller.motor_duty);
//		//pwm__duty_update(Rc_contorller.steer_ch,Rc_contorller.motor_duty);
//	}
//	else RC__flag = 0;
//}
void RC__init(Rem_con contro,INTCInterruptFn func)
{
	pwm__input_measure_config(contro->RC_motor_inch);
	pwm__input_measure_config(contro->RC_steer_inch);
	pwm__input_measure_config(contro->mode_switch);
	//PIT__config(PIT_Timer5,20,64,func,10);
}

void init_RC()
{
	
	EMIOS_0.CH[0].CCR.B.UCPRE=0;	    /* Set channel prescaler to divide by 1 */
	EMIOS_0.CH[0].CCR.B.UCPEN = 1;   /* Enable prescaler; uses default divide by 1 */
	EMIOS_0.CH[0].CCR.B.FREN = 1; 	/* Freeze channel counting when in debug mode */
	EMIOS_0.CH[0].CADR.R =16000;/********设置周期16ms  62.5HZ*******/
	EMIOS_0.CH[0].CCR.B.MODE = 0x50; /* Modulus Counter Buffered (MCB) */
	EMIOS_0.CH[0].CCR.B.BSL = 0x3;

	EMIOS_0.CH[3].CCR.B.BSL = 0x01;				/* Use counter bus B,C,D,or E */
	EMIOS_0.CH[3].CCR.B.EDPOL = 1;			/* Polarity-leading edge sets output/trailing clears*/
	EMIOS_0.CH[3].CCR.B.EDSEL = 1;				/* Both edges triggering*/
	EMIOS_0.CH[3].CCR.B.FCK = 1;	    			/* Input filter bit clock reference is the system clock*/
	EMIOS_0.CH[3].CCR.B.IF = 1;				    /* Input filter bit of 2 clock cycles*/ 
	EMIOS_0.CH[3].CCR.B.MODE = 0b0000100; 		/* Mode is IPWM Input Pulse Width Measurement */
	EMIOS_0.CH[3].CCR.B.FREN = 1;	    			/* Freeze channel counting when in debug mode */
	EMIOS_0.CH[3].CCR.B.DMA = 0;	  				/* Flag/overrun assigned to interrupt request instead of CTU*/
	EMIOS_0.CH[3].CCR.B.FEN = 0;	    			/* Enables Unified Channel FLAG bit to generate an interrupt signal*/

	SIU.PCR[3].B.PA = 1;            			    /* Selects eMIOS path as alternative mode 3 for input signal */
	SIU.PCR[3].B.IBE = 1;

	EMIOS_0.CH[6].CCR.B.BSL = 0x01;				/* Use counter bus B,C,D,or E */
	EMIOS_0.CH[6].CCR.B.EDPOL = 1;			/* Polarity-leading edge sets output/trailing clears*/
	EMIOS_0.CH[6].CCR.B.EDSEL = 1;				/* Both edges triggering*/
	EMIOS_0.CH[6].CCR.B.FCK = 1;	    			/* Input filter bit clock reference is the system clock*/
	EMIOS_0.CH[6].CCR.B.IF = 1;				    /* Input filter bit of 2 clock cycles*/ 
	EMIOS_0.CH[6].CCR.B.MODE = 0b0000100; 		/* Mode is IPWM Input Pulse Width Measurement */
	EMIOS_0.CH[6].CCR.B.FREN = 1;	    			/* Freeze channel counting when in debug mode */
	EMIOS_0.CH[6].CCR.B.DMA = 0;	  				/* Flag/overrun assigned to interrupt request instead of CTU*/
	EMIOS_0.CH[6].CCR.B.FEN = 0;	    			/* Enables Unified Channel FLAG bit to generate an interrupt signal*/

	SIU.PCR[6].B.PA = 1;            			    /* Selects eMIOS path as alternative mode 3 for input signal */
	SIU.PCR[6].B.IBE = 1;  

	EMIOS_0.CH[7].CCR.B.BSL = 0x01;				/* Use counter bus B,C,D,or E */
	EMIOS_0.CH[7].CCR.B.EDPOL = 1;			/* Polarity-leading edge sets output/trailing clears*/
	EMIOS_0.CH[7].CCR.B.EDSEL = 1;				/* Both edges triggering*/
	EMIOS_0.CH[7].CCR.B.FCK = 1;	    			/* Input filter bit clock reference is the system clock*/
	EMIOS_0.CH[7].CCR.B.IF = 1;				    /* Input filter bit of 2 clock cycles*/ 
	EMIOS_0.CH[7].CCR.B.MODE = 0b0000100; 		/* Mode is IPWM Input Pulse Width Measurement */
	EMIOS_0.CH[7].CCR.B.FREN = 1;	    			/* Freeze channel counting when in debug mode */
	EMIOS_0.CH[7].CCR.B.DMA = 0;	  				/* Flag/overrun assigned to interrupt request instead of CTU*/
	EMIOS_0.CH[7].CCR.B.FEN = 0;	    			/* Enables Unified Channel FLAG bit to generate an interrupt signal*/

	SIU.PCR[7].B.PA = 1;            			    /* Selects eMIOS path as alternative mode 3 for input signal */
	SIU.PCR[7].B.IBE = 1; 

	PIT__config(PIT_Timer3,20,64,func_RC,13);
}
void func_RC(void)
{ 
	static uint32_t time1,time2;
	int Steer,Motor,Auto;
	if(EMIOS_0.CH[3].CSR.B.FLAG==1)
	{	
		time2=EMIOS_0.CH[3].CBDR.R;
		time1=EMIOS_0.CH[3].CADR.R;    
		Steer=time1-time2;
		if(time2>time1)
			Steer=time1+EMIOS_0.CH[0].CADR.R-time2;
		EMIOS_0.CH[3].CSR.B.FLAG = 1;											/*Clear Flag*/
	}  
	if(EMIOS_0.CH[6].CSR.B.FLAG==1)
	{	
		time2=EMIOS_0.CH[6].CBDR.R;
		time1=EMIOS_0.CH[6].CADR.R;    
		Motor=time1-time2;
		if(time2>time1)
			Motor=time1+EMIOS_0.CH[0].CADR.R-time2;
		EMIOS_0.CH[6].CSR.B.FLAG = 1;											/*Clear Flag*/
	}  
	if(EMIOS_0.CH[7].CSR.B.FLAG==1)
	{	
		time2=EMIOS_0.CH[7].CBDR.R;//the leading edge 
		time1=EMIOS_0.CH[7].CADR.R;   // the trailing edge
		Auto=time1-time2;//Auto=0是因为没有进入该判断

		if(time2>time1)
			Auto=time1+EMIOS_0.CH[0].CADR.R-time2;
		EMIOS_0.CH[7].CSR.B.FLAG = 1;											/*Clear Flag*/
	}
		if(Steer>2000)
			Steer=2000;
		else if(Steer<1000)
			Steer=1000;
	/**********因为小车驱动由电调改为驱动板，所以对遥控模式进行了重新标定****************/	
		if(Motor>1860)
			Motor = 4800/16.0f;
		else if(Motor<860)
			Motor = -4800/16.0f;
		else if(Motor>=1200&&Motor<=1600)
			Motor = 0;
		else 
			Motor = (int)(8.0f*(Motor-1360)/16.0f);
	/**************************/	
		if(Auto<1300&&Auto>700)
		{
			Info_Lampe.RC_Lampe_flag = 1;
			RC__flag = 1;
			/***********遥控器的转向输出***********/
			EMIOS_0.CH[4].CBDR.R=Steer;
			
			/*********遥控器的前进倒退输出*********/
			if(Motor>=0)
			{
				EMIOS_0.CH[5].CBDR.R = 0.7*Motor;
				EMIOS_0.CH[2].CBDR.R = 0;
			}
			else
			{
				EMIOS_0.CH[5].CBDR.R = 0;
				EMIOS_0.CH[2].CBDR.R = -Motor;
			}
		}
 		else {  Info_Lampe.RC_Lampe_flag = 0;RC__flag = 0;}
	  
	PIT__clear_flag(PIT_Timer3);
}
