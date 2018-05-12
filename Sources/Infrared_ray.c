/*
 * Infrared_ray.c
 *
 *  Created on: Dec 20, 2017
 *      Author: Sanqing Qu
 */

#include "Infrared_ray.h"
#include "math.h"
#include "uart.h"
//红外线测距模块
Ray_t RAY_Module;
void Ray__init()
{
	EMIOS_0.CH[8].CCR.B.UCPRE=0;	    /* Set channel prescaler to divide by 1 */
	EMIOS_0.CH[8].CCR.B.UCPEN = 1;   /* Enable prescaler; uses default divide by 1 */
	EMIOS_0.CH[8].CCR.B.FREN = 1; 	/* Freeze channel counting when in debug mode */
	EMIOS_0.CH[8].CADR.R =50000;	/********设置周期16ms  62.5HZ*******/
	EMIOS_0.CH[8].CCR.B.MODE = 0x50; /* Modulus Counter Buffered (MCB) */
	EMIOS_0.CH[8].CCR.B.BSL = 0x3;

	EMIOS_0.CH[9].CCR.B.BSL = 0x01;				/* Use counter bus B,C,D,or E */
	EMIOS_0.CH[9].CCR.B.EDPOL = 1;			/* Polarity-leading edge sets output/trailing clears*/
	EMIOS_0.CH[9].CCR.B.EDSEL = 1;				/* Both edges triggering*/
	EMIOS_0.CH[9].CCR.B.FCK = 1;	    			/* Input filter bit clock reference is the system clock*/
	EMIOS_0.CH[9].CCR.B.IF = 1;				    /* Input filter bit of 2 clock cycles*/ 
	EMIOS_0.CH[9].CCR.B.MODE = 0b0000100; 		/* Mode is IPWM Input Pulse Width Measurement */
	EMIOS_0.CH[9].CCR.B.FREN = 1;	    			/* Freeze channel counting when in debug mode */
	EMIOS_0.CH[9].CCR.B.DMA = 0;	  				/* Flag/overrun assigned to interrupt request instead of CTU*/
	EMIOS_0.CH[9].CCR.B.FEN = 0;	    			/* Enables Unified Channel FLAG bit to generate an interrupt signal*/

	SIU.PCR[9].B.PA = 1;            			    /* Selects eMIOS path as alternative mode 3 for input signal */
	SIU.PCR[9].B.IBE = 1;

	EMIOS_0.CH[10].CCR.B.BSL = 0x01;				/* Use counter bus B,C,D,or E */
	EMIOS_0.CH[10].CCR.B.EDPOL = 1;			/* Polarity-leading edge sets output/trailing clears*/
	EMIOS_0.CH[10].CCR.B.EDSEL = 1;				/* Both edges triggering*/
	EMIOS_0.CH[10].CCR.B.FCK = 1;	    			/* Input filter bit clock reference is the system clock*/
	EMIOS_0.CH[10].CCR.B.IF = 1;				    /* Input filter bit of 2 clock cycles*/ 
	EMIOS_0.CH[10].CCR.B.MODE = 0b0000100; 		/* Mode is IPWM Input Pulse Width Measurement */
	EMIOS_0.CH[10].CCR.B.FREN = 1;	    			/* Freeze channel counting when in debug mode */
	EMIOS_0.CH[10].CCR.B.DMA = 0;	  				/* Flag/overrun assigned to interrupt request instead of CTU*/
	EMIOS_0.CH[10].CCR.B.FEN = 0;	    			/* Enables Unified Channel FLAG bit to generate an interrupt signal*/

	SIU.PCR[10].B.PA = 1;            			    /* Selects eMIOS path as alternative mode 3 for input signal */
	SIU.PCR[10].B.IBE = 1;  

	EMIOS_0.CH[11].CCR.B.BSL = 0x01;				/* Use counter bus B,C,D,or E */
	EMIOS_0.CH[11].CCR.B.EDPOL = 1;			/* Polarity-leading edge sets output/trailing clears*/
	EMIOS_0.CH[11].CCR.B.EDSEL = 1;				/* Both edges triggering*/
	EMIOS_0.CH[11].CCR.B.FCK = 1;	    			/* Input filter bit clock reference is the system clock*/
	EMIOS_0.CH[11].CCR.B.IF = 1;				    /* Input filter bit of 2 clock cycles*/ 
	EMIOS_0.CH[11].CCR.B.MODE = 0b0000100; 		/* Mode is IPWM Input Pulse Width Measurement */
	EMIOS_0.CH[11].CCR.B.FREN = 1;	    			/* Freeze channel counting when in debug mode */
	EMIOS_0.CH[11].CCR.B.DMA = 0;	  				/* Flag/overrun assigned to interrupt request instead of CTU*/
	EMIOS_0.CH[11].CCR.B.FEN = 0;	    			/* Enables Unified Channel FLAG bit to generate an interrupt signal*/

	SIU.PCR[11].B.PA = 1;            			    /* Selects eMIOS path as alternative mode 3 for input signal */
	SIU.PCR[11].B.IBE = 1; 

	PIT__config(PIT_Timer4,50,64,Ray__distance_bekommen,10);
}

void Ray__distance_bekommen()
{
	static uint32_t time1,time2;
	int Lateral,Back1,Back2;
	char str[8];
	char format[4] = "^_^"; 
	if(EMIOS_0.CH[9].CSR.B.FLAG==1)
	{	
		time2=EMIOS_0.CH[9].CBDR.R;
		time1=EMIOS_0.CH[9].CADR.R;    
		Lateral=time1-time2;
		if(time2>time1)
			Lateral=time1+EMIOS_0.CH[8].CADR.R-time2;
		EMIOS_0.CH[9].CSR.B.FLAG = 1;	/*Clear Flag*/
		RAY_Module.distance_Lateral = Lateral/10.0f;
	}  
	if(EMIOS_0.CH[10].CSR.B.FLAG==1)
	{	
		time2=EMIOS_0.CH[10].CBDR.R;
		time1=EMIOS_0.CH[10].CADR.R;    
		Back1=time1-time2;
		if(time2>time1)
			Back1=time1+EMIOS_0.CH[8].CADR.R-time2;
		EMIOS_0.CH[10].CSR.B.FLAG = 1;	/*Clear Flag*/
		RAY_Module.distance_Backward1 = Back1/10.0f;
	}  
	if(EMIOS_0.CH[11].CSR.B.FLAG==1)
	{	
		time2=EMIOS_0.CH[11].CBDR.R;//the leading edge 
		time1=EMIOS_0.CH[11].CADR.R;   // the trailing edge
		Back2=time1-time2;
		if(time2>time1)
			Back2=time1+EMIOS_0.CH[8].CADR.R-time2;
		EMIOS_0.CH[11].CSR.B.FLAG = 1;	/*Clear Flag*/
		RAY_Module.distance_Backward2 = Back2/10.0f;
	}
//	f2s(RAY_Module.distance_Backward1,str);
//	BlueTx(str);
//	BlueTx(format);
	PIT__clear_flag(PIT_Timer4);

}
