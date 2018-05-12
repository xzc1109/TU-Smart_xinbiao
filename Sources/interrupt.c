/*
 * interrupt.c
 *
 *  Created on: Nov 07, 2017
 *      Author: Sanqing Qu
 */
#include "interrupt.h"
#include "MPC5604C.h"
void enableIrq(void) 
{
	  INTC.CPR.B.PRI = 0;          /* Single Core: Lower INTC's current priority */
	  asm(" wrteei 1");	    	   /* Enable external interrupts */
}
