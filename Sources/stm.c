/*
 * stm.c
 *
 *  Created on: May 10, 2018
 *      Author: yqye
 */

#include"stm.h"
#include "MPC5604C.h"
void STM_Init(uint8_t system_fre,uint8_t target_fre)
{
	STM.CR.B.CPS = system_fre/target_fre;
	STM.CR.B.FRZ = 1;
	STM.CR.B.TEN = 1;
}
