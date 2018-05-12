/*
 * Information_lampe.h
 *
 *  Created on: Jan 23, 2018
 *      Author: SanqingQu
 *	此头文件是用于行车过程中的各个信号灯的控制
 */

#ifndef INFORMATION_LAMPE_H_
#define INFORMATION_LAMPE_H_

#include "MPC5604c.h"
#include "gpio.h"
#include "typedefs.h"
typedef struct 
{
	uint8_t Links_Richtung_lapme; 	// pad C8  pad 40
	uint8_t Recht_Richtung_lampe; 	// pad C7  pad 39
	uint8_t Bremsen_Lampe;			// pad C9  pad 41
	uint8_t RC_Lampe;				// pad C6  pad 38
	
	bool Links_Richtung_lapme_flag;
	bool Recht_Richtung_lampe_flag;
	bool Bremsen_Lampe_flag;
	uint8_t RC_Lampe_flag;
}*Lampe_t, Lampe;

void Information_lampe_init(Lampe_t lampe,uint8_t pad_Links_lampe,uint8_t pad_Recht_lampe,uint8_t pad_Bremsen_lampe,uint8_t pad_Rc_Lampe);

void Information_lampe_control();


#endif /* INFORMATION_LAMPE_H_ */
