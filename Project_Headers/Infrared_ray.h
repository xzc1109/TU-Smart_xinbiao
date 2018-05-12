/*
 * Infrared_ray.h
 *
 *  Created on: Dec 20, 2017
 *      Author: Sanqing Qu
 */

#ifndef INFRARED_RAY_H_
#define INFRARED_RAY_H_

#include "MPC5604C.h"
#include "math.h"
#include "gpio.h"
#include "all.h"

typedef struct
{
//	EMIOSn_CH Lateral_ch;
//	EMIOSn_CH Backward_ch1;
//	EMIOSn_CH Backward_ch2;
	float distance_Lateral;
	float distance_Backward1;
	float distance_Backward2;
}Ray_t,*Ray;


void Ray__init();
void Ray__distance_bekommen();
float get__Parkplatz_platz();
#endif /* INFRARED_RAY_H_ */
