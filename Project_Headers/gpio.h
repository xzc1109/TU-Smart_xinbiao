/*
 * gpio.h
 *
 *  Created on: Nov 7, 2017
 *      Author: Sanqing Qu
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "MPC5604C.h"
#include "stdbool.h"
void GPIO__input__enable(uint8_t pad,bool weak_pull_enable, bool pullup, uint8_t alt_fun);
#define GPIO__read(pad) (SIU.GPDI[pad].B.PDI);

void GPIO__output__enable(uint8_t pad);

void GPIO__output_toggle(uint8_t pad);

void GPIO__output_low(uint8_t pad);

void GPIO__output_high(uint8_t pad);


#endif /* GPIO_H_ */
