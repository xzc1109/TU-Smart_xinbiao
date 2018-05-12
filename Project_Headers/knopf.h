/*
 * Knopf.h
 *
 *  Created on: Jan 26, 2018
 *      Author: SanqingQu
 *   利用SIU的外部中断，实现按键功能的检测 
 */

#ifndef KNOPF_H_
#define KNOPF_H_

#include "MPC5604C.h"
#include"IntcInterrupts.h"
void Knopf_init();
void Knopf_func();


#endif /* KNOPF_H_ */
