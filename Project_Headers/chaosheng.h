/*
 * chaosheng.h
 *
 *  Created on: May 7, 2018
 *      Author: yqye
 */

#ifndef CHAOSHENG_H_
#define CHAOSHENG_H_
#include "MPC5604C.h"
#include "stdbool.h"
#include "stm.h"
#include "IntcInterrupts.h"
#include "DeutschesSpiel.h"

 ///< ��ȡ��ǰϵͳʱ�ӵ�ֵ���� STM__set_frequency() �趨��Ƶ��������
uint8_t chaosheng_InputCapture(void);
void chaosheng_InputCapture2(void);
//void initEMIOS_chaosheng(void) ;

#endif /* CHAOSHENG_H_ */
