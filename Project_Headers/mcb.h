/*
 * mcb.h
 *
 *  Created on: Nov 11, 2017
 *      Author: Sanqing Qu
 */

/*
 * ��ʱû��ͨ�����ԣ���������ʹ��
 * */

#ifndef MCB_H_
#define MCB_H_

#include "MPC5604C.h"
#include "stdbool.h"
#include "pwm.h"
void mcb__config(EMIOSn_CH mcb_ch,bool pullup);
uint32_t mcb__read_counter(EMIOSn_CH mcb_ch);
#endif /* MCB_H_ */
