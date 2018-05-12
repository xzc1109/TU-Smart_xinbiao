/*
 * mcb.h
 *
 *  Created on: Nov 11, 2017
 *      Author: Sanqing Qu
 */

/*
 * 暂时没有通过测试！！！切勿使用
 * */

#ifndef MCB_H_
#define MCB_H_

#include "MPC5604C.h"
#include "stdbool.h"
#include "pwm.h"
void mcb__config(EMIOSn_CH mcb_ch,bool pullup);
uint32_t mcb__read_counter(EMIOSn_CH mcb_ch);
#endif /* MCB_H_ */
