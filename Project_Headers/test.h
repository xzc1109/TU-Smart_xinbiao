/*
 * test.h
 *
 *  Created on: Nov 10, 2017
 *      Author: Sanqing Qu
 */

#ifndef TEST_H_
#define TEST_H_

#include "all.h"
#include "MPC5604C.h"


/*延迟函数针对单片机的主频为64MHz书写,time为延迟的时间 单位为ms*/
void delay(uint16_t time);
void test__main();

#endif /* TEST_H_ */
