/*
 * ccd_capture.h
 *
 *  Created on: Feb 1, 2018
 *      Author: SanqingQu
 */

#ifndef CCD_CAPTURE_H_
#define CCD_CAPTURE_H_

#include"MPC5604C.h"
#include"uart.h"
#include"pit.h"
#include"delay.h"
#include"sci.h"
#include"gpio.h"

#define CCDR_SI  SIU.GPDO[33].R   //���������Դ������Ķ˿� SI  C1
#define CCDR_CLK SIU.GPDO[36].R   //���������Դ������Ķ˿� CLK C4
#define CCDR_LED SIU.GPDO[39].R   //���ڵ��� �����⵽EDGE����ת�����  C7
void ccd_init();

void ccd_capture();

void ccd_threshold_detect();
void ccd_max_min_send(int pic[]);
void ccd_pix60pix100_send(int pic[]);

uint8_t ccd_edge_detect(uint8_t sta_pix,uint8_t end_pix,int threshold,int pic[]);

void ccd_img_send();

#endif /* CCD_CAPTURE_H_ */
