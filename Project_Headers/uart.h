/*
 * uart.h
 *
 *  Created on: Nov 11, 2017
 *      Author: Sanqing Qu
 */

/*
 * Entscheiden Sie PCR[18] als TX und PCR[19] als RX.
 *������������PCR�������շ�
 * */

#ifndef UART_H_
#define UART_H_

#include"MPC5604C.h"
//extern uint8_t datain[3];
typedef struct
{
   float k1;
   float k2;
   float b1;
   float b2;
}pram_t;
void LINFlex_TX(unsigned char data);
void BlueTx(char *send)   ;
void LINFlex_RX(void);
void LINFlex_RX_Interrupt(void);
void initLINFlex_0_UART(uint8_t pri);

void f2s(float f, char* str);//��������f�����������������λС����ת��Ϊ�ַ���.

void UARTitosTX (int n);//������nת��Ϊ�ַ������ô��ڷ���
char* Int_to_char(int n);


void LINFlex_RX_xinbiaotest(void);
void initLINFlex_0_UART_xinbiaotest(uint8_t pri);
#endif /* UART_H_ */
