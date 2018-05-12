/*
 * uart.h
 *
 *  Created on: Nov 11, 2017
 *      Author: Sanqing Qu
 */

/*
 * Entscheiden Sie PCR[18] als TX und PCR[19] als RX.
 *决定用这两个PCR口来做收发
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

void f2s(float f, char* str);//将浮点数f进行四舍五入后保留两位小数并转换为字符串.

void UARTitosTX (int n);//将整数n转化为字符串并用串口发出
char* Int_to_char(int n);


void LINFlex_RX_xinbiaotest(void);
void initLINFlex_0_UART_xinbiaotest(uint8_t pri);
#endif /* UART_H_ */
