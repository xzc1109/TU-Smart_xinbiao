/*
 * delay.c
 *
 *  Created on: Jan 31, 2018
 *      Author: SanqingQu
 */

#include"delay.h"

void delay_us(int us)
{
	int i;
	//ϵͳ��ƵΪ64MHz
	for(i = 0;i<us;i++)
	{
		__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);//__asm(nop)Ϊһ����������ʱ��
		__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);
		__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);
		__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);
		__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);
		__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);
		__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);
		__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);
	}
}
void delay_ms(int ms)
{
	int i;
	for(i = 0;i<ms;i++)delay_us(1000);
}
void delay_s(int s)
{
	int i;
	for( i = 0;i<s;i++)delay_ms(s);
}
