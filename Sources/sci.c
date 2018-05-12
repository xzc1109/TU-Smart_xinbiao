/*
 * sci.c


 *
 *  Created on: Mar 13, 2016
 *      Author: zhongwei
 */
#include"sci.h"

unsigned char SendHexHigh(unsigned char hex) {
	unsigned char temp=0;
	unsigned char high;
	high = hex >> 4;
	if(high < 10) {
		temp=high + '0';
	} else {
		temp=(high - 10) + 'A';
	}
	return temp;
}

unsigned char SendHexLow(unsigned char hex) {
	unsigned char temp=0;
	unsigned char low;
	low = hex & 0x0F;
	if(low < 10) {
		temp=low + '0';
	} else {
		temp=(low - 10) + 'A';
	}
	return temp;
}

unsigned char SendInt1(int currentdata)
{
  unsigned char temp;
  temp=currentdata>>8;
  return SendHexHigh(temp);
}

unsigned char SendInt2(int currentdata)
{
  unsigned char temp;
  temp=currentdata>>8;
  return SendHexLow(temp);
}

unsigned char SendInt3(int currentdata)
{
  unsigned char temp;
  temp=currentdata & 0x00FF;
  return SendHexHigh(temp);
}

unsigned char SendInt4(int currentdata)
{
  unsigned char temp;
  temp=currentdata & 0x00FF;
  return SendHexLow(temp);
}
unsigned char SendUnsignedLong1(unsigned long currentdata)
{
  unsigned char temp;
  temp=currentdata>>24;
  return SendHexHigh(temp);
}
unsigned char SendUnsignedLong2(unsigned long currentdata)
{
  unsigned char temp;
  temp=currentdata>>24;
  return SendHexLow(temp);
}
unsigned char SendUnsignedLong3(unsigned long currentdata)
{
  unsigned char temp;
  temp=currentdata>>16;
  temp=temp&0x00ff;
  return SendHexHigh(temp);
}
unsigned char SendUnsignedLong4(unsigned long currentdata)
{
  unsigned char temp;
  temp=currentdata>>16;
  temp=temp&0x00ff;
  return SendHexLow(temp);
}
unsigned char SendUnsignedLong5(unsigned long currentdata)
{
  unsigned char temp;
  temp=currentdata>>8;
  temp=temp&0x0000ff;
  return SendHexHigh(temp);
}
unsigned char SendUnsignedLong6(unsigned long currentdata)
{
  unsigned char temp;
  temp=currentdata>>8;
  temp=temp&0x0000ff;
  return SendHexLow(temp);
}
unsigned char SendUnsignedLong7(unsigned long currentdata)
{
  unsigned char temp;
  temp=currentdata;
  temp=temp&0x000000ff;
  return SendHexHigh(temp);
}
unsigned char SendUnsignedLong8(unsigned long currentdata)
{
  unsigned char temp;
  temp=currentdata;
  temp=temp&0x000000ff;
  return SendHexLow(temp);
}
