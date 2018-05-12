/*
 * uart.c
 *
 *  Created on: Nov 11, 2017
 *      Author: Xu Zhongcong UND Sanqing Qu 
 *      ��Ҫ�������������ٴ�
 */
#include"uart.h"
#include"stdlib.h"
#include "IntcInterrupts.h"
float Radius,reverse_target_distance; 
uint8_t data[4];
uint8_t pramdata[32]; //�������鳤�ȣ���չͨ��Э��
uint8_t points[7];
uint8_t flagR=0,flagr=0,flagRr=0,Reverse_flag=0,Switch_lane_flag = 0,Switch_lane_trigger;
uint8_t FLAG;
uint8_t lane_flag;//������־,��������Ŀ�공��
uint8_t pram[4];
uint8_t Lampe_test;
extern int ccd_threshold;
void LINFlex_TX(unsigned char data)
{
	LINFLEX_0.BDRL.B.DATA0 = data;       //�������
	while(!LINFLEX_0.UARTSR.B.DTF){}
	LINFLEX_0.UARTSR.B.DTF = 1;
}
void UARTitosTX (int n)
{
  int i,j,sign;
  char s[10];
  if((sign=n)<0)//��¼����
  n=-n;//ʹn��Ϊ����
  i=0;
  do{
       s[i++]=n%10+'0';//ȡ��һ������
  }
  while ((n/=10)>0);//ɾ��������
  if(sign<0)
  s[i++]='-';
  s[i]='\0';
  for(j=i;j>=0;j--)//���ɵ�����������ģ�����Ҫ�������
	  LINFlex_TX(s[j]);
}

char* Int_to_char(int n)
{
	int i,j,sign;
	char temp[10];
	char s[10];
	if((sign=n)<0)//��¼����
		n=-n;//ʹn��Ϊ����
	i=0;
	do{
		temp[i++]=n%10+'0';//ȡ��һ������
	}
	while ((n/=10)>0);//ɾ��������
	if(sign<0)
		temp[i++]='-';
	temp[i]='\0';
	for(j = 0;j<i;j++)
		s[j] = temp[i-j-1];
	s[j] = '\0';
	return s;
}
void f2s(float f, char* str)
{
	
    uint8_t i = 0,j=0;
    char temp[8];
    int n = (int)f;
    f = ((int)(f*100+0.5))/100.0f;//��f�����������뱣����λС��
    f -= n;
    while (n > 0) {
        temp[i++] = n % 10 + '0';
        n /= 10;
    }
//    reverse(str,i);
	for(j = 0;j<i;j++)
		str[j] = temp[i-j-1];
	str[j] = '\0';
	
    str[i++] = '.';
    n = 0;
    while (f > 0 && n < 6) {
        int t = (int)(f * 10);
        str[i++] = t + '0';
        f = f * 10 - t;
        n++;
    }
    str[i] = '\0';
}



void BlueTx(char *send)                             //����������
{
	while(*send!=0x00)
	LINFlex_TX(*send++);
}

void LINFlex_RX(void)
{
	uint8_t temp;
	data[0]=LINFLEX_0.BDRM.B.DATA4;        	// ��ȡ���յ�������
	data[1]=LINFLEX_0.BDRM.B.DATA5;			//
	data[2]=LINFLEX_0.BDRM.B.DATA6;		 //�˰汾����ÿ�����÷�3Byte�ֽڣ������ȡ��˳������
	data[3]=LINFLEX_0.BDRM.B.DATA7;	
	temp=data[3];
	switch (temp)
	{
	case 'X':
		points[0]=data[0]-'0';
		points[1]=data[1]-'0';
		points[2]=data[2]-'0';
		points[6]=1;
	break;
	case 'Y': 
		points[3]=data[0]-'0';
		points[4]=data[1]-'0';
		points[5]=data[2]-'0';
	break;
	case 'T': //�Ƿ�ִ�е��������ı�־λ
//		points[3]=data[0]-'0';
//		points[4]=data[1]-'0';
//		points[5]=data[2]-'0';
		reverse_target_distance=((data[0]-'0')*100+(data[1]-'0')*10+(data[2]-'0'))/100.0f;
		Reverse_flag=1;
	break;
	case 'x':
		points[0]=data[0]-'0';
		points[1]=data[1]-'0';
		points[2]=data[2]-'0';
		points[6]=0;
	break;
	case 'R':
		Radius=((data[0]-'0')*100+(data[1]-'0')*10+(data[2]-'0'));
		flagR=1;
	break;
	case 'r':
		Radius=-((data[0]-'0')*100+(data[1]-'0')*10+(data[2]-'0'));
		flagr=1;
	break;

	case 'A': 
		//K1
		pramdata[0]=data[0]-'0';
		pramdata[1]=data[1]-'0';
		pramdata[2]=data[2]-'0';
		pram[0]=((data[0]-'0')*100+(data[1]-'0')*10+(data[2]-'0'));
	break;
	case 'B':
		//B1
		pramdata[3]=data[0]-'0';
		pramdata[4]=data[1]-'0';
		pramdata[5]=data[2]-'0';
		pram[1]=((data[0]-'0')*100+(data[1]-'0')*10+(data[2]-'0'));
	break;
	case 'C': 
		//K2
		pramdata[6]=data[0]-'0';
		pramdata[7]=data[1]-'0';
		pramdata[8]=data[2]-'0';
		pram[2]=((data[0]-'0')*100+(data[1]-'0')*10+(data[2]-'0'));
	break;
	case 'D': 
		//B2
		pramdata[9]=data[0]-'0';
		pramdata[10]=data[1]-'0';
		pramdata[11]=data[2]-'0';
		pram[3]=((data[0]-'0')*100+(data[1]-'0')*10+(data[2]-'0'));
	break;
	case 'E':
		//K2
		pramdata[12]=data[0]-'0';
		pramdata[13]=data[1]-'0';
		pramdata[14]=data[2]-'0';
	break;
	case 'F':
		//B2
		pramdata[15]=data[0]-'0';
		pramdata[16]=data[1]-'0';
		pramdata[17]=data[2]-'0';
	break;
	case 'W'://�����޸�Ŀ�����
		pramdata[18] = data[0]-'0';
		pramdata[19] = data[1]-'0';
		pramdata[20] = data[2]-'0';
	break;
	case 'S'://�����޸�Ŀ���ٶ�
		pramdata[21] = data[0]-'0';
		pramdata[22] = data[1]-'0';
		pramdata[23] = data[2]-'0';
	break;
	case 'f'://�����ж�������־
		lane_flag = data[2];
		break;
	case 'G':
		//�����л������ı�־λ      Switch_lane_trigger = G �����л�����
		// 					   Switch_lane_trigger = g �����л�����
		Switch_lane_trigger = data[3];
		Switch_lane_flag = data[2];
		break;
	case 'g':
		Switch_lane_trigger = data[3];
		Switch_lane_flag = data[2];
		break;
	case 'L':
		//���ڲ���Infomation_lampe
		Lampe_test = data[2];
		break;
	case'c':
		//����ȷ��CCD����ֵ
        ccd_threshold = ((data[0]-'0')*100+(data[1]-'0')*10+(data[2]-'0'));
        break;
	case 'a':
		    BlueTx("uaaaa");  
		break;
	case 'b': 
			BlueTx("bbbbb"); 
		break;
	default:
		initLINFlex_0_UART(12);
	break;
	}
	if (flagR||flagr) flagRr=1;
	LINFLEX_0.UARTSR.B.DRF = 1;  
	FLAG = 1;	//���ô��ڸ��µı�־
}

void initLINFlex_0_UART(uint8_t pri)
{
//����LINFlex
  LINFLEX_0.LINCR1.B.INIT   = 1;   // �����ʼ��
  LINFLEX_0.LINCR1.B.SLEEP  = 0;  // ��ֹ˯��ģʽ
  LINFLEX_0.LINCR1.B.BF     = 0;  // ���ID��ƥ�䲻�����ж�

  LINFLEX_0.UARTCR.B.UART   = 1;        // ����UARTģʽ
  LINFLEX_0.UARTCR.B.RXEN   = 1;   // �������
  LINFLEX_0.UARTCR.B.TXEN   = 1;   // ������
  LINFLEX_0.UARTCR.B.WL     = 1;        // 8λ����λ
//  LINFLEX_0.UARTCR.B.OP     = 1;      // żУ��
  LINFLEX_0.UARTCR.B.PCE    = 0;  // ��ֹ��żУ��
  LINFLEX_0.UARTCR.B.TDFL   = 0;        // ���ͻ�����Ϊ1���ֽ�
  LINFLEX_0.UARTCR.B.RDFL   = 3;        // ���ջ�����Ϊ4���ֽ�

  LINFLEX_0.LINIBRR.B.DIV_M = 416;      // Baud Rate = 9600, In Case fipg_clock_lin = 64 MHz
  LINFLEX_0.LINFBRR.B.DIV_F = 11;       // Baud Rate = 9600, In Case fipg_clock_lin = 64 MHz

//�����жϣ�ʹ���жϹ���
  LINFLEX_0.LINIER.B.DRIE   = 1;   // ���ݽ�������ж�
  LINFLEX_0.UARTSR.B.DRF    = 1;   // ���������ɱ�־
  LINFLEX_0.UARTSR.B.DTF    = 1;   // ���������ɱ�־
  
  LINFLEX_0.LINCR1.B.INIT   = 0;  // ��Ϊ����ģʽ
  
  SIU.PCR[18].R = 0x0400;    /* MPC56xxB: Configure port B2 as LIN0TX */
  SIU.PCR[19].R = 0x0103;    /* MPC56xxB: Configure port B3 as LIN0RX */
  INTC_InstallINTCInterruptHandler(LINFlex_RX,79,pri); 
}



void LINFlex_RX_xinbiaotest(void)
{
	uint8_t temp;
	data[0]=LINFLEX_0.BDRM.B.DATA4;        	// ��ȡ���յ�������
	data[1]=LINFLEX_0.BDRM.B.DATA5;			//
	data[2]=LINFLEX_0.BDRM.B.DATA6;		 //�˰汾����ÿ�����÷�3Byte�ֽڣ������ȡ��˳������
	data[3]=LINFLEX_0.BDRM.B.DATA7;	
	temp=data[3];
	switch (temp)
	{
	case 'a':
		BlueTx("aabaa");  
	break;
	case 'b': 
		BlueTx("bbbbb"); 
	break;
	
	default:
		initLINFlex_0_UART_xinbiaotest(12);//������ֹ����
	break;
	}
	
	LINFLEX_0.UARTSR.B.DRF = 1;  //�����жϱ�־λ
	
}

void initLINFlex_0_UART_xinbiaotest(uint8_t pri)
{
//����LINFlex
  LINFLEX_0.LINCR1.B.INIT   = 1;   // �����ʼ��
  LINFLEX_0.LINCR1.B.SLEEP  = 0;  // ��ֹ˯��ģʽ
  LINFLEX_0.LINCR1.B.BF     = 0;  // ���ID��ƥ�䲻�����ж�

  LINFLEX_0.UARTCR.B.UART   = 1;        // ����UARTģʽ
  LINFLEX_0.UARTCR.B.RXEN   = 1;   // �������
  LINFLEX_0.UARTCR.B.TXEN   = 1;   // ������
  LINFLEX_0.UARTCR.B.WL     = 1;        // 8λ����λ
//  LINFLEX_0.UARTCR.B.OP     = 1;      // żУ��
  LINFLEX_0.UARTCR.B.PCE    = 0;  // ��ֹ��żУ��
  LINFLEX_0.UARTCR.B.TDFL   = 0;        // ���ͻ�����Ϊ1���ֽ�
  LINFLEX_0.UARTCR.B.RDFL   = 3;        // ���ջ�����Ϊ4���ֽ�

  LINFLEX_0.LINIBRR.B.DIV_M = 34;//416;      // Baud Rate = 9600, In Case fipg_clock_lin = 64 MHz
  LINFLEX_0.LINFBRR.B.DIV_F = 12;       // Baud Rate = 9600, In Case fipg_clock_lin = 64 MHz

//�����жϣ�ʹ���жϹ���
  LINFLEX_0.LINIER.B.DRIE   = 1;   // ���ݽ�������ж�
  LINFLEX_0.UARTSR.B.DRF    = 1;   // ���������ɱ�־
  LINFLEX_0.UARTSR.B.DTF    = 1;   // ���������ɱ�־
  
  LINFLEX_0.LINCR1.B.INIT   = 0;  // ��Ϊ����ģʽ
  
  SIU.PCR[18].R = 0x0400;    /* MPC56xxB: Configure port B2 as LIN0TX */
  SIU.PCR[19].R = 0x0103;    /* MPC56xxB: Configure port B3 as LIN0RX */
  INTC_InstallINTCInterruptHandler(LINFlex_RX_xinbiaotest,79,pri); 
}


