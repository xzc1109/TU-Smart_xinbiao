/*
 * ccd_capture.c
 *
 *  Created on: Feb 1, 2018
 *      Author: SanqingQu
 */

#include"ccd_capture.h"

int img[128];
unsigned char *send;
unsigned char putstring[]="Image";
int ccd_threshold = 302; //Ĭ��200 ���Ը�����λ�����е�������ֵ                   ����170   ����145
void ccd_init()
{
	ADC.MCR.R = 0x20000000;       /*δ��ת�����ݲ��ܱ����ǣ��Ҷ��룻����ת������ģʽ������ǰ��ת����������ֹ��������ͨ��ע����ⴥ����ʹ���ܣ�ģ��ʱ��Ƶ��Ϊ40MHz��CTU����ת��ʹ���ܣ��Զ�ʱ�ӹر�ʹ���ܣ�����ģʽ*/
	ADC.NCMR[1].R = 0x00000004;   /*ʹ��CH34��33ͨ������׼ͨ������λ��������*/
	ADC.CTR[1].R = 0x00008606;    /*ת��ʱ��Ĵ��� ���׼ͨ�������*/
	ADC.MCR.B.NSTART=1;          /* Trigger normal conversions for ADC0 */
	SIU.PCR[26].R = 0x2100;     // ����ccd��AO �˿�    B10
	GPIO__output__enable(33);   // ����ccd��SI �˿�    C1
	GPIO__output__enable(36);   // ����ccd��CLK�˿�    C4
	PIT__config(PIT_Timer0,5,64,ccd_capture,12);
//	PIT__config(PIT_Timer2,10,64,ccd_threshold_detect,11);//������ֵȷ��

}

void ccd_capture()
{
	int A0result;
	static int count;
	int i;
	CCDR_SI = 1;
	delay_us(1);
	CCDR_CLK = 1;
	delay_us(1);
	CCDR_SI = 0;
	delay_us(20);
	while (ADC.CDR[34].B.VALID != 1) {}; /* �ȴ����һ��ɨ�����*/
	A0result = ADC.CDR[34].B.CDATA; /* ��ȡANS1��ת���������*/
	img[0] = A0result;
	CCDR_CLK = 0;
	for(i = 0;i<127;i++)
	{
		delay_us(4);
		CCDR_CLK = 1;
		delay_us(4);
		while (ADC.CDR[34].B.VALID != 1) {}; /* �ȴ�ɨ�����*/
		A0result = ADC.CDR[34].B.CDATA; /* ��ȡANS1��ת���������*/
		img[i+1] = A0result;
		CCDR_CLK = 0;
	}
	delay_us(4);
	CCDR_CLK = 1;
	delay_us(4);
	CCDR_CLK = 0;
	PIT__clear_flag(PIT_Timer0);

	count++;

//	if(count%4 == 0)ccd_img_send();
	//{ccd_max_min_send(img);ccd_pix60pix100_send(img);}//����ʹ�ã������Ҫ�ر�
}
/**
 * sta_pix ��Ե���Ŀ�ʼ���ص�; �������� ȡsta_pix = 70
 * end_pix ��Ե���Ľ������ص�; �������� ȡend_pix = 90 
 * int threshold CCDͼ���ֵ������ֵ; ʵ������ֵ ȡ150
 * int pic[] CCDͼ������ 128*1;
 */
void ccd_threshold_detect()
{
	static ccd_send_flag;
	static int ccd_temp_img[128];
	char format[2] = ":";
	char char_space[3]="  ";
	uint8_t i;
	if(!ccd_send_flag)
		{
			for(i = 0;i<128;i++)ccd_temp_img[i] = img[i]; ccd_send_flag = 1;
		}
	if(ccd_send_flag)
	{
		for(i = 0;i<128;i++)
		{
			BlueTx(Int_to_char(i));
			BlueTx(format);
			BlueTx(Int_to_char(ccd_temp_img[i]));
			BlueTx(char_space);
		}
		ccd_send_flag = 0;
	}
	ccd_max_min_send(ccd_temp_img);
	PIT__clear_flag(PIT_Timer2);
}
uint8_t ccd_edge_detect(uint8_t sta_pix,uint8_t end_pix,int threshold,int pic[])
{
	uint8_t i;
	uint8_t temp=0;
	static uint8_t last,now;
	static int count;
	for(i = 0; i < 128; i++)
	{
		//��CCDͼ����ж�ֵ��
		if(pic[i] >= threshold)
			pic[i] = 1;
		else
			pic[i] = 0;	
	}
	for(i = sta_pix; i <= end_pix; i++)if(pic[i])temp++;
	if(temp >=(end_pix-sta_pix)) 
	{
		if (last) 
		 {
			last=1; return 1;
		 }
		else 
		 {
			last=1;return 0;
		 }
	} 
	else {last=0; return 0;}
//	if (count>=3)return 1; else return 0;//���ϰ���1���ϰ���0
}
void ccd_max_min_send(int pic[])
{
	char char_max[6] = "Max: ";
	char char_min[6] = "Min: ";
	char char_space[4] = "   ";
	int i = 0;
	int max_temp = 0, min_temp = pic[0];
	for(i;i<128;i++)
	{
		if(pic[i]>max_temp) max_temp = pic[i];
		if(pic[i]<min_temp) min_temp = pic[i];
	}
	BlueTx(char_max);
	BlueTx(Int_to_char(max_temp));
	BlueTx(char_space);
	BlueTx(char_min);
	BlueTx(Int_to_char(min_temp));
	BlueTx(char_space);
}
void ccd_pix60pix100_send(int pic[])
{
	char format[2] = ":";
	char char_space[3]="  ";
	int i = 0;
	for(i = 1;i<=128;i++)
	{
		BlueTx(Int_to_char(i));
		BlueTx(format);
		BlueTx(Int_to_char(pic[i]));
		BlueTx(char_space);
	}
}
void ccd_img_send()
{
	unsigned char aa='L';
	uint8_t Ts = 0,Tc = 0;
	send = putstring;
	LINFlex_TX(*send++);
	for(Ts=0;Ts<5;)
	{
		switch(Ts){
		case 0: if(*send!=0x00){
			LINFlex_TX(*send++);
			break;}
		else{
			Ts=1;
			break;}
		case 1:
			LINFlex_TX(aa);
			Ts=2;
			break;
		case 2: 
			LINFlex_TX(SendInt2(img[Tc]));        //����CCDͼ��
			Ts=3;
			break;
		case 3: 
			LINFlex_TX(SendInt3(img[Tc]));
			Ts=4;
			break;
		case 4: 
			LINFlex_TX(SendInt4(img[Tc]));
			if(Tc<127){
				Tc++;
				Ts=2;}
			else{
				Tc=0;
				Ts=5;}
			break;
		}
	}
}
