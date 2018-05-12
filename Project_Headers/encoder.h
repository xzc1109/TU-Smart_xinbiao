/*
 * encode.h
 *
 *  Created on: Oct 29, 2017
 *      Author: Sanqing Qu
 */

#ifndef ENCODER_H_
#define ENCODER_H_
#include"MPC5604C.h"
#include"pwm.h"
typedef struct{
	EMIOSn_CH ch;   	//�������õ�EMIOSͨ��Ϊ����MCB���ܵ�ch16
	float _transmission_ratio;// ������ 0.16515*29/28 = 0.17105// �³�0.15502
	float _radius;		//���ְ뾶 32mm��
	float _speed;		//��������ڵ���ľ����ٶ�
	float _resolution; 	//�������
	uint32_t _last_counter;
	float _time;		//�ɼ�����
	uint8_t dir;		//���ٵķ���
	uint8_t dir_pad;
} ECD_t,*Encoder__t;
/**
 * @ _transmission_ratio 
 */
void Encoder__config(Encoder__t encoder,EMIOSn_CH CH,float transmisson_ratio, float resolution,
		uint32_t time,float radius,uint8_t dir_pad);
void Encoder__init(Encoder__t encoder);
float Speed__bekommen(Encoder__t encoder);
unsigned char Dir__bekommen__left(Encoder__t encoder);
unsigned char Dir__bekommen__right(Encoder__t encoder);
unsigned char NullBekommen(void);


#endif /* ENCODER_H_ */
