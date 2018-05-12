/*
 * xinbiao_control.c
 *
 *  Created on: Apr 17, 2018
 *      Author: yqye
 */

#include"DeutschesSpiel.h"
uint8_t XB_Direction=1;//1��ʾֱ����ת,0��ʾ��ת
typedef struct 
{
	uint8_t flag;//�ж��ű��ڳ����ĸ����޵��ĸ�
	uint8_t error;//��ǰ���복�������ߵ�ƫ��ֵ��
	uint8_t radius;//ת��ʱֱ����OPENCV�����ͼ��Ľ��������׼ת��()
}Xinbiao,*Xinbiao;
Xinbiao YQY;
float Cal_R(int x,int y)
{
	float R;
	R=(x*x+y*y)/(2*x);//�����������Ҫ���ݱ궨�����,ֱ�߰뾶Ϊ����
	return R;
}

Xinbiao Judg(int x, int y)
{
	Xinbiao jud;
	uint8_t R,Distance;
	if(x>=0&&y>=0)
	{
		jud->flag=1;
		jud->radius=Cal_R(x,y);//
		jud->error=x;
	}
	else if(x<0&&y>=0)
	{
		jud->flag=2;
		jud->radius=Cal_R(x,y);//
		jud->error=x;
	}
	else if(x<0&&y<0)
	{
		jud->flag=3;
		jud->radius=0.00001;//ԽСԽ��
		jud->error=x;
	}
	else 
	{
		jud->flag=4;
		jud->radius=0.00001;//ԽСԽ��
		jud->error=x;
	}
	
}


float Matlab_PWM_calculate_XB(float Radius,bool Direction)//�����ʾ����
{
	float PWM_val;
	if (Direction)//��ת
		{
	       PWM_val=(-582.5*atan(0.2572/Radius)+927.8);//����Ҫ�궨��������
	       return PWM_val;
		}
	else
		{

		   PWM_val=(472.2*atan(0.2572/Radius)+928.7);
		   return PWM_val;
		}
}

void Steer__control_update_XB(Steer steer,float radius,float points[1][2])//��Щ������ʱ��ֱ���ã�
{
	static float Last_radius=999.0f;
	uint8_t direction;
	float temp,der;
	float pwm_duty_1,pwm_duty_2,pwm_duty;//������Ƶ�pwm_duty
	float steer_center=926;
	{
		  if (radius>0.0f) direction=1; else direction=0;
		  if (radius>0.0f&&radius<60.0f) radius=60.0f;
		  if (radius<0.0f&&radius>-60.0f)radius=-60.0f;
		  if (radius==0.0f) radius=999.0f;
		  else
		  {
			der=(float)fabs(radius-Last_radius);
			if (der<50.0f) PID__config(&steer->steer_pid,0,0,0.30f,10,500,80,500);
			else            PID__config(&steer->steer_pid,0,0,0.30f,10,500,80,500);//���D�ֶ�
			temp=radius+PID__update(&steer->steer_pid,radius,Last_radius);
			Last_radius=radius;
		    pwm_duty_1 = Matlab_PWM_calculate(fabs(temp)/100.0f,direction)/10000.0f;/*������Ҫ��PWMֵ*/
		  }
	}

		  pwm_duty_2 = 275.5/10000000.0f * (-23.875f - points[0][1]) + 0.0926f;
		  pwm_duty_2=pwm_duty_2/1.0f;
		  if(lane_flag==1)pwm_duty = pwm_duty_2;
		  else pwm_duty = (pwm_duty_1*1.0f + pwm_duty_2*0.0f);
		  Steer__output(steer,pwm_duty);

}

void XB_control_main(Xinbiao H)
{
	if(H->flag==4||H->flag==3)
	{
		if(XB_Direction==1)
		{
			Steer__output(&steer,CENTER);//����Ϊ�����CENTERҪ��
		}
		else
		{
			Steer__output(&steer,CENTER);//����Ϊ�Ҵ���
		}
	}
	else if��H->flag==1��
	{
		;
	}
	else
	{
		;
	}
}
