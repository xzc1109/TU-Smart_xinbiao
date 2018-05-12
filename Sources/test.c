/*
 * test.c
 *
 *  Created on: Nov 10, 2017
 *      Author: Sanqing Qu
 */
#include"test.h"
#include "MPC5604C.h"
#include "math.h"
#include "gpio.h"
#include "all.h"

void func2()
{
//	static uint8_t temp[3]="0";
	uint8_t temp_flag;
	float steer_duty = 0.09;
	float target_distance;
//	if( (temp[0]!=data[0])||(temp[1]!=data[1])||(temp[2]!=data[2]))
//	{
//		temp_flag = data[2];
//		temp[0] = data[0];
//		temp[1] = data[1];
//		temp[2] = data[2];
//		motor.distance = 0;
//	}

	Speed__bekommen(&ecd);
	motor.distance += ecd._speed * 0.02;
	if(Dir__bekommen(&ecd)) 
		ecd._speed = -ecd._speed;
	
	PIT__clear_flag(PIT_Timer1);
	if(FLAG)
	{
		temp_flag = data[3];
		motor.distance = 0;
		FLAG = 0;	//清除串口更新的标志
	}
	else return;
	Motor__output(&motor,0.20);
	switch(temp_flag)
	{
		case 'r':
			steer_duty = CENTER - ((data[0]-'0')*100+(data[1]-'0')*10+(data[2]-'0'))*0.0001f;
//			Motor__output(&motor,0.106);
			Steer__output(&steer,steer_duty);
			break;
		case 'l':
			steer_duty = CENTER + ((data[0]-'0')*100+(data[1]-'0')*10+(data[2]-'0'))*0.0001f;
//			Motor__output(&motor,0.106);
			Steer__output(&steer,steer_duty);
			break;
		default:
			target_distance = 1.0;
//			Motor__output(&motor,0.0945);
			break;
	}
	                    
}


void func3()
{
	//用于测试开环速度
	static float pwm_duty;
	int speed_temp;//用于串口发送速度;
	PIT__clear_flag(PIT_Timer1);
	/*------------光编部分--------------*/
	Speed__bekommen(&ecd);
	motor.distance += ecd._speed * 0.02;
	if(RC__flag)
		return;
	else
	{  
		if (FLAG)
		{
			switch(data[3])
			{
			case'A':
				pwm_duty=(pramdata[0]*100+pramdata[1]*10+pramdata[2])/1000.0f;
				break;
			case'B':
				pwm_duty=-(pramdata[3]*100+pramdata[4]*10+pramdata[5])/1000.0f;
				break;
			default:
				pwm_duty = 0.0f;
			}
			FLAG=0;
			motor.distance=0;
		}
		Steer__output(&steer,CENTER);
		Motor__output(&motor,pwm_duty);
	}
	if (motor.distance>=4.5) Motor__output(&motor,0);
	BlueTx("S");   
	speed_temp = (unsigned int)(ecd._speed*10.0f);
//	UARTitosTX (speed_temp);
	BlueTx(Int_to_char(speed_temp));
	BlueTx("S");   
	BlueTx("  "); 
	
}

void Speed__pid_test()
{
	static char spd_str[6];
	static uint8_t count;
	static int count_spd0;
	uint8_t direction;
	float pwm_duty;
	if(FLAG)
	{
		pid_pram.kp=(pramdata[0]*100+pramdata[1]*10+pramdata[2])/100.f;
		pid_pram.ki=(pramdata[3]*100+pramdata[4]*10+pramdata[5])/100.f;
		pid_pram.kd=(pramdata[6]*100+pramdata[7]*10+pramdata[8])/100.f;
		pid_pram.per=(pramdata[9]*100+pramdata[10]*10+pramdata[11]);
		pid_pram.ier=(pramdata[12]*100+pramdata[13]*10+pramdata[14]);
		pid_pram.der=(pramdata[15]*100+pramdata[16]*10+pramdata[17]);
		//通过串口更新PID参数
		PID__config(&(motor.motor_pid), pid_pram.kp, pid_pram.ki,pid_pram.kd,10,pid_pram.per,pid_pram.ier,pid_pram.der);
		//通过串口更新目前距离
		motor.target_distance = (pramdata[18]*100+pramdata[19]*10+pramdata[20])/100.f;
		//通过串口更新目前速度
		motor.target_speed = (pramdata[21]*100+pramdata[22]*10+pramdata[23])/100.f;
		//每串口更新一次，清零当前车前进的距离.
		motor.distance = 0.0f;
		FLAG = 0;
	}
	if(RC__flag)
	{
		PIT__clear_flag(PIT_Timer1);	
		return;
	}
	/*------------光编部分--------------*/
	Speed__bekommen(&ecd);
	motor.distance += ecd._speed * 0.01;
	if(Dir__bekommen(&ecd)) 
		ecd._speed = -ecd._speed;
	motor.actual_speed = ecd._speed;
	/***************NUC 遥控 辅助调试*******************/
	PIT__clear_flag(PIT_Timer1);
	if (Radius>0.0f) direction=1; else direction=0;
	if (Radius>0.0f&&Radius<60.0f) Radius=60.0f;
	if (Radius<0.0f&&Radius>-60.0f)Radius=-60.0f;
//	if (Radius>700.0f||Radius<-700.0f) 	
//	{
//		pwm_duty=CENTER;
//	}
//	else
	{
		pwm_duty = Matlab_PWM_calculate(fabs(Radius)/100.0f,direction)/10000.0f;
	}
//	Steer__output(&steer,pwm_duty);/*计算需要的PWM值*/
	/***********************************************/
	if(motor.distance<motor.target_distance)
	{
		motor.duty = PID__update(&((motor).motor_pid), motor.target_speed, motor.actual_speed);
		if(motor.duty>=1) motor.duty = 1.0f;
		if(motor.duty<=-1) motor.duty = -1.0f;
		Motor__output(&motor,motor.duty);
	}
	else
	{
		motor.target_speed = 0.0f;
		motor.duty = PID__update(&((motor).motor_pid), motor.target_speed, motor.actual_speed);
		if(motor.duty>=1) motor.duty = 1.0f;
		if(motor.duty<=-1) motor.duty = -1.0f;
		Motor__output(&motor,motor.duty);
	}
	if(ecd._speed<=0.001f&&ecd._speed>= -0.001f)count_spd0++;
	else
		count_spd0 = 0;
	if(count_spd0 >= 5) return;
	if(count%1==0 )
	{
		/****将速度传回上位机******/
		BlueTx("S");  
	
		f2s(ecd._speed,spd_str);
		BlueTx(spd_str);   
		BlueTx("  "); 
		/**********************/
	}
	count++;
}
void Steer_testfunc()
{
	int i = 0 ;
	while(i<5)
	{
		Info_Lampe.Links_Richtung_lapme_flag = 1;
		Info_Lampe.RC_Lampe_flag = 1;
		Info_Lampe.Bremsen_Lampe_flag = 0;
		motor.distance = 0;
		while(motor.distance <1.0f)
		{
			Steer__output(&steer,0.091);
			Motor__output(&motor,0);
		}
		Info_Lampe.Bremsen_Lampe_flag = 1;
		motor.distance = 0;
		while(motor.distance<2.5f)
		{
			Steer__output(&steer,0.113-i*0.001);
			Motor__output(&motor,0);
		}
		Motor__output(&motor,0);
		Info_Lampe.Bremsen_Lampe_flag = 1;
		delay(1000);
		i++;
	}
	Motor__output(&motor,0);
}

void Information_lampe_test()
{
	switch(Lampe_test)
	{
	case '1': //刹车灯亮
		Info_Lampe.Bremsen_Lampe_flag = 1;
		break;
	case '2': //左灯亮
		Info_Lampe.Links_Richtung_lapme_flag = 1;
		break;
	case '3': //右灯亮
		Info_Lampe.Recht_Richtung_lampe_flag = 1;
		break;
	case '4': //RC灯亮
		Info_Lampe.Bremsen_Lampe_flag = 0;
		Info_Lampe.Links_Richtung_lapme_flag = 0;
		Info_Lampe.Recht_Richtung_lampe_flag = 0;
		Info_Lampe.RC_Lampe_flag = 1;
		break;
	default://全灭
		Info_Lampe.Bremsen_Lampe_flag = 0;
		Info_Lampe.Links_Richtung_lapme_flag = 0;
		Info_Lampe.Recht_Richtung_lampe_flag = 0;
		Info_Lampe.RC_Lampe_flag = 0;
		break;
	}
	PIT__clear_flag(PIT_Timer2);
}
void Vertical_Parking(void)
{
	float pwm_duty,distance;
	reverse_target_distance/=1.0f;
	Steer__output(&steer,CENTER);
	distance = reverse_target_distance - 0.60f;
	motor.distance=0;
	Motor__output(&motor,0.16);
	while (motor.distance<distance){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0; return;}}; //0.40
	Motor__output(&motor,0);
	distance = 0.60*3.1415926573/2.0f;
	Steer__output(&steer,0.113);
	delay(100);
	motor.distance=0;
	Motor__output(&motor,0.16);
	while (motor.distance<distance){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0; return;}}; //0.40
	Steer__output(&steer,CENTER);
	motor.distance=0;
	Motor__output(&motor,0.16);
	while (motor.distance<0.33f){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0; return;}}; //0.40
	Motor__output(&motor,0);
}
