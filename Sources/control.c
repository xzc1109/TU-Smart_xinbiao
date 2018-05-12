/*
 * control.c
 *
 *  Created on: Oct 29, 2017
 *      Author: Sanqing Qu
 */

#include"all.h"
#include "Information_lampe.h"
#include "uart.h"
#include "delay.h"
extern Pixel_t Photo;
extern uint8_t lane_flag,Reverse_flag;
extern uint8_t pram[4];
extern uint8_t pramdata[24];
extern uint8_t points[7];
extern pid_pram_t pid_pram;
extern uint8_t points[7];
uint8_t Reverse_finish=0;
void Pixel__update(Pixel photo,Flag flag,uint8_t error,uint8_t angle)
{
	photo->flag = flag;
	photo->error = error;
	photo->angle = angle;
}
void Motor__config(Motor motor,EMIOSn_CH forward_ch,EMIOSn_CH backward_ch,float kp,float ki,float kd,uint32_t period_ms,float perror_max,float ierror_max,float derror_max )
{
	motor->backward_ch = backward_ch;
	motor->forward_ch = forward_ch;
	pwm__config(forward_ch);
	pwm__config(backward_ch);
	/**EMIOS9用于驱动板的pwm3通道输出持续高电平**/
	pwm__config(EMIOS_CH9);
	pwm__duty_update(EMIOS_CH9,1.0f);
	/***************************************/
	PID__config(&(motor->motor_pid), kp, ki, kd, period_ms, perror_max, ierror_max, derror_max);
}
void Steer__config(Steer steer,EMIOSn_CH ch,float kp,float ki,float kd,uint32_t period_ms ,float perror_max ,float ierror_max ,float derror_max )
{
	steer->ch = ch;
	pwm__config(ch);
	PID__config(&(steer->steer_pid), kp, ki, kd, period_ms, perror_max, ierror_max, derror_max);
}

void Steer__output(Steer steer,float duty)
{
	pwm__duty_update(steer->ch,duty);
}
void Motor__output(Motor motor,float duty)
{
	if(duty>=0.09)
	{
		pwm__duty_update(motor->forward_ch,duty);
		pwm__duty_update(motor->backward_ch,0);
	}
	else if(duty<=-0.09)
	{
		pwm__duty_update(motor->backward_ch,-1.0f*duty);
		pwm__duty_update(motor->forward_ch,0);
	}
	else
	{
		pwm__duty_update(motor->backward_ch,0);
		pwm__duty_update(motor->forward_ch,0);
	}
}
void Motor__control_update(Motor motor,float speed)
{	
	float temp;
	if ((!Reverse_finish)&&(!Reverse_flag))
	{
		motor->target_speed = 0.9f;
	}
	else if((!Reverse_finish)&&Reverse_flag) 
	{
		motor->target_speed = speed;
	}
	else
	{
//		pid_pram.kp=(pramdata[0]*100+pramdata[1]*10+pramdata[2])/100.f;
//		pid_pram.ki=(pramdata[3]*100+pramdata[4]*10+pramdata[5])/100.f;
//		pid_pram.kd=(pramdata[6]*100+pramdata[7]*10+pramdata[8])/100.f;
//		pid_pram.per=(pramdata[9]*100+pramdata[10]*10+pramdata[11]);
//		pid_pram.ier=(pramdata[12]*100+pramdata[13]*10+pramdata[14]);
//		pid_pram.der=(pramdata[15]*100+pramdata[16]*10+pramdata[17]);
	switch(lane_flag)
	{
	case'1'://在直道
		motor->target_speed = 1.70f;
	break;
	case'2'://z2w
		motor->target_speed = 1.3f;
	break;
    	case'3'://w2w
		motor->target_speed = 1.3f;
	break;
	case'4'://wrz
		motor->target_speed = 1.3f;
	break;
	default:
		motor->target_speed = 0.0f;
	break;	
	}
	}
	/*************对电机的PID参数进行分段*****************/
	temp=fabs(motor->target_speed-motor->actual_speed);
	{
	  if(temp>=1.6f)                 PID__config(&motor->motor_pid,0.38f,0.14f,0,10,10,70,10);
	  else if(temp>=1.4f&&temp<1.6f) PID__config(&motor->motor_pid,0.36f,0.14f,0,10,10,75,10);
	  else if(temp>=1.2f&&temp<1.4f) PID__config(&motor->motor_pid,0.34f,0.14f,0,10,10,80,10);
	  else if(temp>=1.0f&&temp<1.2f) PID__config(&motor->motor_pid,0.32f,0.15f,0,10,10,85,10);
      else if(temp>=0.8f&&temp<1.0f) PID__config(&motor->motor_pid,0.30f,0.15f,0,10,10,90,10);
      else if(temp>=0.7f&&temp<0.8f) PID__config(&motor->motor_pid,0.26f,0.16f,0,10,10,100,10);
      else if(temp>=0.6f&&temp<0.7f) PID__config(&motor->motor_pid,0.26f,0.16f,0,10,10,100,10);
      else if(temp>=0.5f&&temp<0.6f) PID__config(&motor->motor_pid,0.24f,0.17f,0,10,10,100,10);
      else if(temp>=0.4f&&temp<0.5f) PID__config(&motor->motor_pid,0.24f,0.18f,0,10,10,100,10);
      else if(temp>=0.3f&&temp<0.4f) PID__config(&motor->motor_pid,0.22f,0.20f,0,10,10,105,10);
      else if(temp>=0.2f&&temp<0.3f) PID__config(&motor->motor_pid,0.20f,0.22f,0,10,10,110,10);
      else if(temp>=0.05f&&temp<0.2f)PID__config(&motor->motor_pid,0.15f,0.24f,0,10,10,115,10);
      else                           PID__config(&motor->motor_pid,0,0,0,10,10,128,10);
	}
	/***************************************************/
	motor->duty = PID__update(&((motor)->motor_pid), motor->target_speed, motor->actual_speed);
	if(motor->duty>=1) motor->duty = 1.0f;
	if(motor->duty<=-1) motor->duty = -1.0f;
	Motor__output(motor,motor->duty);
}
void Obstacle_Motor__control_update(Motor motor)
{	
	float temp;
	{
//		pid_pram.kp=(pramdata[0]*100+pramdata[1]*10+pramdata[2])/100.f;
//		pid_pram.ki=(pramdata[3]*100+pramdata[4]*10+pramdata[5])/100.f;
//		pid_pram.kd=(pramdata[6]*100+pramdata[7]*10+pramdata[8])/100.f;
//		pid_pram.per=(pramdata[9]*100+pramdata[10]*10+pramdata[11]);
//		pid_pram.ier=(pramdata[12]*100+pramdata[13]*10+pramdata[14]);
//		pid_pram.der=(pramdata[15]*100+pramdata[16]*10+pramdata[17]);
	switch(lane_flag)
	{
	case'1'://在直道
		motor->target_speed = 1.7f;
	break;
	case'2'://z2w
		motor->target_speed = 1.3f;
	break;
    	case'3'://w2w
		motor->target_speed = 1.3f;
	break;
	case'4'://wrz
		motor->target_speed = 1.3f;
	break;
	case'6'://停车
		motor->target_speed = 0.0f;
//		delay_s(3); //停车时间有上位机NUC确定
	break;
	default:
		motor->target_speed = 0.0f;
	break;	
	}
	}
	/*************对电机的PID参数进行分段*****************/
	temp=fabs(motor->target_speed-motor->actual_speed);
	{
	  if(temp>=1.6f)                 PID__config(&motor->motor_pid,0.38f,0.14f,0,10,10,70,10);
	  else if(temp>=1.4f&&temp<1.6f) PID__config(&motor->motor_pid,0.36f,0.14f,0,10,10,75,10);
	  else if(temp>=1.2f&&temp<1.4f) PID__config(&motor->motor_pid,0.34f,0.14f,0,10,10,80,10);
	  else if(temp>=1.0f&&temp<1.2f) PID__config(&motor->motor_pid,0.32f,0.15f,0,10,10,85,10);
      else if(temp>=0.8f&&temp<1.0f) PID__config(&motor->motor_pid,0.30f,0.15f,0,10,10,90,10);
      else if(temp>=0.7f&&temp<0.8f) PID__config(&motor->motor_pid,0.26f,0.16f,0,10,10,100,10);
      else if(temp>=0.6f&&temp<0.7f) PID__config(&motor->motor_pid,0.26f,0.16f,0,10,10,100,10);
      else if(temp>=0.5f&&temp<0.6f) PID__config(&motor->motor_pid,0.24f,0.17f,0,10,10,100,10);
      else if(temp>=0.4f&&temp<0.5f) PID__config(&motor->motor_pid,0.24f,0.18f,0,10,10,100,10);
      else if(temp>=0.3f&&temp<0.4f) PID__config(&motor->motor_pid,0.22f,0.20f,0,10,10,105,10);
      else if(temp>=0.2f&&temp<0.3f) PID__config(&motor->motor_pid,0.20f,0.22f,0,10,10,110,10);
      else if(temp>=0.05f&&temp<0.2f)PID__config(&motor->motor_pid,0.15f,0.24f,0,10,10,115,10);
      else                           PID__config(&motor->motor_pid,0,0,0,10,10,128,10);
	}
	/***************************************************/
	motor->duty = PID__update(&((motor)->motor_pid), motor->target_speed, motor->actual_speed);
	if(motor->duty>=1) motor->duty = 1.0f;
	if(motor->duty<=-1) motor->duty = -1.0f;
	Motor__output(motor,motor->duty);
}
void Steer__control_update(Steer steer,float radius,float points[1][2])
{
	static float Last_radius=999.0f;
	uint8_t direction;
	float temp,der;
	float pwm_duty_1,pwm_duty_2,pwm_duty;//方向控制的pwm_duty
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
			else            PID__config(&steer->steer_pid,0,0,0.30f,10,500,80,500);//舵机D分段
			temp=radius+PID__update(&steer->steer_pid,radius,Last_radius);
			Last_radius=radius;
		    pwm_duty_1 = Matlab_PWM_calculate(fabs(temp)/100.0f,direction)/10000.0f;/*计算需要的PWM值*/
		  }
	}
//	else if(lane_flag==1)
//	{
//=======
//	if (radius>0.0f) direction=1; else direction=0;
//	if (radius>0.0f&&radius<60.0f) radius=60.0f;
//	if (radius<0.0f&&radius>-60.0f)radius=-60.0f;
//	if (radius==0.0f) radius=999.0f;
//	else
//	{
//		der=(float)fabs(radius-Last_radius);
//		if (der<50.0f) PID__config(&steer->steer_pid,0,0,0,10,500,80,500);
//		else            PID__config(&steer->steer_pid,0,0,0.20f,10,500,80,500);//舵机D分段
//		temp=radius+PID__update(&steer->steer_pid,radius,Last_radius);
//		Last_radius=radius;
//		pwm_duty_1 = Matlab_PWM_calculate(fabs(temp)/100.0f,direction)/10000.0f;/*计算需要的PWM值*/
//		//Steer__output(steer,pwm_duty_1);
//	}
//>>>>>>> 7071752da2f019b5d662fb617766c8830a1db5d6
	/*****************近场X向偏差***********************/   
	//           车头   ^x
	//                |
	//                |
	//                |
	//          车尾 0 |--------->y 车的坐标系，原点在小车的后轴的中点处 point[][0] ist x  point[][1] ist y
	/*车的y向偏差即对赛道中心线的偏差*/
		  pwm_duty_2 = 275.5/10000000.0f * (-23.875f - points[0][1]) + 0.0926f;
		  pwm_duty_2=pwm_duty_2/1.0f;
		  if(lane_flag==1)pwm_duty = pwm_duty_2;
		  else pwm_duty = (pwm_duty_1*1.0f + pwm_duty_2*0.0f);
		  Steer__output(steer,pwm_duty);
//	if(points[0][1]>5.0f)
//	{
//		y_set0=(-pram[0]*((points[0][1])*(points[0][1])/10000.0f)-pram[1]); // 120 / 80 --6此处重新标定，用黄老板的方法
//	}
//
//	else if(wpts[0][1]<-5.0f)
//	{
//		y_set0=(pram[2]*((points[0][1])*(points[0][1])/10000.0f)+pram[3]);  //120 / 80  --6
//	}
//	else 
//	{
//		y_set0=0;
//	}
//	if(y_set0>134) y_set0=134;
//	else if(y_set0<-156) y_set0=-156;//对近场打角进行限幅
//	y_set0 += steer_center;
	//	  pwm_duty_2=y_set0/10000.0f;
	/*****************************************************/
//	pwm_duty=(pwm_duty_1+pwm_duty_2)/2.0f;
//	Steer__output(steer,y_set0/10000.0f);
}

void Control__main()
{

	//Motor__config(Motor motor,EMIOSn_CH ch,float kp,float ki,float kd,uint32_t period_ms = 20,float perror_max = 5,float ierror_max = 2,float derror_max = 2)
	//Steer__config(Steer steer,EMIOSn_CH ch,float kp,float ki,float kd,uint32_t period_ms = 20,float perror_max = 70,float ierror_max = 40,float derror_max = 40)
	
	while(1)
	{
		
	}
}
float PWM_calculate(float Radius,bool Direction)
{
	float PWM_val;
	uint8_t i;
	float L_PWM_Table[17]={0.098,0.099,0.100,0.101,0.103,0.104,0.105,0.106,0.107,0.108,0.109,0.110,0.111,0.112,0.113,0.114},
		  L_RAD_Table[17]={2.25,1.94,1.65,1.50,1.28,1.18,1.10,1.025,0.95,0.85,0.75,0.70,0.65,0.62,0.61,0.59,0.58},
		  R_PWM_Table[20]={0.086,0.085,0.084,0.083,0.082,0.081,0.080,0.079,0.078,0.077,0.076,0.075,0.074,0.073,0.072,0.071,0.070,0.069,0.068,0.067},
		  R_RAD_Table[20]={2.48,1.92,1.75,1.53,1.33,1.25,1.17,1.04,0.96,0.92,0.85,0.78,0.75,0.73,0.68,0.65,0.63,0.61,0.60,0.59};
	if (Direction)//右转
	{
		for (i=0;i<15;i++)
			if (R_RAD_Table[i]>Radius&&R_RAD_Table[i+1]<Radius)
			{
               	PWM_val=R_PWM_Table[i+1]+(Radius-R_RAD_Table[i+1])/(R_RAD_Table[i]-R_RAD_Table[i+1])*(R_PWM_Table[i]-R_PWM_Table[i+1]);
               	return PWM_val;
			}
	}
	else
	{
		for (i=0;i<19;i++)
			if (L_RAD_Table[i]>Radius&&L_RAD_Table[i+1]<Radius)
			{
               	PWM_val=L_PWM_Table[i]+(L_RAD_Table[i]-Radius)/(L_RAD_Table[i]-L_RAD_Table[i+1])*(L_PWM_Table[i+1]-L_PWM_Table[i]);
               	return PWM_val;
			}
	}
}
float Matlab_PWM_calculate(float Radius,bool Direction)
{
	float PWM_val;
	if (Direction)//右转
		{
	       PWM_val=(-582.5*atan(0.2572/Radius)+927.8);
	       return PWM_val;
		}
	else
		{

		   PWM_val=(472.2*atan(0.2572/Radius)+928.7);
		   return PWM_val;
		}
}
/****************************************************************************************************/
//float steer_controller(float points[1][2]){// 舵机控制
//    float yerror;
//	uint8_t direction;
//	uint8_t center=926;
//           车头   ^x
//                |
//                |
//                |
//          车尾 0 |--------->y 车的坐标系，原点在小车的后轴的中点处 point[][0] ist x  point[][1] ist y

    /* 偏差法 */

    /*车的y向偏差即对赛道中心线的偏差*/
//    if(points[0][1] > 5.0f)
//    {
//    	yerror=-pram[0]*((points[0][1])*(points[0][1])/1000) - pram[1]; // 120 / 80 --6此处重新标定，用黄老板的方法
//    }
//    else if(points[0][1] <- 5.0f)
//    {
//        yerror=pram[2]*((points[0][1])*(points[0][1])/1000) + pram[3];  //120 / 80  --6
//    }
//    else 
//    {
//        yerror = 0;
//    }

//    if(yerror>90) yerror = 200;
//    else if(yerror<-90) yerror = -200;

//    yerror += steer_center;//加上舵机中位值,需要标定中位值

//#if 0
//    /* 曲率法 */
//    if(fabs(points[0][1])<10.0f ) y_set1 = 910;
//    else{
//    	turn_Radius = fabs((points[0][0]*points[0][0]+points[0][1]*points[0][1])/2/points[0][1])/1000;//单位为m
//    	if (points[0][1]<0)direction=links;else direction=rechts;
//    	//if ((turn_Radius>=0.59)&&(turn_Radius<=2.48)) 
//    	y_set1 =Matlab_PWM_calculate(turn_Radius,direction);
////    	else y_set1 =910;
//    }
//
//#endif
//	duty=(center+yerror)/10000;/*将两个偏差加权求平均之后去计算对应的PWM值*/
//    return duty;
//}
/*************************************************************************************/

//void Control__func()
//{		//PIT中断服务函数
//	Flag flag;
//	flag = Photo.flag;
//	switch(flag)
//	{
//		case Pingxing_Tingche:
//			
// 
//			break;
//		case Chuizhi_Tingche:
//
//
//			break;
//		case Zhidao:
//
//
//			break;
//		case Wandao:
//			/*弯道采用动态KP的方法,即KP是error的函数*/
//			
//			break;
//		case Shizi:
//
//
//			break;
//		case Shizi_Tingche:
//
//
//			break;
//		default:
//			return;//不可能出现只能找到远场点的情况
//	}
//}
