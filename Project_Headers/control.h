/*
 * control.h
 *
 *  Created on: Oct 29, 2017
 *      Author: Sanqing Qu
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include"MPC5604C.h"
#include"pwm.h"
#include"pid.h"
//typedef enum{
//	EMIOS_CH1 = 1,EMIOS_CH2,EMIOS_CH3,EMIOS_CH4,EMIOS_CH5,EMIOS_CH6,
//	EMIOS_CH7,EMIOS_CH8,EMIOS_CH9,EMIOS_CH10,EMIOS_CH11,EMIOS_CH12,EMIOS_13,
//	EMIOS_CH14,EMIOS_CH15,EMIOS_CH16,EMIOS_CH17,EMIOS_CH18,EMIOS_CH19,EMIOS_CH20,
//	EMIOS_CH21,EMIOS_CH22,EMIOS_CH23
//}EMIOSn_CH;
#define CENTER 0.0842
typedef enum
{
	Pingxing_Tingche = 1,Chuizhi_Tingche,Zhidao,Wandao,Shizi,Shizi_Tingche
}Flag;
typedef struct 
{
	Flag flag;//根据判断的类型来判断如何行进
	uint8_t error;//当前车与车道中心线的偏差值
	uint8_t angle;//转弯时直接由OPENCV处理的图像的结果导出基准转角()
}Pixel_t,*Pixel;
typedef struct
{
	EMIOSn_CH forward_ch;
	EMIOSn_CH backward_ch;
	pid_t motor_pid;
	uint8_t dir;
	float target_speed;
	float actual_speed;
	float duty;
	float distance;
	float target_distance;
}Motor_t,*Motor;
typedef struct
{
	EMIOSn_CH ch;
	pid_t steer_pid;
	float angle; //实际目标输出角，有Photo里的angle和error共同确定
	float duty;
}Steer_t,*Steer;
typedef struct
{
   float kp;
   float ki;
   float kd;
   float per;
   float ier;
   float der;
}pid_pram_t;
void Pixel__update(Pixel photo,Flag flag,uint8_t error,uint8_t angle);

void Motor__config(Motor motor,EMIOSn_CH forward_ch,EMIOSn_CH backward_ch,float kp,float ki,float kd,uint32_t period_ms,float perror_max,float ierror_max,float derror_max );

void Steer__config(Steer steer,EMIOSn_CH ch,float kp,float ki,float kd,uint32_t period_ms,float perror_max,float ierror_max ,float derror_max);

void Steer__output(Steer steer,float duty);
void Motor__output(Motor motor,float duty);

void Control__func();
void Control__main();
void Motor__control_update(Motor motor,float speed);
void Obstacle_Motor__control_update(Motor motor);
void Steer__control_update(Steer steer,float Radius,float points[1][2]);
float steer_controller(float points[1][2]);
float Matlab_PWM_calculate(float Radius,bool Direction);
float PWM_calculate(float Radius,bool Direction);
#define Motor_targetspeed_update(motor, target_speed) ((motor)->target_speed = target_speed);
#define Steer_angle_update(steer, target_angle) ((steer)->angle = target_angle);
#define rechts 1
#define links  0

#endif /* CONTROL_H_ */
