/*
 * xinbiao_main.c
 *
 *  Created on: Apr 10, 2018
 *      Author: yqye
 */

#include"DeutschesSpiel.h"

#define CENTER 0.0842
static ECD_t ecd_right;
static ECD_t ecd_left;
static Steer_t steer;
static Motor_t motor_right;
static Motor_t motor_left;
static float pwm_actul=CENTER,pwm_target;
static uint8_t check_flag,check_status;
static pid_pram_t pid_pram;
static float reverse_speed=0;
static pid_t Reverse_motor_left_pid;
static uint8_t cs_time1;

extern bool ParkingMode_flag,ObstacleMode_flag;
extern uint8_t data[4];
extern uint8_t points[7];
extern uint8_t pramdata[24];
//uint8_t flagx=0,flagy=0,flagxy=0;
extern uint8_t flagR,flagr,flagRr,Reverse_flag,Switch_lane_flag,Switch_lane_trigger;
extern float Radius,reverse_target_distance; 
extern uint8_t FLAG;//作为外界查询串口中断的一个标志位
extern Rem_con_t Rc_contorller;
extern uint8_t RC__flag;
extern Ray_t RAY_Module;
extern uint8_t Reverse_finish;
extern Lampe Info_Lampe;
extern uint8_t Lampe_test;
extern int ccd_threshold;
float duty;
//extern int ccd_threshold;
extern int img[128];

static uint8_t chaoshenpad=9;
void chaosheng()
{
	cs_time1 = chaosheng_InputCapture();
	ccd_edge_detect(57,75,200,img);
}	
void demo(void)
{
	uint8_t obstacle[5];
	uint8_t obstacle_flag;
	PIT__clear_flag(PIT_Timer1);//清除中断标志位
	/*******更新目前车的车速信息**********/
	Speed__bekommen(&ecd_left);
	if(Dir__bekommen__left(&ecd_left)) 
		ecd_left._speed = -ecd_left._speed;
	motor_left.actual_speed = ecd_left._speed;
	
	Speed__bekommen(&ecd_right);
	if(Dir__bekommen__right(&ecd_right)) 
		ecd_right._speed = -ecd_right._speed;
	motor_right.actual_speed = ecd_right._speed;
	/***********************************/
	obstacle[4] = obstacle[3];
	obstacle[3] = obstacle[2];
	obstacle[2] = obstacle[1];
	obstacle[1] = obstacle[0];
	obstacle[0] = ccd_edge_detect(50,75,ccd_threshold,img);
//				if (obstacle[0]) LINFlex_TX('Y');else LINFlex_TX('N');
	if((!obstacle[4])&&(!obstacle[3])&&(!obstacle[2])&&(!obstacle[1])&&(obstacle[0]))  obstacle_flag=1;
	if (obstacle_flag)
	{
		Steer__output(&steer,0.078);
		delay_ms(300);
		Steer__output(&steer,0.0842);
		Motor__output(&motor_left,0);//为左轮
		Motor__output(&motor_right,0);
	}
	//Obstacle_motor_left__control_update(&motor_left);
	//Steer__control_update(&steer,Radius,wpts);
}

void Init_DeutschesAuto()
{
		/***********配置主频、配置电机、舵机********/
		initModesAndClock(); 
		enableIrq();
		Motor__config(&motor_left,EMIOS_CH19,EMIOS_CH20,0.35f,0.20f,0,10,10,10,10);
		Motor__config(&motor_right,EMIOS_CH22,EMIOS_CH21,0.35f,0.20f,0,10,10,10,10);
		Encoder__config(&ecd_left,EMIOS_CH24,0.17105,512,10,0.032,17);
		Encoder__config(&ecd_right,EMIOS_CH8,0.17105,512,10,0.032,17);
		Encoder__init(&ecd_right);
		Encoder__init(&ecd_left);
		Steer__config(&steer,EMIOS_CH4, 0,0,0.13,20,10,10,500);//控制转向
		Steer__output(&steer,0.09);
		Motor__output(&motor_left,0.15f);//为左轮
		Motor__output(&motor_right,0.15f);
		
		/************初始化串口通信**************/
		initLINFlex_0_UART(12);
		//initLINFlex_0_UART_xinbiaotest(12);
		/*************初始化线性CCD****************/
		ccd_init();//采用PIT_Tiemr2         优先级12            周期为10ms
		//initEMIOS_chaosheng();
		GPIO__output__enable(chaoshenpad);//用A9口初始超声输入信号
		//PIT__config(1,10,64,chaosheng, 10);
		GPIO__output__enable(7);//A7口输给inhabit
		GPIO__output_low(7);
		GPIO__output__enable(8);
		GPIO__output_high(8);
		
		EMIOS__init();
		STM_Init(64,64);
		PIT__config(PIT_Timer1,10,64,demo,10);
		while(1)
		{

//			duty = ((data[0]-'0')*100.0f+(data[1]-'0')*10.0f+(data[2]-'0'))/10000.0f;
//			Steer__output(&steer,duty);
//			cs_time1=chaosheng_InputCapture();


		}
}




