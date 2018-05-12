/*
 * final_edition.c
 *
 *  Created on: Feb 3, 2018
 *      Author: SanqingQu
 */

#include"DeutschesSpiel.h"

#define CENTER 0.0926
static ECD_t ecd;
static Steer_t steer;
static Motor_t motor;
static float pwm_actul=CENTER,pwm_target;
static uint8_t check_flag,check_status;
static pid_pram_t pid_pram;
static float reverse_speed=0;
static pid_t Reverse_motor_pid;

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
extern int img[128];

void Init_DeutschesAuto()
{
		/***********配置主频、配置电机、舵机********/
		initModesAndClock(); 
		enableIrq();
		Motor__config(&motor,EMIOS_CH5,EMIOS_CH2,0.35f,0.20f,0,10,10,10,10);
		Encoder__config(&ecd,EMIOS_CH16,0.17105,512,10,0.032,17);
		Encoder__init(&ecd);
		Steer__config(&steer,EMIOS_CH4, 0,0,0.13,20,10,10,500);//控制转向
		Steer__output(&steer,CENTER);
		/*---------------------------------------*/
		
		/*-----------各功能模块初始化-------------*/
		/***********初始化RC监控模式*************/
		init_RC(); //采用的PIT_Timer3 	优先级13 	 控制周期为20ms
		
		/***********初始化红外线测距模块***********/
		Ray__init();//采用的PIT_Timer4      优先级10		控制周期为50ms
		
		/***********初始化按模式键检测模块***********/
		Knopf_init();//采用SIUL模块的EIRQ 优先级13 
		//	 EIRQ[5]---PC[2]----PCR[34]  ObstacleMode
	    //   EIRQ[6]---PC[3]----PCR[35]  ParkingMode
		
		/**********初始化小车信号灯模块***********/
		//小车信号灯采用PIT_Timer5控制 	 	优先级8		控制周期为50ms
		Information_lampe_init(&Info_Lampe,40,39,41,38);
		
		/************初始化串口通信**************/
		initLINFlex_0_UART(12);
		
		/*************初始化线性CCD****************/
		ccd_init();//采用PIT_Tiemr2         优先级12            周期为10ms
		
	
		GPIO__output__enable(13);
		GPIO__output__enable(12);
		GPIO__output__enable(14);
//		while((!ParkingMode_flag)&&(!ObstacleMode_flag))
//		{
//			Steer__output(&steer,CENTER);
//			Motor__output(&motor,0.0f);
//			SIU.GPDO[12].B.PDO=0;
//		    SIU.GPDO[14].B.PDO=1;
//		    SIU.GPDO[13].B.PDO=1;
//		}

		{
//			delay_ms(100);
			PIT__config(PIT_Timer1,10,64,ParkingMode,10);
		    SIU.GPDO[14].B.PDO=0;
		    SIU.GPDO[12].B.PDO=1;
		    SIU.GPDO[13].B.PDO=1;
		}
//		else if(ObstacleMode_flag)
//		{
//			delay_ms(100);
//			PIT__config(PIT_Timer1,10,64,ObstacleMode,10);
//		    SIU.GPDO[13].B.PDO=0;
//		    SIU.GPDO[12].B.PDO=1;
//		    SIU.GPDO[14].B.PDO=1;
//		}
		while(1)
		{
			if(ParkingMode_flag&&Reverse_flag) 
				ParkingMode_Reverse();
			if(ObstacleMode_flag&&Switch_lane_flag) 
				ObstacleMode_Switchlane();
		}
}

void ObstacleMode()
{
	static float last_speed;  //上一次车速，用于刹车检测
	static uint8_t Bremsen_count,Accelerate_count;
	static float position[3];
	static uint8_t count;
	float wpts[1][2];
	
	PIT__clear_flag(PIT_Timer1);//清除中断标志位
	
	/*******更新目前车的车速信息**********/
	Speed__bekommen(&ecd);
	if(Dir__bekommen(&ecd)) 
		ecd._speed = -ecd._speed;
	motor.actual_speed = ecd._speed;
	/********检测小车的刹车状态*********/
	if(last_speed - ecd._speed >=0.015f && last_speed>0.0f && ecd._speed>0.0f)
	{
		if(Bremsen_count>=2)
		{
			Info_Lampe.Bremsen_Lampe_flag = 1;
			Accelerate_count = 0;
		}
		Bremsen_count++;
	}
	else if(last_speed - ecd._speed <=-0.015f && last_speed<0.0f && ecd._speed<0.0f)
	{
		if(Bremsen_count>=2)
		{
			Info_Lampe.Bremsen_Lampe_flag = 1;
			Accelerate_count = 0;
		}
		Bremsen_count++;
	}
	else if(fabs(last_speed) - fabs(ecd._speed) <=-0.015f)
	{
		if(Accelerate_count>=3)
			Info_Lampe.Bremsen_Lampe_flag = 0;
		Accelerate_count++;
		Bremsen_count = 0;    
	}
	if(-0.15f<ecd._speed<0.15f)	Info_Lampe.Bremsen_Lampe_flag = 0;//+-0.15f 为停车速度死区   若停车，则刹车灯灭；
	last_speed = ecd._speed;
	/************************************/ //检测目前小车是否处于RC模式
	if(RC__flag)
	{
		Info_Lampe.RC_Lampe_flag = 1;
		return;//检测到目前处于RC控制状态
	}else
		Info_Lampe.RC_Lampe_flag = 0;
	/***********************************/  //更新小车的行进距离信息
	motor.distance += ecd._speed * 0.01;
	if(Switch_lane_flag) return;
	if(count%2==0) //舵机的控制周期限制为20ms
	{
		if (points[6]) {wpts[0][1]=(points[0])*100+(points[1])*10+(points[2]);} 
		else if (!points[6]){wpts[0][1]=-((points[0])*100+(points[1])*10+(points[2]));}
		wpts[0][0]=(points[3])*100+(points[4])*10+(points[5]);  
		Steer__control_update(&steer,Radius,wpts);
	}
	count++;	
	Obstacle_Motor__control_update(&motor); //
}

void ParkingMode()
{
	static float Ray_dis_last;//红外模块的上次距离记录值
	static float last_speed;  //上一次车速，用于刹车检测
	static uint8_t Bremsen_count,Accelerate_count;
	static uint8_t Obstacle_count;//障碍物边缘的计数,用于确定车库
	static float position[3];
	static uint8_t flag_first=0,flag_second=0;
	static uint8_t count;
	static float lateral_dis_vorletzt,lateral_dis_letzt,lateral_dis_zurzeit;//用于记录侧向红外模块的测量结果
	static float lateral_ccd[5];//用于记录CCD模块的测量结果
	float wpts[1][2];
	char P[8];
	char format[4] = "  "; 
	PIT__clear_flag(PIT_Timer1);//清除中断标志位
	
	/*******更新目前车的车速信息**********/
	Speed__bekommen(&ecd);
	if(Dir__bekommen(&ecd)) 
		ecd._speed = -ecd._speed;
	motor.actual_speed = ecd._speed;
	/********检测小车的刹车状态*********/
	if(fabs(last_speed) - fabs(ecd._speed) >=0.015f)
	{
		if(Bremsen_count>=3)
		{
			Info_Lampe.Bremsen_Lampe_flag = 1;
			Accelerate_count = 0;
		}
		Bremsen_count++;
	}
	else if(fabs(last_speed) - fabs(ecd._speed) <=-0.015f)
	{
		if(Accelerate_count>=3)
			Info_Lampe.Bremsen_Lampe_flag = 0;
		Accelerate_count++;
		Bremsen_count = 0;    
	}
	if(-0.10f<ecd._speed<0.10f)	Info_Lampe.Bremsen_Lampe_flag = 0;//若停车，则刹车灯灭；
	last_speed = ecd._speed;
	/************************************/ //检测目前小车是否处于RC模式
	if(RC__flag)
	{
		Info_Lampe.RC_Lampe_flag = 1;
		return;//检测到目前处于RC控制状态
	}else
		Info_Lampe.RC_Lampe_flag = 0;
	/***********************************/  //更新小车的行进距离信息
	motor.distance += ecd._speed * 0.01;
	/***********************************/
	Motor__control_update(&motor,reverse_speed); //
	if(Reverse_flag)	return;
	
	/*--------侧向CCD模块检测车库-------*/
#if 1
	if(!Reverse_finish)
	{
		lateral_ccd[4] = lateral_ccd[3];
		lateral_ccd[3] = lateral_ccd[2];
		lateral_ccd[2] = lateral_ccd[1];
		lateral_ccd[1] = lateral_ccd[0];
		lateral_ccd[0] = ccd_edge_detect(50,75,ccd_threshold,img);
//				if (lateral_ccd[0]) LINFlex_TX('Y');else LINFlex_TX('N');
		if((!lateral_ccd[4])&&(!lateral_ccd[3])&&(!lateral_ccd[2])&&(!lateral_ccd[1])&&(lateral_ccd[0]))
		{
			position[Obstacle_count]=motor.distance;
			Obstacle_count++;
			f2s(position[Obstacle_count-1],P);
			LINFlex_TX(Obstacle_count+'0');
			BlueTx(P);
			BlueTx(format);
		}
		else if((lateral_ccd[4])&&(!lateral_ccd[3])&&(!lateral_ccd[2])&&(!lateral_ccd[1])&&(!lateral_ccd[0])&&Obstacle_count!=0)
		{
			position[Obstacle_count]=motor.distance;
			Obstacle_count++;
			LINFlex_TX(Obstacle_count+'0');
			f2s(position[Obstacle_count-1],P);
			BlueTx(P);
			BlueTx(format);
		}
		if (Obstacle_count==3) //说明找到一个可能的车库
		{
			Obstacle_count = 1; 
			if ((position[2]-position[1])>0.50f&&(position[2]-position[1])<80.0f)//横向车库的库长限制.
			{Reverse_flag=1;reverse_speed=0;BlueTx("JA");return;} else 	BlueTx("NEIN");
			f2s(position[2]-position[1],P);
			BlueTx(P);
			position[0] = position[2];
			position[1] = 0;
			position[2] = 0;
		}
	}
#endif
	
#if 0
	/*--------侧向红外模块检测车库-------*/
	if(!Reverse_finish)
	{
		lateral_dis_vorletzt = lateral_dis_letzt;
		lateral_dis_letzt = lateral_dis_zurzeit;
		lateral_dis_zurzeit = RAY_Module.distance_Lateral;
		if(lateral_dis_vorletzt>250.0f&&lateral_dis_letzt<250.0f&&lateral_dis_zurzeit<250.0f)
		{
			position[Obstacle_count]=motor.distance-0.158;//0.158(传动比0.15502);//红外散射角引起的误差需要标定去除
			Obstacle_count++;
			f2s(position[Obstacle_count-1],P);
			LINFlex_TX(Obstacle_count+'0');
			BlueTx(P);
			BlueTx(format);
		}
		else if(lateral_dis_vorletzt<250.0f&&lateral_dis_letzt>250.0f&&lateral_dis_zurzeit>250.0f&&Obstacle_count!=0)
		{
			position[Obstacle_count]=motor.distance-0.304;//0.304;
			Obstacle_count++;
			LINFlex_TX(Obstacle_count+'0');
			f2s(position[Obstacle_count-1],P);
			BlueTx(P);
			BlueTx(format);
		}
		if (Obstacle_count==3) //说明找到一个可能的车库
		{
			Obstacle_count = 1; 
			if ((position[2]-position[1])>0.50f&&(position[2]-position[1])<80.0f)//横向车库的库长限制.
			{Reverse_flag=1;reverse_speed=0;BlueTx("JA");return;} else 	BlueTx("NEIN");
			f2s(position[2]-position[1],P);
			BlueTx(P);
			position[0] = position[2];
			position[1] = 0;
			position[2] = 0;
		}
	}

	/*********************侧向红外2*****************************/
	lateral_dis_vorletzt[1] = lateral_dis_letzt[1];
	lateral_dis_letzt[1] = lateral_dis_zurzeit[1];
	lateral_dis_zurzeit[1] = RAY_Module.distance_Backward1;
	if(lateral_dis_vorletzt[1]>500.0f&&lateral_dis_letzt[1]<500.0f&&lateral_dis_zurzeit[1]<500.0f)
	{
		position[1][Obstacle_count]=motor.distance-0.158;//红外散射角引起的误差需要标定去除
		Obstacle_count++;
		f2s(position[1][Obstacle_count-1],P);
		LINFlex_TX(Obstacle_count+'0');
		LINFlex_TX('B');
		BlueTx(P);
		BlueTx(format);
	}
	else if(lateral_dis_vorletzt[1]<500.0f&&lateral_dis_letzt[1]>500.0f&&lateral_dis_zurzeit[1]>500.0f&&Obstacle_count!=0)
	{
		position[1][Obstacle_count]=motor.distance-0.304;
		Obstacle_count++;
		LINFlex_TX(Obstacle_count+'0');
		LINFlex_TX('B');
		f2s(position[1][Obstacle_count-1],P);
		BlueTx(P);
		BlueTx(format);
	}
	if (Obstacle_count==3) //说明找到一个可能的车库
	{
		Obstacle_count = 1; 
		if ((position[1][2]-position[1][1])>0.55f&&(position[1][2]-position[1][1])<0.9f)//横向车库的库长限制.
		{
			flag_second=1;
			BlueTx("JA_B");
			//				Reverse_flag=1;Motor__output(&motor,0);BlueTx("JA");return;
		} else 	
			BlueTx("NEIN_B");
		f2s(position[1][2]-position[1][1],P);
		BlueTx(P);
		position[1][0] = position[1][2];
		position[1][1] = 0;
		position[1][2] = 0;
	}
#endif
	if(count%2==0) //舵机的控制周期限制为20ms
	{
		if (points[6]) {wpts[0][1]=(points[0])*100+(points[1])*10+(points[2]);} 
		else if (!points[6]){wpts[0][1]=-((points[0])*100+(points[1])*10+(points[2]));}
		wpts[0][0]=(points[3])*100+(points[4])*10+(points[5]);  
		Steer__control_update(&steer,Radius,wpts);
	}
	count++;	
}
void ObstacleMode_Switchlane()
{

	/*******车道切换********
	用于切换赛道的标志位      Switch_lane_trigger = G 向左切换赛道
						 Switch_lane_trigger = g 向右切换赛道
					     Switch_lane_flag = 1  直道
	 * 					 Switch_lane_flag =	2  右弯
	 * 					 Switch_lane_flag =	3  左弯
	*********************/
	switch(Switch_lane_trigger)
	{
		case'G':
			switch(Switch_lane_flag)
			{
				case'1':
					/*******位于直道 向左切换车道*******/
					Info_Lampe.Links_Richtung_lapme_flag = 1;	//打开左转向灯
					motor.distance=0;
					Steer__output(&steer,0.113); //左转打死方向
					Motor__output(&motor,0.20);
					while (motor.distance<0.20f) {if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
					motor.distance=0;
					Steer__output(&steer,CENTER); //直行
					Motor__output(&motor,0.20);
					while (motor.distance<0.01f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
					motor.distance=0;
					Steer__output(&steer,0.068); //右转打死方向
					Motor__output(&motor,0.20);
					while (motor.distance<0.20f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
					//暂时保留，以便调试 ------直道调试通过
					Steer__output(&steer,CENTER); //直行
					Motor__output(&motor,0);
					Switch_lane_flag = 0;
					Info_Lampe.Links_Richtung_lapme_flag = 0;	//关闭左转向灯
					break;
				case'2':
					/*******位于右弯 向左切换车道*******/
					Info_Lampe.Links_Richtung_lapme_flag = 1;	//打开左转向灯
					motor.distance = 0;
					Steer__output(&steer,0.113); //左转打死方向
					Motor__output(&motor,0.20);
					while (motor.distance<0.25f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
					motor.distance = 0;
					Steer__output(&steer,0.068); //右转打死方向
					Motor__output(&motor,0.20);
					while (motor.distance<0.50f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
//					//暂时保留，以便调试	
//					Steer__output(&steer,CENTER); //直行
//					Motor__output(&motor,0);
//					delay(500);			
					
					Switch_lane_flag = 0;
					Info_Lampe.Links_Richtung_lapme_flag = 0;	//关闭左转向灯
					break;
				case'3':
					/*******位于左弯 向左切换车道*******/
					Info_Lampe.Links_Richtung_lapme_flag = 1;	//打开左转向灯
					motor.distance = 0;
					Steer__output(&steer,0.113); //右转打死方向
					Motor__output(&motor,0.20);
					while (motor.distance<0.60f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
					motor.distance = 0;
					Steer__output(&steer,CENTER); //右转打死方向
					Motor__output(&motor,0.20);
					while (motor.distance<0.10f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
//					//暂时保留，以便调试		
//					Steer__output(&steer,CENTER); 
//					Motor__output(&motor,0);
//					delay(500);	
					
					Switch_lane_flag = 0;
					Info_Lampe.Links_Richtung_lapme_flag = 0;	//关闭左转向灯
					break;
				default:
					break;
			}
			break;
		case'g':
			switch(Switch_lane_flag)
			{
				case'1':
					/*******位于直道 向右切换车道*******/
					Info_Lampe.Recht_Richtung_lampe_flag = 1;	//打开右转向灯
					motor.distance=0;
					Steer__output(&steer,0.068); //右转打死方向
					Motor__output(&motor,0.20);
					while (motor.distance<0.15f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
					motor.distance=0;
					Steer__output(&steer,CENTER); //直行
					Motor__output(&motor,0.30);
					while (motor.distance<0.01f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
					motor.distance=0;
					Steer__output(&steer,0.113); //左转打死方向
					Motor__output(&motor,0.20);
					while (motor.distance<0.20f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
//					//暂时保留，以便调试
//					Steer__output(&steer,CENTER); //直行
//					Motor__output(&motor,0);
					
					Switch_lane_flag = 0;
					Info_Lampe.Recht_Richtung_lampe_flag = 0;	//关闭右转向灯
					break;
				case'2':
					/*******位于右弯 向右切换车道*******/
					Info_Lampe.Recht_Richtung_lampe_flag = 1;	//打开右转向灯
					motor.distance = 0;
					Steer__output(&steer,0.068); //右转打死方向
					Motor__output(&motor,0.20);
					while (motor.distance<0.60f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
					motor.distance = 0;
					Steer__output(&steer,CENTER); //右转打死方向
					Motor__output(&motor,0.20);
					while (motor.distance<0.10f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
//					//暂时保留，以便调试		
//					Steer__output(&steer,CENTER); 
//					Motor__output(&motor,0);
//					delay(500);	
					
					Switch_lane_flag = 0;
					Info_Lampe.Recht_Richtung_lampe_flag = 0;	//关闭右转向灯
					break;
				case'3':
					/*******位于左弯 向右切换车道*******/
					Info_Lampe.Recht_Richtung_lampe_flag = 1;	//打开右转向灯
					motor.distance = 0;
					Steer__output(&steer,0.068); //左转打死方向
					Motor__output(&motor,0.20);
					while (motor.distance<0.25f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
					motor.distance = 0;
					Steer__output(&steer,0.113); //右转打死方向
					Motor__output(&motor,0.20);
					while (motor.distance<0.50f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
//					//暂时保留，以便调试		
//					Steer__output(&steer,CENTER); 
//					Motor__output(&motor,0);
//					delay(500);		
					
					Switch_lane_flag = 0;
					Info_Lampe.Recht_Richtung_lampe_flag = 0;	//关闭右转向灯
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
}

void ParkingMode_Reverse()
{	 
	//这是进行倒车的处理程序
	/*发现可以停的车库之后先直行*/
	float target_distance[10];
	/**************倒车长度列表*****************/
	target_distance[0] = -0.12f;//停车之后的后退
	target_distance[1] =  0.38f;//往左走一小段
	target_distance[2] = -0.30f;//第一段倒车圆弧
	target_distance[3] = -0.42f;//第二段倒车圆弧
	target_distance[4] =  0.07f;//第一次进库挣扎
	target_distance[5] = -0.06f;//第二次进库挣扎
	target_distance[6] =  0.06f;//第一次出库挣扎
	target_distance[7] = -0.07f;//第二次出库挣扎
	target_distance[8] =  0.40f;//出库第一段圆弧
	target_distance[9] =  0.35f;//出库第二段圆弧
	/******************************************/
	PIT__stop(PIT_Timer0);
	//		reverse_target_distance/=1.0f;
	//	    reverse_target_distance+=0.10;//0.264;  //m
	Steer__output(&steer,CENTER);
	reverse_speed=0;
	motor.distance=0;
	Steer__output(&steer,CENTER);
	reverse_speed=-0.75f;
	while (motor.distance>-0.01){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0;PIT__restart(PIT_Timer0); return;}};//刹车
	motor.distance=0;
	Steer__output(&steer,CENTER);
	reverse_speed=-0.8f;
	while (motor.distance>target_distance[0]){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0;PIT__restart(PIT_Timer0); return;}};//停车之后先后退 //0.23
	reverse_speed=0;
	delay_ms(200);
	motor.distance=0;
	Steer__output(&steer,0.113);
	delay_ms(100);
	reverse_speed=0.8f;
	while (motor.distance<target_distance[1]){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0;PIT__restart(PIT_Timer0); return;}};//向左侧走一段 //0.32
	reverse_speed=0;
	delay_ms(200);
	motor.distance=0;
	PID__config(&Reverse_motor_pid,0.9,0,0,20,10,0,0);
	Steer__output(&steer,0.068);
	delay_ms(100);
	reverse_speed=-0.8f;
	while (motor.distance>target_distance[2]){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0;PIT__restart(PIT_Timer0); return;}};//判断是否达到目标第一次转向倒车距离  //-0.25
	Steer__output(&steer,CENTER);
	reverse_speed=0;
	delay_ms(200);
	motor.distance=0;	
	Steer__output(&steer,0.113);
	delay_ms(100);
	while (motor.distance>target_distance[3])//0.45
	{
		if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0;PIT__restart(PIT_Timer0); return;};
		//			   if(RAY_Module.distance_Backward1<=45.0f) break;
		//			   Motor__output(&motor,-0.16);//判断是否达到目标第二次直行倒车距离
		reverse_speed=-0.8f;
	}
	Steer__output(&steer,CENTER);
	reverse_speed=0;
	delay_ms(200);
	motor.distance=0;	
	Steer__output(&steer,0.068);
	delay_ms(100);
	reverse_speed=0.8f;
	while (motor.distance<target_distance[4]){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0; PIT__restart(PIT_Timer0);return;}};//判断是否达到目标第一次调整直行倒车距离 //0.07
	Steer__output(&steer,CENTER);
	reverse_speed=0;
	delay_ms(200);
	motor.distance=0;	
	Steer__output(&steer,0.113);
	delay_ms(100);
	reverse_speed=-0.8f;
	while (motor.distance>target_distance[5])
	{
		if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0;PIT__restart(PIT_Timer0); return;}
		//			if(RAY_Module.distance_Backward1<=45.0f) break;
	}//判断是否达到目标第二次调整倒车距离 //-0.07
	reverse_speed=0;
	Steer__output(&steer,CENTER);
#if 1
	//视规则决定是否闪灯
	Info_Lampe.Recht_Richtung_lampe_flag = 1; //右转向灯
	Info_Lampe.Links_Richtung_lapme_flag = 1; //左转向灯
	Info_Lampe.Bremsen_Lampe_flag = 1;        //RC模式灯
	delay_ms(1000); 
	Info_Lampe.Recht_Richtung_lampe_flag = 0; //右转向灯
	Info_Lampe.Links_Richtung_lapme_flag = 0; //左转向灯
	Info_Lampe.Bremsen_Lampe_flag = 0;        //RC模式灯
#endif
	delay_ms(2000);
	/*******结束停车后准备出库********/
	Info_Lampe.Links_Richtung_lapme_flag = 1; //左转向灯
	motor.distance=0;
	Steer__output(&steer,0.113);
	delay_ms(100);                             
	reverse_speed=0.8f;
	while (motor.distance<target_distance[6]){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0;PIT__restart(PIT_Timer0); return;}};//第一次出库调整
	reverse_speed=0;
	delay_ms(100); 
	motor.distance=0;
	Steer__output(&steer,0.068);
	delay_ms(100);
	reverse_speed=-0.8f;
	while (motor.distance>target_distance[7]){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0;PIT__restart(PIT_Timer0); return;}};//第二次出库调整
	motor.distance=0;
	Steer__output(&steer,0.113);
	reverse_speed=0.8f;
	while (motor.distance<target_distance[8]){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0;PIT__restart(PIT_Timer0); return;}}; //0.40出库第一段
	motor.distance=0;
	Steer__output(&steer,0.068);
	reverse_speed=0.8f;
	while (motor.distance<target_distance[9]){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0;PIT__restart(PIT_Timer0); return;}};//出库第二段
	Steer__output(&steer,CENTER);
	Reverse_flag=0;
	Reverse_finish=1;
	Info_Lampe.Links_Richtung_lapme_flag = 0; //左转向灯
	/*****************************/ 
}

void ParkingMode_Vertical_Parking()
{
	float pwm_duty,distance;
	reverse_target_distance/=1.0f;
	Steer__output(&steer,CENTER);
	distance = reverse_target_distance - 0.60f;
	motor.distance=0;
	Motor__output(&motor,0.16);
	while (motor.distance<distance){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0; return;}}; //0.40
	Motor__output(&motor,0);
	distance = 0.60*3.1415926535/2.0f;
	Steer__output(&steer,0.113);
	delay_ms(200);
	motor.distance=0;
	Motor__output(&motor,0.16);
	while (motor.distance<distance){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0; return;}}; //0.40
	Steer__output(&steer,CENTER);
	motor.distance=0;
	Motor__output(&motor,0.16);
	while (motor.distance<0.33f){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0; return;}}; //0.40
	Motor__output(&motor,0);
}


