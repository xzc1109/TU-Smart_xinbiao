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
extern uint8_t FLAG;//��Ϊ����ѯ�����жϵ�һ����־λ
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
		/***********������Ƶ�����õ�������********/
		initModesAndClock(); 
		enableIrq();
		Motor__config(&motor,EMIOS_CH5,EMIOS_CH2,0.35f,0.20f,0,10,10,10,10);
		Encoder__config(&ecd,EMIOS_CH16,0.17105,512,10,0.032,17);
		Encoder__init(&ecd);
		Steer__config(&steer,EMIOS_CH4, 0,0,0.13,20,10,10,500);//����ת��
		Steer__output(&steer,CENTER);
		/*---------------------------------------*/
		
		/*-----------������ģ���ʼ��-------------*/
		/***********��ʼ��RC���ģʽ*************/
		init_RC(); //���õ�PIT_Timer3 	���ȼ�13 	 ��������Ϊ20ms
		
		/***********��ʼ�������߲��ģ��***********/
		Ray__init();//���õ�PIT_Timer4      ���ȼ�10		��������Ϊ50ms
		
		/***********��ʼ����ģʽ�����ģ��***********/
		Knopf_init();//����SIULģ���EIRQ ���ȼ�13 
		//	 EIRQ[5]---PC[2]----PCR[34]  ObstacleMode
	    //   EIRQ[6]---PC[3]----PCR[35]  ParkingMode
		
		/**********��ʼ��С���źŵ�ģ��***********/
		//С���źŵƲ���PIT_Timer5���� 	 	���ȼ�8		��������Ϊ50ms
		Information_lampe_init(&Info_Lampe,40,39,41,38);
		
		/************��ʼ������ͨ��**************/
		initLINFlex_0_UART(12);
		
		/*************��ʼ������CCD****************/
		ccd_init();//����PIT_Tiemr2         ���ȼ�12            ����Ϊ10ms
		
	
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
	static float last_speed;  //��һ�γ��٣�����ɲ�����
	static uint8_t Bremsen_count,Accelerate_count;
	static float position[3];
	static uint8_t count;
	float wpts[1][2];
	
	PIT__clear_flag(PIT_Timer1);//����жϱ�־λ
	
	/*******����Ŀǰ���ĳ�����Ϣ**********/
	Speed__bekommen(&ecd);
	if(Dir__bekommen(&ecd)) 
		ecd._speed = -ecd._speed;
	motor.actual_speed = ecd._speed;
	/********���С����ɲ��״̬*********/
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
	if(-0.15f<ecd._speed<0.15f)	Info_Lampe.Bremsen_Lampe_flag = 0;//+-0.15f Ϊͣ���ٶ�����   ��ͣ������ɲ������
	last_speed = ecd._speed;
	/************************************/ //���ĿǰС���Ƿ���RCģʽ
	if(RC__flag)
	{
		Info_Lampe.RC_Lampe_flag = 1;
		return;//��⵽Ŀǰ����RC����״̬
	}else
		Info_Lampe.RC_Lampe_flag = 0;
	/***********************************/  //����С�����н�������Ϣ
	motor.distance += ecd._speed * 0.01;
	if(Switch_lane_flag) return;
	if(count%2==0) //����Ŀ�����������Ϊ20ms
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
	static float Ray_dis_last;//����ģ����ϴξ����¼ֵ
	static float last_speed;  //��һ�γ��٣�����ɲ�����
	static uint8_t Bremsen_count,Accelerate_count;
	static uint8_t Obstacle_count;//�ϰ����Ե�ļ���,����ȷ������
	static float position[3];
	static uint8_t flag_first=0,flag_second=0;
	static uint8_t count;
	static float lateral_dis_vorletzt,lateral_dis_letzt,lateral_dis_zurzeit;//���ڼ�¼�������ģ��Ĳ������
	static float lateral_ccd[5];//���ڼ�¼CCDģ��Ĳ������
	float wpts[1][2];
	char P[8];
	char format[4] = "  "; 
	PIT__clear_flag(PIT_Timer1);//����жϱ�־λ
	
	/*******����Ŀǰ���ĳ�����Ϣ**********/
	Speed__bekommen(&ecd);
	if(Dir__bekommen(&ecd)) 
		ecd._speed = -ecd._speed;
	motor.actual_speed = ecd._speed;
	/********���С����ɲ��״̬*********/
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
	if(-0.10f<ecd._speed<0.10f)	Info_Lampe.Bremsen_Lampe_flag = 0;//��ͣ������ɲ������
	last_speed = ecd._speed;
	/************************************/ //���ĿǰС���Ƿ���RCģʽ
	if(RC__flag)
	{
		Info_Lampe.RC_Lampe_flag = 1;
		return;//��⵽Ŀǰ����RC����״̬
	}else
		Info_Lampe.RC_Lampe_flag = 0;
	/***********************************/  //����С�����н�������Ϣ
	motor.distance += ecd._speed * 0.01;
	/***********************************/
	Motor__control_update(&motor,reverse_speed); //
	if(Reverse_flag)	return;
	
	/*--------����CCDģ���⳵��-------*/
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
		if (Obstacle_count==3) //˵���ҵ�һ�����ܵĳ���
		{
			Obstacle_count = 1; 
			if ((position[2]-position[1])>0.50f&&(position[2]-position[1])<80.0f)//���򳵿�Ŀⳤ����.
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
	/*--------�������ģ���⳵��-------*/
	if(!Reverse_finish)
	{
		lateral_dis_vorletzt = lateral_dis_letzt;
		lateral_dis_letzt = lateral_dis_zurzeit;
		lateral_dis_zurzeit = RAY_Module.distance_Lateral;
		if(lateral_dis_vorletzt>250.0f&&lateral_dis_letzt<250.0f&&lateral_dis_zurzeit<250.0f)
		{
			position[Obstacle_count]=motor.distance-0.158;//0.158(������0.15502);//����ɢ�������������Ҫ�궨ȥ��
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
		if (Obstacle_count==3) //˵���ҵ�һ�����ܵĳ���
		{
			Obstacle_count = 1; 
			if ((position[2]-position[1])>0.50f&&(position[2]-position[1])<80.0f)//���򳵿�Ŀⳤ����.
			{Reverse_flag=1;reverse_speed=0;BlueTx("JA");return;} else 	BlueTx("NEIN");
			f2s(position[2]-position[1],P);
			BlueTx(P);
			position[0] = position[2];
			position[1] = 0;
			position[2] = 0;
		}
	}

	/*********************�������2*****************************/
	lateral_dis_vorletzt[1] = lateral_dis_letzt[1];
	lateral_dis_letzt[1] = lateral_dis_zurzeit[1];
	lateral_dis_zurzeit[1] = RAY_Module.distance_Backward1;
	if(lateral_dis_vorletzt[1]>500.0f&&lateral_dis_letzt[1]<500.0f&&lateral_dis_zurzeit[1]<500.0f)
	{
		position[1][Obstacle_count]=motor.distance-0.158;//����ɢ�������������Ҫ�궨ȥ��
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
	if (Obstacle_count==3) //˵���ҵ�һ�����ܵĳ���
	{
		Obstacle_count = 1; 
		if ((position[1][2]-position[1][1])>0.55f&&(position[1][2]-position[1][1])<0.9f)//���򳵿�Ŀⳤ����.
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
	if(count%2==0) //����Ŀ�����������Ϊ20ms
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

	/*******�����л�********
	�����л������ı�־λ      Switch_lane_trigger = G �����л�����
						 Switch_lane_trigger = g �����л�����
					     Switch_lane_flag = 1  ֱ��
	 * 					 Switch_lane_flag =	2  ����
	 * 					 Switch_lane_flag =	3  ����
	*********************/
	switch(Switch_lane_trigger)
	{
		case'G':
			switch(Switch_lane_flag)
			{
				case'1':
					/*******λ��ֱ�� �����л�����*******/
					Info_Lampe.Links_Richtung_lapme_flag = 1;	//����ת���
					motor.distance=0;
					Steer__output(&steer,0.113); //��ת��������
					Motor__output(&motor,0.20);
					while (motor.distance<0.20f) {if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
					motor.distance=0;
					Steer__output(&steer,CENTER); //ֱ��
					Motor__output(&motor,0.20);
					while (motor.distance<0.01f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
					motor.distance=0;
					Steer__output(&steer,0.068); //��ת��������
					Motor__output(&motor,0.20);
					while (motor.distance<0.20f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
					//��ʱ�������Ա���� ------ֱ������ͨ��
					Steer__output(&steer,CENTER); //ֱ��
					Motor__output(&motor,0);
					Switch_lane_flag = 0;
					Info_Lampe.Links_Richtung_lapme_flag = 0;	//�ر���ת���
					break;
				case'2':
					/*******λ������ �����л�����*******/
					Info_Lampe.Links_Richtung_lapme_flag = 1;	//����ת���
					motor.distance = 0;
					Steer__output(&steer,0.113); //��ת��������
					Motor__output(&motor,0.20);
					while (motor.distance<0.25f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
					motor.distance = 0;
					Steer__output(&steer,0.068); //��ת��������
					Motor__output(&motor,0.20);
					while (motor.distance<0.50f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
//					//��ʱ�������Ա����	
//					Steer__output(&steer,CENTER); //ֱ��
//					Motor__output(&motor,0);
//					delay(500);			
					
					Switch_lane_flag = 0;
					Info_Lampe.Links_Richtung_lapme_flag = 0;	//�ر���ת���
					break;
				case'3':
					/*******λ������ �����л�����*******/
					Info_Lampe.Links_Richtung_lapme_flag = 1;	//����ת���
					motor.distance = 0;
					Steer__output(&steer,0.113); //��ת��������
					Motor__output(&motor,0.20);
					while (motor.distance<0.60f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
					motor.distance = 0;
					Steer__output(&steer,CENTER); //��ת��������
					Motor__output(&motor,0.20);
					while (motor.distance<0.10f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
//					//��ʱ�������Ա����		
//					Steer__output(&steer,CENTER); 
//					Motor__output(&motor,0);
//					delay(500);	
					
					Switch_lane_flag = 0;
					Info_Lampe.Links_Richtung_lapme_flag = 0;	//�ر���ת���
					break;
				default:
					break;
			}
			break;
		case'g':
			switch(Switch_lane_flag)
			{
				case'1':
					/*******λ��ֱ�� �����л�����*******/
					Info_Lampe.Recht_Richtung_lampe_flag = 1;	//����ת���
					motor.distance=0;
					Steer__output(&steer,0.068); //��ת��������
					Motor__output(&motor,0.20);
					while (motor.distance<0.15f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
					motor.distance=0;
					Steer__output(&steer,CENTER); //ֱ��
					Motor__output(&motor,0.30);
					while (motor.distance<0.01f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
					motor.distance=0;
					Steer__output(&steer,0.113); //��ת��������
					Motor__output(&motor,0.20);
					while (motor.distance<0.20f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
//					//��ʱ�������Ա����
//					Steer__output(&steer,CENTER); //ֱ��
//					Motor__output(&motor,0);
					
					Switch_lane_flag = 0;
					Info_Lampe.Recht_Richtung_lampe_flag = 0;	//�ر���ת���
					break;
				case'2':
					/*******λ������ �����л�����*******/
					Info_Lampe.Recht_Richtung_lampe_flag = 1;	//����ת���
					motor.distance = 0;
					Steer__output(&steer,0.068); //��ת��������
					Motor__output(&motor,0.20);
					while (motor.distance<0.60f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
					motor.distance = 0;
					Steer__output(&steer,CENTER); //��ת��������
					Motor__output(&motor,0.20);
					while (motor.distance<0.10f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
//					//��ʱ�������Ա����		
//					Steer__output(&steer,CENTER); 
//					Motor__output(&motor,0);
//					delay(500);	
					
					Switch_lane_flag = 0;
					Info_Lampe.Recht_Richtung_lampe_flag = 0;	//�ر���ת���
					break;
				case'3':
					/*******λ������ �����л�����*******/
					Info_Lampe.Recht_Richtung_lampe_flag = 1;	//����ת���
					motor.distance = 0;
					Steer__output(&steer,0.068); //��ת��������
					Motor__output(&motor,0.20);
					while (motor.distance<0.25f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
					motor.distance = 0;
					Steer__output(&steer,0.113); //��ת��������
					Motor__output(&motor,0.20);
					while (motor.distance<0.50f){if(RC__flag) {Info_Lampe.Links_Richtung_lapme_flag = 0;Info_Lampe.Recht_Richtung_lampe_flag = 0;Switch_lane_flag = 0;return;}}
//					//��ʱ�������Ա����		
//					Steer__output(&steer,CENTER); 
//					Motor__output(&motor,0);
//					delay(500);		
					
					Switch_lane_flag = 0;
					Info_Lampe.Recht_Richtung_lampe_flag = 0;	//�ر���ת���
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
	//���ǽ��е����Ĵ������
	/*���ֿ���ͣ�ĳ���֮����ֱ��*/
	float target_distance[10];
	/**************���������б�*****************/
	target_distance[0] = -0.12f;//ͣ��֮��ĺ���
	target_distance[1] =  0.38f;//������һС��
	target_distance[2] = -0.30f;//��һ�ε���Բ��
	target_distance[3] = -0.42f;//�ڶ��ε���Բ��
	target_distance[4] =  0.07f;//��һ�ν�������
	target_distance[5] = -0.06f;//�ڶ��ν�������
	target_distance[6] =  0.06f;//��һ�γ�������
	target_distance[7] = -0.07f;//�ڶ��γ�������
	target_distance[8] =  0.40f;//�����һ��Բ��
	target_distance[9] =  0.35f;//����ڶ���Բ��
	/******************************************/
	PIT__stop(PIT_Timer0);
	//		reverse_target_distance/=1.0f;
	//	    reverse_target_distance+=0.10;//0.264;  //m
	Steer__output(&steer,CENTER);
	reverse_speed=0;
	motor.distance=0;
	Steer__output(&steer,CENTER);
	reverse_speed=-0.75f;
	while (motor.distance>-0.01){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0;PIT__restart(PIT_Timer0); return;}};//ɲ��
	motor.distance=0;
	Steer__output(&steer,CENTER);
	reverse_speed=-0.8f;
	while (motor.distance>target_distance[0]){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0;PIT__restart(PIT_Timer0); return;}};//ͣ��֮���Ⱥ��� //0.23
	reverse_speed=0;
	delay_ms(200);
	motor.distance=0;
	Steer__output(&steer,0.113);
	delay_ms(100);
	reverse_speed=0.8f;
	while (motor.distance<target_distance[1]){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0;PIT__restart(PIT_Timer0); return;}};//�������һ�� //0.32
	reverse_speed=0;
	delay_ms(200);
	motor.distance=0;
	PID__config(&Reverse_motor_pid,0.9,0,0,20,10,0,0);
	Steer__output(&steer,0.068);
	delay_ms(100);
	reverse_speed=-0.8f;
	while (motor.distance>target_distance[2]){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0;PIT__restart(PIT_Timer0); return;}};//�ж��Ƿ�ﵽĿ���һ��ת�򵹳�����  //-0.25
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
		//			   Motor__output(&motor,-0.16);//�ж��Ƿ�ﵽĿ��ڶ���ֱ�е�������
		reverse_speed=-0.8f;
	}
	Steer__output(&steer,CENTER);
	reverse_speed=0;
	delay_ms(200);
	motor.distance=0;	
	Steer__output(&steer,0.068);
	delay_ms(100);
	reverse_speed=0.8f;
	while (motor.distance<target_distance[4]){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0; PIT__restart(PIT_Timer0);return;}};//�ж��Ƿ�ﵽĿ���һ�ε���ֱ�е������� //0.07
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
	}//�ж��Ƿ�ﵽĿ��ڶ��ε����������� //-0.07
	reverse_speed=0;
	Steer__output(&steer,CENTER);
#if 1
	//�ӹ�������Ƿ�����
	Info_Lampe.Recht_Richtung_lampe_flag = 1; //��ת���
	Info_Lampe.Links_Richtung_lapme_flag = 1; //��ת���
	Info_Lampe.Bremsen_Lampe_flag = 1;        //RCģʽ��
	delay_ms(1000); 
	Info_Lampe.Recht_Richtung_lampe_flag = 0; //��ת���
	Info_Lampe.Links_Richtung_lapme_flag = 0; //��ת���
	Info_Lampe.Bremsen_Lampe_flag = 0;        //RCģʽ��
#endif
	delay_ms(2000);
	/*******����ͣ����׼������********/
	Info_Lampe.Links_Richtung_lapme_flag = 1; //��ת���
	motor.distance=0;
	Steer__output(&steer,0.113);
	delay_ms(100);                             
	reverse_speed=0.8f;
	while (motor.distance<target_distance[6]){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0;PIT__restart(PIT_Timer0); return;}};//��һ�γ������
	reverse_speed=0;
	delay_ms(100); 
	motor.distance=0;
	Steer__output(&steer,0.068);
	delay_ms(100);
	reverse_speed=-0.8f;
	while (motor.distance>target_distance[7]){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0;PIT__restart(PIT_Timer0); return;}};//�ڶ��γ������
	motor.distance=0;
	Steer__output(&steer,0.113);
	reverse_speed=0.8f;
	while (motor.distance<target_distance[8]){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0;PIT__restart(PIT_Timer0); return;}}; //0.40�����һ��
	motor.distance=0;
	Steer__output(&steer,0.068);
	reverse_speed=0.8f;
	while (motor.distance<target_distance[9]){if(RC__flag) {Reverse_flag = 0; Reverse_finish = 0;PIT__restart(PIT_Timer0); return;}};//����ڶ���
	Steer__output(&steer,CENTER);
	Reverse_flag=0;
	Reverse_finish=1;
	Info_Lampe.Links_Richtung_lapme_flag = 0; //��ת���
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


