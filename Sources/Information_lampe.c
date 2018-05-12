/*
 * Information_lampe.c
 *
 *  Created on: Jan 23, 2018
 *      Author: SanqingQu
 */

#include "Information_lampe.h"
#include "pit.h"
Lampe Info_Lampe;
#define Test_flag 1
void Information_lampe_control()
{
	/* Information_lampe_control 周期为50ms 
	 * RC模式灯的频率为     1HZ 	duty = 0.5;
	 * 左右转向灯的频率为  2Hz 	duty = 0.5;
	 * 输出低电平处于工作模式 输出高电平处于关闭模式
	 * 刹车灯一旦触发，至少持续0.1s
	 * 左右转向灯，RC模式灯一旦触发则立即点亮，然后按着要求频率进行闪烁
	 */
	static int i;
	static uint8_t  count_bremsen,start_links,start_recht,start_rc;
	if (Info_Lampe.Bremsen_Lampe_flag) 
		{GPIO__output_low(Info_Lampe.Bremsen_Lampe);count_bremsen = 1;}
	else
		{	if(count_bremsen != 0)
				{GPIO__output_low(Info_Lampe.Bremsen_Lampe); count_bremsen--;}
			else
				{GPIO__output_high(Info_Lampe.Bremsen_Lampe);count_bremsen = 0;}
		}
	if (Info_Lampe.Links_Richtung_lapme_flag)
		{	if(start_links == 0)GPIO__output_low(Info_Lampe.Links_Richtung_lapme);start_links = 1;
			if(i%5== 0)
			GPIO__output_toggle(Info_Lampe.Links_Richtung_lapme);
		}
	else
		{
			GPIO__output_high(Info_Lampe.Links_Richtung_lapme);
			start_links = 0;
		}	
	if(Info_Lampe.Recht_Richtung_lampe_flag)
		{	if(start_recht==0)GPIO__output_low(Info_Lampe.Recht_Richtung_lampe); start_recht = 1;
			if(i%5 == 0)
				GPIO__output_toggle(Info_Lampe.Recht_Richtung_lampe);
		}
	else
		{
			GPIO__output_high(Info_Lampe.Recht_Richtung_lampe);
			start_recht = 0;
		}
		
	if(Info_Lampe.RC_Lampe_flag)
		{
			if(start_rc == 0)GPIO__output_low(Info_Lampe.RC_Lampe); start_rc = 1;
			if(i%10 == 0)
				GPIO__output_toggle(Info_Lampe.RC_Lampe);
		}
	else
		{
			GPIO__output_high(Info_Lampe.RC_Lampe);
			start_rc = 0;
		}
	i++;
	PIT__clear_flag(PIT_Timer5);
}
void Information_lampe_init(Lampe_t lampe,uint8_t pad_Links_lampe,uint8_t pad_Recht_lampe,uint8_t pad_Bremsen_lampe,uint8_t pad_RC_lampe)
{
	lampe->Bremsen_Lampe = pad_Bremsen_lampe;
	lampe->Links_Richtung_lapme = pad_Links_lampe;
	lampe->Recht_Richtung_lampe = pad_Recht_lampe;
	lampe->RC_Lampe = pad_RC_lampe;
	GPIO__output__enable(lampe->Bremsen_Lampe);
	GPIO__output__enable(lampe->Links_Richtung_lapme);
	GPIO__output__enable(lampe->Recht_Richtung_lampe);
	GPIO__output__enable(lampe->RC_Lampe);
	PIT__config(PIT_Timer5,50,64,Information_lampe_control,12);
}

