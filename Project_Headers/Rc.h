/*
 * Rc.h
 *
 *  Created on: Nov 16, 2017
 *      Author: Sanqing Qu
 */

#ifndef RC_H_
#define RC_H_

#include "MPC5604C.h"
#include "math.h"
#include "gpio.h"
#include "all.h"


/*这里采用INPWM读取遥控器发射的PWM脉冲的宽度，达到软件限副的作用*/

/*RC模式的配置应在配置完电机和舵机之后*/
typedef struct
{
	/*注意RC模式的输入通道选用*/
	EMIOSn_CH motor_ch;
	EMIOSn_CH steer_ch;
	EMIOSn_CH RC_motor_inch;
	EMIOSn_CH RC_steer_inch;
	EMIOSn_CH mode_switch;
	uint32_t motor_duty;
	uint32_t steer_duty;
	uint32_t mode_duty;
}Rem_con_t,*Rem_con;

void RC__config(Rem_con contro,EMIOSn_CH motor_ch,EMIOSn_CH steer_ch,EMIOSn_CH RC_motor_inch,EMIOSn_CH RC_steer_inch,EMIOSn_CH mode_switch);
void RC__init(Rem_con contro,INTCInterruptFn func);
void func_RC(void);
void init_RC(void);
//void RC_check();

#endif /* RC_H_ */
