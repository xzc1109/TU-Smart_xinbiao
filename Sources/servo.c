/**
 * @file servo.c
 * @date Apr 24, 2017
 * @author 黄禹尧
 * 
 * @brief 
 */

#include "servo.h"

static servo_t my_servo;// 单例模式：舵机实例

#define servo__calc_pwm(angle) ((angle - my_servo.servo_mdl.c2) / my_servo.servo_mdl.c1) /// 已知目标打角，反求占空比

void servo__config(EMIOSn_CH ch, float servo_c1, float servo_c2){
	my_servo.ch = ch;
	my_servo.servo_mdl.c1 = servo_c1;
	my_servo.servo_mdl.c2 = servo_c2;
//	if(PWM_config(ch, servo__calc_pwm(0.0f), 50, 1, 1)==0) return;
//	if(PWM_config(ch, servo__calc_pwm(0.0f), 50, 0, 1)==0) return;
//	if(PWM_config(ch, servo__calc_pwm(0.0f), 50, 0, 0)==0) return;
	while(1){}
}

void servo__turn_to(float angle){
	if (fabs(angle) > 0.66f) angle = (angle>0?1:-1) * 0.66f; 
//	PWM_duty(my_servo.ch, servo__calc_pwm(angle));
}
