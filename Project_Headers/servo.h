/**
 * @file servo.h
 * @date Apr 21, 2017
 * @author 黄禹尧
 * 
 * @brief 
 */

#ifndef SERVO_H_
#define SERVO_H_

#include "MPC5604C.h"
#include <math.h>
#include "encoder.h"


typedef struct {
	EMIOSn_CH ch;
	/**
	 * 基于舵机转向实验数据的线性拟合模型。
	 * 表征了PWM占空比与舵机转角之前的转化关系：
	 *  angle(rad) = c1 * pwm + c2;
	 * 
	 * ======= MATLAB拟合代码========
	 *  %% 舵机转向实验数据
	 *  % left_wheel_angle，right_wheel_angle 为左右轮在不同舵机PWM占空比下的转角（左正右负，单位deg）
	 *  % steer_pwm_duty 舵机PWM测试占空比
	 *  load('steer_experiment_data.mat');
	 *  left_wheel_angle = left_wheel_angle / 180 * pi;
	 *  right_wheel_angle = right_wheel_angle / 180 * pi;
	 *  %% 直接对平均车轮转角进行拟合
	 *  servo_model = polyfit(steer_pwm_duty,(left_wheel_angle+right_wheel_angle)/2,1);
	 *  plot(steer_pwm_duty, (left_wheel_angle+right_wheel_angle)/2,...
	 *      steer_pwm_duty, polyval(servo_model, steer_pwm_duty));
	 * ===== MATLAB拟合代码结束========
	 * 
	 * 德国赛的C1 = 17.1700
	 * 德国赛的C2 = -1.5422
	 */
	struct {float c1, c2;} servo_mdl;
	float _angle;
}servo_t;

// 单例模式：舵机

/**
 * @brief 初始化转向舵机模块
 */
void servo__config(EMIOSn_CH ch, float servo_c1, float servo_c2);

/**
 * @brief 舵机动作函数
 * @param angle 舵机目标转角，向左为正，单位为弧度
 * @note 防止舵机堵转损坏，限制最大前轮平均打角为 +- 38 deg, 约 0.66 rad
 */
void servo__turn_to(float angle);

#endif /* SERVO_H_ */
