/**
 * @file servo.h
 * @date Apr 21, 2017
 * @author ����Ң
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
	 * ���ڶ��ת��ʵ�����ݵ��������ģ�͡�
	 * ������PWMռ�ձ�����ת��֮ǰ��ת����ϵ��
	 *  angle(rad) = c1 * pwm + c2;
	 * 
	 * ======= MATLAB��ϴ���========
	 *  %% ���ת��ʵ������
	 *  % left_wheel_angle��right_wheel_angle Ϊ�������ڲ�ͬ���PWMռ�ձ��µ�ת�ǣ������Ҹ�����λdeg��
	 *  % steer_pwm_duty ���PWM����ռ�ձ�
	 *  load('steer_experiment_data.mat');
	 *  left_wheel_angle = left_wheel_angle / 180 * pi;
	 *  right_wheel_angle = right_wheel_angle / 180 * pi;
	 *  %% ֱ�Ӷ�ƽ������ת�ǽ������
	 *  servo_model = polyfit(steer_pwm_duty,(left_wheel_angle+right_wheel_angle)/2,1);
	 *  plot(steer_pwm_duty, (left_wheel_angle+right_wheel_angle)/2,...
	 *      steer_pwm_duty, polyval(servo_model, steer_pwm_duty));
	 * ===== MATLAB��ϴ������========
	 * 
	 * �¹�����C1 = 17.1700
	 * �¹�����C2 = -1.5422
	 */
	struct {float c1, c2;} servo_mdl;
	float _angle;
}servo_t;

// ����ģʽ�����

/**
 * @brief ��ʼ��ת����ģ��
 */
void servo__config(EMIOSn_CH ch, float servo_c1, float servo_c2);

/**
 * @brief �����������
 * @param angle ���Ŀ��ת�ǣ�����Ϊ������λΪ����
 * @note ��ֹ�����ת�𻵣��������ǰ��ƽ�����Ϊ +- 38 deg, Լ 0.66 rad
 */
void servo__turn_to(float angle);

#endif /* SERVO_H_ */
