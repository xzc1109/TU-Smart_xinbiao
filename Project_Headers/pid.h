/*
 * pid.h
 *
 *  Created on: Oct 29, 2017
 *      Author: Sanqing Qu
 */
#ifndef PID_H_
#define PID_H_

#include"MPC5604C.h"
#include"math.h"
typedef struct {
	float Kp, perror, perror_max;
	float Ki, ierror, ierror_max;
	float Kd, derror, derror_max;
	float period_sec;
	float _output;
} pid_t, *pid_object_t;

/**
 * @param perror_max ����ƫ��������ֵ
 * @param ierror_max ���ֿ�������ֵ
 * @param derror_max ΢�ַ�ֵ����
 * @param period_ms ��������
 */
void PID__config(pid_object_t pid, float kp, float ki, float kd,
		uint32_t period_ms, float perror_max, float ierror_max,
		float derror_max);
/**
 * ����Ӧ��PIT�����ж��е��ô˺������Ե������¿������е�����
 * @param actual �ⲿ����������ĸ�����ֵ����Ŀ��ֵͬ���٣�
 */
float PID__update(pid_object_t pid, float target, float actual);

#define PID__get_output(pid) ((pid)->_output)
#define PID__update_kp(pid, kp) ((pid)->Kp = kp)
#define PID__update_ki(pid, ki) ((pid)->Ki = ki)
#define PID__update_kd(pid, kd) ((pid)->Kd = kd)
#define PID__update_params(pid, kp, ki, kd) ((pid)->Kp = kp, (pid)->Ki = ki, (pid)->Kd = kd)
#define __min__(a,b) ((fabs((a))<(b)) ? (a) : (((a)>0?1.0f:-1.0f)*(b)))
#endif /* PID_H_ */
