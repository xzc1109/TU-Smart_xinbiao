/*
 * pid.c
 *
 *  Created on: Oct 29, 2017
 *      Author: Sanqing Qu
 */
#include"MPC5604C.h"
#include"pid.h"

void PID__config(pid_object_t pid, float kp, float ki, float kd,
		uint32_t period_ms, float perror_max, float ierror_max,
		float derror_max) {
	
	pid->period_sec = period_ms / 1000.0f;
	
	pid->Kp = kp;
	pid->perror = 0.0f;
	pid->perror_max = perror_max;

	pid->Ki = ki * pid->period_sec;
	pid->ierror = 0.0f;
	pid->ierror_max = ierror_max;

	pid->Kd = kd / pid->period_sec;
	pid->derror = 0.0f;
	pid->derror_max = derror_max;
	
	pid->_output = 0.0f;
}

float PID__update(pid_object_t pid, float target, float actual) {

	float perror;
	float derror;
	
	perror = target - actual;
	perror = __min__(perror, pid->perror_max);
	
	derror = perror - pid->perror; //΢��
	if(fabs(derror) > pid->derror_max) //΢���޷�
	{
		pid->derror = pid->derror * 0.5 + derror * 0.5; //��ͨ
	}else{
		pid->derror = derror;
	}

	pid->ierror += perror; //����
	pid->ierror = __min__(pid->ierror, pid->ierror_max); //���ֿ�����

	pid->perror = perror;

	return pid->_output = pid->Kp * perror + pid->Ki * pid->ierror + pid->Kd * pid->derror;
}
