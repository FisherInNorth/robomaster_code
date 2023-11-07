#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stdint.h"
#include "stm32f4xx.h"

//can stdid
#define CAN_LoopBack_ID				 0x200		//���ڻ���ģʽ�Լ����
#define	CAN_3508Motor1_ID  	   0x201	//0x20+���ID 
#define	CAN_3508Motor2_ID      0x202	//1-4�ֱ�Ϊ���� 
#define	CAN_3508Motor3_ID      0x203
#define	CAN_3508Motor4_ID      0x204
#define CAN_3508Motor5_ID      0x205	//5��6Ϊ��Ԯצ 
#define CAN_3508Motor6_ID      0x206
#define CAN_3508Motor7_ID      0x207	//����Ǿ�Ԯ�� 

//���ת��pid�����ṹ��
typedef struct{
	int err;
	int last_err;
	int err_integration;
	int target_speed;
	int actual_speed;
	int P_OUT;
	int I_OUT;
	int D_OUT;
	int PID_OUT;
	float kp,ki,kd;
}VPID_t;

//�����е�ǶȲ���
typedef struct{
	int err;
	int last_err;
	int err_integration;
	int actual_angle;
	int target_angle;
	int P_OUT;
	int I_OUT;
	int D_OUT;
	int PID_OUT;
  int total_angle;	
}APID_t;


//��������ṹ��
typedef struct{
	int start_angle;			//�����ʼ�Ƕ�ֵ
	int start_angle_flag;	//��¼�����ʼ�Ƕ�ֵ��flag
	int stop_angle;				//����ֹͣ����ʱ��ĽǶ�ֵ
	
	int last_angle;				//��һ�η��صĽǶ�ֵ
	int round_cnt;				//��Կ���ʱת����Ȧ��
	
	int actual_current;		//�����ʵ����
	int target_current;		//���Ŀ�����
	int integralSeparation;// �����ֱ��� 
	int integral_max;  //����޷� 
	int vPID_max;  //������ֵ 
	int aPID_OUT_MAX;		//�������ٶ�
	VPID_t vpid;
	APID_t apid;
	
	int stop_flag;
	int count;
	int pwm_num;
}MOTOR_t;

//�������ת��������ת�Ľṹ��
typedef enum{
	out,
	in,
	forward,
	back,
	right,
	left,
	down,
	up,
	stop,
}MOTOR_MOVE_t;

#define PI 			3.141593f
#define RPM2RAD 0.104720f										//ת��ת���ٶ�		1 rpm = 2pi/60 rad/s 
#define RPM2VEL 0.523599f										//ת��ת���ٶ�		vel = rpn*pi*D/60  cm/s
#define VEL2RPM 1.909859f										//���ٶ�תת��
#define M2006_REDUCTION_RATIO 36.000000f		//��������ٱ�
#define M3508_REDUCTION_RATIO 19.000000f		//��������ٱ�


#endif
