#ifndef __MOTOR_H
#define __MOTOR_H

#include "pid.h"
#include "kinematics.h"
#include "can.h"
#include "referee.h"
#include <math.h>

#define CHASSIS_NO_FORCE   3
#define CHASSIS_FOLLOW   	 1
#define CHASSIS_NORMAL     0
#define CHASSIS_SPIN       2

#define CLOCKWISE          1
#define ANTICLOCKWISE      2

#define GIMBAL_HEAD_ANGLE  161   //161  342

#define POWER_LIMIT         80.0f
#define WARNING_POWER       40.0f   
#define WARNING_POWER_BUFF  50.0f 
#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, 
//#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
//#define POWER_TOTAL_CURRENT_LIMIT       20000.0f

#define follow_pid_realize() \
				do{ \
						switch_flag=FOLLOW;      \
						pid_realize(&(chassis_center.pid));   \
					  switch_flag=NUL;  \
				}while(0)
				
#define vpid_chassis_realize() \
				do{ \
						switch_flag=CHASSIS;      \
						pid_realize(&(chassis_motor1.pid));   \
						pid_realize(&(chassis_motor2.pid));   \
						pid_realize(&(chassis_motor3.pid));   \
						pid_realize(&(chassis_motor4.pid));   \
					  switch_flag=NUL;  \
				}while(0)
				
//#define chassis_spin(vx,vy) \
//				do{ \
//					float theta;       \
//					theta=Get_chassis_theta();   \
//					vx = (float)(chassis_control_order.vy_set*sin(theta) + chassis_control_order.vx_set*cos(theta));    \
//					vy = (float)(chassis_control_order.vy_set*cos(theta) - chassis_control_order.vx_set*sin(theta));    \
//				}while(0)

			
typedef struct{
	
	float start_angle;			//�����ʼ�Ƕ�ֵ
	int start_angle_flag;	//��¼�����ʼ�Ƕ�ֵ��flag
	int switch_mode_flag;  //��¼ģʽת���Ƕ�ֵ��flag
	int stop_angle;				//����ֹͣ����ʱ��ĽǶ�ֵ
	float target_angle;
	
	float actual_angle;			//��ǰ��ʵ�Ƕ�ֵ
	float last_angle;				//��һ�η��صĽǶ�ֵ
	float switch_mode_angle;  //��¼ģʽת���Ƕ�ֵ
	int round_cnt;				//��Կ���ʱת����Ȧ��
	int total_angle;			//�ܹ�ת���ļ���
	
	float actual_speed;			//�����ʵ�ٶ�,rpm
	int target_speed;			//���Ŀ���ٶ�,rpm  ת/min
	int last_speed;       //�����һ�λش����ٶ�ֵ
	int actual_current;		//�����ʵ����
	int target_current;		//���Ŀ�����
	//int temp;							//����¶ȣ�2006�����֧�֣�3508֧�֣�
	PID_t pid;
	uint8_t spin_dirt;
}MOTOR_t;

typedef struct
{
	int16_t vx_set;
	int16_t vy_set;
	int16_t wz_set;
	uint32_t chassis_mode;
	uint32_t last_chassis_mode;
	uint16_t gimbal_6020_angle;
	uint16_t last_gimbal_6020_angle;
}CHASSIS_CONTROL_ORDER_t;

typedef struct
{
	int16_t real_vx;
	int16_t real_vy;
}REAl_CHASSIS_SPEED_t;


extern MOTOR_t chassis_motor1,chassis_motor2,chassis_motor3,chassis_motor4,chassis_center;
extern CHASSIS_CONTROL_ORDER_t chassis_control_order;
extern POWER_PID_t p_pid;
extern BUFFER_PID_t b_pid;
extern uint8_t fly_flag;
void chassis_move(void);
#endif

