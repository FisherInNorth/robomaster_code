#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "pid.h"
#include "BSP_Motor.h"

typedef struct
{
	float linear_vel;			//���ٶ�
	float rpm;						//ת��Ȧÿ����
}CHASSIS_SPEED_t;

typedef struct
{
	CHASSIS_SPEED_t target_speed;			
	CHASSIS_SPEED_t actual_speed;						
}CHASSIS_WHEEL_t;

//typedef struct
//{
//	Wheel_t wheel1;
//	Wheel_t wheel2;
//	Wheel_t wheel3;
//	Wheel_t wheel4;
//}Kinematics_t;

#define wheel_diameter  10.000000f			//����ֱ��
#define half_width  25.000000f		//���̰��
#define half_length  35.000000f		//���̰볤
#define MAX_BASE_LINEAR_SPEED    120.817f    //�������ƽ���ٶȣ���λcm/s   
#define MAX_BASE_ROTATIONAL_SPEED    7.260570f  //7.260570f    //���������ת�ٶȣ���λrad/s    
#define NORMAL_LINEAR_SPEED    70.0f
#define NORMAL_ROTATIONAL_SPEED  0.5f

#define MAX_MOTOR_SPEED   15336				//������ת�٣��궨�巽���޸�   ��Χ0 - 10000   15336   
#define MAX_BASE_LINEAR_SPEED    120.817f    //�������ƽ���ٶȣ���λcm/s   
#define MAX_BASE_ROTATIONAL_SPEED    7.260570f  //7.260570f    //���������ת�ٶȣ���λrad/s    
#define NORMAL_LINEAR_SPEED    70.0f
#define NORMAL_ROTATIONAL_SPEED  0.5f

void Chassis_Init(void);


#endif

