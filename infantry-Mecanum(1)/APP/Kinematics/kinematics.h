#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include "chassis_move.h"
#include <stdlib.h>
#define wheel_diameter  15.000000f			//����ֱ��
#define half_width  19.000000f		//���̰��
#define half_length  17.000000f		//���̰볤

#define PI 			3.141593f
#define PI2     2*PI
#define RPM2RAD 0.104720f										//ת��ת���ٶ�		1 rpm = 2pi/60 rad/s 
#define RPM2VEL 0.523599f										//ת��ת���ٶ�		vel = rpn*pi*D/60  cm/s
#define VEL2RPM 1.909859f										//���ٶ�תת��
#define M2006_REDUCTION_RATIO 36.000000f		//��������ٱ�
#define M3508_REDUCTION_RATIO 19.000000f		//��������ٱ�
#define GM6020_ENCODER_ANGLE  8192.0f


#define MAX_MOTOR_SPEED   14336				//������ת�٣��궨�巽���޸�   ��Χ0 - 10000   15336   
#define MAX_BASE_LINEAR_SPEED    120.817f    //�������ƽ���ٶȣ���λcm/s   
#define MAX_BASE_ROTATIONAL_SPEED    7.260570f    //���������ת�ٶȣ���λrad/s    
#define NORMAL_LINEAR_SPEED          70.0f
#define NORMAL_ROTATIONAL_SPEED      0.5f


#define set_chassis_speed(motor1_speed,motor2_speed,motor3_speed,motor4_speed) \
        do{                                                                    \
					chassis_motor1.pid.speed_loop.vpid.target_speed = motor1_speed;		                         \
	        chassis_motor2.pid.speed_loop.vpid.target_speed = motor2_speed;                             \
	        chassis_motor3.pid.speed_loop.vpid.target_speed = motor3_speed;                             \
	        chassis_motor4.pid.speed_loop.vpid.target_speed = motor4_speed;                             \
				}while(0)                                                              \

				
				
typedef struct
{
	float linear_vel;			//���ٶ�
	float rpm;						//ת��Ȧÿ����
}Speed_t;

typedef struct
{
	Speed_t target_speed;			
	Speed_t actual_speed;						
}Wheel_t;

//���̼������ĵ���/���ٶ�
typedef struct
{
	float linear_x;	//m/s
	float linear_y;
	float angular_z; //���ٶ�rpm
}Velocities_t;

typedef struct
{
	float target_angular;
	float actual_angular;
  float target_angle;
	float actual_angle;
}Application_t;

typedef struct
{
	Wheel_t wheel1;
	Wheel_t wheel2;
	Wheel_t wheel3;
	Wheel_t wheel4;
	
	Velocities_t target_velocities;		//Ŀ�����ٶ�
	Velocities_t actual_velocities; 	//ʵ�����ٶ�
}Kinematics_t;

extern Kinematics_t Kinematics;
void BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z);
void Get_Base_Velocities(void);
int find_max(void);

#endif
