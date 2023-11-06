#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "pid.h"
#include "BSP_Motor.h"

typedef struct
{
	float linear_vel;			//线速度
	float rpm;						//转速圈每分钟
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

#define wheel_diameter  10.000000f			//轮子直径
#define half_width  25.000000f		//底盘半宽
#define half_length  35.000000f		//底盘半长
#define MAX_BASE_LINEAR_SPEED    120.817f    //底盘最大平移速度，单位cm/s   
#define MAX_BASE_ROTATIONAL_SPEED    7.260570f  //7.260570f    //底盘最大旋转速度，单位rad/s    
#define NORMAL_LINEAR_SPEED    70.0f
#define NORMAL_ROTATIONAL_SPEED  0.5f

#define MAX_MOTOR_SPEED   15336				//电机最大转速，宏定义方便修改   范围0 - 10000   15336   
#define MAX_BASE_LINEAR_SPEED    120.817f    //底盘最大平移速度，单位cm/s   
#define MAX_BASE_ROTATIONAL_SPEED    7.260570f  //7.260570f    //底盘最大旋转速度，单位rad/s    
#define NORMAL_LINEAR_SPEED    70.0f
#define NORMAL_ROTATIONAL_SPEED  0.5f

void Chassis_Init(void);


#endif

