#ifndef _KINEMATIC_H
#define _KINEMATIC_H
#include "stdint.h"

#define half_width  25.000000f		//底盘半宽
#define half_length  35.000000f		//底盘半长

#define RPM2VEL 0.523599f										//转速转线速度		vel = rpn*pi*D/60  cm/s
#define VEL2RPM 1.909859f										//线速度转转度
#define M3508_REDUCTION_RATIO 19.000000f		//齿轮箱减速比

#define MAX_MOTOR_SPEED   15336				//电机最大转速，宏定义方便修改   范围0 - 10000   15336   
#define MAX_BASE_LINEAR_SPEED    120.817f    //底盘最大平移速度，单位cm/s   
#define MAX_BASE_ROTATIONAL_SPEED    7.260570f  //7.260570f    //底盘最大旋转速度，单位rad/s    
	
typedef struct
{
	float linear_vel;			//线速度
	float rpm;						//转速圈每分钟
}Speed_t;

typedef struct
{
	Speed_t target_speed;			
	Speed_t actual_speed;						
}Wheel_t;

//底盘几何中心的线/角速度
typedef struct
{
	float linear_x;	//m/s
	float linear_y;
	float angular_z; //角速度rpm
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
  volatile uint16_t pulse_num;
           uint16_t pwm_pulse;	
}Stepper_motor_t;

typedef struct
{
	uint16_t rescue_claw_pulse_right;
	uint16_t rescue_claw_pulse_left;

	Wheel_t wheel1;
	Wheel_t wheel2;
	Wheel_t wheel3;
	Wheel_t wheel4;
	
	Velocities_t target_velocities;		//目标线速度
	Velocities_t actual_velocities; 	//实际线速度
	Application_t handle_L;
	Application_t handle_R;
	Application_t clamping_claw;
	Application_t flip;
	Stepper_motor_t stepper_motor_left;
	Stepper_motor_t stepper_motor_right;
	Application_t steering_part;

}Kinematics_t;



extern Kinematics_t Kinematics;

void BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z);
void Get_Base_Velocities(void);
void chassic_speed_control(float speed_x,float speed_y,float speed_r);		//将三个方向速度转换为电机转速
void handle_speed_control(float gimbal_y_angle);
void flip_speed_control(float motor6_speed,float motor7_speed);   
void card_angle_control(float card_angle);   
#endif
