#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stdint.h"
#include "stm32f4xx.h"

//can stdid
#define CAN_LoopBack_ID				 0x200		//用于环回模式自检调试
#define	CAN_3508Motor1_ID  	   0x201	//0x20+电调ID 
#define	CAN_3508Motor2_ID      0x202	//1-4分别为底盘 
#define	CAN_3508Motor3_ID      0x203
#define	CAN_3508Motor4_ID      0x204
#define CAN_3508Motor5_ID      0x205	//5、6为救援爪 
#define CAN_3508Motor6_ID      0x206
#define CAN_3508Motor7_ID      0x207	//这个是救援卡 

//电机转速pid参数结构体
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

//电机机械角度参数
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


//电机参数结构体
typedef struct{
	int start_angle;			//电机初始角度值
	int start_angle_flag;	//记录电机初始角度值的flag
	int stop_angle;				//发送停止命令时候的角度值
	
	int last_angle;				//上一次返回的角度值
	int round_cnt;				//相对开机时转过的圈数
	
	int actual_current;		//电机真实电流
	int target_current;		//电机目标电流
	int integralSeparation;// 抗积分饱和 
	int integral_max;  //输出限幅 
	int vPID_max;  //输出最大值 
	int aPID_OUT_MAX;		//即最大角速度
	VPID_t vpid;
	APID_t apid;
	
	int stop_flag;
	int count;
	int pwm_num;
}MOTOR_t;

//电机往内转或者往外转的结构体
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
#define RPM2RAD 0.104720f										//转速转角速度		1 rpm = 2pi/60 rad/s 
#define RPM2VEL 0.523599f										//转速转线速度		vel = rpn*pi*D/60  cm/s
#define VEL2RPM 1.909859f										//线速度转转度
#define M2006_REDUCTION_RATIO 36.000000f		//齿轮箱减速比
#define M3508_REDUCTION_RATIO 19.000000f		//齿轮箱减速比


#endif
