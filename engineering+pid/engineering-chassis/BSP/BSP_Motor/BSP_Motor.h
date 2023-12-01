#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stdint.h"

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

typedef struct
{
	int16_t vx_set;
	int16_t vy_set;
	int16_t wz_set;
}CHASSIS_CONTROL_ORDER_t;

typedef enum{
	out,
	in,
	stop,
}MOTOR_MOVE_t;

#endif
