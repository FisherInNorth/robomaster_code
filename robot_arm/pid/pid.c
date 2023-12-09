/**
  ******************************************************************************
  * @file    Project/HARDWARE/motor.c
  * @author  Siyuan Qiao&Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief
  ******************************************************************************
  * @attention
  ******************************************************************************
	// * 范围  ：3508  数据范围                       金属头转速为0.22rps,即13.3rpm
//						给定电流 -16384 ~ 0 ~ +16384
//						转子角度	0~8191 等价	0-360°
//						转子转速	max=450	RPM
//						实际转矩电流
//						电机温度
//			   ：GM6020数据范围
//						给定电压 -30000 ~ 0 ~ +30000
//						转子角度	0~8191 等价	0-360°
//						转子转速	max=350	RPM
//						实际转矩电流
//						电机温度
//			   ：P2006 数据范围
//						给定电流 -10000 ~ 0 ~ +10000
//						转子角度	0~8191 等价	0-360°
//						转子转速	max=450	RPM
//						实际输出转矩
      ..................NEUQ_SUDO..................

*/
#include "pid.h"


// 函数: VPID_Init()
// 描述: 电机速度pid参数初始化
// 参数：电机速度pid参数结构体
// 输出：无
// 内部函数，用户无需调用
void VPID_Init(VPID_t *vpid)
{
	vpid->target_speed=0;
	vpid->actual_speed=0;
	vpid->err=0;
	vpid->last_err=0;
	vpid->err_integration=0;
	vpid->P_OUT=0;
	vpid->I_OUT=0;
	vpid->D_OUT=0;
	vpid->PID_OUT=0;
}

// 函数: VPID_Init_All()
// 描述: 所有电机速度pid参数初始化
// 参数：无
// 输出：无
void VPID_Init_All()	
{
	VPID_Init(&motor1.vpid);
	VPID_Init(&motor2.vpid);
	VPID_Init(&motor3.vpid);
	VPID_Init(&motor4.vpid);
	VPID_Init(&motor5.vpid);
	VPID_Init(&motor6.vpid);
	VPID_Init(&motor7.vpid);
}




// 函数: APID_Init()
// 描述: 电机机械角度pid参数初始化
// 参数：电机机械角度pid参数结构体
// 输出：无
// 内部函数，用户无需调用
void APID_Init(APID_t *apid)
{
	apid->actual_angle=0;
	apid->target_angle=0;
	apid->total_angle=0;
	apid->err=0;
	apid->err_integration=0;
	apid->P_OUT=0;
	apid->I_OUT=0;
	apid->D_OUT=0;
	apid->PID_OUT=0;
}

// 函数: APID_Init_All()
// 描述: 所有电机机械角度pid参数初始化
// 参数：无
// 输出：无
void APID_Init_All()
{
	APID_Init(&motor1.apid);
	APID_Init(&motor2.apid);
	APID_Init(&motor3.apid);
	APID_Init(&motor4.apid);
  APID_Init(&motor5.apid);
	APID_Init(&motor6.apid);
	APID_Init(&motor7.apid);
}

/**
* @function函数:vpid_realize()
* @brief描述:vpid的计算公式
* @param输入:pid参数
* @retval返回值:无
 */

void vpid_realize(MOTOR_t *motor,float kp,float ki,float kd)
{
	motor->vpid.err = motor->vpid.target_speed - motor->vpid.actual_speed;
	if(abs(motor->vpid.err) <= motor->integralSeparation)		//积分分离
		motor->vpid.err_integration += motor->vpid.err;
	if(motor->vpid.err_integration > motor->integral_max)		//抗积分饱和
		motor->vpid.err_integration = motor->integral_max;
	else if(motor->vpid.err_integration < -motor->integral_max)
		motor->vpid.err_integration = -motor->integral_max;
	
	motor->vpid.P_OUT = kp * motor->vpid.err;								//P项
	motor->vpid.I_OUT = ki * motor->vpid.err_integration;		//I项
	motor->vpid.D_OUT = kd * (motor->vpid.err-motor->vpid.last_err);//D项
	motor->vpid.last_err=motor->vpid.err;
	//输出限幅
	if(abs(motor->vpid.err) <= 2)
	  motor->vpid.PID_OUT=0;
	else
	{	
		if((motor->vpid.P_OUT + motor->vpid.I_OUT + motor->vpid.D_OUT)> motor->vPID_max) 
			motor->vpid.PID_OUT = motor->vPID_max;
	  	else if((motor->vpid.P_OUT + motor->vpid.I_OUT + motor->vpid.D_OUT) < -motor->vPID_max) 
			pid->PID_OUT = -motor->vPID_max;
	  	else
			motor->vpid.PID_OUT = motor->vpid.P_OUT + motor->vpid.I_OUT + motor->vpid.D_OUT;
    }
}

	/**
	* @function函数:apid_realize()
	* @brief描述:apid的计算公式
	* @param输入:pid参数
	* @retval返回值:无
  */
void apid_realize(MOTOR_t *motor,float kp,float ki,float kd)
{

	motor->apid.err = motor->apid.target_angle - motor->apid.total_angle;
	motor->apid.P_OUT = kp * motor->apid.err;
	motor->apid.D_OUT = kd * (motor->apid.err-motor->apid.last_err);
	motor->apid.last_err = motor->apid.err;
  if((motor->apid.P_OUT + motor->apid.D_OUT) > aPID_OUT_MAX) 
  {
			  
	  motor->apid.PID_OUT = aPID_OUT_MAX;
	}
	else if((motor->apid.P_OUT + motor->apid.D_OUT) < -aPID_OUT_MAX) 
	{
		motor->apid.PID_OUT = -aPID_OUT_MAX;
	}
	else
	{
		motor->apid.PID_OUT = motor->apid.P_OUT + motor->apid.D_OUT;
	}
	set_speed(motor, motor->apid.PID_OUT);
}

void set_angle(MOTOR_t *motor, motor_can2_1_angle) 
{
	motor->apid.target_angle = motor_can2_1_angle;
} 

void set_speed(MOTOR_t *motor, motor_speed) 
{
	motor->vpid.target_speed = motor_speed;
}


