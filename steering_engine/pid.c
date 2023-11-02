#include "pid.h"
#include "BSP_motor.h"
#include "algorithm.h"
#include "stm32F4xx.h"
#include "stdint.h"



/**
* @function函数:vpid_realize()
* @brief描述:vpid的计算公式
* @param输入:pid参数
* @retval返回值:无
 */

void Vpid_Realize(MOTOR_t *motor,float kp,float ki,float kd)
{
	motor->vpid.err = motor->vpid.target_speed - motor->vpid.actual_speed;
	if(abs(motor->vpid.err) <= IntegralSeparation)		//积分分离
		motor->vpid.err_integration += motor->vpid.err;
	if(motor->vpid.err_integration > Integral_max)		//抗积分饱和
		motor->vpid.err_integration = Integral_max;
	else if(motor->vpid.err_integration < -Integral_max)
		motor->vpid.err_integration = -Integral_max;
	
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
			motor->vpid.PID_OUT = -motor->vPID_max;
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
void Apid_Realize(MOTOR_t *motor,float kp,float ki,float kd)
{

	motor->apid.err = motor->apid.target_angle - motor->apid.total_angle;
	motor->apid.P_OUT = kp * motor->apid.err;
	motor->apid.D_OUT = kd * (motor->apid.err-motor->apid.last_err);
	motor->apid.last_err = motor->apid.err;
  if((motor->apid.P_OUT + motor->apid.D_OUT) > motor->aPID_OUT_MAX) 
  {
			  
	  motor->apid.PID_OUT = motor->aPID_OUT_MAX;
	}
	else if((motor->apid.P_OUT + motor->apid.D_OUT) < -motor->aPID_OUT_MAX) 
	{
		motor->apid.PID_OUT = -motor->aPID_OUT_MAX;
	}
	else
	{
		motor->apid.PID_OUT = motor->apid.P_OUT + motor->apid.D_OUT;
	}
}


/**
* @brief  电机can回调赋值函数
* @param MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current
* @retval void
* @attention
*/
void record_motor_callback(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current)
{
	motor->last_angle = motor->apid.actual_angle;
	motor->apid.actual_angle = angle;
	motor->vpid.actual_speed = speed;
	motor->actual_current = current;
	if(motor->start_angle_flag == 0)
	{
		motor->start_angle = angle;
		motor->start_angle_flag++;	//只在启动时记录一次初始角度
	}

	if(motor->apid.actual_angle - motor->last_angle > 4096)
		motor->round_cnt --;
	else if (motor->apid.actual_angle - motor->last_angle < -4096)
		motor->round_cnt ++;
	motor->apid.total_angle = motor->round_cnt * 8192 + motor->apid.actual_angle;	// - motor->start_angle;
}
