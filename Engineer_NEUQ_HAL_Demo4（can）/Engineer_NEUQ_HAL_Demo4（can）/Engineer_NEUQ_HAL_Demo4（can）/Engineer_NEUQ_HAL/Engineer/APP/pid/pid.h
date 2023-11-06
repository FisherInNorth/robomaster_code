#ifndef _PID_H
#define _PID_H


//速度环 
/*#define CHASSIC_Integral_max         5000            //抗积分饱和
#define CHASSIC_IntegralSeparation   500             //积分分离
#define CHASSIC_vPID_max             8000            //输出限幅
#define GIMBAL_Integral_max         500
#define GIMBAL_IntegralSeparation   20
#define GIMBAL_vPID_max             3000
#define FLIP_vPID_max             5000
#define BELT_vPID_max             5000*/
//上面这个放到 vpid环中 

#include "BSP_Motor.h"

void APID_Init(APID_t *apid); 
void APID_Init_All(void);
void Apid_Realize(MOTOR_t *motor,float kp,float ki,float kd);
void Vpid_Realize(MOTOR_t *motor,float kp,float ki,float kd); 
void record_motor_callback(MOTOR_t *motor, unsigned short angle, short speed, short current);
#endif

