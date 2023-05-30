#ifndef __PID_H__
#define __PID_H__

#include "TypeDef.h"

void Vpid_Calc(MOTOR_t *motor, float kp, float ki, float kd, float IntegralSeparation, float Integral_max, float vPID_max);
void Apid_Calc(MOTOR_t *motor, float kp, float ki, float kd, float IntegralSeparation, float Integral_max, float aPID_max);
void ChassisMotorPID_Calc(MOTOR_t *motor, float * PID_Parameters);
void OutboardLiftMotor_PID_Calc(MOTOR_t *motor, float * vPID_Parameters, float * aPID_Parameters);
void MineralMotorPID_Calc(MOTOR_t *motor, float * PID_Parameters);
	
#endif
