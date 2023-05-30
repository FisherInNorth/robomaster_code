#ifndef __PID_H__
#define __PID_H__

#include "TypeDef.h"

void Vpid_Calc(MOTOR_t *motor, float kp, float ki, float kd, float IntegralSeparation, float Integral_max, float vPID_max);
void Apid_Calc(MOTOR_t *motor, float kp, float ki, float kd, float IntegralSeparation, float Integral_max, float aPID_max);
void Longitudinal_Motor_PID_Calc(MOTOR_t *motor, float * aPID_Parameters, float * vPID_Parameters);
void InsideLiftMotor_PID_Calc(MOTOR_t *motor, float * aPID_Parameters, float * vPID_Parameters);
void HorizontalMotor_PID_Calc(MOTOR_t *motor, float * aPID_Parameters, float * vPID_Parameters);
void ChuckYaw_PID_Calc(MOTOR_t *motor, float * aPID_Parameters, float * vPID_Parameters);
void ChuckPitch_PID_Calc(MOTOR_t *motor, float * aPID_Parameters, float * vPID_Parameters);
void ChuckRoll_PID_Calc(MOTOR_t *motor, float * aPID_Parameters, float * vPID_Parameters);

#endif
