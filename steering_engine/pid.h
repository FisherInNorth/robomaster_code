#ifndef _PID_H
#define _PID_H

#include "BSP_Motor.h"

#define Integral_max         500
#define IntegralSeparation   20

void APID_Init(APID_t *apid); 
void APID_Init_All(void);
void Apid_Realize(MOTOR_t *motor,float kp,float ki,float kd);
void Vpid_Realize(MOTOR_t *motor,float kp,float ki,float kd); 
void record_motor_callback(MOTOR_t *motor, unsigned short angle, short speed, short current);
#endif

