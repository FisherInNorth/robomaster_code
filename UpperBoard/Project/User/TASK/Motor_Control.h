#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include "Motor.h"
#include "ChassisBoard.h"
#include "micromath.h"
#include "pid.h"

extern int calibrate_flag;
extern int calibrate_lift_flag;
extern int calibrate_chuck_flag;
extern int calibrate_longitudinal_flag;

void Motor_Control(void);
void Motor_Calibrate(void);
void Motor_Lift_Calibrate(void);
void Motor_Chuck_Calibrate(void);
void Motor_Longitudinal_Calibrate(void);

#endif

