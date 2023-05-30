#ifndef __CHASSISMOVE_TASK_H__
#define __CHASSISMOVE_TASK_H__

#include "stm32f4xx_hal.h"
#include "DR16.h"
#include "kinematics.h"
#include "Motor.h"

extern float chassis_vx,chassis_vy,chassis_wz;


void ChassisMove(void);





#endif

