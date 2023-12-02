#ifndef __CHASSISMOVE_TASK_H__
#define __CHASSISMOVE_TASK_H__

#include "stm32f4xx_hal.h"
#include "DR16.h"
#include "kinematics.h"

extern int16_t chassis_vx;
extern int16_t chassis_vy;
extern int16_t chassis_wz;


void ChassisMove(void);





#endif

