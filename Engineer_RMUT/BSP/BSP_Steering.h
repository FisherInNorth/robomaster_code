#ifndef _STEERING_H
#define _STEERING_H

#include "tim.h"
#include "BSP_Motor.h"

void set_steering_gear_dutyfactor(uint16_t dutyfactor);
void set_steering_gear_angle(uint16_t angle_temp);
void TIM2_PWM_Init(void);
#endif
