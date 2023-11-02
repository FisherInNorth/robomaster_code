#ifndef _STEERING_ENGINE_H
#define _STEERING_ENGINE_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"

#define PWM_PERIOD_COUNT 1000-1
#define PWM_PRESCALER_COUNT 1680-1

void TIM_Steering_Engine_PWM_Init(void);
void set_steering_gear_dutyfactor(uint16_t dutyfactor);
void set_steering_gear_angle(uint16_t angle_temp);
void set_steering_gear_dutyfactor_repeat(uint16_t dutyfactor);
void set_steering_gear_angle_repeat(uint16_t angle_temp);

#endif
