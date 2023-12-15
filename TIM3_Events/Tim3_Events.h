#ifndef _TIM3_EVENTS_H
#define _TIM3_EVENTS_H

#include <string.h>
#include <stdio.h>
#include "led.h"
#include "bsp_debug_usart.h"
#include "bsp_uart7.h"
#include "steering_engine.h"
#include "motor.h"
#include "kinematic.h"
#include "remote_code.h"
#include "angle_pid.h"
#include "speed_pid.h"
#include "stm32f4xx_tim.h"
#include "steering_engine.h"
#include "stepper_motor.h"
#include "limit_switch.h"
#include "power.h"
#include "key.h"


void Robo_Move(void);							//机器人运动控制
void Debug_Key(void);							//调试按键

#define pressed     0
#define unpressed   1

/***********pid参数宏定义************/
#define v_chassic_p  pid_t.chassic_pid.speed_loop.kp  
#define v_chassic_i  pid_t.chassic_pid.speed_loop.ki  
#define v_chassic_d  pid_t.chassic_pid.speed_loop.kd  

#define v_handle_p  pid_t.handle.speed_loop.kp 
#define v_handle_i  pid_t.handle.speed_loop.ki
#define v_handle_d  pid_t.handle.speed_loop.kd

#define a_handle_p  pid_t.handle.position_loop.kp 
#define a_handle_i  pid_t.handle.position_loop.ki
#define a_handle_d  pid_t.handle.position_loop.kd

#define v_flip_p  pid_t.flip.speed_loop.kp
#define v_flip_i  pid_t.flip.speed_loop.ki
#define v_flip_d  pid_t.flip.speed_loop.kd

#define a_flip_p  pid_t.flip.position_loop.kp 
#define a_flip_i  pid_t.flip.position_loop.ki
#define a_flip_d  pid_t.flip.position_loop.kd

#define v_card_p  pid_t.card.speed_loop.kp  
#define v_card_i  pid_t.card.speed_loop.ki  
#define v_card_d  pid_t.card.speed_loop.kd  

#define a_card_p  pid_t.card.position_loop.kp 
#define a_card_i  pid_t.card.position_loop.ki
#define a_card_d  pid_t.card.position_loop.kd

#define v_resuce_p  pid_t.resuce.speed_loop.kp  
#define v_resuce_i  pid_t.resuce.speed_loop.ki  
#define v_resuce_d  pid_t.resuce.speed_loop.kd  

#define v_mineral_p  pid_t.mineral.speed_loop.kp  
#define v_mineral_i  pid_t.mineral.speed_loop.ki  
#define v_mineral_d  pid_t.mineral.speed_loop.kd  

#endif


