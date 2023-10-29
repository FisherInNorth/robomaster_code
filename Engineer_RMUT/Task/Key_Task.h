#ifndef __KEY_H__
#define __KEY_H__

#include "BSP_Handle.h"
#include "BSP_Mineral.h"
#include "BSP_Flip.h"
#include "Remote_Task.h"
#include "BSP_Uart.h"
#include "BSP_Chassis.h"
#include "remote_control.h"
#include "BSP_Steering.h"
#include "BSP_Clamp.h"

extern MOTOR_MOVE_t flip_move;
extern MOTOR_MOVE_t rescue_move;
extern MOTOR_MOVE_t rescue_card;
extern MOTOR_MOVE_t handle_move;
extern MOTOR_MOVE_t mineral_longitudinal_move;
extern MOTOR_MOVE_t mineral_widthwise_move;
extern MOTOR_MOVE_t push1,push2;
extern MOTOR_MOVE_t steering_move;

void Key_Task(void);

#endif

