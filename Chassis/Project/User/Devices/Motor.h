#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f4xx_hal.h"
#include "bsp_can.h"
#include "TypeDef.h"

#define CANRX_Chassis_Motor1_ID 		0x201
#define CANRX_Chassis_Motor2_ID 		0x202
#define CANRX_Chassis_Motor3_ID 		0x203
#define CANRX_Chassis_Motor4_ID 		0x204
#define CANRX_Outboard_Lift_MotorL_ID		0x201
#define CANRX_Outboard_Lift_MotorR_ID		0x202
#define CANRX_Mineral_Motor1_ID     0x205
#define CANRX_Mineral_Motor2_ID     0x206
#define CANRX_Mineral_Motor3_ID     0x207
#define CANRX_Mineral_Motor4_ID     0x208

extern MOTOR_t chassis_motor1;//第一象限的电机
extern MOTOR_t chassis_motor2;//第二象限的电机
extern MOTOR_t chassis_motor3;//第三象限的电机
extern MOTOR_t chassis_motor4;//第四象限的电机
extern MOTOR_t outboard_lift_motorL;//外侧左边抬升的电机
extern MOTOR_t outboard_lift_motorR;//外侧右边抬升的电机
extern MOTOR_t mineral_motor1;//1号矿石运动电机
extern MOTOR_t mineral_motor2;//2号矿石运动电机
extern MOTOR_t mineral_motor3;//3号矿石运动电机
extern MOTOR_t mineral_motor4;//4号矿石运动电机

extern uint8_t start_flag;

//extern TaskHandle_t Motor_Task_Handle;


void CAN1_Motor_HandleMsg(uint32_t StdId, unsigned char * Data);
void CAN2_Motor_HandleMsg(uint32_t StdId, unsigned char * Data);
void Debug_CAN_Send_Mineral_Msg(uint8_t* msg);
void Debug_CAN_Send_Flip_Msg(uint8_t* msg);
void CAN_Send_Flip_Msg(uint8_t* msg);
void abs_motor(void);
void CAN1_Motor_ControlMsg(void);
void CAN2_Motor_ControlMsg(void);
#endif
