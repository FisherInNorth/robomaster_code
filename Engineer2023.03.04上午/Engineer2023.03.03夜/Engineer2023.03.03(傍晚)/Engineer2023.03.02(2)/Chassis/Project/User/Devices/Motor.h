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

extern MOTOR_t chassis_motor1;//第一象限的电机
extern MOTOR_t chassis_motor2;//第二象限的电机
extern MOTOR_t chassis_motor3;//第三象限的电机
extern MOTOR_t chassis_motor4;//第四象限的电机
extern MOTOR_t outboard_lift_motorL;//外侧左边抬升的电机
extern MOTOR_t outboard_lift_motorR;//外侧右边抬升的电机

//extern TaskHandle_t Motor_Task_Handle;

void Mineral_Init(void);
void Set_Mineral_F_Speed(int target_speed);
void Set_Mineral_B_Speed(int target_speed);
void Set_Mineral_L_Speed(int target_speed);
void Set_Mineral_R_Speed(int target_speed);
void CAN1_Motor_HandleMsg(uint32_t StdId, unsigned char * Data);
void CAN2_Motor_HandleMsg(uint32_t StdId, unsigned char * Data);
void Debug_CAN_Send_Mineral_Msg(uint8_t* msg);
void Debug_CAN_Send_Flip_Msg(uint8_t* msg);
void CAN_Send_Flip_Msg(uint8_t* msg);

#endif
