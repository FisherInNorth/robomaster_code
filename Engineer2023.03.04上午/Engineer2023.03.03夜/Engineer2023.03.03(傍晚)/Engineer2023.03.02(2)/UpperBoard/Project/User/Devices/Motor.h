#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f4xx_hal.h"
#include "bsp_can.h"
#include "TypeDef.h"

#define CANRX_Longitudinal_MotorL_ID 		0x201
#define CANRX_Longitudinal_MotorR_ID 		0x202
#define CANRX_Inside_Lift_MotorL_ID		0x201
#define CANRX_Inside_Lift_MotorR_ID		0x202
#define CANRX_Horizontal_Motor_ID 0x203
#define CANRX_Chuck_Yaw_ID 0x203 //can2
#define CANRX_Chuck_Pitch_ID 0x204 //can2
#define CANRX_Chuck_Roll_ID 0x205 //can2

extern MOTOR_t longitudinal_motorL;//控制纵向移动左侧的电机
extern MOTOR_t longitudinal_motorR;//控制纵向移动右侧的电机
extern MOTOR_t inside_lift_motorL;//内侧左边抬升的电机
extern MOTOR_t inside_lift_motorR;//内侧右边抬升的电机
extern MOTOR_t horizontal_motor;
extern MOTOR_t chuck_yaw;//吸盘电机
extern MOTOR_t chuck_pitch;//吸盘电机
extern MOTOR_t chuck_roll;//吸盘电机

//extern TaskHandle_t Motor_Task_Handle;

void Mineral_Init(void);
void Horizontal_Init(void);
void Chuck_Yaw_Init(void);
void Chuck_Pitch_Init(void);
void Chuck_Roll_Init(void);

void CAN1_Motor_HandleMsg(uint32_t StdId, unsigned char * Data);
void CAN2_Motor_HandleMsg(uint32_t StdId, unsigned char * Data);
void Debug_CAN_Send_Mineral_Msg(uint8_t* msg);
void Debug_CAN_Send_Flip_Msg(uint8_t* msg);
void CAN_Send_Flip_Msg(uint8_t* msg);

#endif
