#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f4xx_hal.h"
#include "bsp_can.h"
#include "TypeDef.h"

#define CANRX_Longitudinal_MotorL_ID 		0x201
#define CANRX_Longitudinal_MotorR_ID 		0x202
#define CANRX_Inside_Lift_MotorL_ID		  0x203
#define CANRX_Inside_Lift_MotorR_ID		  0x204

#define CANRX_Horizontal_Motor_ID     0x201
#define CANRX_Chuck_Yaw_ID            0x202
#define CANRX_Chuck_Pitch_ID          0x203
#define CANRX_Chuck_Roll_ID           0x204


extern MOTOR_t longitudinal_motorL;//���������ƶ����ĵ��
extern MOTOR_t longitudinal_motorR;//���������ƶ��Ҳ�ĵ��
extern MOTOR_t inside_lift_motorL;//�ڲ����̧���ĵ��
extern MOTOR_t inside_lift_motorR;//�ڲ��ұ�̧���ĵ��
extern MOTOR_t horizontal_motor;//���Ƶĵ��
extern MOTOR_t chuck_yaw;//yaw�ĵ��
extern MOTOR_t chuck_roll;//roll�ĵ��
extern MOTOR_t chuck_pitch;//pitch�ĵ��
extern uint8_t start_flag;

void Mineral_Init(void);
void motor_angle_init(void);
void CAN1_Motor_HandleMsg(uint32_t StdId, unsigned char * Data);
void CAN2_Motor_HandleMsg(uint32_t StdId, unsigned char * Data);
void Debug_CAN_Send_Mineral_Msg(uint8_t* msg);
void Debug_CAN_Send_Flip_Msg(uint8_t* msg);
void CAN_Send_Flip_Msg(uint8_t* msg);
void Custom_Robot_Control(void);
void CAN1_Motor_ControlMsg(void);
void CAN2_Motor_ControlMsg(void);

#endif
