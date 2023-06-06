#ifndef __REMOTE_CONTROL_H
#define __REMOTE_CONTROL_H

#include "dma.h"
#include "usart.h"
#include "ChassisMove_Task.h"

#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u
//死区值
#define DEADLINE 100
//遥控器值范围
#define RC_MIDD 1024
#define RC_MAXX 1684
#define RC_MINN 364

//底盘速度范围
#define X_SPEED_MAXX 255
#define X_SPEED_MINN -255

#define Y_SPEED_MAXX 255
#define Y_SPEED_MINN -255

#define Z_SPEED_MAXX 255
#define Z_SPEED_MINN -255

#define NORMAL_LINEAR_SPEED          70.0f
#define NORMAL_ROTATIONAL_SPEED      0.5f

//遥控器数据结构体
typedef __packed struct
{
	__packed struct
	{
			int16_t ch[5];
			char s[2];
	} rc;
	__packed struct
	{
			int16_t x;
			int16_t y;
			int16_t z;
			uint8_t press_l;
			uint8_t press_r;
	} mouse;
	__packed struct
	{
			uint16_t v;
	} key;

} RC_ctrl_t;
//遥控器接收值

/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_W ((uint16_t)1 << 0)              //1
#define KEY_S ((uint16_t)1 << 1)              //2
#define KEY_A ((uint16_t)1 << 2)              //4
#define KEY_D ((uint16_t)1 << 3)              //8
#define KEY_SHIFT ((uint16_t)1 << 4)          //16
#define KEY_CTRL ((uint16_t)1 << 5)           //32
#define KEY_Q ((uint16_t)1 << 6)              //64
#define KEY_E ((uint16_t)1 << 7)              //128
#define KEY_R ((uint16_t)1 << 8)              //256
#define KEY_F ((uint16_t)1 << 9)              //512
#define KEY_G ((uint16_t)1 << 10)             //1024
#define KEY_Z ((uint16_t)1 << 11)             //2048
#define KEY_X ((uint16_t)1 << 12)             //4096
#define KEY_C ((uint16_t)1 << 13)             //8192
#define KEY_V ((uint16_t)1 << 14)             //16384
#define KEY_B ((uint16_t)1 << 15)             //32768

#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/**********************************************************************************
   * 键盘通道:15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
   *          V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
************************************************************************************/
/* ----------------------- Data Struct ------------------------------------- */

extern uint8_t calibrate_lift_flag,flag_v;

void remote_control_init(void);
void RC_IRQHandler(void);
//void remote_control(void);
void HandleDR16_Data(void);
void Key_Control(void);
void Key_Mode_Judge(void);
void Motor_Lift_Calibrate(void);
void judge_v(void);
#endif
