#ifndef __REMOTE_CONTROL_H
#define __REMOTE_CONTROL_H

#include "dma.h"
#include "usart.h"

#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
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
   * ����ͨ��:15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
   *          V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
************************************************************************************/
/* ----------------------- Data Struct ------------------------------------- */


//ң�������ݽṹ��
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
//ң��������ֵ

typedef struct{
	
	float target_angle;
	float target_speed;
}GIMBAL_RC_t;

typedef struct{
	GIMBAL_RC_t yaw;
	GIMBAL_RC_t pitch;
	
	float x_speed;
	float y_speed;
	float r_speed;
	
}RC_GET_t;

extern const RC_ctrl_t *get_remote_control_point(void);
extern RC_ctrl_t rc_ctrl;
extern RC_GET_t rc_sent;
void remote_control_init(void);
void RC_IRQHandler(void);
void RC_restart(uint16_t dma_buf_num);
void RC_unable(void);

#endif
