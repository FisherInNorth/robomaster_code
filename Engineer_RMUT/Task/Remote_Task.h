#ifndef __REMOTE_TASK_H
#define __REMOTE_TASK_H

#include "main.h"
#include "remote_control.h"

#define x_CH_width            rc_ctrl.rc.ch[0]         //x方向通道脉宽   右摇杆左右
#define y_CH_width            rc_ctrl.rc.ch[1]         //y方向通道脉宽   右摇杆上下
#define r_CH_width            rc_ctrl.rc.ch[2]         //r方向通道脉宽   左摇杆左右
#define i_CH_width            rc_ctrl.rc.ch[3]         //左摇杆上下
#define	DJI_Motion_Round		  	rc_ctrl.rc.ch[4]					//使用左手滑轮控制云台yaw
#define RIGHT_LEVER           rc_ctrl.rc.s[0]          //右拨杆
#define LEFT_LEVER            rc_ctrl.rc.s[1]          //左拨杆
#define Lever_down        	  2   					 //拨到最下
#define Lever_mid      				3  				     //拨到中间 
#define Lever_up					    1              //拨到上 

//各方向初始值
#define x_initial_value       1024
#define y_initial_value       1024
#define r_initial_value       1024
#define i_initial_value       1024
//各方向最大最小值
#define x_max_value           1684
#define x_min_value           364
#define y_max_value           1684
#define y_min_value           364
#define r_max_value           1684
#define r_min_value           364
#define i_max_value           1684
#define i_min_value           364

void Remote_Control(void);
void left_act_up(void);
void left_act_down(void);
void left_act_mid(void);


#endif

