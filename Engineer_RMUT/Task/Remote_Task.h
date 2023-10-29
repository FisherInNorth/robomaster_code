#ifndef __REMOTE_TASK_H
#define __REMOTE_TASK_H

#include "main.h"
#include "remote_control.h"

#define x_CH_width            rc_ctrl.rc.ch[0]         //x����ͨ������   ��ҡ������
#define y_CH_width            rc_ctrl.rc.ch[1]         //y����ͨ������   ��ҡ������
#define r_CH_width            rc_ctrl.rc.ch[2]         //r����ͨ������   ��ҡ������
#define i_CH_width            rc_ctrl.rc.ch[3]         //��ҡ������
#define	DJI_Motion_Round		  	rc_ctrl.rc.ch[4]					//ʹ�����ֻ��ֿ�����̨yaw
#define RIGHT_LEVER           rc_ctrl.rc.s[0]          //�Ҳ���
#define LEFT_LEVER            rc_ctrl.rc.s[1]          //�󲦸�
#define Lever_down        	  2   					 //��������
#define Lever_mid      				3  				     //�����м� 
#define Lever_up					    1              //������ 

//�������ʼֵ
#define x_initial_value       1024
#define y_initial_value       1024
#define r_initial_value       1024
#define i_initial_value       1024
//�����������Сֵ
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

