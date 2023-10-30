#include "rc_task.h"
#include "remote_control.h"
#include "bsp_math.h"
#include "gimbal_task.h"
#include "vision_task.h" 
#include "shoot_task.h"

KEY_CONTROL control_mode=KEY_OFF;//����ģʽ
FIGHT_CONTROL fight_mode=FIGHT_ON;//ս��ģʽ

int calibrate_start_flag=0;

static int deadline_judge(uint8_t a);
//�����ж�
static int deadline_judge(uint8_t a) 
{
	if(control_mode==KEY_OFF)
	{
		if(abs(a-RC_MIDD)<=DEADLINE) return 1;
	    else return 0;
	}
   	if(control_mode==KEY_ON)
	{
		if(abs(a-KEY_MIDD)<=KEY_DEADLINE) return 1;
	    else return 0;
	}
	return 1;
}
//����ģʽѡ��
void control_mode_judge(void)
{
	if(rc_ctrl.rc.ch[0]!=0||rc_ctrl.rc.ch[1]!=0||rc_ctrl.rc.ch[2]!=0||rc_ctrl.rc.ch[3]!=0||rc_ctrl.rc.ch[4]!=0)
		control_mode=KEY_OFF;
	if(KEY_board||MOUSE_x||MOUSE_y||MOUSE_z)
		control_mode=KEY_ON;
}

//ң��������ģʽ
void remote_control_data(void)
{
	calibrate_start_flag=0;
	One_Shoot_flag=0;
    Ten_Shoot_flag=0;

		/*                ģʽѡ��                 */
	if(switch_is_up(rc_ctrl.rc.s[1]))  //����   �����˶�ģʽѡ��
	{
		if(switch_is_up(rc_ctrl.rc.s[0]) && gimbal_set_mode!=GIMBAL_INIT) // �涯
		{
			gimbal_set_mode = GIMBAL_ABSOLUTE_ANGLE; //����ע��
		}
		if(switch_is_mid(rc_ctrl.rc.s[0]) && gimbal_set_mode!=GIMBAL_INIT ) // С����
		{
			gimbal_set_mode = GIMBAL_TOP_ANGLE;  //����ע��
		}
		if(switch_is_down(rc_ctrl.rc.s[0])) //  ����У׼ģʽ
		{
			gimbal_set_mode=GIMBAL_ZERO_FORCE;
			//calibrate_start_flag=1;
			return;
		}
		
	}
	if(switch_is_mid(rc_ctrl.rc.s[1])) // �� �Ӿ�ѡ��
	{
		if(switch_is_up(rc_ctrl.rc.s[0])) // ��
		{
			//control_mode=KEY_ON;
		}
		if(switch_is_mid(rc_ctrl.rc.s[0])) // ��
		{
			vision_mode=VISION_OFF;
		}
		if(switch_is_down(rc_ctrl.rc.s[0]))
		{
      vision_mode=VISION_ON;
		}
	}
	if(switch_is_down(rc_ctrl.rc.s[1])) // ��  ���ӵ�
	{
		rc_shoot.left_fric.target_speed = -SHOOT_FRIC_SPEED;
		rc_shoot.right_fric.target_speed = SHOOT_FRIC_SPEED;
		if(switch_is_up(rc_ctrl.rc.s[0]))// ����
		{
			One_Shoot_flag=1;
		}
		if(switch_is_mid(rc_ctrl.rc.s[0])) // ��
		{
			
		}
		if(switch_is_down(rc_ctrl.rc.s[0])) //  ����
		{
			Ten_Shoot_flag=1;
		}
	}
	if(deadline_judge(rc_ctrl.rc.ch[0])==0)
	{
		rc_sent.y_speed=limits_change(Y_SPEED_MAXX,Y_SPEED_MINN,rc_ctrl.rc.ch[0],RC_MAXX,RC_MINN);
	}
	else rc_sent.y_speed=0;
	if(deadline_judge(rc_ctrl.rc.ch[1])==0)
	{
		rc_sent.x_speed=limits_change(X_SPEED_MAXX,X_SPEED_MINN,rc_ctrl.rc.ch[1],RC_MAXX,RC_MINN);
	}
	else rc_sent.x_speed=0;
	if(deadline_judge(rc_ctrl.rc.ch[4])==0)
	{
		rc_sent.r_speed=limits_change(Z_SPEED_MAXX,Z_SPEED_MINN,rc_ctrl.rc.ch[4],RC_MAXX,RC_MINN);
	}
	else rc_sent.r_speed=0;
	
	if(deadline_judge(rc_ctrl.rc.ch[2])==0)
	{
		rc_sent.yaw.target_speed=limits_change(RC_YAW_SPEED_MAXX,RC_YAW_SPEED_MINN,rc_ctrl.rc.ch[2],RC_MAXX,RC_MINN);
		rc_sent.yaw.target_angle=limits_change(RC_YAW_ANGLE_MAXX,RC_YAW_ANGLE_MINN,rc_ctrl.rc.ch[2],RC_MAXX,RC_MINN);
	}
	else
	{
		rc_sent.yaw.target_speed=0;
		rc_sent.yaw.target_angle=0;
	}
	if(deadline_judge(rc_ctrl.rc.ch[3])==0)
	{
		rc_sent.pitch.target_speed=limits_change(RC_PITCH_SPEED_MAXX,RC_PITCH_SPEED_MINN,rc_ctrl.rc.ch[3],RC_MAXX,RC_MINN);
		rc_sent.pitch.target_angle=-limits_change(RC_PITCH_ANGLE_MAXX,RC_PITCH_ANGLE_MINN,rc_ctrl.rc.ch[3],RC_MAXX,RC_MINN);
	}
	else
	{
		rc_sent.pitch.target_speed=0;
		rc_sent.pitch.target_angle=0;
	}
}
//�������ģʽ

void key_control_data(void)
{
	One_Shoot_flag=0;
    Ten_Shoot_flag=0;
	//ս��ģʽ�ж�
	if(KEY_board&SHIFT_key) fight_mode=RUN_AWAY;
	else fight_mode=FIGHT_ON;
	
	/*���Ƹ�Ϊ��ϸ*/
	if(fight_mode==FIGHT_ON)
	{
		/*       ������̨      */
		rc_sent.yaw.target_angle=limits_change(KEY_YAW_ANGLE_MAXX_ON,KEY_YAW_ANGLE_MINN_ON,MOUSE_x,KEY_MAXX,KEY_MINN);
		if(deadline_judge(MOUSE_y)==0)
		rc_sent.pitch.target_angle=limits_change(KEY_PITCH_ANGLE_MAXX_ON,KEY_PITCH_ANGLE_MINN_ON,MOUSE_y,KEY_MAXX,KEY_MINN);
	    else
		rc_sent.pitch.target_angle=0.0f;	
		if(MOUSE_pre_left==1) One_Shoot_flag=1;
		if(MOUSE_pre_right==1) Ten_Shoot_flag=1;
		
		/*       ���Ƶ���     */
		if(KEY_board&ws_key)
		{
			if(KEY_board&W_key)
		    {
			    rc_sent.x_speed= 60;
		    }
		    if(KEY_board&S_key)
		    {
			    rc_sent.x_speed= -60;
		    }
		}
		else rc_sent.x_speed=0;

		if(KEY_board&ad_key)
		{
			if(KEY_board&A_key)
		    {
			    rc_sent.y_speed=-40;
		    }
		    if(KEY_board&D_key)
		    {
			    rc_sent.y_speed= 40;
		    }
		}
		else rc_sent.y_speed=0;
		//����
		if(KEY_board&Q_key)
		{
			gimbal_set_mode = GIMBAL_ABSOLUTE_ANGLE;
		}
		
		if(KEY_board&E_key)
		{
			gimbal_set_mode = GIMBAL_TOP_ANGLE;
		}
		//�Ӿ�
		if(KEY_board&R_key)
		{
			vision_mode=VISION_OFF;
		}
		
		if(KEY_board&F_key)
		{
			vision_mode=VISION_ON;
		}
	}
	
   	if(fight_mode==RUN_AWAY)
	{
		/*       ������̨      */
		rc_sent.yaw.target_angle=limits_change(KEY_YAW_ANGLE_MAXX_RUN,KEY_YAW_ANGLE_MINN_RUN,MOUSE_x,KEY_MAXX,KEY_MINN);
		if(deadline_judge(MOUSE_y)==0)
		rc_sent.pitch.target_angle=limits_change(KEY_PITCH_ANGLE_MAXX_RUN,KEY_PITCH_ANGLE_MINN_RUN,MOUSE_y,KEY_MAXX,KEY_MINN);
	    
		if(MOUSE_pre_left==1) One_Shoot_flag=1;
		if(MOUSE_pre_right==1) Ten_Shoot_flag=1;
		
		/*       ���Ƶ���     */
		if(KEY_board&ws_key)
		{
			if(KEY_board&W_key)
		    {
			    rc_sent.x_speed= 170;
		    }
		    if(KEY_board&S_key)
		    {
			    rc_sent.x_speed= -170;
		    }
		}
		else rc_sent.x_speed=0;

		if(KEY_board&ad_key)
		{
			if(KEY_board&A_key)
		    {
			    rc_sent.y_speed=-120;
		    }
		    if(KEY_board&D_key)
		    {
			    rc_sent.y_speed= 120;
		    }
		}
		else rc_sent.y_speed=0;
		//����
		if(KEY_board&Q_key)
		{
			gimbal_set_mode = GIMBAL_ABSOLUTE_ANGLE;
		}
		
		if(KEY_board&E_key)
		{
			gimbal_set_mode = GIMBAL_TOP_ANGLE;
		}
		//�Ӿ�
		if(KEY_board&R_key)
		{
			vision_mode=VISION_OFF;
		}
		
		if(KEY_board&F_key)
		{
			vision_mode=VISION_ON;
		}
	}
	
}
