#ifndef __RC_TASK_H
#define __RC_TASK_H

/*                 ң����                   */
//����ֵ
#define DEADLINE 30
//ң����ֵ��Χ
#define RC_MIDD 0
#define RC_MAXX 660
#define RC_MINN -660
//��̨�Ƕ��ٶȷ�Χ
#define RC_YAW_SPEED_MAXX 30
#define RC_YAW_SPEED_MINN -30
#define RC_YAW_ANGLE_MAXX 14
#define RC_YAW_ANGLE_MINN -14

#define RC_PITCH_SPEED_MAXX 6
#define RC_PITCH_SPEED_MINN -6
#define RC_PITCH_ANGLE_MAXX 8
#define RC_PITCH_ANGLE_MINN -8

//�����ٶȷ�Χ
#define X_SPEED_MAXX 500
#define X_SPEED_MINN -500

#define Y_SPEED_MAXX 400
#define Y_SPEED_MINN -400

#define Z_SPEED_MAXX 500
#define Z_SPEED_MINN -500


/************************** ���� ******************************/
#define KEY_DEADLINE 2
//ս��ģʽʱ�Ŀ��Ʒ�Χ
#define KEY_PITCH_ANGLE_MAXX_ON 15
#define KEY_PITCH_ANGLE_MINN_ON -15

#define KEY_YAW_ANGLE_MAXX_ON 40
#define KEY_YAW_ANGLE_MINN_ON -40

//����ģʽʱ�Ŀ��Ʒ�Χ
#define KEY_PITCH_ANGLE_MAXX_RUN 15
#define KEY_PITCH_ANGLE_MINN_RUN -15

#define KEY_YAW_ANGLE_MAXX_RUN 80
#define KEY_YAW_ANGLE_MINN_RUN -80

//ң����ֵ��Χ
#define KEY_MIDD 0
#define KEY_MAXX 300
#define KEY_MINN -300

#define MOUSE_x               rc_ctrl.mouse.x     
#define MOUSE_y               rc_ctrl.mouse.y
#define MOUSE_z               rc_ctrl.mouse.z
#define MOUSE_pre_left        rc_ctrl.mouse.press_l
#define MOUSE_pre_right       rc_ctrl.mouse.press_r
#define KEY_board             rc_ctrl.key.v

#define W_key                0x000001
#define S_key                0x000002
#define ws_key               0x000003   
	
#define A_key                0x000004
#define D_key                0x000008
#define ad_key               0x00000C

#define Q_key                0x000040
#define E_key                0x000080
#define qe_key               0x0000C0

#define R_key                0x000100
#define F_key                0x000200
#define qe_key               0x0000C0
	
#define SHIFT_key            0x000010
#define CTRL_key             0x000020
	
#define C_key                0x002000
#define G_key                0x000400  
#define V_key                0x004000  
#define NULL_Key             0x000000

typedef enum{
    KEY_ON= 0, //���̿�
    KEY_OFF,    //
}KEY_CONTROL;

typedef enum{
    RUN_AWAY= 0, //���̿�
    FIGHT_ON,    //
}FIGHT_CONTROL;

void remote_control_data(void);
void control_mode_judge(void);
void key_control_data(void);

extern int calibrate_start_flag;
extern KEY_CONTROL control_mode;
extern FIGHT_CONTROL fight_mode;
#endif
