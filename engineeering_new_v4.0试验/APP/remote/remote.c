#include "remote.h"
#include "motor.h"
#include "bsp_led.h"


/**************************************************
ս������DJIң��������
**************************************************/
void DJI_Remote_Control(void)
{
		DJI_Remote_Chassis();
		DJI_Remote_Gimball();
		DJI_Remote_Action();
}

/**************************************************
ս������PCң��������
**************************************************/
void PC_Remote_Control(void)
{
	  //PC_Remote_Chassis();
		PC_Remote_Gimball();
		PC_Remote_Action();
}
/***************************************************
ң�������̿��ƴ���
���ܣ���ҡ�˻ش�����ֵת��Ϊ���̵��˶�
���ã����̿��ƴ��롢ң������������
��ʽ���ٶȿ���
������� ����Ϊ�����˶�
����ƽ���˶�
������ת�˶�
***************************************************/

void DJI_Remote_Chassis(void)
{
//	float tempX=0,tempY=0,tempZ=0;
//	tempX = (rc.ch1-DJI_CHMID)/DJI_XVAL;
//	tempY = (rc.ch0-DJI_CHMID)/DJI_YVAL;
//	tempZ = (rc.ch2-DJI_CHMID)/DJI_ZVAL;
//	Chassis_Ctrl(ChassisX,tempX);
//	Chassis_Ctrl(ChassisY,tempY);
//	Chassis_Ctrl(ChassisZ,tempZ);
}
/**************************************************
ң������̨���ƴ���
���ܣ���ҡ�˺Ͳ��ֻش�����ֵת��Ϊ��̨���˶�
���ã���̨���ƴ��롢ң������������
��ʽ���Ƕȿ���
������� ����Ϊ��̨�˶�
����pitch�˶�
����yaw�˶�
**************************************************/
void DJI_Remote_Gimball(void)
{
//	uint16_t Yaw_angle,Pitch_angle;
//	Yaw_angle 	= (rc.sw - DJI_MIN)*(YAW_MAXANGLE - YAW_MINANGLE)/(DJI_MAX - DJI_MIN)+YAW_MINANGLE;
//	Pitch_angle = (rc.ch3 - DJI_MIN)*(PITCH_MAXANGLE - PITCH_MINANGLE)/(DJI_MAX - DJI_MIN)+PITCH_MINANGLE;
//	Gimbal_Ctrl(GimbalYaw,0,Yaw_angle);
//	Gimbal_Ctrl(GimbalPitch,0,Pitch_angle);	
}
/**************************************************
ս����������
���ܣ������Ƹ����ݣ��ж�ս������
���ã���̨�����̿��ƴ��룬ң������������
��ʽ���߼�����
������� ����Ϊս������
	Action_Rest			((3<<8)|3)	//��λ״̬ �޶���
	Action_Shoot_H	((1<<8)|2)	//���ٿ���	Ħ�������ٽϴ󣬲����ָ���ת��
	Action_Fire_H		((1<<8)|1)	//���ٵ���	Ħ�������ٽϴ󣬲����ֽǶ�ת��
	Action_Shoot_L	((2<<8)|2)	//���ٿ���	Ħ�������ٽ�С�������ָ���ת��
	Action_Fire_L		((2<<8)|1)	//���ٵ���	Ħ�������ٽ�С�������ֽǶ�ת��
	Action_Unknown1	((1<<8)|3)	//������������С���ݵ�
	Action_Unknown2	((2<<8)|3)	//
	Action_Unknown3	((3<<8)|1)	//
	Action_Unknown4	((3<<8)|2)	//
**************************************************/
void DJI_Remote_Action(void)
{
	switch(DJI_Action_Flag)
	{
		case DJI_Action_Reset:;
		{
			LED_Control(0x11);
			LASER_OFF;
		}
		break;
		case DJI_Action_Shoot_H:
		{
			LED_Control(0x22);
		}
		break;
		case DJI_Action_Fire_H:
		{
			LED_Control(0x44);
		}
		break;
		case DJI_Action_Shoot_L:
		{
			LED_Control(0x88);
		}
		break;
		case DJI_Action_Fire_L:
		{
			LED_Control(0x81);
		}
		break;
		/*case DJI_Action_Unknown1:
		{
			LED_Control(0x42);
			pid_clear(&pid_wheel_speed[0]);
			pid_clear(&pid_wheel_speed[1]);
			pid_clear(&pid_wheel_speed[2]);
			pid_clear(&pid_wheel_speed[3]);
			LASER_ON;
		}
		break;*/
		case DJI_Action_Unknown2:
		{
			LED_Control(0x24);
			LASER_ON;
		}
		break;
		case DJI_Action_Unknown3:
		{
			LED_Control(0x18);
		}
		break;
		case DJI_Action_Unknown4:
		{
			LED_Control(0x33);
		}
		break;
		default:	break;					
		
	}
	
}

int16_t PC_temp_rpm_up(int16_t PC_temp_rpm,uint8_t XYZFlag)
{
	int16_t temp_rpm_up = PC_temp_rpm;
/*******************�����ٶȲ�ͬ����***********************/
//	if (temp_rpm_up > 0)
//	{
//		temp_rpm_up = temp_rpm_up + RPM_STEP;
//		if (temp_rpm_up > temp_rpm_max)
//			temp_rpm_up = temp_rpm_max;
//	}
//	else
//	{
//		temp_rpm_up = temp_rpm_up - RPM_STEP;
//		if (temp_rpm_up < temp_rpm_min)
//			temp_rpm_up = temp_rpm_min;
//	} 
/******************�����ٶ���ͬ����***********************/
	if (XYZFlag)
	{
		temp_rpm_up = temp_rpm_up + RPM_STEP;
		if (temp_rpm_up > XYRPM_MAX)
			temp_rpm_up = XYRPM_MAX;
	}
	else
	{
		temp_rpm_up = temp_rpm_up + RPM_STEP;
		if (temp_rpm_up > ZRPM_MAX)
			temp_rpm_up = ZRPM_MAX;
	}
	return (temp_rpm_up);
}
int16_t PC_temp_rpm_down(int16_t PC_temp_rpm,uint8_t XYZFlag)
{
	int16_t temp_rpm_down = PC_temp_rpm;
	if (XYZFlag)
	{
		temp_rpm_down = temp_rpm_down - RPM_STEP;
		if (temp_rpm_down < 0)
			temp_rpm_down = 0;
	}
	else
	{
		temp_rpm_down = temp_rpm_down - RPM_STEP;
		if (temp_rpm_down < 0)
			temp_rpm_down = 0;
	}
	return (temp_rpm_down);
}
/*********************************************************************
PC���̿��ƴ���
���ܣ������̻ش�����ֵת��Ϊ���̵��˶�
���ã����̿��ƴ��롢PC��������
��ʽ���ٶȿ���
������� ����Ϊ�����˶�
		W
A		S		D
ʵ��ǰ�����ˡ�WA��ת��WD��ת //����SD��ת��AS��ת����
Shift Ϊ���ٰ�ť �����ά�˶�
Shift +			 	W
					A		S		D
ʵ�ֶ�Ӧ����ļ���ǰ�����ˡ�WA��ת��WD��ת //����SD��ת��AS��ת����
Ctrl Ϊ���ٰ�ť �����ά�˶�
Ctrl +			 	W
					A		S		D
ʵ�ֶ�Ӧ����ļ���ǰ�����ˡ�WA��ת��WD��ת //����SD��ת��AS��ת����
E��						�ٶ�ֵ��ʼ��
Shift	+	E��		ɲ����ͣ
Shift + Ctrl	��̨�Ƕȸ�λ
*********************************************************************/
/*void PC_Remote_Chassis(void)
{
	
	int16_t PC_temp_rpm_X,PC_temp_rpm_Y,PC_temp_rpm_Z;//��������м��� ��Ϊ��ֵ
	int16_t PC_set_rpm_X,PC_set_rpm_Y,PC_set_rpm_Z;		//����ṹ���������
	switch (PC_KEY_VALUE & 0x0004)
	{
		case PC_Motion_XP:
		{
			if (PC_Press_Shift)
				PC_set_rpm_X = PC_temp_rpm_up(PC_temp_rpm_X,XY_Flag);
			else if (PC_Press_Ctrl)
				PC_set_rpm_X = PC_temp_rpm_down(PC_temp_rpm_X,XY_Flag);
			else
				PC_set_rpm_X = PC_temp_rpm_X;
		}break;
		case PC_Motion_XN:
		{
			if (PC_Press_Shift)
				PC_set_rpm_X = (-PC_temp_rpm_up(PC_temp_rpm_X,XY_Flag));
			else if (PC_Press_Ctrl)
				PC_set_rpm_X = (-PC_temp_rpm_down(PC_temp_rpm_X,XY_Flag));
			else
				PC_set_rpm_X = (-PC_temp_rpm_X);
		}break;
		case PC_Motion_YP:
		{
			if (PC_Press_Shift)
				PC_set_rpm_Y = PC_temp_rpm_up(PC_temp_rpm_Y,XY_Flag);
			else if (PC_Press_Ctrl)
				PC_set_rpm_Y = PC_temp_rpm_down(PC_temp_rpm_Y,XY_Flag);
			else
				PC_set_rpm_Y = PC_temp_rpm_Y;
		}break;
		case PC_Motion_YN:
		{
			if (PC_Press_Shift)
				PC_set_rpm_Y = (-PC_temp_rpm_up(PC_temp_rpm_Y,XY_Flag));
			else if (PC_Press_Ctrl)
				PC_set_rpm_Y = (-PC_temp_rpm_down(PC_temp_rpm_Y,XY_Flag));
			else
				PC_set_rpm_Y = (-PC_temp_rpm_Y);
		}break;
		case PC_Motion_ZP:
		{
			if (PC_Press_Shift)
				PC_set_rpm_Z = PC_temp_rpm_up(PC_temp_rpm_Z,Z_Flag);
			else if (PC_Press_Ctrl)
				PC_set_rpm_Z = PC_temp_rpm_down(PC_temp_rpm_Z,Z_Flag);
			else
				PC_set_rpm_Z = PC_temp_rpm_Z;
		}break;
		case PC_Motion_ZN:
		{
			if (PC_Press_Shift)
				PC_set_rpm_Z = (-PC_temp_rpm_up(PC_temp_rpm_Z,Z_Flag));
			else if (PC_Press_Ctrl)
				PC_set_rpm_Z = (-PC_temp_rpm_down(PC_temp_rpm_Z,Z_Flag));
			else
				PC_set_rpm_Z = (-PC_temp_rpm_Z);
		}break;
		case PC_Motion_Reset:
		{
			PC_temp_rpm_X = XRPM_RESET;
			PC_temp_rpm_Y = YRPM_RESET;
			PC_temp_rpm_Z = ZRPM_RESET;
		}break;//�����ٶȼ���������
		case PC_Action_Reset:
		{
			
		}break;//��̨�ǶȻ���
		default:
		{
			PC_set_rpm_X = 0;
			PC_set_rpm_Y = 0;
			PC_set_rpm_Z = 0;
		}break;//���̾�ֹ�������ٶ�Ϊ0
	}
	//���ٶ�ֵ����������API

}
*/
/******************************************************************
�����̨����
���ܣ������ƫ�Ƶ��ٶ�ת��Ϊ��̨yaw��pitch��ת���ٶ�
���ã���̨���ƴ��롢PC��������
��ʽ���ٶȿ���
������� ����Ϊ��̨�˶�
mouse X ��̨��yaw����ת��		//����λ 360��
mouse Y ��̨��pitch����ת��	//����λ ������
mouse Z ����

*******************************************************************/
void PC_Remote_Gimball(void)
{
	
}

/******************************************************************
�����̨����
���ܣ�����갴����ֵת��Ϊ��̨����ָ��
���ã���̨���ƴ��롢PC��������
��ʽ���߼�����
������� ����Ϊ��̨����
mouse left_key 	�������
mouse right_key	�Ҽ�����
*******************************************************************/
void PC_Remote_Action(void)
{
	if (PC_Mouse_Left & ~PC_Mouse_Right)
	{
		
	}//��������
	else if (PC_Mouse_Right & PC_Mouse_Left)
	{
		
	}//����
	else
	{
		
	}//�޶���
}



