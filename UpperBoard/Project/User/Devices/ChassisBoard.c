#include "ChassisBoard.h"
#include "bsp_led.h"
#include "DR16.h"

unsigned char Lift_Motor_State=0;						//̧�����״̬
unsigned char Longitudinal_Motor_State=0;		//�������״̬
unsigned char Horizontal_Motor_State=0;			//���Ƶ��״̬
unsigned char Chuck_Yaw_State=0;		        //Yaw���״̬
unsigned char Chuck_Pitch_State=0;					//Pitcḩ�����״̬
unsigned char Chuck_Roll_State=0;		        //roll���״̬
unsigned char PumpAll_State=0;		        //����״̬
unsigned char View_Servo_State=0;		        //ͼ�����
unsigned char Longitudinal_Ready_State=0;	//�������һ��ȡ���ջ�ʱ�Ƿ��Ѿ���ȫ�ջ�
unsigned char Lift_Ready_State=0;         //̧������Ƿ�̧��Ԥ���߶�
unsigned char Roll_Ready_State=0;
unsigned char Longitudinal_Location=0;
unsigned char Lift_Location=0;
unsigned char Chuck_Roll_Location=0;
unsigned char HP_Zero=0;
unsigned char Pump_State=0;
unsigned char Out_Lift_Motor_State=0;
unsigned char Chassis_vx_State=0;
unsigned char Chassis_vy_State=0;
unsigned char Chassis_wz_State=0;


//...(����״̬��־λ)...

unsigned char ChassisBoard_Data[100] = {0};
unsigned char Transmit_ChassisBoard_Data[16] = {0};
unsigned char pChassisBoard_DataH = 0;
unsigned char pChassisBoard_DataL = 0;

unsigned char target_ChassisBoard_Data;

void Resolve_ChassisBoard_Data(void)
{
	while(pChassisBoard_DataL != pChassisBoard_DataH)
	{
		pChassisBoard_DataL ++;
		if(pChassisBoard_DataL == 100)
		{
			pChassisBoard_DataL = 0;
		}
		
		target_ChassisBoard_Data = ChassisBoard_Data[pChassisBoard_DataL];
		
		switch(target_ChassisBoard_Data >> 4)
		{
//			case 0x0001:	Lift_Motor_State = target_ChassisBoard_Data & 0x0f; break;
//			case 0x0002:	Longitudinal_Motor_State = target_ChassisBoard_Data & 0x0f; break;
//			case 0x0003:  Horizontal_Motor_State = target_ChassisBoard_Data & 0x0f;break;
//			case 0x0004:  Chuck_Yaw_State = target_ChassisBoard_Data & 0x0f;break;
//			case 0x0005:  Chuck_Pitch_State = target_ChassisBoard_Data & 0x0f;break;
//			case 0x0006:  Chuck_Roll_State = target_ChassisBoard_Data & 0x0f;break;
//			case 0x0007:  View_Servo_State = target_ChassisBoard_Data & 0x0f;break;
			case 0x0008:  HP_Zero = target_ChassisBoard_Data & 0x0f;break;

			//...(���֧��ʮ����)...
			default:break;
		}
	}
}

void Load_ChassisBoard_Data(void)
{
	Transmit_ChassisBoard_Data[0] = 0x01 << 4;
	Transmit_ChassisBoard_Data[0] |= Longitudinal_Ready_State;
	
	Transmit_ChassisBoard_Data[1] = 0x02 << 4;
	Transmit_ChassisBoard_Data[1] |= Lift_Ready_State;
	
	Transmit_ChassisBoard_Data[2] = 0x03 << 4;
	Transmit_ChassisBoard_Data[2] |= Roll_Ready_State;
	
	Transmit_ChassisBoard_Data[3] = 0x04 << 4;
	Transmit_ChassisBoard_Data[3] |= Longitudinal_Location & 0x0f;
	
	Transmit_ChassisBoard_Data[4] = 0x05 << 4;
	Transmit_ChassisBoard_Data[4] |= Longitudinal_Location >> 4;
	
	Transmit_ChassisBoard_Data[5] = 0x06 << 4;
	Transmit_ChassisBoard_Data[5] |= Lift_Location & 0x0f;
	
	Transmit_ChassisBoard_Data[6] = 0x07 << 4;
	Transmit_ChassisBoard_Data[6] |= Lift_Location >> 4;
	
	Transmit_ChassisBoard_Data[7] = 0x08 << 4;
	Transmit_ChassisBoard_Data[7] |= Chuck_Roll_Location & 0x0f;
	
	Transmit_ChassisBoard_Data[8] = 0x09 << 4;
	Transmit_ChassisBoard_Data[8] |= Chuck_Roll_Location >> 4;
	
	Transmit_ChassisBoard_Data[9] = 0x0A << 4;
	Transmit_ChassisBoard_Data[9] |= Out_Lift_Motor_State;
	
	Transmit_ChassisBoard_Data[10] = 0x0B << 4;
	Transmit_ChassisBoard_Data[10] |= Chassis_vx_State;
	
	Transmit_ChassisBoard_Data[11] = 0x0C << 4;
	Transmit_ChassisBoard_Data[11] |= Chassis_vy_State;
	
	Transmit_ChassisBoard_Data[12] = 0x0D << 4;
	Transmit_ChassisBoard_Data[12] |= Chassis_wz_State;
	
	Transmit_ChassisBoard_Data[13] = 0x0E << 4;
	Transmit_ChassisBoard_Data[13] |= Pump_State;
	
	Transmit_ChassisBoard_Data[14] = 0x0F << 4;
	Transmit_ChassisBoard_Data[14] |= flag_v;
}