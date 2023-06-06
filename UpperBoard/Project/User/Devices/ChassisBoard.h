#ifndef __CHASSISBOARD_H__
#define __CHASSISBOARD_H__

extern unsigned char Lift_Motor_State;						//̧�����״̬
extern unsigned char Longitudinal_Motor_State;		//�������״̬
extern unsigned char Horizontal_Motor_State;			//���Ƶ��״̬
extern unsigned char Chuck_Yaw_State;		        //Yaw���״̬
extern unsigned char Chuck_Pitch_State;					//Pitcḩ�����״̬
extern unsigned char Chuck_Roll_State;		        //roll���״̬
extern unsigned char PumpAll_State;		        //����״̬
extern unsigned char View_Servo_State;		        //ͼ�����
extern unsigned char Longitudinal_Ready_State;	//�������һ��ȡ���ջ�ʱ�Ƿ��Ѿ���ȫ�ջ�
extern unsigned char Lift_Ready_State;          //̧������Ƿ�ﵽԤ���߶�
extern unsigned char ChassisBoard_Data[];
extern unsigned char Transmit_ChassisBoard_Data[];
extern unsigned char pChassisBoard_DataH;
extern unsigned char pChassisBoard_DataL;
extern unsigned char Roll_Ready_State;
extern unsigned char Longitudinal_Location;
extern unsigned char Lift_Location;
extern unsigned char Chuck_Roll_Location;
extern unsigned char HP_Zero;
extern unsigned char Pump_State;
extern unsigned char Out_Lift_Motor_State;
extern unsigned char Chassis_vx_State;
extern unsigned char Chassis_vy_State;
extern unsigned char Chassis_wz_State;


void Resolve_ChassisBoard_Data(void);
void Load_ChassisBoard_Data(void);

#endif
