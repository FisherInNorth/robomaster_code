#include "Motor.h"
#include "pid.h"


MOTOR_t chassis_motor1;//��һ���޵ĵ��
MOTOR_t chassis_motor2;//�ڶ����޵ĵ��
MOTOR_t chassis_motor3;//�������޵ĵ��
MOTOR_t chassis_motor4;//�������޵ĵ��
MOTOR_t outboard_lift_motorL;//������̧���ĵ��
MOTOR_t outboard_lift_motorR;//����ұ�̧���ĵ��

extern float ChassisMotor1_PID_Parameters;
extern float ChassisMotor2_PID_Parameters;
extern float ChassisMotor3_PID_Parameters;
extern float ChassisMotor4_PID_Parameters;
extern float Outboard_MotorL_vPID_Parameters;
extern float Outboard_MotorL_aPID_Parameters;
extern float Outboard_MotorR_vPID_Parameters;
extern float Outboard_MotorR_aPID_Parameters;

static void Set_Chassis_Current(void);
static void Set_Lift_Current(void);
static void Motor_RecordData(MOTOR_t *motor, unsigned char * data);





/**
	* @function����:Mineral_Init
	* @brief����:��е�۳�ʼ��
	* @param����:��
	* @retval����ֵ:��
  */
void Mineral_Init(void)
{
//	mineral_longitudinal_move=stop;           mineral_widthwise_move=stop;
//	mineral_F_motor.round_cnt=0.0f;			      mineral_B_motor.round_cnt=0.0f;
//	mineral_F_motor.target_current=0;		      mineral_B_motor.target_current=0;
//	mineral_F_motor.actual_current=0;		      mineral_B_motor.actual_current=0;
//	mineral_F_motor.vpid.target_speed=0;			mineral_B_motor.vpid.target_speed=0;
//	mineral_F_motor.vpid.actual_speed=0.0f;	  mineral_B_motor.vpid.actual_speed=0.0f;
//	mineral_F_motor.vpid.err=0;               mineral_B_motor.vpid.err=0;
//	mineral_F_motor.vpid.last_err=0;          mineral_B_motor.vpid.last_err=0;
//	mineral_F_motor.vpid.err_integration=0;   mineral_B_motor.vpid.err_integration=0; 
//	mineral_F_motor.vpid.P_OUT=0;             mineral_B_motor.vpid.P_OUT=0;
//	mineral_F_motor.vpid.I_OUT=0;             mineral_B_motor.vpid.I_OUT=0;
//	mineral_F_motor.vpid.D_OUT=0;             mineral_B_motor.vpid.D_OUT=0;
//	mineral_F_motor.vpid.PID_OUT=0;           mineral_B_motor.vpid.PID_OUT=0;
//	mineral_F_motor.vPID_max=3000;						mineral_B_motor.vPID_max=3000;

//	mineral_R_motor.round_cnt=0.0f;			      mineral_L_motor.round_cnt=0.0f;
//	mineral_R_motor.target_current=0;		      mineral_L_motor.target_current=0;
//	mineral_R_motor.actual_current=0;		      mineral_L_motor.actual_current=0;
//	mineral_R_motor.vpid.target_speed=0;			mineral_L_motor.vpid.target_speed=0;
//	mineral_R_motor.vpid.actual_speed=0.0f;	  mineral_L_motor.vpid.actual_speed=0.0f;
//	mineral_R_motor.vpid.err=0;               mineral_L_motor.vpid.err=0;
//	mineral_R_motor.vpid.last_err=0;          mineral_L_motor.vpid.last_err=0;
//	mineral_R_motor.vpid.err_integration=0;   mineral_L_motor.vpid.err_integration=0; 
//	mineral_R_motor.vpid.P_OUT=0;             mineral_L_motor.vpid.P_OUT=0;
//	mineral_R_motor.vpid.I_OUT=0;             mineral_L_motor.vpid.I_OUT=0;
//	mineral_R_motor.vpid.D_OUT=0;             mineral_L_motor.vpid.D_OUT=0;
//	mineral_R_motor.vpid.PID_OUT=0;           mineral_L_motor.vpid.PID_OUT=0;
//	mineral_R_motor.vPID_max=3000;						mineral_L_motor.vPID_max=3000;

}

void Flip_Init(void)
{
////	flip_move=stop;
//	flip_R_motor.target_current=0;
//	flip_R_motor.actual_current=0;
//	flip_R_motor.round_cnt=0.0f;
//	flip_R_motor.vpid.target_speed=0;
//	flip_R_motor.vpid.actual_speed=0.0f;
//	flip_R_motor.vpid.err=0;
//	flip_R_motor.vpid.last_err=0;
//	flip_R_motor.vpid.err_integration=0;
//	flip_R_motor.vpid.P_OUT=0;
//	flip_R_motor.vpid.I_OUT=0;
//	flip_R_motor.vpid.D_OUT=0;
//	flip_R_motor.vpid.PID_OUT=0;
//	flip_R_motor.vPID_max=6000;
//	flip_L_motor.target_current=0;
//	flip_L_motor.actual_current=0;
//	flip_L_motor.round_cnt=0.0f;
//	flip_L_motor.vpid.target_speed=0;
//	flip_L_motor.vpid.actual_speed=0.0f;
//	flip_L_motor.vpid.err=0;
//	flip_L_motor.vpid.last_err=0;
//	flip_L_motor.vpid.err_integration=0;
//	flip_L_motor.vpid.P_OUT=0;
//	flip_L_motor.vpid.I_OUT=0;
//	flip_L_motor.vpid.D_OUT=0;
//	flip_L_motor.vpid.PID_OUT=0;
//	flip_L_motor.vPID_max=6000;
//	
//	flip_R_motor.aPID_OUT_MAX=23000;
//	flip_R_motor.apid.err=0;
//	flip_R_motor.apid.last_err=0;
//	flip_R_motor.apid.err_integration=0;
//	flip_R_motor.apid.actual_angle=0;
//	flip_R_motor.apid.target_angle=0;
//	flip_R_motor.apid.P_OUT=0;
//	flip_R_motor.apid.I_OUT=0;
//	flip_R_motor.apid.D_OUT=0;
//	flip_R_motor.apid.PID_OUT=0;
//  flip_R_motor.apid.total_angle=0;	
//	flip_L_motor.aPID_OUT_MAX=23000;
//	flip_L_motor.apid.err=0;
//	flip_L_motor.apid.last_err=0;
//	flip_L_motor.apid.err_integration=0;
//	flip_L_motor.apid.actual_angle=0;
//	flip_L_motor.apid.target_angle=0;
//	flip_L_motor.apid.P_OUT=0;
//	flip_L_motor.apid.I_OUT=0;
//	flip_L_motor.apid.D_OUT=0;
//	flip_L_motor.apid.PID_OUT=0;
//  flip_L_motor.apid.total_angle=0;	
}

/**
	* @function	������Set_Mineral_X_Speed
	* @brief		���������õ����Ŀ���ٶ�
	* @param		���룺Ŀ���ٶ�
	* @retval	����ֵ����
	*/
//void Set_Mineral_F_Speed(int target_speed)
//{
//	mineral_F_motor.vpid.target_speed = target_speed*36; 
//}
//void Set_Mineral_B_Speed(int target_speed)
//{
//	mineral_B_motor.vpid.target_speed = target_speed*36;
//}
//void Set_Mineral_R_Speed(int target_speed)
//{
//	mineral_R_motor.vpid.target_speed = target_speed*36;
//}
//void Set_Mineral_L_Speed(int target_speed)
//{
//	mineral_L_motor.vpid.target_speed = target_speed*36;
//}


/**
	* @function	������Motor_HandleMsg
	* @brief		��������������Ϣ
	* @param		���룺CAN���ܵ��ı���ID����������
	*									���������ʽ�����ȣ�8
	*															 1��ת�ӻ�е�Ƕȸ�8λ	3.ת��ת�ٸ�8λ	5.ʵ�����ת�ظ�8λ	7.��
	*															 2��ת�ӻ�е�Ƕȵ�8λ	4.ת��ת�ٵ�8λ	6.ʵ�����ת�ص�8λ	8.��
	* @retval	����ֵ����
	*/
void CAN1_Motor_HandleMsg(uint32_t StdId, unsigned char * Data)
{
	switch (StdId)
	{
		case CANRX_Chassis_Motor1_ID:		Motor_RecordData(&chassis_motor1,Data);	ChassisMotorPID_Calc(&chassis_motor1, &ChassisMotor1_PID_Parameters);	Set_Chassis_Current();	break;
		case CANRX_Chassis_Motor2_ID:		Motor_RecordData(&chassis_motor2,Data);	ChassisMotorPID_Calc(&chassis_motor2, &ChassisMotor2_PID_Parameters);	Set_Chassis_Current();	break;
		case CANRX_Chassis_Motor3_ID:		Motor_RecordData(&chassis_motor3,Data);	ChassisMotorPID_Calc(&chassis_motor3, &ChassisMotor3_PID_Parameters);	Set_Chassis_Current();	break;
		case CANRX_Chassis_Motor4_ID:		Motor_RecordData(&chassis_motor4,Data);	ChassisMotorPID_Calc(&chassis_motor4, &ChassisMotor4_PID_Parameters);	Set_Chassis_Current();	break;
	}
//	Vpid_Calc(motor, Mineral_Kp, Mineral_Ki, Mineral_Kd);
//	Set_Mineral_Current();
}

void CAN2_Motor_HandleMsg(uint32_t StdId, unsigned char * Data)
{
	switch (StdId)
	{
		case CANRX_Outboard_Lift_MotorL_ID:	Motor_RecordData(&outboard_lift_motorL,Data);	OutboardLiftMotor_PID_Calc(&outboard_lift_motorL, &Outboard_MotorL_aPID_Parameters, &Outboard_MotorL_vPID_Parameters); Set_Lift_Current(); break;
		case CANRX_Outboard_Lift_MotorR_ID:	Motor_RecordData(&outboard_lift_motorR,Data);	OutboardLiftMotor_PID_Calc(&outboard_lift_motorR, &Outboard_MotorR_aPID_Parameters, &Outboard_MotorR_vPID_Parameters); Set_Lift_Current(); break;
	}
//	Vpid_Calc(motor, Mineral_Kp, Mineral_Ki, Mineral_Kd);
//	Set_Mineral_Current();
}

/**
	* @function	������Motor_RecordData
	* @brief		���������µ����Ϣ
	* @param		���룺����ṹ�塢��������
	*									���������ʽ�����ȣ�8
	*															 1��ת�ӻ�е�Ƕȸ�8λ	3.ת��ת�ٸ�8λ	5.ʵ�����ת�ظ�8λ	7.��
	*															 2��ת�ӻ�е�Ƕȵ�8λ	4.ת��ת�ٵ�8λ	6.ʵ�����ת�ص�8λ	8.��
	* @retval	����ֵ����
	*/
static void Motor_RecordData(MOTOR_t *motor, unsigned char * data)
{
	motor->last_angle 				= 	motor->apid.actual_angle;
	motor->apid.actual_angle 	= 	(int16_t)((data[0] << 8) + data[1]);
	motor->vpid.actual_speed 	= 	(int16_t)((data[2] << 8) + data[3]);
	motor->actual_current 		= 	(int16_t)((data[4] << 8) + data[5]);
	
	//��¼��ʼ�Ƕ�
	if(motor->start_angle_flag == 0)
	{
		motor->start_angle = (int16_t)((data[0] << 8) + data[1]);
		motor->start_angle_flag++;
	}

	//�������������νǶȱ仯����4096,��ô���ת����һȦ
	if(motor->apid.actual_angle - motor->last_angle > 4096)
	{
		motor->round_cnt --;
	}
	else if(motor->apid.actual_angle - motor->last_angle < -4096)
	{
		motor->round_cnt ++;
	}
	motor->apid.total_angle = motor->round_cnt * 8192 + motor->apid.actual_angle;	// - motor->start_angle;
}


/**
	* @function	������Debug_CAN_Send_Mineral_Msg
	* @brief		������CAN����Ħ���ֵ���ʼ�(���ĸ�������ʯ��̬�ĵ��)(��������ʱʹ��)
	* @param		���룺����Ϊ8������
	* @retval	����ֵ����
	*/
void Debug_CAN_Send_Mineral_Msg(uint8_t* msg)
{		
	CAN1_Send_Msg(0x200, msg);
}


/**
	* @function	������CAN_Send_Flip_Msg
	* @brief		������CAN���ͼ�צ����ʼ�(�����Ƽ�צ��ת�ĵ��)
	* @param		���룺����Ϊ8������
	* @retval	����ֵ����
	*/
void Debug_CAN_Send_Flip_Msg(uint8_t* msg)
{
	CAN1_Send_Msg(0x1FF, msg);
}


/**
	* @function	������Set_Mineral_Current
	* @brief		������CAN���͵��̵���ʼ�
	* @param		���룺����Ϊ8������
	* @retval	����ֵ����
	*/
static void Set_Chassis_Current(void)
{
	unsigned char current_msg[8];
	
	//���Ŀ�����Ϊ�ٶ�pid���
	chassis_motor1.target_current = chassis_motor1.vpid.PID_OUT;	//
	chassis_motor2.target_current = chassis_motor2.vpid.PID_OUT;	//
	chassis_motor3.target_current = chassis_motor3.vpid.PID_OUT;	//
	chassis_motor4.target_current = chassis_motor4.vpid.PID_OUT;	//

	current_msg[0] = chassis_motor1.target_current >> 8;			//1�ŵ��������8λ
	current_msg[1] = chassis_motor1.target_current & 0xff;		//1�ŵ��������8λ
	current_msg[2] = chassis_motor2.target_current >> 8;			//2�ŵ��������8λ
	current_msg[3] = chassis_motor2.target_current & 0xff;		//2�ŵ��������8λ

	current_msg[4] = chassis_motor3.target_current >> 8;			//3�ŵ��������8λ
	current_msg[5] = chassis_motor3.target_current & 0xff;		//3�ŵ��������8λ
	current_msg[6] = chassis_motor4.target_current >> 8;			//4�ŵ��������8λ
	current_msg[7] = chassis_motor4.target_current & 0xff;		//4�ŵ��������8λ

	//can��������֡
	CAN1_Send_Msg(0x200, current_msg);
}

/**
	* @function	������Set_Mineral_Current
	* @brief		������CAN����Ħ���ֵ���ʼ�(���ĸ�������ʯ��̬�ĵ��)
	* @param		���룺����Ϊ8������
	* @retval	����ֵ����
	*/
static void Set_Lift_Current(void)
{
	unsigned char current_msg[8];
	
	//���Ŀ�����Ϊ�ٶ�pid���
	outboard_lift_motorL.target_current = outboard_lift_motorL.vpid.PID_OUT;	//
	outboard_lift_motorR.target_current = outboard_lift_motorR.vpid.PID_OUT;	//

	current_msg[0] = outboard_lift_motorL.target_current >> 8;			//���̧�����������8λ
	current_msg[1] = outboard_lift_motorL.target_current & 0xff;		//���̧�����������8λ
	current_msg[2] = outboard_lift_motorR.target_current >> 8;			//�Ҳ�̧�����������8λ
	current_msg[3] = outboard_lift_motorR.target_current & 0xff;		//�Ҳ�̧�����������8λ


	//can��������֡
	CAN2_Send_Msg(0x200, current_msg);
}
