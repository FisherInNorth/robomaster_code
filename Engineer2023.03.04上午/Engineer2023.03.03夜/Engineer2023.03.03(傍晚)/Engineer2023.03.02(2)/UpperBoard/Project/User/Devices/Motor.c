#include "Motor.h"
#include "pid.h"

MOTOR_t longitudinal_motorL;//���������ƶ����ĵ��
MOTOR_t longitudinal_motorR;//���������ƶ��Ҳ�ĵ��

MOTOR_t inside_lift_motorL;//�ڲ����̧���ĵ��
MOTOR_t inside_lift_motorR;//�ڲ��ұ�̧���ĵ��

MOTOR_t horizontal_motor;//���ƺ����ƶ��ĵ��
MOTOR_t chuck_yaw; //��������yaw
MOTOR_t chuck_pitch; //��������pitch
MOTOR_t chuck_roll; //��������pitch


/*�������������ƶ��ĵ�����ٶȻ��ͽǶȻ�pid����*/
extern float Longitudinal_MotorL_vPID_Parameters;
extern float Longitudinal_MotorL_aPID_Parameters;
extern float Longitudinal_MotorR_vPID_Parameters;
extern float Longitudinal_MotorR_aPID_Parameters;
/*�����ڲ�̧��������ٶȻ��ͽǶȻ�pid����*/
extern float Inside_MotorL_vPID_Parameters;
extern float Inside_MotorL_aPID_Parameters;
extern float Inside_MotorR_vPID_Parameters;
extern float Inside_MotorR_aPID_Parameters;
/*���ƺ����ƶ��ĵ�����ٶȻ��ͽǶȻ�pid����*/
extern float horizontal_motor_vPID_Parameters;
extern float horizontal_motor_aPID_Parameters;
/*��������yaw������ٶȻ��ͽǶȻ�pid����*/
extern float chuck_yaw_vPID_Parameters;
extern float chuck_yaw_aPID_Parameters;
/*��������pitch������ٶȻ��ͽǶȻ�pid����*/
extern float chuck_pitch_vPID_Parameters;
extern float chuck_pitch_aPID_Parameters;
/*��������roll������ٶȻ��ͽǶȻ�pid����*/
extern float chuck_roll_vPID_Parameters;
extern float chuck_roll_aPID_Parameters;

static void Set_Longitudinal_Motor_Current(void);
static void Set_Lift_Current(void);
static void Set_Horizontal_Motor_Current(void);
static void Set_Chuck_Yaw_Current(void);
static void Set_Chuck_Pitch_Current(void);
static void Set_Chuck_Roll_Current(void);
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

void Horizontal_Init(void)
{
	horizontal_motor.target_current=0;
	horizontal_motor.actual_current=0;
	horizontal_motor.round_cnt=0.0f;
	horizontal_motor.vpid.target_speed=0;
	horizontal_motor.vpid.actual_speed=0.0f;
	horizontal_motor.vpid.err=0;
	horizontal_motor.vpid.last_err=0;
	horizontal_motor.vpid.err_integration=0;
	horizontal_motor.vpid.P_OUT=0;
	horizontal_motor.vpid.I_OUT=0;
	horizontal_motor.vpid.D_OUT=0;
	horizontal_motor.vpid.PID_OUT=0;
	horizontal_motor.vPID_max=3000;
	horizontal_motor.aPID_OUT_MAX=15000;
	horizontal_motor.apid.err=0;
	horizontal_motor.apid.last_err=0;
	horizontal_motor.apid.err_integration=0;
	horizontal_motor.apid.actual_angle=0;
	horizontal_motor.apid.target_angle=0;
	horizontal_motor.apid.P_OUT=0;
	horizontal_motor.apid.I_OUT=0;
	horizontal_motor.apid.D_OUT=0;
	horizontal_motor.apid.PID_OUT=0;
  horizontal_motor.apid.total_angle=0;	

}

void Chuck_Yaw_Init()
{
	chuck_yaw.target_current=0;
	chuck_yaw.actual_current=0;
	chuck_yaw.round_cnt=0.0f;
	chuck_yaw.vpid.target_speed=0;
	chuck_yaw.vpid.actual_speed=0.0f;
	chuck_yaw.vpid.err=0;
	chuck_yaw.vpid.last_err=0;
	chuck_yaw.vpid.err_integration=0;
	chuck_yaw.vpid.P_OUT=0;
	chuck_yaw.vpid.I_OUT=0;
	chuck_yaw.vpid.D_OUT=0;
	chuck_yaw.vpid.PID_OUT=0;
	chuck_yaw.vPID_max=3000;
	chuck_yaw.aPID_OUT_MAX=15000;
	chuck_yaw.apid.err=0;
	chuck_yaw.apid.last_err=0;
	chuck_yaw.apid.err_integration=0;
	chuck_yaw.apid.actual_angle=0;
	chuck_yaw.apid.target_angle=0;
	chuck_yaw.apid.P_OUT=0;
	chuck_yaw.apid.I_OUT=0;
	chuck_yaw.apid.D_OUT=0;
	chuck_yaw.apid.PID_OUT=0;
  chuck_yaw.apid.total_angle=0;
}

void Chuck_Pitch_Init()
{
	chuck_pitch.target_current=0;
	chuck_pitch.actual_current=0;
	chuck_pitch.round_cnt=0.0f;
	chuck_pitch.vpid.target_speed=0;
	chuck_pitch.vpid.actual_speed=0.0f;
	chuck_pitch.vpid.err=0;
	chuck_pitch.vpid.last_err=0;
	chuck_pitch.vpid.err_integration=0;
	chuck_pitch.vpid.P_OUT=0;
	chuck_pitch.vpid.I_OUT=0;
	chuck_pitch.vpid.D_OUT=0;
	chuck_pitch.vpid.PID_OUT=0;
	chuck_pitch.vPID_max=3000;
	chuck_pitch.aPID_OUT_MAX=15000;
	chuck_pitch.apid.err=0;
	chuck_pitch.apid.last_err=0;
	chuck_pitch.apid.err_integration=0;
	chuck_pitch.apid.actual_angle=0;
	chuck_pitch.apid.target_angle=0;
	chuck_pitch.apid.P_OUT=0;
	chuck_pitch.apid.I_OUT=0;
	chuck_pitch.apid.D_OUT=0;
	chuck_pitch.apid.PID_OUT=0;
  chuck_pitch.apid.total_angle=0;
}

void Chuck_Roll_Init()
{
	chuck_roll.target_current=0;
	chuck_roll.actual_current=0;
	chuck_roll.round_cnt=0.0f;
	chuck_roll.vpid.target_speed=0;
	chuck_roll.vpid.actual_speed=0.0f;
	chuck_roll.vpid.err=0;
	chuck_roll.vpid.last_err=0;
	chuck_roll.vpid.err_integration=0;
	chuck_roll.vpid.P_OUT=0;
	chuck_roll.vpid.I_OUT=0;
	chuck_roll.vpid.D_OUT=0;
	chuck_roll.vpid.PID_OUT=0;
	chuck_roll.vPID_max=3000;
	chuck_roll.aPID_OUT_MAX=15000;
	chuck_roll.apid.err=0;
	chuck_roll.apid.last_err=0;
	chuck_roll.apid.err_integration=0;
	chuck_roll.apid.actual_angle=0;
	chuck_roll.apid.target_angle=0;
	chuck_roll.apid.P_OUT=0;
	chuck_roll.apid.I_OUT=0;
	chuck_roll.apid.D_OUT=0;
	chuck_roll.apid.PID_OUT=0;
  chuck_roll.apid.total_angle=0;
}
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
		case CANRX_Inside_Lift_MotorL_ID:	
			Motor_RecordData(&inside_lift_motorL,Data);	
			InsideLiftMotor_PID_Calc(&inside_lift_motorL, &Inside_MotorL_aPID_Parameters, &Inside_MotorL_vPID_Parameters); 
			Set_Lift_Current(); 
		break;
		case CANRX_Inside_Lift_MotorR_ID:	
			Motor_RecordData(&inside_lift_motorR,Data);	
			InsideLiftMotor_PID_Calc(&inside_lift_motorR, &Inside_MotorR_aPID_Parameters, &Inside_MotorR_vPID_Parameters);
			Set_Lift_Current(); 
		break;
		case CANRX_Horizontal_Motor_ID:	
			Motor_RecordData(&horizontal_motor,Data);	
			HorizontalMotor_PID_Calc(&horizontal_motor, &horizontal_motor_aPID_Parameters, &horizontal_motor_vPID_Parameters); 
			Set_Horizontal_Motor_Current(); 
		break;
	}
}

void CAN2_Motor_HandleMsg(uint32_t StdId, unsigned char * Data)
{
	switch (StdId)
	{
		case CANRX_Longitudinal_MotorL_ID:		
			Motor_RecordData(&longitudinal_motorL,Data);	
			Longitudinal_Motor_PID_Calc(&longitudinal_motorL, &Longitudinal_MotorL_aPID_Parameters, &Longitudinal_MotorL_vPID_Parameters);	
			Set_Longitudinal_Motor_Current();	
		break;
		case CANRX_Longitudinal_MotorR_ID:		
			Motor_RecordData(&longitudinal_motorR,Data);	
			Longitudinal_Motor_PID_Calc(&longitudinal_motorR, &Longitudinal_MotorR_aPID_Parameters, &Longitudinal_MotorR_vPID_Parameters);	
			Set_Longitudinal_Motor_Current();	
		break;
		case CANRX_Chuck_Yaw_ID:	
			Motor_RecordData(&chuck_yaw,Data);	
			ChuckYaw_PID_Calc(&chuck_yaw, &chuck_yaw_aPID_Parameters, &chuck_yaw_vPID_Parameters); 
			Set_Chuck_Yaw_Current(); 
		break;
		case CANRX_Chuck_Pitch_ID:	
			Motor_RecordData(&chuck_pitch,Data);	
			ChuckYaw_PID_Calc(&chuck_pitch, &chuck_pitch_aPID_Parameters, &chuck_pitch_vPID_Parameters); 
			Set_Chuck_Pitch_Current(); 
		break;
		case CANRX_Chuck_Roll_ID:	
			Motor_RecordData(&chuck_roll,Data);	
			ChuckRoll_PID_Calc(&chuck_roll, &chuck_roll_aPID_Parameters, &chuck_roll_vPID_Parameters); 
			Set_Chuck_Roll_Current(); 
		break;
	}
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
	* @brief		������CAN����Ħ���ֵ���ʼ�(���ĸ�������ʯ��̬�ĵ��)
	* @param		���룺����Ϊ8������
	* @retval	����ֵ����
	*/
static void Set_Lift_Current(void)
{
	unsigned char current_msg[8];
	
	//���Ŀ�����Ϊ�ٶ�pid���
	inside_lift_motorL.target_current = inside_lift_motorL.vpid.PID_OUT;	//
	inside_lift_motorR.target_current = inside_lift_motorR.vpid.PID_OUT;	//

	current_msg[0] = inside_lift_motorL.target_current >> 8;			//���̧�����������8λ
	current_msg[1] = inside_lift_motorL.target_current & 0xff;		//���̧�����������8λ
	current_msg[2] = inside_lift_motorR.target_current >> 8;			//�Ҳ�̧�����������8λ
	current_msg[3] = inside_lift_motorR.target_current & 0xff;		//�Ҳ�̧�����������8λ


	//can��������֡
	CAN1_Send_Msg(0x200, current_msg);
}

static void Set_Longitudinal_Motor_Current(void)
{
	unsigned char current_msg[8];
	
	//���Ŀ�����Ϊ�ٶ�pid���
	longitudinal_motorL.target_current = longitudinal_motorL.vpid.PID_OUT;	//
	longitudinal_motorR.target_current = longitudinal_motorR.vpid.PID_OUT;	//

	current_msg[0] = longitudinal_motorL.target_current >> 8;			//1�ŵ��������8λ
	current_msg[1] = longitudinal_motorL.target_current & 0xff;		//1�ŵ��������8λ
	current_msg[2] = longitudinal_motorR.target_current >> 8;			//2�ŵ��������8λ
	current_msg[3] = longitudinal_motorR.target_current & 0xff;		//2�ŵ��������8λ

	//can��������֡
	CAN2_Send_Msg(0x200, current_msg);
}

static void Set_Horizontal_Motor_Current(void)
{
	unsigned char current_msg[8];
	
	//���Ŀ�����Ϊ�ٶ�pid���
	horizontal_motor.target_current = horizontal_motor.vpid.PID_OUT;	//

	current_msg[4] = horizontal_motor.target_current >> 8;			//1�ŵ��������8λ
	current_msg[5] = horizontal_motor.target_current & 0xff;		//1�ŵ��������8λ
//	current_msg[4] = 0 >> 8;			//1�ŵ��������8λ
//	current_msg[5] = 0 & 0xff;		//1�ŵ��������8λ

	//can��������֡
	CAN1_Send_Msg(0x200, current_msg);
}

static void Set_Chuck_Yaw_Current(void)
{
	unsigned char current_msg[8];
	
	//���Ŀ�����Ϊ�ٶ�pid���
	chuck_yaw.target_current = chuck_yaw.vpid.PID_OUT;	//

	current_msg[4] = chuck_yaw.target_current >> 8;			//1�ŵ��������8λ
	current_msg[5] = chuck_yaw.target_current & 0xff;		//1�ŵ��������8λ


	//can��������֡
	CAN2_Send_Msg(0x200, current_msg);
}

static void Set_Chuck_Pitch_Current(void)
{
	unsigned char current_msg[8];
	
	//���Ŀ�����Ϊ�ٶ�pid���
	chuck_pitch.target_current = chuck_pitch.vpid.PID_OUT;	//

	current_msg[6] = chuck_pitch.target_current >> 8;			//1�ŵ��������8λ
	current_msg[7] = chuck_pitch.target_current & 0xff;		//1�ŵ��������8λ


	//can��������֡
	CAN2_Send_Msg(0x200, current_msg);
}

static void Set_Chuck_Roll_Current(void)
{
	unsigned char current_msg[8];
	
	//���Ŀ�����Ϊ�ٶ�pid���
	chuck_roll.target_current = chuck_roll.vpid.PID_OUT;	//

	current_msg[0] = chuck_roll.target_current >> 8;			//1�ŵ��������8λ
	current_msg[1] = chuck_roll.target_current & 0xff;		//1�ŵ��������8λ


	//can��������֡
	CAN2_Send_Msg(0x201, current_msg);
}
