#include "Motor.h"
#include "pid.h"
#include "protocol.h"
#include "referee.h"

MOTOR_t longitudinal_motorL;//控制纵向移动左侧的电机
MOTOR_t longitudinal_motorR;//控制纵向移动右侧的电机

MOTOR_t inside_lift_motorL;//内侧左边抬升的电机
MOTOR_t inside_lift_motorR;//内侧右边抬升的电机

MOTOR_t horizontal_motor;//控制横向移动的电机
MOTOR_t chuck_yaw; //控制吸盘yaw
MOTOR_t chuck_pitch; //控制吸盘pitch
MOTOR_t chuck_roll; //控制吸盘pitch
uint8_t start_flag=0;

/*两个控制纵向移动的电机的速度环和角度环pid参数*/
extern float Longitudinal_MotorL_vPID_Parameters;
extern float Longitudinal_MotorL_aPID_Parameters;
extern float Longitudinal_MotorR_vPID_Parameters;
extern float Longitudinal_MotorR_aPID_Parameters;

/*两个内侧抬升电机的速度环和角度环pid参数*/
extern float Inside_MotorL_vPID_Parameters;
extern float Inside_MotorL_aPID_Parameters;
extern float Inside_MotorR_vPID_Parameters;
extern float Inside_MotorR_aPID_Parameters;

/*控制横向移动的电机的速度环和角度环pid参数*/
extern float horizontal_motor_vPID_Parameters;
extern float horizontal_motor_aPID_Parameters;

/*控制吸盘yaw电机的速度环和角度环pid参数*/
extern float chuck_yaw_vPID_Parameters;
extern float chuck_yaw_aPID_Parameters;

/*控制吸盘pitch电机的速度环和角度环pid参数*/
extern float chuck_pitch_vPID_Parameters;
extern float chuck_pitch_aPID_Parameters;

/*控制吸盘roll电机的速度环和角度环pid参数*/
extern float chuck_roll_vPID_Parameters;
extern float chuck_roll_aPID_Parameters;

extern custom_robot_data_t custom_robot_data;



static void Set_CAN1_Low_Current(void);
static void Motor_RecordData(MOTOR_t *motor, unsigned char * data);
static void Set_CAN2_Low_Current(void);



void motor_angle_init(void)
{
//	longitudinal_motorL.last_angle=
	inside_lift_motorL.last_angle=4096;
	inside_lift_motorR.last_angle=4096;
	chuck_pitch.last_angle=4096;
	chuck_roll.last_angle=4096;

}
/**
	* @function	函数：Motor_HandleMsg
	* @brief		描述：处理电机消息
	* @param		输入：CAN接受到的报文ID、数据数组
	*									数据数组格式：长度：8
	*															 1：转子机械角度高8位	3.转子转速高8位	5.实际输出转矩高8位	7.空
	*															 2：转子机械角度低8位	4.转子转速低8位	6.实际输出转矩低8位	8.空
	* @retval	返回值：无
	*/
void CAN1_Motor_HandleMsg(uint32_t StdId, unsigned char * Data)
{
	switch (StdId)
	{
	
		
		case CANRX_Inside_Lift_MotorL_ID:	
			Motor_RecordData(&inside_lift_motorL,Data);	
//			InsideLiftMotor_PID_Calc(&inside_lift_motorL, &Inside_MotorL_aPID_Parameters, &Inside_MotorL_vPID_Parameters); 
//			Set_CAN1_Low_Current();
//			start_flag=1;
		break;
		
		case CANRX_Inside_Lift_MotorR_ID:	
			Motor_RecordData(&inside_lift_motorR,Data);	
//			InsideLiftMotor_PID_Calc(&inside_lift_motorR, &Inside_MotorR_aPID_Parameters, &Inside_MotorR_vPID_Parameters);
//			Set_CAN1_Low_Current();
		break;
	}
}

void CAN2_Motor_HandleMsg(uint32_t StdId, unsigned char * Data)
{
	switch (StdId)
	{
		case CANRX_Longitudinal_MotorL_ID:		
			Motor_RecordData(&longitudinal_motorL,Data);	
//			Longitudinal_Motor_PID_Calc(&longitudinal_motorL, &Longitudinal_MotorL_aPID_Parameters, &Longitudinal_MotorL_vPID_Parameters);	
//			Set_CAN2_Low_Current();
			
		break;
		
		case CANRX_Longitudinal_MotorR_ID:		
			Motor_RecordData(&longitudinal_motorR,Data);	
//			Longitudinal_Motor_PID_Calc(&longitudinal_motorR, &Longitudinal_MotorR_aPID_Parameters, &Longitudinal_MotorR_vPID_Parameters);	
//			Set_CAN2_Low_Current();
		break;
		
//		case CANRX_Horizontal_Motor_ID:	
//			Motor_RecordData(&horizontal_motor,Data);	
//			HorizontalMotor_PID_Calc(&horizontal_motor, &horizontal_motor_aPID_Parameters, &horizontal_motor_vPID_Parameters); 
//			Set_CAN2_Low_Current(); 
//		break;
//		case CANRX_Chuck_Yaw_ID:	
//			Motor_RecordData(&chuck_yaw,Data);	
//			ChuckYaw_PID_Calc(&chuck_yaw, &chuck_yaw_aPID_Parameters, &chuck_yaw_vPID_Parameters); 
//			Set_CAN2_Low_Current(); 
//		break;
		case CANRX_Chuck_Pitch_ID:	
			Motor_RecordData(&chuck_pitch,Data);	
//			ChuckYaw_PID_Calc(&chuck_pitch, &chuck_pitch_aPID_Parameters, &chuck_pitch_vPID_Parameters); 
//			Set_CAN2_Low_Current(); 
		break;
		case CANRX_Chuck_Roll_ID:	
			Motor_RecordData(&chuck_roll,Data);	
			start_flag=1;
//			ChuckRoll_PID_Calc(&chuck_roll, &chuck_roll_aPID_Parameters, &chuck_roll_vPID_Parameters); 
//			Set_CAN2_Low_Current(); 
		break;
	}
}

void CAN1_Motor_ControlMsg(void)
{
	
  InsideLiftMotor_PID_Calc(&inside_lift_motorL, &Inside_MotorL_aPID_Parameters, &Inside_MotorL_vPID_Parameters);
  InsideLiftMotor_PID_Calc(&inside_lift_motorR, &Inside_MotorR_aPID_Parameters, &Inside_MotorR_vPID_Parameters);
  Set_CAN1_Low_Current();	
}

void CAN2_Motor_ControlMsg(void)
{
	Longitudinal_Motor_PID_Calc(&longitudinal_motorL, &Longitudinal_MotorL_aPID_Parameters, &Longitudinal_MotorL_vPID_Parameters);
  Longitudinal_Motor_PID_Calc(&longitudinal_motorR, &Longitudinal_MotorR_aPID_Parameters, &Longitudinal_MotorR_vPID_Parameters);
	ChuckPitch_PID_Calc(&chuck_pitch, &chuck_pitch_aPID_Parameters, &chuck_pitch_vPID_Parameters);
	ChuckRoll_PID_Calc(&chuck_roll, &chuck_roll_aPID_Parameters, &chuck_roll_vPID_Parameters);
	Set_CAN2_Low_Current();
}

/**
	* @function	函数：Motor_RecordData
	* @brief		描述：更新电机信息
	* @param		输入：电机结构体、数据数组
	*									数据数组格式：长度：8
	*															 1：转子机械角度高8位	3.转子转速高8位	5.实际输出转矩高8位	7.空
	*															 2：转子机械角度低8位	4.转子转速低8位	6.实际输出转矩低8位	8.空
	* @retval	返回值：无
	*/
static void Motor_RecordData(MOTOR_t *motor, unsigned char * data)
{
	motor->apid.actual_angle 	= 	(int16_t)((data[0] << 8) + data[1]);
	motor->vpid.actual_speed 	= 	(int16_t)((data[2] << 8) + data[3]);
	motor->actual_current 		= 	(int16_t)((data[4] << 8) + data[5]);
	
	//记录初始角度
	if(motor->start_angle_flag == 0)
	{
		motor->start_angle = (int16_t)((data[0] << 8) + data[1]);
		motor->start_angle_flag++;
	}

	//如果电机相邻两次角度变化超过4096,那么电机转过了一圈
	if(motor->apid.actual_angle - motor->last_angle > 4096)
	{
		motor->round_cnt --;
	}
	else if(motor->apid.actual_angle - motor->last_angle < -4096)
	{
		motor->round_cnt ++;
	}
	motor->apid.total_angle = motor->round_cnt * 8192 + motor->apid.actual_angle - motor->start_angle;
	motor->last_angle 				= 	motor->apid.actual_angle;
}


/**
	* @function	函数：Set_Mineral_Current
	* @brief		描述：CAN发送摩擦轮电机邮件(即四个调整矿石姿态的电机)
	* @param		输入：长度为8的数组
	* @retval	返回值：无
	*/
static void Set_CAN1_Low_Current(void)
{
	unsigned char current_msg[8];
	
	//电机目标电流为速度pid输出
	inside_lift_motorL.target_current = inside_lift_motorL.vpid.PID_OUT;	//
	inside_lift_motorR.target_current = inside_lift_motorR.vpid.PID_OUT;	//
	
	longitudinal_motorL.target_current = longitudinal_motorL.vpid.PID_OUT;	//
	longitudinal_motorR.target_current = longitudinal_motorR.vpid.PID_OUT;	//

//	current_msg[0] = longitudinal_motorL.target_current >> 8;			//1号电机电流高8位
//	current_msg[1] = longitudinal_motorL.target_current & 0xff;		//1号电机电流低8位
//	current_msg[2] = longitudinal_motorR.target_current >> 8;			//2号电机电流高8位
//	current_msg[3] = longitudinal_motorR.target_current & 0xff;		//2号电机电流低8位
	
	current_msg[4] = inside_lift_motorL.target_current >> 8;			//左侧抬升电机电流高8位
	current_msg[5] = inside_lift_motorL.target_current & 0xff;		//左侧抬升电机电流低8位
	current_msg[6] = inside_lift_motorR.target_current >> 8;			//右侧抬升电机电流高8位
	current_msg[7] = inside_lift_motorR.target_current & 0xff;		//右侧抬升电机电流低8位
	
	//can发送数据帧
	CAN1_Send_Msg(0x200, current_msg);
}



static void Set_CAN2_Low_Current(void)
{
	unsigned char current_msg[8];
	
	//电机目标电流为速度pid输出
	horizontal_motor.target_current = horizontal_motor.vpid.PID_OUT;	//
	chuck_yaw.target_current = chuck_yaw.vpid.PID_OUT;
	chuck_pitch.target_current = chuck_pitch.vpid.PID_OUT;	//
	chuck_roll.target_current = chuck_roll.vpid.PID_OUT;	//
	
	longitudinal_motorL.target_current = longitudinal_motorL.vpid.PID_OUT;	//
	longitudinal_motorR.target_current = longitudinal_motorR.vpid.PID_OUT;	//

	current_msg[0] = longitudinal_motorL.target_current >> 8;			//1号电机电流高8位
	current_msg[1] = longitudinal_motorL.target_current & 0xff;		//1号电机电流低8位
	current_msg[2] = longitudinal_motorR.target_current >> 8;			//2号电机电流高8位
	current_msg[3] = longitudinal_motorR.target_current & 0xff;		//2号电机电流低8位
	
//	current_msg[0] = horizontal_motor.target_current >> 8;			//1号电机电流高8位
//	current_msg[1] = horizontal_motor.target_current & 0xff;		//1号电机电流低8位
//	current_msg[2] = chuck_yaw.target_current >> 8;			//1号电机电流高8位
//	current_msg[3] = chuck_yaw.target_current & 0xff;		//1号电机电流低8位
	
	current_msg[4] = chuck_pitch.target_current >> 8;			//1号电机电流高8位
	current_msg[5] = chuck_pitch.target_current & 0xff;		//1号电机电流低8位
	current_msg[6] = chuck_roll.target_current >> 8;			//1号电机电流高8位
	current_msg[7] = chuck_roll.target_current & 0xff;		//1号电机电流低8位
	
	//can发送数据帧
	CAN2_Send_Msg(0x200, current_msg);
}

//自定义控制器解包
//void Custom_Robot_Control(void)
//{
//	chuck_pitch.apid.target_angle = (int16_t)((custom_robot_data.data[0] << 8)+(custom_robot_data.data[1]));
//	chuck_roll.apid.target_angle  = (int16_t)((custom_robot_data.data[2] << 8)+(custom_robot_data.data[3]));
//}
