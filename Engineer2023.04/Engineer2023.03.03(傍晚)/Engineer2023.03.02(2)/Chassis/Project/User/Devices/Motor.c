#include "Motor.h"
#include "pid.h"


MOTOR_t chassis_motor1;//第一象限的电机
MOTOR_t chassis_motor2;//第二象限的电机
MOTOR_t chassis_motor3;//第三象限的电机
MOTOR_t chassis_motor4;//第四象限的电机
MOTOR_t outboard_lift_motorL;//外侧左边抬升的电机
MOTOR_t outboard_lift_motorR;//外侧右边抬升的电机

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
	* @function函数:Mineral_Init
	* @brief描述:机械臂初始化
	* @param输入:无
	* @retval返回值:无
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
	* @function	函数：Set_Mineral_X_Speed
	* @brief		描述：设置电机的目标速度
	* @param		输入：目标速度
	* @retval	返回值：无
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
	motor->last_angle 				= 	motor->apid.actual_angle;
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
	motor->apid.total_angle = motor->round_cnt * 8192 + motor->apid.actual_angle;	// - motor->start_angle;
}


/**
	* @function	函数：Debug_CAN_Send_Mineral_Msg
	* @brief		描述：CAN发送摩擦轮电机邮件(即四个调整矿石姿态的电机)(仅供调试时使用)
	* @param		输入：长度为8的数组
	* @retval	返回值：无
	*/
void Debug_CAN_Send_Mineral_Msg(uint8_t* msg)
{		
	CAN1_Send_Msg(0x200, msg);
}


/**
	* @function	函数：CAN_Send_Flip_Msg
	* @brief		描述：CAN发送夹爪电机邮件(即控制夹爪旋转的电机)
	* @param		输入：长度为8的数组
	* @retval	返回值：无
	*/
void Debug_CAN_Send_Flip_Msg(uint8_t* msg)
{
	CAN1_Send_Msg(0x1FF, msg);
}


/**
	* @function	函数：Set_Mineral_Current
	* @brief		描述：CAN发送底盘电机邮件
	* @param		输入：长度为8的数组
	* @retval	返回值：无
	*/
static void Set_Chassis_Current(void)
{
	unsigned char current_msg[8];
	
	//电机目标电流为速度pid输出
	chassis_motor1.target_current = chassis_motor1.vpid.PID_OUT;	//
	chassis_motor2.target_current = chassis_motor2.vpid.PID_OUT;	//
	chassis_motor3.target_current = chassis_motor3.vpid.PID_OUT;	//
	chassis_motor4.target_current = chassis_motor4.vpid.PID_OUT;	//

	current_msg[0] = chassis_motor1.target_current >> 8;			//1号电机电流高8位
	current_msg[1] = chassis_motor1.target_current & 0xff;		//1号电机电流低8位
	current_msg[2] = chassis_motor2.target_current >> 8;			//2号电机电流高8位
	current_msg[3] = chassis_motor2.target_current & 0xff;		//2号电机电流低8位

	current_msg[4] = chassis_motor3.target_current >> 8;			//3号电机电流高8位
	current_msg[5] = chassis_motor3.target_current & 0xff;		//3号电机电流低8位
	current_msg[6] = chassis_motor4.target_current >> 8;			//4号电机电流高8位
	current_msg[7] = chassis_motor4.target_current & 0xff;		//4号电机电流低8位

	//can发送数据帧
	CAN1_Send_Msg(0x200, current_msg);
}

/**
	* @function	函数：Set_Mineral_Current
	* @brief		描述：CAN发送摩擦轮电机邮件(即四个调整矿石姿态的电机)
	* @param		输入：长度为8的数组
	* @retval	返回值：无
	*/
static void Set_Lift_Current(void)
{
	unsigned char current_msg[8];
	
	//电机目标电流为速度pid输出
	outboard_lift_motorL.target_current = outboard_lift_motorL.vpid.PID_OUT;	//
	outboard_lift_motorR.target_current = outboard_lift_motorR.vpid.PID_OUT;	//

	current_msg[0] = outboard_lift_motorL.target_current >> 8;			//左侧抬升电机电流高8位
	current_msg[1] = outboard_lift_motorL.target_current & 0xff;		//左侧抬升电机电流低8位
	current_msg[2] = outboard_lift_motorR.target_current >> 8;			//右侧抬升电机电流高8位
	current_msg[3] = outboard_lift_motorR.target_current & 0xff;		//右侧抬升电机电流低8位


	//can发送数据帧
	CAN2_Send_Msg(0x200, current_msg);
}
