#include "Motor.h"
#include "pid.h"


MOTOR_t chassis_motor1;//第一象限的电机
MOTOR_t chassis_motor2;//第二象限的电机
MOTOR_t chassis_motor3;//第三象限的电机
MOTOR_t chassis_motor4;//第四象限的电机
MOTOR_t outboard_lift_motorL;//外侧左边抬升的电机
MOTOR_t outboard_lift_motorR;//外侧右边抬升的电机
MOTOR_t mineral_motor1;//1号矿石运动电机
MOTOR_t mineral_motor2;//2号矿石运动电机
MOTOR_t mineral_motor3;//3号矿石运动电机
MOTOR_t mineral_motor4;//4号矿石运动电机


extern float ChassisMotor1_PID_Parameters;
extern float ChassisMotor2_PID_Parameters;
extern float ChassisMotor3_PID_Parameters;
extern float ChassisMotor4_PID_Parameters;
extern float Outboard_MotorL_vPID_Parameters;
extern float Outboard_MotorL_aPID_Parameters;
extern float Outboard_MotorR_vPID_Parameters;
extern float Outboard_MotorR_aPID_Parameters;
extern float MineralMotor_PID_Parameters;


static void Set_Chassis_Current(void);
static void Set_Lift_Current(void);
static void Set_Mineral_Current(void);
static void Motor_RecordData(MOTOR_t *motor, unsigned char * data);

int abs_chassis1 = 0;
int abs_chassis2 = 0;
int abs_chassis3 = 0;
int abs_chassis4 = 0;

uint8_t start_flag=0;

void motor_angle_init(void)
{
	outboard_lift_motorL.last_angle=4096;
	outboard_lift_motorR.last_angle=4096;

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
		case CANRX_Chassis_Motor1_ID:		Motor_RecordData(&chassis_motor1,Data);/*	ChassisMotorPID_Calc(&chassis_motor1, &ChassisMotor1_PID_Parameters);	Set_Chassis_Current();*/	break;
		case CANRX_Chassis_Motor2_ID:		Motor_RecordData(&chassis_motor2,Data);/*	ChassisMotorPID_Calc(&chassis_motor2, &ChassisMotor2_PID_Parameters);	Set_Chassis_Current();*/	break;
		case CANRX_Chassis_Motor3_ID:		Motor_RecordData(&chassis_motor3,Data);/*	ChassisMotorPID_Calc(&chassis_motor3, &ChassisMotor3_PID_Parameters);	Set_Chassis_Current();*/	break;
		case CANRX_Chassis_Motor4_ID:		Motor_RecordData(&chassis_motor4,Data);/*	ChassisMotorPID_Calc(&chassis_motor4, &ChassisMotor4_PID_Parameters);	Set_Chassis_Current();*/	break;
	}
//	Vpid_Calc(motor, Mineral_Kp, Mineral_Ki, Mineral_Kd);
//	Set_Mineral_Current();
}

void CAN2_Motor_HandleMsg(uint32_t StdId, unsigned char * Data)
{
	switch (StdId)
	{
		case CANRX_Outboard_Lift_MotorL_ID:	Motor_RecordData(&outboard_lift_motorL,Data);if(start_flag==0) start_flag++;/*	OutboardLiftMotor_PID_Calc(&outboard_lift_motorL, &Outboard_MotorL_aPID_Parameters, &Outboard_MotorL_vPID_Parameters); Set_Lift_Current();*/ break;
		case CANRX_Outboard_Lift_MotorR_ID:	Motor_RecordData(&outboard_lift_motorR,Data);if(start_flag==1) start_flag++;/*	OutboardLiftMotor_PID_Calc(&outboard_lift_motorR, &Outboard_MotorR_aPID_Parameters, &Outboard_MotorR_vPID_Parameters); Set_Lift_Current();*/ break;
		case CANRX_Mineral_Motor1_ID:    Motor_RecordData(&mineral_motor1,Data); MineralMotorPID_Calc(&mineral_motor1, &MineralMotor_PID_Parameters); Set_Mineral_Current(); break;
		case CANRX_Mineral_Motor2_ID:    Motor_RecordData(&mineral_motor2,Data); MineralMotorPID_Calc(&mineral_motor2, &MineralMotor_PID_Parameters); Set_Mineral_Current(); break;
		case CANRX_Mineral_Motor3_ID:    Motor_RecordData(&mineral_motor3,Data); MineralMotorPID_Calc(&mineral_motor3, &MineralMotor_PID_Parameters); Set_Mineral_Current(); break;
		case CANRX_Mineral_Motor4_ID:    Motor_RecordData(&mineral_motor4,Data); MineralMotorPID_Calc(&mineral_motor4, &MineralMotor_PID_Parameters); Set_Mineral_Current(); break;
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
	motor->apid.total_angle = motor->round_cnt * 8192 + motor->apid.actual_angle- motor->start_angle;//+ motor->start_angle;
	motor->last_angle 				= 	motor->apid.actual_angle;
}



/**
	* @function	函数：Set_Chassis_Current
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
	* @brief		描述：CAN发送外抬升电机邮件
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

static void Set_Mineral_Current(void)
{
	unsigned char current_msg[8];
	
	
	//电机目标电流为速度pid输出
  mineral_motor1.target_current = mineral_motor1.vpid.PID_OUT;	//
	mineral_motor2.target_current = mineral_motor2.vpid.PID_OUT;	//
	mineral_motor3.target_current = mineral_motor3.vpid.PID_OUT;	//
	mineral_motor4.target_current = mineral_motor4.vpid.PID_OUT;	//

	current_msg[0] = mineral_motor1.target_current >> 8;			//1号矿石运动电机电流高8位
	current_msg[1] = mineral_motor1.target_current & 0xff;		//1号矿石运动电机电流低8位
	current_msg[2] = mineral_motor2.target_current >> 8;			//2号矿石运动电机电流高8位
	current_msg[3] = mineral_motor2.target_current & 0xff;		//2号矿石运动电机电流低8位
	current_msg[4] = mineral_motor3.target_current >> 8;			//3号矿石运动电机电流高8位
	current_msg[5] = mineral_motor3.target_current & 0xff;		//3号矿石运动电机电流低8位
	current_msg[6] = mineral_motor4.target_current >> 8;			//4号矿石运动电机电流高8位
	current_msg[7] = mineral_motor4.target_current & 0xff;		//4号矿石运动电机电流低8位

	//can发送数据帧
	CAN2_Send_Msg(0x1FF, current_msg);
}

void abs_motor(void)
{
	abs_chassis1 = abs(chassis_motor1.vpid.actual_speed);
	abs_chassis2 = abs(chassis_motor2.vpid.actual_speed);
	abs_chassis3 = abs(chassis_motor3.vpid.actual_speed);
	abs_chassis4 = abs(chassis_motor4.vpid.actual_speed);
}

void CAN1_Motor_ControlMsg(void)
{
	ChassisMotorPID_Calc(&chassis_motor1, &ChassisMotor1_PID_Parameters);
	ChassisMotorPID_Calc(&chassis_motor2, &ChassisMotor2_PID_Parameters);
	ChassisMotorPID_Calc(&chassis_motor3, &ChassisMotor3_PID_Parameters);
	ChassisMotorPID_Calc(&chassis_motor4, &ChassisMotor4_PID_Parameters);
	Set_Chassis_Current();
}

void CAN2_Motor_ControlMsg(void)
{
	OutboardLiftMotor_PID_Calc(&outboard_lift_motorL, &Outboard_MotorL_aPID_Parameters, &Outboard_MotorL_vPID_Parameters);
	OutboardLiftMotor_PID_Calc(&outboard_lift_motorR, &Outboard_MotorR_aPID_Parameters, &Outboard_MotorR_vPID_Parameters);
//	MineralMotorPID_Calc(&mineral_motor1, &MineralMotor_PID_Parameters);
//	MineralMotorPID_Calc(&mineral_motor2, &MineralMotor_PID_Parameters);
//	MineralMotorPID_Calc(&mineral_motor3, &MineralMotor_PID_Parameters);
//	MineralMotorPID_Calc(&mineral_motor4, &MineralMotor_PID_Parameters);
	Set_Lift_Current();
//	Set_Mineral_Current();
}
