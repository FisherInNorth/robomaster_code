#include "BSP_Chassis.h"
#include "BSP_Uart.h"
#include "Remote_Task.h"
#include "algorithm.h"
#include "pid.h"
#include "BSP_Motor.h"
#include "BSP_Can.h"

MOTOR_t Chassis_motor1;//前面的电机
MOTOR_t Chassis_motor2;//后面的电机
MOTOR_t Chassis_motor3;//右边的电机
MOTOR_t Chassis_motor4;//左边的电机

CHASSIS_WHEEL_t Chassis_Wheel_1;
CHASSIS_WHEEL_t Chassis_Wheel_2;
CHASSIS_WHEEL_t Chassis_Wheel_3;
CHASSIS_WHEEL_t Chassis_Wheel_4;

int Chassis_Send_Speed1;
int Chassis_Send_Speed2;
int Chassis_Send_Speed3;
int Chassis_Send_Speed4;
int X_Value;
int Y_Value;
int Key_Vw,Key_Vs,Key_Va,Key_Vd;
int Liner_X,Liner_Y,Angular_Z;
int Motion_Round;
extern uint8_t Keyboard_Mode;
void Chassis_Init(void)
{
	Liner_X=Liner_Y=Angular_Z=0;Key_Vw=Key_Vs=Key_Va=Key_Vd=0;X_Value=Y_Value=0;Motion_Round=0;
	Chassis_Send_Speed1=Chassis_Send_Speed2=Chassis_Send_Speed3=Chassis_Send_Speed4=0;
	Chassis_motor1.round_cnt=Chassis_motor2.round_cnt=Chassis_motor3.round_cnt=Chassis_motor4.round_cnt=0.0f;
	Chassis_motor1.vpid.target_speed=Chassis_motor2.vpid.target_speed=Chassis_motor3.vpid.target_speed=Chassis_motor4.vpid.target_speed=0;
	Chassis_motor1.vpid.actual_speed=Chassis_motor2.vpid.actual_speed=Chassis_motor3.vpid.actual_speed=Chassis_motor4.vpid.actual_speed=0.0f;
	Chassis_motor1.target_current=Chassis_motor2.target_current=Chassis_motor3.target_current=Chassis_motor4.target_current=0;
	Chassis_motor1.actual_current=Chassis_motor2.actual_current=Chassis_motor3.actual_current=Chassis_motor4.actual_current=0;
	Chassis_motor1.vpid.err=Chassis_motor2.vpid.err=Chassis_motor3.vpid.err=Chassis_motor4.vpid.err=0;
	Chassis_motor1.vpid.last_err=Chassis_motor2.vpid.last_err=Chassis_motor3.vpid.last_err=Chassis_motor4.vpid.last_err=0;
	Chassis_motor1.vpid.err_integration=Chassis_motor2.vpid.err_integration=Chassis_motor3.vpid.err_integration=Chassis_motor4.vpid.err_integration=0;
	Chassis_motor1.vpid.P_OUT=Chassis_motor2.vpid.P_OUT=Chassis_motor3.vpid.P_OUT=Chassis_motor4.vpid.P_OUT=0;
	Chassis_motor1.vpid.I_OUT=Chassis_motor2.vpid.I_OUT=Chassis_motor3.vpid.I_OUT=Chassis_motor4.vpid.I_OUT=0;
	Chassis_motor1.vpid.D_OUT=Chassis_motor2.vpid.D_OUT=Chassis_motor3.vpid.D_OUT=Chassis_motor4.vpid.D_OUT=0;
	Chassis_motor1.vpid.PID_OUT=Chassis_motor2.vpid.PID_OUT=Chassis_motor3.vpid.PID_OUT=Chassis_motor4.vpid.PID_OUT=0;
}
/**
  * @brief  将遥控器摇杆输出映射到机器人x, y轴速度上
	* @param int width,int mid,int min,int max
	* @retval float	speed
	* @attention 无
  */ 

static float caculate_linear_speed(int width,int mid,int min,int max)
{
	float speed=0;
	if(width>=(mid+2))		//中间消除波动
		speed=(1.0*(width-(mid+2))/(max-(mid+2))*MAX_BASE_LINEAR_SPEED);
	else if(width<=(mid-2))
		speed=(1.0*(width-(mid-2))/((mid-2)-min)*MAX_BASE_LINEAR_SPEED);
	else
		speed=0;
	return speed;
}

/**
  * @brief  将遥控器摇杆输出映射到机器人z轴速度上
	* @param int width,int mid,int min,int max
	* @retval float	speed
	* @attention 无
  */ 

static float caculate_rotational_speed(int width,int mid,int min,int max)
{
	float speed=0;
	if(width>=(mid+2))		//中间消除波动
		speed=(1.0*(width-(mid+2))/(max-(mid+2))*MAX_BASE_ROTATIONAL_SPEED);
	else if(width<=(mid-2))
		speed=(1.0*(width-(mid-2))/((mid-2)-min)*MAX_BASE_ROTATIONAL_SPEED);
	else
		speed=0;
	return speed;
}

	/**
	* @function函数:BaseVel_To_WheelVel()
	* @brief描述:逆运动学公式,把想要得到的底盘速度转换为轮子的线速度
	* @param输入:底盘的三轴速度
	* @retval返回值:无
  */
void BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z)
{
	Chassis_Wheel_1.target_speed.linear_vel = linear_x + linear_y + angular_z*(half_width+half_length);
	Chassis_Wheel_2.target_speed.linear_vel = linear_x - linear_y - angular_z*(half_width+half_length);
	Chassis_Wheel_3.target_speed.linear_vel = linear_x - linear_y + angular_z*(half_width+half_length);
	Chassis_Wheel_4.target_speed.linear_vel = linear_x +  linear_y - angular_z*(half_width+half_length); 
	
	Chassis_Wheel_1.target_speed.rpm = Chassis_Wheel_1.target_speed.linear_vel * VEL2RPM;
	Chassis_Wheel_2.target_speed.rpm = Chassis_Wheel_2.target_speed.linear_vel * VEL2RPM;
	Chassis_Wheel_3.target_speed.rpm = Chassis_Wheel_3.target_speed.linear_vel * VEL2RPM;
	Chassis_Wheel_4.target_speed.rpm = Chassis_Wheel_4.target_speed.linear_vel * VEL2RPM;
		
	Chassis_motor1.vpid.target_speed =   (int)(Chassis_Wheel_1.target_speed.rpm * M3508_REDUCTION_RATIO);
	Chassis_motor2.vpid.target_speed = - (int)(Chassis_Wheel_2.target_speed.rpm * M3508_REDUCTION_RATIO);
	Chassis_motor3.vpid.target_speed =   (int)(Chassis_Wheel_3.target_speed.rpm * M3508_REDUCTION_RATIO);
	Chassis_motor4.vpid.target_speed = - (int)(Chassis_Wheel_4.target_speed.rpm * M3508_REDUCTION_RATIO);
}


/**
	* @function函数:chassic_speed_control()
	* @brief描述:底盘速度控制，将宏观速度指令解算，赋值到具体电机
	* @param输入:底盘速度
	* @retval返回值:无
  */
void Chassic_Speed_Control(float speed_x, float speed_y, float speed_r)
{
		int max;
//		stop_flag_1 = 0;
		BaseVel_To_WheelVel(speed_x, speed_y, speed_r);    //速度换算
 
		max=find_max(Chassis_motor1.vpid.target_speed,Chassis_motor2.vpid.target_speed,Chassis_motor3.vpid.target_speed,Chassis_motor4.vpid.target_speed);
		if(max>MAX_MOTOR_SPEED)
		{
			Chassis_motor1.vpid.target_speed=(int)(Chassis_motor1.vpid.target_speed*MAX_MOTOR_SPEED*1.0/max);
			Chassis_motor2.vpid.target_speed=(int)(Chassis_motor2.vpid.target_speed*MAX_MOTOR_SPEED*1.0/max);
			Chassis_motor3.vpid.target_speed=(int)(Chassis_motor3.vpid.target_speed*MAX_MOTOR_SPEED*1.0/max);
			Chassis_motor4.vpid.target_speed=(int)(Chassis_motor4.vpid.target_speed*MAX_MOTOR_SPEED*1.0/max);
		}
		
		Chassis_Send_Speed1= Chassis_motor1.vpid.target_speed;
		Chassis_Send_Speed2= Chassis_motor2.vpid.target_speed;
		Chassis_Send_Speed3= Chassis_motor3.vpid.target_speed;
		Chassis_Send_Speed4= Chassis_motor4.vpid.target_speed;
		
}	
/**
	* @function函数:Chassis_Task
	* @brief描述:底盘任务执行，底盘速度运算与发送
	* @param输入:无
	* @retval返回值:无
  */
void Chassis_Task(void)
{
	if(Keyboard_Mode==1)
	{
		Y_Value=Key_Vw+Key_Vs;//这里是反的，测出来就是这样的
		X_Value=-Key_Va-Key_Vd;
		Motion_Round=rc_ctrl.mouse.x*880/660;
	}
	else if(Keyboard_Mode==0)
	{
		X_Value=x_CH_width;
		Y_Value=y_CH_width;
		Motion_Round=-DJI_Motion_Round;
	}
	Liner_X = caculate_linear_speed(Y_Value+1024, y_initial_value, y_min_value, y_max_value); //前后左右
	Liner_Y = caculate_linear_speed(X_Value+1024, x_initial_value, x_min_value, x_max_value); //这里是反的，上一届就这么写的
	Angular_Z = caculate_rotational_speed((Motion_Round+1024 )* 2, r_initial_value * 2, r_min_value * 2, r_max_value * 2); //旋转
	
	Chassic_Speed_Control(Liner_X,Liner_Y,Angular_Z);
}

/*
	void Set_Chassis_Speed(int motor1_speed,int motor2_speed,int motor3_speed,int motor4_speed)
	{
		Chassis_motor1.vpid.target_speed = motor1_speed;		                         
		Chassis_motor2.vpid.target_speed = motor2_speed;                             
		Chassis_motor3.vpid.target_speed = motor3_speed;                             
		Chassis_motor4.vpid.target_speed = motor4_speed;                             
		Chassis_motor1.target_speed = motor1_speed;		                               
		Chassis_motor2.target_speed = motor2_speed;                                  
		Chassis_motor3.target_speed = motor3_speed;                                  
		Chassis_motor4.target_speed = motor4_speed;                                  
	}

*/

