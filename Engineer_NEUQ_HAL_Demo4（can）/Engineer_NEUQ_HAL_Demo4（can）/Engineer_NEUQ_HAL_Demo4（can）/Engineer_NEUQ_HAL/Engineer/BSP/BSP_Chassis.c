#include "BSP_Chassis.h"
#include "BSP_Uart.h"
#include "Remote_Task.h"
#include "algorithm.h"
#include "pid.h"
#include "BSP_Motor.h"
#include "BSP_Can.h"

MOTOR_t Chassis_motor1;//ǰ��ĵ��
MOTOR_t Chassis_motor2;//����ĵ��
MOTOR_t Chassis_motor3;//�ұߵĵ��
MOTOR_t Chassis_motor4;//��ߵĵ��

CHASSIS_WHEEL_t Chassis_Wheel_1;
CHASSIS_WHEEL_t Chassis_Wheel_2;
CHASSIS_WHEEL_t Chassis_Wheel_3;
CHASSIS_WHEEL_t Chassis_Wheel_4;

uint16_t Chassis_Send_Speed1;
uint16_t Chassis_Send_Speed2;
uint16_t Chassis_Send_Speed3;
uint16_t Chassis_Send_Speed4;

int Liner_X,Liner_Y,Angular_Z;

void Chassis_Init(void)
{
	Liner_X=Liner_Y=Angular_Z=0;
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
	Chassis_motor1.vPID_max=Chassis_motor2.vPID_max=Chassis_motor3.vPID_max=Chassis_motor4.vPID_max=3000;
}
/**
  * @brief  ��ң����ҡ�����ӳ�䵽������x, y���ٶ���
	* @param int width,int mid,int min,int max
	* @retval float	speed
	* @attention ��
  */ 

static float caculate_linear_speed(int width,int mid,int min,int max)
{
	float speed=0;
	if(width>=(mid+2))		//�м���������
		speed=(1.0*(width-(mid+2))/(max-(mid+2))*MAX_BASE_LINEAR_SPEED);
	else if(width<=(mid-2))
		speed=(1.0*(width-(mid-2))/((mid-2)-min)*MAX_BASE_LINEAR_SPEED);
	else
		speed=0;
	return speed;
}

/**
  * @brief  ��ң����ҡ�����ӳ�䵽������z���ٶ���
	* @param int width,int mid,int min,int max
	* @retval float	speed
	* @attention ��
  */ 

static float caculate_rotational_speed(int width,int mid,int min,int max)
{
	float speed=0;
	if(width>=(mid+2))		//�м���������
		speed=(1.0*(width-(mid+2))/(max-(mid+2))*MAX_BASE_ROTATIONAL_SPEED);
	else if(width<=(mid-2))
		speed=(1.0*(width-(mid-2))/((mid-2)-min)*MAX_BASE_ROTATIONAL_SPEED);
	else
		speed=0;
	return speed;
}

	/**
	* @function����:BaseVel_To_WheelVel()
	* @brief����:���˶�ѧ��ʽ,����Ҫ�õ��ĵ����ٶ�ת��Ϊ���ӵ����ٶ�
	* @param����:���̵������ٶ�
	* @retval����ֵ:��
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
	* @function����:chassic_speed_control()
	* @brief����:�����ٶȿ��ƣ�������ٶ�ָ����㣬��ֵ��������
	* @param����:�����ٶ�
	* @retval����ֵ:��
  */
void Chassic_Speed_Control(float speed_x, float speed_y, float speed_r)
{
		int max;
//		stop_flag_1 = 0;
		BaseVel_To_WheelVel(speed_x, speed_y, speed_r);    //�ٶȻ���
 
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
	* @function����:Chassis_Task
	* @brief����:��������ִ�У������ٶ������뷢��
	* @param����:��
	* @retval����ֵ:��
  */
void Chassis_Task(void)
{
	Liner_X = caculate_linear_speed(y_CH_width, y_initial_value, y_min_value, y_max_value); //ǰ������
	Liner_Y = caculate_linear_speed(x_CH_width, x_initial_value, x_min_value, x_max_value);
	Angular_Z = caculate_rotational_speed(r_CH_width * 2, r_initial_value * 2, r_min_value * 2, r_max_value * 2); //��ת
	
	Chassic_Speed_Control(Liner_X,Liner_Y,Angular_Z);
	RC_Chassis_Speed_Send(Chassis_Send_Speed1,Chassis_Send_Speed2,Chassis_Send_Speed3,Chassis_Send_Speed4);

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

