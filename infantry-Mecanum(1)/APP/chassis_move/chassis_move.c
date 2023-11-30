#include "chassis_move.h"
#include "fuzzy_pid.h"

static float chassis_follow(void);
static void chassis_speed_control(float speed_x, float speed_y, float speed_r);
static float chassis_power_loop(uint16_t target_power,float actual_power,float last_power);
static void chassis_move_mode(void);
static void can_send_chassis_current(void);
static void power_limitation_jugement(void);
static float chassis_buffer_loop(uint16_t buffer);
static void chassis_fly(uint16_t buffer);
static void speed_optimize(void);
static float Get_chassis_theta(void);
CHASSIS_CONTROL_ORDER_t chassis_control_order;
MOTOR_t chassis_motor1,chassis_motor2,chassis_motor3,chassis_motor4,chassis_center;
POWER_PID_t p_pid;
BUFFER_PID_t b_pid;
REAl_CHASSIS_SPEED_t real_chassis_speed;
uint8_t fly_flag;
int8_t max_d_speed_x;
int8_t max_d_speed_y;
/**
  * @breif         �����˶�����
  * @param[in]     none 
	* @param[out]    none
  * @retval        none     
  */
void chassis_move(void)
{
	//�Ż��ٶ�
//	speed_optimize();
	//ģʽѡ��
	chassis_move_mode();
	//pid����
	vpid_chassis_realize();
	//��������
//	power_limitation_jugement();
	//���͵���
	can_send_chassis_current();
}
/**
  * @breif         ��ȡ��̨�����֮��ļн�
  * @param[in]     none
	* @param[out]    ��̨�����֮��ļн�(������)
  * @retval        none     
  */
static float Get_chassis_theta(void)
{
	float temp,temp2,angle;
	if(chassis_center.actual_angle<GIMBAL_HEAD_ANGLE)
		temp=chassis_center.actual_angle+360.0f;
	else temp=chassis_center.actual_angle;
	temp2=temp-GIMBAL_HEAD_ANGLE;	
	angle=temp2/360.0f*2*PI;
	return angle;
}
float theta; 
void chassis_spin(float *vx,float *vy) 
{					
	   
	theta=Get_chassis_theta(); 
	*vx = (float)((float)(chassis_control_order.vy_set)*sin(theta) + (float)(chassis_control_order.vx_set)*cos(theta)); 
	*vy = (float)((float)(chassis_control_order.vy_set)*cos(theta) - (float) (chassis_control_order.vx_set)*sin(theta));   
}
  

/**
  * @breif         ���̹�������
  * @param[in]     none 
	* @param[out]    ������ƺ���ĸ��������ֵ
  * @retval        none     
  */
float current_scale,BUFFER_TOTAL_CURRENT_LIMIT=5000.0f,POWER_TOTAL_CURRENT_LIMIT=9000.0f;
float temp3,temp1,temp2,speed1,speed2,speed3,speed4,total_current_limit,total_current,power,last_power;
float power_scale,buffer_scale;
uint16_t max_power,buffer;
static void power_limitation_jugement(void)
{
	total_current=0;
	last_power=power;
	get_chassis_power_and_buffer_and_max(&power,&buffer,&max_power);

	//����ʱʹ��
	if(fly_flag)
		chassis_fly(buffer);
	else
		//���ʻ�
		{
			power_scale=chassis_power_loop(temp3,power,last_power);
			buffer_scale=chassis_buffer_loop(buffer);
			temp2=power_scale*buffer_scale;
			
			//�˴�������ϵ���˵�Ŀ���ٶ���
			temp1=chassis_motor1.pid.speed_loop.vpid.PID_OUT*temp2;
			chassis_motor1.pid.speed_loop.vpid.PID_OUT=(int16_t)temp1;
			temp1=chassis_motor2.pid.speed_loop.vpid.PID_OUT*temp2;
			chassis_motor2.pid.speed_loop.vpid.PID_OUT=(int16_t)temp1;
			temp1=chassis_motor3.pid.speed_loop.vpid.PID_OUT*temp2;
			chassis_motor3.pid.speed_loop.vpid.PID_OUT=(int16_t)temp1;
			temp1=chassis_motor4.pid.speed_loop.vpid.PID_OUT*temp2;
			chassis_motor4.pid.speed_loop.vpid.PID_OUT=(int16_t)temp1;
		}
}
/**
  * @breif         ���̹��ʻ�����
  * @param[in]     target_power���趨��Ŀ��ֵ
	* @param[in]     target_power�����ص���ʵֵ  
	* @param[in]     last_power����һ�η��ص���ʵֵ
	* @param[out]    �ĸ�������������
  * @retval        none     
  */
static float chassis_power_loop(uint16_t target_power,float actual_power,float last_power)
{
	float temp;
	if(actual_power>(float)(0.8f*(float)target_power))
	{
		p_pid.target_power=(float)target_power;
		p_pid.actual_power=actual_power;
		//�˴�����pid����
		Fuzzytrans(target_power,actual_power,last_power);
		power_pid_realize(&p_pid);
		//�˴��������ϵ��
		temp=((float)p_pid.PID_OUT/(float)target_power)+1.0f;
	

	}
	else 
		temp=1;
	return temp;
}

static float chassis_buffer_loop(uint16_t buffer)
{
	float temp;
	if(buffer<30)
	{
		b_pid.target_buffer=60;
		b_pid.actual_buffer=buffer;
		buffer_pid_realize(&b_pid);
		temp=((float)b_pid.PID_OUT/(float)b_pid.target_buffer)+1.0f;
	}
	else 
		temp=1;
	
	return temp;
}

/**
  * @breif         ���̷��º�������ֹ����º󻺳���������
  * @param[in]     buffer�����̻�������
	* @param[out]    �ĸ�������������
  * @retval        none     
  */
static void chassis_fly(uint16_t buffer)
{
	if(buffer<20)
	{
		chassis_motor1.pid.speed_loop.vpid.PID_OUT*=0.5f;
		chassis_motor2.pid.speed_loop.vpid.PID_OUT*=0.5f;
		chassis_motor3.pid.speed_loop.vpid.PID_OUT*=0.5f;
		chassis_motor4.pid.speed_loop.vpid.PID_OUT*=0.5f;
	}
}

/**
  * @breif         �����ٶ��Ż������ٶȱ仯��ƽ��
  * @param[in]     chassis_control_order.vx_set��x�����ٶ�
	* @param[in]     chassis_control_order.vy_set��y�����ٶ�
	* @param[out]    real_chassis_speed.real_vx���Ż���x�����ٶ�
	* @param[out]    real_chassis_speed.real_vy���Ż���y�����ٶ�
  * @retval        none     
  */
static void speed_optimize(void)
{
	
	static int16_t last_xspeed,last_yspeed;
	real_chassis_speed.real_vx=chassis_control_order.vx_set;
	real_chassis_speed.real_vy=chassis_control_order.vy_set;
	
	if(chassis_control_order.vx_set>last_xspeed)
		max_d_speed_x=5;
	else if(chassis_control_order.vx_set<-last_xspeed)
		max_d_speed_x=10;
	if(abs(chassis_control_order.vx_set-last_xspeed)>max_d_speed_x)
	{
		if(chassis_control_order.vx_set>last_xspeed)
			real_chassis_speed.real_vx+=max_d_speed_x;
		else if(chassis_control_order.vx_set<-last_xspeed)
			real_chassis_speed.real_vx-=max_d_speed_x;
	}

	if(chassis_control_order.vy_set>last_yspeed)
		max_d_speed_y=5;
	else if(chassis_control_order.vy_set<-last_yspeed)
		max_d_speed_y=10;	
	if(abs(chassis_control_order.vy_set-last_yspeed)>max_d_speed_y)
	{
		if(chassis_control_order.vy_set>last_yspeed)
			real_chassis_speed.real_vx+=max_d_speed_y;
		else if(chassis_control_order.vy_set<-last_yspeed)
			real_chassis_speed.real_vy-=max_d_speed_y;
	}
	
	last_xspeed=real_chassis_speed.real_vx;
	last_yspeed=real_chassis_speed.real_vy;
}


/**
  * @breif         �˶�ѧ�ֽ⣬���������ĵ��ٶ�ת��Ϊ�ĸ����ӵ��ٶ�
  * @param[in]     speed_x��x�����ٶ�
	* @param[in]     speed_y��y�����ٶ�
	* @param[in]     speed_r����ת�ٶ�
	* @param[out]    �ĸ������Ŀ���ٶ�
  * @retval        none     
  */
static void chassis_speed_control(float speed_x, float speed_y, float speed_r)
{
	int max;
		//�ٶȻ��㣬�˶�ѧ�ֽ�
	BaseVel_To_WheelVel(speed_x, speed_y, speed_r);
	
	max=find_max();
	if(max>MAX_MOTOR_SPEED)
	{
		chassis_motor1.target_speed=(int)(chassis_motor1.target_speed*MAX_MOTOR_SPEED*1.0/max);
		chassis_motor2.target_speed=(int)(chassis_motor2.target_speed*MAX_MOTOR_SPEED*1.0/max);
		chassis_motor3.target_speed=(int)(chassis_motor3.target_speed*MAX_MOTOR_SPEED*1.0/max);
		chassis_motor4.target_speed=(int)(chassis_motor4.target_speed*MAX_MOTOR_SPEED*1.0/max);
	}
	set_chassis_speed(chassis_motor1.target_speed, chassis_motor2.target_speed, chassis_motor3.target_speed, chassis_motor4.target_speed);
}	

/**
  * @breif         ����ģʽ��ͨ���ǶȻ���Ŀ��Ƕ�ת��ΪĿ���ٶ�
  * @param[in]     none
	* @param[out]    ������ת�ٶ�
  * @retval        none     
  */
static float chassis_follow(void)
{
	//��̨ǹ�ڶ�Ӧ�ĽǶ�ֵ
	chassis_center.pid.position_loop.apid.target_angle=GIMBAL_HEAD_ANGLE;
	chassis_center.pid.position_loop.apid.actual_angle=chassis_control_order.gimbal_6020_angle;
	follow_pid_realize();
	return (float)chassis_center.pid.position_loop.apid.PID_OUT;
}
float vx,vy,wz;
/**
  * @breif         ѡ������˶�ģʽ
  * @param[in]     none
	* @param[out]    ��������������ٶ�
  * @retval        none     
  */
static void chassis_move_mode(void)
{

	vx=(float)chassis_control_order.vx_set;
	vy=(float)chassis_control_order.vy_set;
	wz=(float)chassis_control_order.wz_set;
	CHASSIS_vPID_max=10000;
	switch(chassis_control_order.chassis_mode)
	{
		case CHASSIS_NORMAL:
		break;
		//case CHASSIS_NO_FORCE:
		case CHASSIS_NO_FORCE:
		{   wz=0;
			vx=0;
			vy=0;
			}
		break;
		case CHASSIS_FOLLOW:
		{			
			if(vx<=0.0f&&vy<=0.0f)
			{
				CHASSIS_vPID_max=4000;
			}
			wz=-1.0f*chassis_follow();
		}
		break;
		case CHASSIS_SPIN:
		{
			if(vx==0&&vy==0&&wz==0)
			{
				CHASSIS_vPID_max=9000;
			}
			chassis_spin(&vx,&vy);
			wz=2.0f;
		}
		break;
		default:break;

	}
	chassis_speed_control(vx,vy,wz);
}

/**
  * @breif         �����ĸ�����ĵ���
  * @param[in]     none
	* @param[out]    �ĸ�����ĵ���ֵ
  * @retval        none     
  */
static void can_send_chassis_current(void)
{
	static uint8_t cdata[8];
	cdata[0]=(chassis_motor1.pid.speed_loop.vpid.PID_OUT)>>8;
	cdata[1]=(chassis_motor1.pid.speed_loop.vpid.PID_OUT)&0xFF;
	cdata[2]=(chassis_motor2.pid.speed_loop.vpid.PID_OUT)>>8;
	cdata[3]=(chassis_motor2.pid.speed_loop.vpid.PID_OUT)&0xFF;
	cdata[4]=(chassis_motor3.pid.speed_loop.vpid.PID_OUT)>>8;
	cdata[5]=(chassis_motor3.pid.speed_loop.vpid.PID_OUT)&0xFF;
	cdata[6]=(chassis_motor4.pid.speed_loop.vpid.PID_OUT)>>8;
	cdata[7]=(chassis_motor4.pid.speed_loop.vpid.PID_OUT)&0xFF;
	
	Can_Tx_Message(&hcan1,cdata);
}


