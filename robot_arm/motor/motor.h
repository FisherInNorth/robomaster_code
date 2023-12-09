#ifndef MOTOR_H
#define MOTOR_H

//can stdid
#define CAN_LoopBack_ID				 0x200		//用于环回模式自检调试
#define	CAN_3508Motor1_ID  	   0x201	//0x20+电调ID 
#define	CAN_3508Motor2_ID      0x202	//1-4分别为底盘 
#define	CAN_3508Motor3_ID      0x203
#define	CAN_3508Motor4_ID      0x204
#define CAN_3508Motor5_ID      0x205	//5、6为救援爪 
#define CAN_3508Motor6_ID      0x206
#define CAN_3508Motor7_ID      0x207	//这个是救援卡 

//电机转速pid参数结构体
typedef struct{
	int err;
	int last_err;
	int err_integration;
	int target_speed;
	int actual_speed;
	int P_OUT;
	int I_OUT;
	int D_OUT;
	int PID_OUT;
}VPID_t;

//电机机械角度参数
typedef struct{
	int err;
	int last_err;
	int err_integration;
	int actual_angle;
	int target_angle;
	int P_OUT;
	int I_OUT;
	int D_OUT;
	int PID_OUT;
  	int total_angle;	
}APID_t;


//电机参数结构体
typedef struct{
	int start_angle;			//电机初始角度值
	int start_angle_flag;	//记录电机初始角度值的flag
	int stop_angle;				//发送停止命令时候的角度值
	int last_angle;				//上一次返回的角度值
	int round_cnt;				//相对开机时转过的圈数
	int actual_current;		//电机真实电流
	int target_current;		//电机目标电流
	int integralSeparation;// 抗积分饱和 
	int integral_max;  //输出限幅 
	int vPID_max  //输出最大值 
	VPID_t vpid;
	APID_t apid;
}MOTOR_t;

extern MOTOR_t motor1, motor2, motor3, motor4, motor5, motor6, motor7;

void record_motor_callback(MOTOR_t *motor, unsigned short angle, short speed, short current);
void motor_Init(void);			//电机初始化
void motor_all_Init(MOTOR_t *motor);
void set_chassis_current(void);	//设置电机电流
void set_rescueclaw_current(void);	//设置翻转装置电机电流
void set_rescuecard_current(void);

#endif



