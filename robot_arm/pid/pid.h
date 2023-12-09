#ifndef _PID_H
#define _PID_H

//角度环 
#define aPID_OUT_MAX          30000		//即最大角速度
//速度环 
/*#define CHASSIC_Integral_max         5000            //抗积分饱和
#define CHASSIC_IntegralSeparation   500             //积分分离
#define CHASSIC_vPID_max             8000            //输出限幅
#define GIMBAL_Integral_max         500
#define GIMBAL_IntegralSeparation   20
#define GIMBAL_vPID_max             3000
#define FLIP_vPID_max             5000
#define BELT_vPID_max             5000*/
//上面这个放到 vpid环中 



//pid参数
#define pid_init() \
	do{                 \
	/*底盘速度环*/                        \
		pid_t.chassic_pid.speed_loop.kp = 3.05 ; \
		pid_t.chassic_pid.speed_loop.ki = 0.27 ; \
		pid_t.chassic_pid.speed_loop.kd = 0; \
		pid_t.chassic_pid.integral_max = 5000; \
		pid_t.chassic_pid.integralSeparation = 500;\
		\
		\
		pid_t.trigger_pid.speed_loop.kp = 2.5; \
		pid_t.trigger_pid.speed_loop.ki = 0.05; \
		pid_t.trigger_pid.speed_loop.kd = 0; \
        \
		pid_t.handle.position_loop.kp = 0.22; \
		pid_t.handle.position_loop.ki = 0; \
		pid_t.handle.position_loop.kd = 0.15; \
					                                    \
		pid_t.handle.speed_loop.kp = 4; \
		pid_t.handle.speed_loop.ki = 0.8; \
		pid_t.handle.speed_loop.kd = 1.6; \
					                                    \
		pid_t.flip.speed_loop.kp = 4; \
		pid_t.flip.speed_loop.ki = 0.8; \
		pid_t.flip.speed_loop.kd = 1.6; \
																							\
		pid_t.flip.position_loop.kp = 0.22; \
		pid_t.flip.position_loop.ki = 0; \
		pid_t.flip.position_loop.kd = 0.15; \
					\
		pid_t.card.speed_loop.kp = 8; \
		pid_t.card.speed_loop.ki = 1.2; \
		pid_t.card.speed_loop.kd = 0.5; \
					\
		pid_t.card.position_loop.kp = 0.065; \
		pid_t.card.position_loop.ki = 0; \
		pid_t.card.position_loop.kd = 0.05; \
					\
   		pid_t.resuce.speed_loop.kp = 2.3; \
		pid_t.resuce.speed_loop.ki = 0.54; \
		pid_t.resuce.speed_loop.kd = 0.25; \
	}while(0) 


//电机pid结构体 
typedef struct
{
	float kp;			
	float ki;	
  	float kd;	
}Parameter_t;

typedef struct
{
  	Parameter_t position_loop;
	Parameter_t speed_loop;
}PID_Loop_t;
	
typedef struct
{
  	PID_Loop_t chassic_pid;
	PID_Loop_t trigger_pid;
	PID_Loop_t handle;
	PID_Loop_t flip;
	PID_Loop_t card;
	PID_Loop_t resuce;
	PID_Loop_t mineral;
}PID_t;

extern PID_t pid_t;


void APID_Init(APID_t *apid); 
void APID_Init_All(void);
void apid_realize(MOTOR_t *motor,float kp,float ki,float kd);
void vpid_realize(MOTOR_t *motor,float kp,float ki,float kd); 
void set_angle(MOTOR_t *motor, motor_can2_1_angle);
void set_speed(MOTOR_t *motor, motor_speed);
#endif

