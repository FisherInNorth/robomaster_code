#ifndef MOTOR_H
#define MOTOR_H

//can stdid
#define CAN_LoopBack_ID				 0x200		//���ڻ���ģʽ�Լ����
#define	CAN_3508Motor1_ID  	   0x201	//0x20+���ID 
#define	CAN_3508Motor2_ID      0x202	//1-4�ֱ�Ϊ���� 
#define	CAN_3508Motor3_ID      0x203
#define	CAN_3508Motor4_ID      0x204
#define CAN_3508Motor5_ID      0x205	//5��6Ϊ��Ԯצ 
#define CAN_3508Motor6_ID      0x206
#define CAN_3508Motor7_ID      0x207	//����Ǿ�Ԯ�� 

//���ת��pid�����ṹ��
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

//�����е�ǶȲ���
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


//��������ṹ��
typedef struct{
	int start_angle;			//�����ʼ�Ƕ�ֵ
	int start_angle_flag;	//��¼�����ʼ�Ƕ�ֵ��flag
	int stop_angle;				//����ֹͣ����ʱ��ĽǶ�ֵ
	int last_angle;				//��һ�η��صĽǶ�ֵ
	int round_cnt;				//��Կ���ʱת����Ȧ��
	int actual_current;		//�����ʵ����
	int target_current;		//���Ŀ�����
	int integralSeparation;// �����ֱ��� 
	int integral_max;  //����޷� 
	int vPID_max  //������ֵ 
	VPID_t vpid;
	APID_t apid;
}MOTOR_t;

extern MOTOR_t motor1, motor2, motor3, motor4, motor5, motor6, motor7;

void record_motor_callback(MOTOR_t *motor, unsigned short angle, short speed, short current);
void motor_Init(void);			//�����ʼ��
void motor_all_Init(MOTOR_t *motor);
void set_chassis_current(void);	//���õ������
void set_rescueclaw_current(void);	//���÷�תװ�õ������
void set_rescuecard_current(void);

#endif



