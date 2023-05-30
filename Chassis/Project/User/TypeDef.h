#ifndef __TYPEDEF_H__
#define __TYPEDEF_H__


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
	float kp,ki,kd;
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
//	int integralSeparation;// �����ֱ��� 
//	int integral_max;  		//����޷� 
//	int vPID_max;  				//������ֵ 
//	int aPID_OUT_MAX;			//�������ٶ�
	VPID_t vpid;
	APID_t apid;
	
	int stop_flag;
	int count;
	int pwm_num;
}MOTOR_t;



#endif
