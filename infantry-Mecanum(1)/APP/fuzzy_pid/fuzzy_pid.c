#include "fuzzy_pid.h"


#define NB   -6
#define NM	 -4
#define NS	 -2
#define ZO	 0
#define PS	 2
#define PM	 4
#define PB	 6
 
 //�����
static const float ruleKp[7][7]={
	PB,	PB,	PM,	PM,	PS,	PS,	ZO,
	PB,	PB,	PM,	PM,	PS,	ZO,	ZO,
	PM,	PM,	PM,	PS,	ZO,	NS,	NM,
	PM,	PS,	PS,	ZO,	NS,	NM,	NM,
	PS,	PS,	ZO,	NS,	NS,	NM,	NM,
	ZO,	ZO,	NS,	NM,	NM,	NM,	NB,
	ZO,	NS,	NS,	NM,	NM,	NB,	NB
};
 
static const float ruleKi[7][7]={
	NB,	NB,	NB,	NM,	NM,	ZO,	ZO,
	NB,	NB,	NM,	NM,	NS,	ZO,	ZO,
	NM,	NM,	NS,	NS,	ZO,	PS,	PS,
	NM,	NS,	NS,	ZO,	PS,	PS,	PM,
	NS,	NS,	ZO,	PS,	PS,	PM,	PM,
	ZO,	ZO,	PS,	PM,	PM,	PB,	PB,
	ZO,	ZO,	PS,	PM,	PB,	PB,	PB
};
 
static const float ruleKd[7][7]={
	PS,	PS,	ZO,	ZO,	ZO,	PB,	PB,
	NS,	NS,	NS,	NS,	ZO,	NS,	PM,
	NB,	NB,	NM,	NS,	ZO,	PS,	PM,
	NB,	NM,	NM,	NS,	ZO,	PS,	PM,
	NB,	NM,	NS,	NS,	ZO,	PS,	PS,
	NM,	NS,	NS,	NS,	ZO,	PS,	PS,
	PS,	ZO,	ZO,	ZO,	ZO,	PB,	PB
};
 
//��ʼ���ṹ�����
FUZZYPID FPID={
	0,0,0,0,0,0,
	0,0,1,1,1
};

//�����ȼ��㺯��
static void CalcMembership(float *ms,float qv,int * index)
{
 if((qv>=NB)&&(qv<NM))
 {
 index[0]=0;
 index[1]=1;
 ms[0]=-0.5f*qv-2.0f;  //y=-0.5x-2.0
 ms[1]=0.5f*qv+3.0f;  //y=0.5x+3.0
 }
 else if((qv>=NM)&&(qv<NS))
 {
 index[0]=1;
 index[1]=2;
 ms[0]=-0.5f*qv-1.0f;  //y=-0.5x-1.0
 ms[1]=0.5f*qv+2.0f;  //y=0.5x+2.0
 }
 else if((qv>=NS)&&(qv<ZO))
 {
 index[0]=2;
 index[1]=3;
 ms[0]=-0.5f*qv;  //y=-0.5x
 ms[1]=0.5f*qv+1.0f;  //y=0.5x+1.0
 }
 else if((qv>=ZO)&&(qv<PS))
 {
 index[0]=3;
 index[1]=4;
 ms[0]=-0.5f*qv+1.0f;  //y=-0.5x+1.0
 ms[1]=0.5f*qv;  //y=0.5x
 }
 else if((qv>=PS)&&(qv<PM))
 {
 index[0]=4;
 index[1]=5;
 ms[0]=-0.5f*qv+2.0f;  //y=-0.5x+2.0
 ms[1]=0.5f*qv-1.0f;  //y=0.5x-1.0
 }
 else if((qv>=PM)&&(qv<=PB))
 {
 index[0]=5;
 index[1]=6;
 ms[0]=-0.5f*qv+3.0f;  //y=-0.5x+3.0
 ms[1]=0.5f*qv-2.0f;  //y=0.5x-2.0
  }
}

//����ֵ����������(-6->6)
static void LinearQuantization(FUZZYPID *vPID,float _Real_Value,float *qValue)
{
  float thisError;
  float deltaError;

  thisError=vPID->setVaule-_Real_Value;   //���㵱ǰƫ��
  deltaError=thisError-vPID->lasterror;   //����ƫ������
	
  //E��EC������
  qValue[0]=6.0f*thisError/(vPID->maximum-vPID->minimum);
  qValue[1]=3.0f*deltaError/(vPID->maximum-vPID->minimum);

}


//��ģ��
static void FuzzyComputation (FUZZYPID *vPID,float _Real_Value)
{
	//����ֵ
 float qValue[2]={0,0};  

 int indexE[2]={0,0};     //e�ڹ�����е�����
 float msE[2]={0,0};      //e��������
 
 int indexEC[2]={0,0};    //ec�ڹ�����е�����
 float msEC[2]={0,0};      //ec��������
 
 //pid����ֵ
 float pidvalue[3];
 
//����
 LinearQuantization(vPID,_Real_Value,qValue);
//����e�������Ⱥ�����
 CalcMembership(msE,qValue[0],indexE);
//����ec�������Ⱥ�����
 CalcMembership(msEC,qValue[1],indexEC);
 
 //�������ķ�����pid����ֵ
 pidvalue[0]=msE[0]*(msEC[0]*ruleKp[indexE[0]][indexEC[0]]+msEC[1]*ruleKp[indexE[0]][indexEC[1]]) 
            +msE[1]*(msEC[0]*ruleKp[indexE[1]][indexEC[0]]+msEC[1]*ruleKp[indexE[1]][indexEC[1]]);
 pidvalue[1]=msE[0]*(msEC[0]*ruleKi[indexE[0]][indexEC[0]]+msEC[1]*ruleKi[indexE[0]][indexEC[1]])
            +msE[1]*(msEC[0]*ruleKi[indexE[1]][indexEC[0]]+msEC[1]*ruleKi[indexE[1]][indexEC[1]]);
 pidvalue[2]=msE[0]*(msEC[0]*ruleKd[indexE[0]][indexEC[0]]+msEC[1]*ruleKd[indexE[0]][indexEC[1]])
            +msE[1]*(msEC[0]*ruleKd[indexE[1]][indexEC[0]]+msEC[1]*ruleKd[indexE[1]][indexEC[1]]);

//pid��������
 vPID->date_kp=vPID->qKp*pidvalue[0];
 vPID->date_ki=vPID->qKi*pidvalue[1];
 vPID->date_kd=vPID->qKd*pidvalue[2];

}

/**
  * @breif         ģ��pid����ں�����ʵ������ʱֻ����ô˺���
  * @param[in]     _Set_Vaule���趨��Ŀ��ֵ����target value
	* @param[in]     _Measure_Vaule�����ص���ʵֵ����actual value  
	* @param[in]     pre_Measure_Vaule����һ�η��ص���ʵֵ
	* @param[out]    vPID->date_kp��kp�������������Լ���д��pid��kp��
	* @param[out]    vPID->date_ki��ki�������������Լ���д��pid��ki��
	* @param[out]    vPID->date_kd��kd�������������Լ���д��pid��kd��
  * @retval        none     
  */

/*�������������*/
void Fuzzytrans(float _Set_Vaule,float _Measure_Vaule,float pre_Measure_Vaule) 
{	
	FPID.setVaule=(_Set_Vaule-5);  
	//��һ�ε�ƫ��ֵ
	FPID.lasterror=_Set_Vaule-pre_Measure_Vaule;
	
	//����ϵ�������Ը���ʵ���������
	FPID.qKp=1;
	FPID.qKi=1;
	FPID.qKi=1;

	//�������������Сֵ
	FPID.maximum=_Set_Vaule; 
	FPID.minimum=0;
	FuzzyComputation(&FPID,_Measure_Vaule);
}

