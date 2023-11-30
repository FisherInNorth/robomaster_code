#ifndef __FUZZY_PID_H
#define __FUZZY_PID_H

//����ģ��PID�ṹ��
typedef struct
{
 float setVaule;  //�趨ֵ
 float date_kp; //����ֵ����
 float date_ki;  //����ֵ����
 float date_kd;  //΢��ֵ����
 float lasterror; //ǰһ��ƫ��
 float preerror;//ǰ����ƫ��

 float maximum; //����ֵ������
 float minimum;  //����ֵ������
 
 float qKp;    //kp����������ϵ��
 float qKi;      //ki����������ϵ��
 float qKd;    //kd����������ϵ��
}FUZZYPID;

//�ⲿ�ļ��ɼ�
extern FUZZYPID FPID;
//ģ��PID����
void Fuzzytrans(float _Set_Vaule,float _Measure_Vaule,float pre_Measure_Vaule);

#endif

