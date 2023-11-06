/**
  ******************************************************************************
  * @file    Project/USER/main.c 
  * @author  Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   
  ******************************************************************************
  * @attention ���ļ���Ÿ����Զ������ѧ�����Լ�һЩ�㷨
  ******************************************************************************
      ..................NEUQ_SUDO..................
*/
#include "algorithm.h"


/**
  *@brief  �Զ��������ֵ����
  */
int abs(int input)
{
	if(input < 0)
		input = -input;
	return input;
}
/**
   * @function������find_max
   * @brief�����������ֵ
   * @param���룺����*4
   * @retval����ֵ�����ֵtemp
   */
int find_max(int elem1, int elem2, int elem3, int elem4)
{
  int temp = 0;
  temp = abs(elem1);
  if(abs(elem2) > temp)
    temp = abs(elem2);
  if(abs(elem3) > temp)
    temp = abs(elem3);
  if(abs(elem4) > temp)
    temp = abs(elem4);
  return temp;
}

/**
  *@brief  �������˲�����
           Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
           R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա��       
  */
float KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R)
                                        
{
        float R = MeasureNoise_R;
        float Q = ProcessNiose_Q;

        static        float x_last = 0;

        float x_mid = x_last;
        float x_now;

        static        float p_last = 0;

        float p_mid ;
        float p_now;
        float kg;       

        x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
        p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
        kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
        x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
               
        p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance       

        p_last = p_now; //����covarianceֵ
        x_last = x_now; //����ϵͳ״ֵ̬

        return x_now;               
}

/*-------------------------------------------------------------------------------------------------------------*/



