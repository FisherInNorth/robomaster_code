/**
  ******************************************************************************
  * @file    Project/USER/flag.c 
  * @author  neuq robomaster
  * @version V1.0.0
  * @date    5.2021
  * @brief   
  ******************************************************************************
  * @attention
  ******************************************************************************
      ..................NEUQ_SUDO..................
*/  
#include "flag.h"
/*----------- ��ŷ���ʶ��-----------*/
uint16_t Solenoid_valve_flag = 0;     //���������ŷ�
uint16_t Solenoid_handle_flag = 0;    //ץ�ֵ�ŷ�

/*----------- �˶�ѧ���Ʊ�ʶ�� ------------*/
uint16_t stop_flag_1=0;
uint16_t stop_flag_4=0;
uint16_t test_flag=0;
uint16_t handle_test_flag=0;
uint16_t stop_flag_2=0;

/*---------- ���������ʶ�� ---------------*/
uint16_t off_line_flag=0;

/*---------- ������ʶ�� -----------*/
uint16_t lever_flag=0;   //�жϲ����Ƿ�����һ������
uint16_t lever_start=0;
uint16_t key_flag=0;     //�жϰ����Ƿ񳤰�

uint16_t handle_flag=0;
/*--------handle---------*/
uint16_t handle_angle_flag=2;  

uint16_t F_key_flag=0;

/*--------��λ����--------*/

uint16_t limit_flag=0;

