#include "chassis_task.h"
#include "supercap.h"
#include "kinematics.h"
/**
  * @breif         ����������������ʵ�����Ƕ�ʱ���жϣ�1ms����һ��
  * @param[in]     none
	* @param[out]    none
  * @retval        none     
  */

void CHASSIS_TASK()
{
	static int time_count=1;
	time_count++;
	
	//Զ��ң�أ�����ʱʹ��
	//remote_control();
	//chassis_control_order.chassis_mode=0;
	if(time_count%10==0)
	{
		Get_Base_Velocities();
		send_gimbal_data_2();
		//��ȡ����ϵͳ
		referee_unpack_fifo_data();
		//����̨��������
		send_gimbal_data();
		//��������
	//	supercap();
	}
	
	
	if(time_count%7==0)
	{
		//�����˶�
		chassis_move();
	}
	

	
	if(time_count>=1000)			//���������־    1s
	{time_count=1;}

}
