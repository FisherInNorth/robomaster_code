
#include "referee.h"



frame_header Referee_Data_Head;


ext_game_state 			Game_State;
ext_game_result 		Game_Result;
ext_game_robot_HP 	Game_Robot_HP;

ext_event_data 									Field_Event;
ext_supply_projectile_action 		Supply_Projectile_Action;
ext_supply_projectile_booking 	Supply_Projectile_Booking;
ext_referee_warning 						Referee_Warning;


ext_game_robot_state 			Robot_State;
ext_power_heat_data 			Power_Heat_Data;
ext_game_robot_pos	 			Game_Robot_Pos;
ext_buff_musk 						Buff_Musk;
aerial_robot_energy 			Robot_Energy;
ext_robot_hurt 						Robot_Hurt;
ext_shoot_data 						Shoot_Data;
ext_bullet_remaining 			Bullet_Remaining;
ext_student_interactive_data 		Student_Interactive_Data;

void m_memcpy(void *pd,const void *ps,uint16_t len)
{
	const uint8_t* tempps = ps;
	uint8_t* temppd = pd;
	uint16_t i;
	for (i=0;i<len;i++)
	{temppd[i] = tempps[i];}
}

/******************************************************
* @fn Init_Referee_Struct_Data
*
* @brief ��ʼ����ϵͳ�ش����ṹ��
* @pData None
* @return None.
* @note ����������ʼǰ����һ��
*/
void Init_Referee_Struct_Data(void)
{
    memset(&Referee_Data_Head, 0, sizeof(frame_header));
	
    memset(&Game_State, 0, sizeof(ext_game_state));
    memset(&Game_Result, 0, sizeof(ext_game_result));
    memset(&Game_Robot_HP, 0, sizeof(ext_game_robot_HP));

    memset(&Field_Event, 0, sizeof(ext_event_data));
    memset(&Supply_Projectile_Action, 0, sizeof(ext_supply_projectile_action));
    memset(&Supply_Projectile_Booking, 0, sizeof(ext_supply_projectile_booking));
    memset(&Referee_Warning, 0, sizeof(ext_referee_warning));

    memset(&Robot_State, 0, sizeof(ext_game_robot_state));
    memset(&Power_Heat_Data, 0, sizeof(ext_power_heat_data));
    memset(&Game_Robot_Pos, 0, sizeof(ext_game_robot_pos));
    memset(&Buff_Musk, 0, sizeof(ext_buff_musk));
    memset(&Robot_Energy, 0, sizeof(aerial_robot_energy));
    memset(&Robot_Hurt, 0, sizeof(ext_robot_hurt));
    memset(&Shoot_Data, 0, sizeof(ext_shoot_data));
    memset(&Bullet_Remaining, 0, sizeof(ext_bullet_remaining));

    memset(&Student_Interactive_Data, 0, sizeof(ext_student_interactive_data));
}

/******************************************************
* @fn Referee_Data_Solve
*
* @brief ������������ �����Ӧ�ṹ����
* @pData �������ݵ�ָ��
* @return None.
* @note DMA�ж��е���
*/

void Referee_Data_Solve(uint8_t *frame)
{

    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&Referee_Data_Head, frame, sizeof(frame_header));
    index += sizeof(frame_header);
    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&Game_State, frame + index, sizeof(ext_game_state));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&Game_Result, frame + index, sizeof(ext_game_result));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&Game_Robot_HP, frame + index, sizeof(ext_game_robot_HP));
        }
        break;


        case FIELD_EVENTS_CMD_ID:
        {
            memcpy(&Field_Event, frame + index, sizeof(ext_event_data));
        }
        break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&Supply_Projectile_Action, frame + index, sizeof(ext_supply_projectile_action));
        }
        break;
        case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
        {
            memcpy(&Supply_Projectile_Booking, frame + index, sizeof(ext_supply_projectile_booking));
        }
        break;
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&Referee_Warning, frame + index, sizeof(ext_referee_warning));
        }
        break;

        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&Robot_State, frame + index, sizeof(ext_game_robot_state));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&Power_Heat_Data, frame + index, sizeof(ext_power_heat_data));
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&Game_Robot_Pos, frame + index, sizeof(ext_game_robot_pos));
        }
        break;
        case BUFF_MUSK_CMD_ID:
        {
            memcpy(&Buff_Musk, frame + index, sizeof(ext_buff_musk));
        }
        break;
        case AERIAL_ROBOT_ENERGY_CMD_ID:
        {
            memcpy(&Robot_Energy, frame + index, sizeof(aerial_robot_energy));
        }
        break;
        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&Robot_Hurt, frame + index, sizeof(ext_robot_hurt));
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&Shoot_Data, frame + index, sizeof(ext_shoot_data));
        }
        break;
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&Bullet_Remaining, frame + index, sizeof(ext_bullet_remaining));
        }
        break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
            memcpy(&Student_Interactive_Data, frame + index, sizeof(ext_student_interactive_data));
        }
				break;
        default:
        {
            break;
        }
    }
}

/******************************************************
* @fn Get_Robot_HP
*
* @brief //�õ�������Ѫ��
* @pData None
* @return ������Ѫ�� uint16
* @note None.
*/
uint16_t Get_Robot_HP(void) 					
{
	return Robot_State.remain_HP;
}

/******************************************************
* @fn Get_Shoot_Heatlimit
*
* @brief �õ���������������
* @pData None
* @return �������� uint16
* @note None.
*/
uint16_t Get_Shoot_Heatlimit(void)	//
{
	return Robot_State.shooter_heat0_cooling_limit;
}

/******************************************************
* @fn Get_Chassis_Power
*
* @brief �õ������˵��̹���
* @pData None
* @return ���̹��� float
* @note None.
*/
float Get_Chassis_Power(void)					//
{
	return Power_Heat_Data.chassis_power;
}

/******************************************************
* @fn Get_Chassis_PowerBuff
*
* @brief �õ������˹��ʻ���
* @pData None
* @return ���ʻ��� uint16
* @note None.
*/
uint16_t Get_Chassis_PowerBuff(void)	//
{
	return Power_Heat_Data.chassis_power_buffer;
}

/******************************************************
* @fn Get_Shoot_Heat
*
* @brief �õ�������ǹ������
* @pData None
* @return ǹ������ uint16
* @note None.
*/
uint16_t Get_Shoot_Heat(void)					//
{
	return Power_Heat_Data.shooter_heat0;
}
/******************************************************
* @fn Get_HP_Type
*
* @brief �õ�������Ѫ���仯����
* @pData None
* @return Ѫ���仯���� uint8
* @note None.
*/
uint8_t Get_HP_Type(void)							//
{
	uint8_t temp_flag;
	temp_flag = Robot_Hurt.hurt_type>>4;
	return temp_flag;
}
 uint8_t Get_Robot_Stats(void)
 {
	 return Robot_State.robot_level;
 }
