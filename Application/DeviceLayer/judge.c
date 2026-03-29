#include "judge.h"

uint8_t flag;

void Shooting_Fri_Speed_Adapt(void)
{
	
/*�û��������**********************************************************/

#define SPEED_SAVE_NUM 2			  // �ٶȱ������
	const float add_kp = 7.f;		  // ��������
	const float minus_kp = 7.f;		  // ��������
	#if HERO_TYPE==2
	const float over_blind_err = 0.2; // ���������ڲ�����
	#else
	const float over_blind_err = 0.2; // ���������ڲ�����
	#endif
	
	const float less_blind_err = 0.2; // ���ڶ����ڲ�����
	const float max_adapt_range = 100; // ��󵥴ε�����

	/*��������**************************************************************/
	static uint8_t normal_speed_flag;	

	static float last_speed[SPEED_SAVE_NUM] = {0};					  // ������һ���ٶ�����
	float now_speed = Board_Rx_Info.bullet_speed; // ��ǰ�ٶ�

	uint8_t over_cnt = 0, less_cnt = 0;								  // ����Ŀ���ٶȼ�����С��Ŀ���ٶȼ���
	
    /*ִ�е��ٵ���������*****************************************************/
	
//	if(communicate.car_data0_tx_info->car_state.bit.is_open_adapt==0)
//	{
//		return ;
//	}
	
//#ifndef FriSpeedAdaptEnabled
//	return;
//#endif
	if (Board_Rx_Info.flag.bit.is_fric_on != 1)		  // ����δ��ʼ��
		if (shoot.base_info.fric_info.target_fric_B_L_speed == 0) // Ħ����Ŀ���ٶ�Ϊ0
				if (my_abs(Board_Rx_Info.bullet_speed - shoot.config.target_bullet_speed) > 3) // �յ����ݹ�������
				{
					return;
				}
	//�����٣����������½�
	if(now_speed>12.f)
	{
		shoot.config.target_B_friction_speed -= 20;
//		shoot->config.target_F_friction_speed -= 40;
		return;
	}
	
	/*����Ŀǰ�洢�����ﵯ�ٵ����******************************************/
	for (uint8_t i = 0; i < SPEED_SAVE_NUM; i++)
	{
		if (last_speed[i] == 0)
		{
			// ����ҵ�һ��Ԫ��Ϊ�㣬����ѭ��
			continue;
		}
		else if (last_speed[i] > shoot.config.target_bullet_speed)
		{
			over_cnt++;
		}
		else if (shoot.config.target_bullet_speed > last_speed[i])
		{
			less_cnt++;
		}
	}

	/*�����������Ħ�����ٶ�***********************************************/
	//ʩ���ش�����
	if (now_speed - shoot.config.target_bullet_speed > over_blind_err) // �ٶȴ���Ŀ���ٶ�
	{
		if (over_cnt * minus_kp > max_adapt_range)//�޷�
			return;
		shoot.config.target_B_friction_speed -= over_cnt * minus_kp;
		shoot.config.target_F_friction_speed -= over_cnt * minus_kp;
	}
 
	else if (shoot.config.target_bullet_speed - now_speed > less_blind_err) // �ٶ�С��Ŀ���ٶ�
	{
		if (less_cnt * add_kp > max_adapt_range/*||normal_speed_flag==1*/)
			return;
		if(less_cnt>=2)//�����������������ڵ��ٲ���ߵ���
		{
			shoot.config.target_B_friction_speed += less_cnt * add_kp;
//			shoot->config.target_F_friction_speed += less_cnt * add_kp;
		}
		
	}

	/*���浱ǰ�ٶȵ�����**************************************************/
	for (uint8_t i = 1; i < SPEED_SAVE_NUM; i++)
	{
		last_speed[i] = last_speed[i - 1];
	}
	last_speed[0] = now_speed;
}

