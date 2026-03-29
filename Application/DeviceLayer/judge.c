#include "judge.h"

uint8_t flag;

void Shooting_Fri_Speed_Adapt(void)
{
	
/*用户定义参数**********************************************************/

#define SPEED_SAVE_NUM 2			  // 速度保存个数
	const float add_kp = 7.f;		  // 增加增益
	const float minus_kp = 7.f;		  // 减少增益
	#if HERO_TYPE==2
	const float over_blind_err = 0.2; // 超过多少内不调整
	#else
	const float over_blind_err = 0.2; // 超过多少内不调整
	#endif
	
	const float less_blind_err = 0.2; // 低于多少内不调整
	const float max_adapt_range = 100; // 最大单次调整量

	/*函数变量**************************************************************/
	static uint8_t normal_speed_flag;	

	static float last_speed[SPEED_SAVE_NUM] = {0};					  // 保存上一发速度数组
	float now_speed = Board_Rx_Info.bullet_speed; // 当前速度

	uint8_t over_cnt = 0, less_cnt = 0;								  // 大于目标速度计数，小于目标速度计数
	
    /*执行弹速调整的条件*****************************************************/
	
//	if(communicate.car_data0_tx_info->car_state.bit.is_open_adapt==0)
//	{
//		return ;
//	}
	
//#ifndef FriSpeedAdaptEnabled
//	return;
//#endif
	if (Board_Rx_Info.is_fric_on != 1)		  // 发射未初始化
		if (shoot.base_info.fric_info.target_fric_B_L_speed == 0) // 摩擦轮目标速度为0
				if (my_abs(Board_Rx_Info.bullet_speed - shoot.config.target_bullet_speed) > 3) // 收到数据过于离谱
				{
					return;
				}
	//超弹速！！！大量下降
	if(now_speed>12.f)
	{
		shoot.config.target_B_friction_speed -= 20;
//		shoot->config.target_F_friction_speed -= 40;
		return;
	}
	
	/*计算目前存储数组里弹速的情况******************************************/
	for (uint8_t i = 0; i < SPEED_SAVE_NUM; i++)
	{
		if (last_speed[i] == 0)
		{
			// 如果找到一个元素为零，跳出循环
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

	/*根据情况调整摩擦轮速度***********************************************/
	//施密特触发器
	if (now_speed - shoot.config.target_bullet_speed > over_blind_err) // 速度大于目标速度
	{
		if (over_cnt * minus_kp > max_adapt_range)//限幅
			return;
		shoot.config.target_B_friction_speed -= over_cnt * minus_kp;
		shoot.config.target_F_friction_speed -= over_cnt * minus_kp;
	}
 
	else if (shoot.config.target_bullet_speed - now_speed > less_blind_err) // 速度小于目标速度
	{
		if (less_cnt * add_kp > max_adapt_range/*||normal_speed_flag==1*/)
			return;
		if(less_cnt>=2)//数组里面两个都低于弹速才提高弹速
		{
			shoot.config.target_B_friction_speed += less_cnt * add_kp;
//			shoot->config.target_F_friction_speed += less_cnt * add_kp;
		}
		
	}

	/*保存当前速度到数组**************************************************/
	for (uint8_t i = 1; i < SPEED_SAVE_NUM; i++)
	{
		last_speed[i] = last_speed[i - 1];
	}
	last_speed[0] = now_speed;
}

