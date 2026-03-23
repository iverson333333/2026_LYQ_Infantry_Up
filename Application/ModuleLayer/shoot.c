#include "shoot.h"
//下板收指令+热量限制->发给上板0或1+拨盘复位是否完成->（上板发视觉is_ready->视觉发回上板enable_shoot->）拨盘动标志位1
shoot_t shoot=
{
	.fric_b_l=&rm_motor[B_L_Fric],
	.fric_b_r=&rm_motor[B_R_Fric],
	.fric_b_up=&rm_motor[B_UP_Fric],
	
	.work=Shoot_Work,
	
	.config.target_bullet_speed=11.7f,	
	.config.target_B_friction_speed=3040,     //3050//4550,4350,4452（21度16.04），4320（22度16.2）,4290,4530,3585
	
	.target = 0,

};


/*堵转处理*/
void Shoot_stuck_deal(shoot_t *shoot)
{
	if((my_abs(shoot->fric_b_l->ctrl->speed_ctrl->out)>=4000 && my_abs(rm_motor[B_L_Fric].rx_info->speed)<=20)  //堵转判断
	 ||(my_abs(shoot->fric_b_r->ctrl->speed_ctrl->out)>=4000 && my_abs(rm_motor[B_R_Fric].rx_info->speed)<=20)
	 ||(my_abs(shoot->fric_b_up->ctrl->speed_ctrl->out)>=4000 && my_abs(rm_motor[B_UP_Fric].rx_info->speed)<=20))
	{
		shoot->stuck_count++;
		if(shoot->stuck_count>=100)
		{
			
		}
	}
}

/*发射pid计算*/
void Shoot_pid_cal(shoot_t *shoot)
{
	shoot->fric_b_l->ctrl->speed_ctrl->target=shoot->base_info.fric_info.target_fric_B_L_speed;
	shoot->fric_b_r->ctrl->speed_ctrl->target=shoot->base_info.fric_info.target_fric_B_R_speed;	
	shoot->fric_b_up->ctrl->speed_ctrl->target=shoot->base_info.fric_info.target_fric_B_UP_speed;			
	if((my_abs(rm_motor[B_L_Fric].rx_info->encoder_speed)<=500 &&//不在发射不控摩擦轮
		 my_abs(rm_motor[B_R_Fric].rx_info->encoder_speed)<=500 &&
		 my_abs(rm_motor[B_UP_Fric].rx_info->encoder_speed)<=500 && Board_Rx_Info.is_fric_on == 0 )
	  || Board_Rx_Info.is_rc_online == 1)
	{
		shoot->fric_b_l->tx_info->torque=0;
		shoot->fric_b_r->tx_info->torque=0;
		shoot->fric_b_up->tx_info->torque=0;
	}
	else//////////////////////////////////////////在发射控摩擦轮
	{
		rm_motor[B_R_Fric].single_set_speed(&rm_motor[B_R_Fric]);
		rm_motor[B_L_Fric].single_set_speed(&rm_motor[B_L_Fric]);
		rm_motor[B_UP_Fric].single_set_speed(&rm_motor[B_UP_Fric]);
	}
}

/*外部获取*/
static uint32_t t;
void Shoot_extern_get(shoot_t *shoot)
{
  shoot->is_on_fric = Board_Rx_Info.is_fric_on;
	if(shoot->is_on_fric == 1)
  {
		shoot->base_info.fric_info.target_fric_B_L_speed = shoot->config.target_B_friction_speed;
		shoot->base_info.fric_info.target_fric_B_R_speed = -shoot->config.target_B_friction_speed;
		shoot->base_info.fric_info.target_fric_B_UP_speed = -shoot->config.target_B_friction_speed;
	}
	else
	{
		shoot->base_info.fric_info.target_fric_B_L_speed = 0;
		shoot->base_info.fric_info.target_fric_B_R_speed = 0;
		shoot->base_info.fric_info.target_fric_B_UP_speed = 0;
		
	}
}

/*离线保护*/
void Shoot_offline_detect(shoot_t *shoot)
{
	
}

/**
 * @brief 打弹命令执行时间计算
 * 
 * @param flag 0；命令开始执行  1：接收到弹速
 */
void Shooting_Cmd_Excute_Tick_Calculating(uint8_t flag)
{
	static uint32_t cmd_start_tick = 0;
	static uint32_t rx_bullet_tick = 0;
	static uint8_t rx_bullet_cnt = 0;
	static uint8_t reset_cnt_flag = 0;
	
	const uint8_t buf_length = 100;
	if (flag == 0)//命令开始执行
	{
		cmd_start_tick = HAL_GetTick();
	}
	else if (flag == 1)//接收到弹速
	{
		rx_bullet_tick = HAL_GetTick();
		vision.shooting_cmd_excute_tick = rx_bullet_tick - cmd_start_tick;
		vision.shooting_cmd_excute_tick_buf[rx_bullet_cnt]=vision.shooting_cmd_excute_tick;
		#if 1
		//移动指针
		rx_bullet_cnt++;
		//回归零点
		if(rx_bullet_cnt>=buf_length-1)
		{
			rx_bullet_cnt=0;
			reset_cnt_flag=1;
		}
		//计算平均数
		float shooting_cmd_excute_tick_sum;
		
		if(reset_cnt_flag==1)//如果回到原点过，直接遍历
		{
			
			for(uint8_t i=0;i<buf_length;i++)
			{
				shooting_cmd_excute_tick_sum+=vision.shooting_cmd_excute_tick_buf[i];
			}
			vision.shooting_cmd_excute_tick_mean=shooting_cmd_excute_tick_sum/buf_length;
		}
		else//多少个就多少个
		{
			for(uint8_t i=0;i<rx_bullet_cnt;i++)
			{
				shooting_cmd_excute_tick_sum+=vision.shooting_cmd_excute_tick_buf[i];
			}
			vision.shooting_cmd_excute_tick_mean=shooting_cmd_excute_tick_sum/rx_bullet_cnt;
		}
		#endif
		
	}
}

/*发射板间更新*/
//void Shoot_Board_Update(shoot_t *shoot)
//{
//	Board_Tx_Info.vision_yaw_tar = vision.EtoV->yaw_offset;
//	Board_Tx_Info.hit_enable = shoot->base_info.is_enable_shoot;
//	Board_Tx_Info.is_find_Target = shoot->target.find_Target;	
//	Board_Tx_Info.launch_timer = shoot->base_info.launch_timer;
//}

/*发射总控*/
void Shoot_Work(shoot_t *shoot)
{
	static uint16_t last_shoot_count,shoot_count = 0;
	last_shoot_count = shoot_count;
	shoot_count = Board_Rx_Info.shoot_count;
	
	Shoot_offline_detect(shoot);
	Shooting_Cmd_Excute_Tick_Calculating(1);
	if(shoot_count != last_shoot_count)
	{
    Speed_Statistic();
	  Shooting_Fri_Speed_Adapt();
	}
	Shoot_extern_get(shoot);
	Shoot_pid_cal(shoot);
}