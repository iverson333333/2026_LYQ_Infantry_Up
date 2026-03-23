/**
 * @file        judge_protocol.c
 * @author      RobotPilots@2022
 * @Version     V1.0
 * @date        3-November-2021
 * @brief       judge_protocol
 */

#include "judge_protocol.h"
#include "vision_protocol.h"
#include "communicate.h"
#include "drv_uart.h"
#include "drv_can.h"
#include "string.h"
#include "usart.h"
#include "crc.h"
#include "can.h"

void Determine_ID(void)//判断自己是哪个队伍
{
	if(judge.game_robot_status.robot_id < 10)//本机器人的ID，红方
	{ 
		judge.ids.teammate_hero 		 	= 1;
		judge.ids.teammate_engineer  = 2;
		judge.ids.teammate_infantry3 = 3;
		judge.ids.teammate_infantry4 = 4;
		judge.ids.teammate_infantry5 = 5;
		judge.ids.teammate_plane		 	= 6;
		judge.ids.teammate_sentry		= 7;
		
		judge.ids.client_hero 		 	= 0x0101;
		judge.ids.client_engineer  = 0x0102;
		judge.ids.client_infantry3 = 0x0103;
		judge.ids.client_infantry4 = 0x0104;
		judge.ids.client_infantry5 = 0x0105;
		judge.ids.client_plane			= 0x0106;
		
		if     (judge.game_robot_status.robot_id == hero_red)//不断刷新放置在比赛中更改颜色
			judge.self_client = judge.ids.client_hero;
		else if(judge.game_robot_status.robot_id == engineer_red)
			judge.self_client = judge.ids.client_engineer;
		else if(judge.game_robot_status.robot_id == infantry3_red)
			judge.self_client = judge.ids.client_infantry3;
		else if(judge.game_robot_status.robot_id == infantry4_red)
			judge.self_client = judge.ids.client_infantry4;
		else if(judge.game_robot_status.robot_id == infantry5_red)
			judge.self_client = judge.ids.client_infantry5;
		else if(judge.game_robot_status.robot_id == plane_red)
			judge.self_client = judge.ids.client_plane;
		
	}
	else //蓝方
	{
		judge.ids.teammate_hero 		 	= 101;
		judge.ids.teammate_engineer  = 102;
		judge.ids.teammate_infantry3 = 103;
		judge.ids.teammate_infantry4 = 104;
		judge.ids.teammate_infantry5 = 105;
		judge.ids.teammate_plane		 	= 106;
		judge.ids.teammate_sentry		= 107;
		
		judge.ids.client_hero 		 	= 0x0165;
		judge.ids.client_engineer  = 0x0166;
		judge.ids.client_infantry3 = 0x0167;
		judge.ids.client_infantry4 = 0x0168;
		judge.ids.client_infantry5 = 0x0169;
		judge.ids.client_plane			= 0x016A;
		
		if     (judge.game_robot_status.robot_id == hero_blue)
			judge.self_client = judge.ids.client_hero;
		else if(judge.game_robot_status.robot_id == engineer_blue)
			judge.self_client = judge.ids.client_engineer;
		else if(judge.game_robot_status.robot_id == infantry3_blue)
			judge.self_client = judge.ids.client_infantry3;
		else if(judge.game_robot_status.robot_id == infantry4_blue)
			judge.self_client = judge.ids.client_infantry4;
		else if(judge.game_robot_status.robot_id == infantry5_blue)
			judge.self_client = judge.ids.client_infantry5;
		else if(judge.game_robot_status.robot_id == plane_blue)
			judge.self_client = judge.ids.client_plane;
		
	}

}

judge_t judge = {
	.power_heat_data_offline_max_cnt = 30,
};
uint16_t frame_length;
void judge_update(judge_t *judge_sen,uint8_t *rxBuf)
{
	if(Verify_CRC8_Check_Sum(rxBuf, LEN_FRAME_HEAD) == true)
	{
		memcpy(&judge_sen->fream_header, rxBuf, LEN_FRAME_HEAD);//5个字节
		frame_length = LEN_FRAME_HEAD + LEN_CMD_ID + judge_sen->fream_header.data_length + LEN_FRAME_TAIL;
		if(judge_sen->fream_header.sof == JUDGE_FRAME_HEADER)
		{
			if(Verify_CRC16_Check_Sum(rxBuf, frame_length) == true) 
			{
				uint32_t cmd_id = rxBuf[5] | (rxBuf[6]<<8);
				judge_sen->power_heat_data_offline_cnt = 0;		
				switch(cmd_id)
				{
					case ID_game_state:					
						memcpy(&judge_sen->ext_game_status,rxBuf+7, judge_sen->fream_header.data_length);					
					break;
					case ID_game_robot_state:
						//judge.game_robot_status.robot_id机器人id
						memcpy(&judge_sen->game_robot_status,rxBuf+7, judge_sen->fream_header.data_length);		
						Determine_ID();					
					break;
					case ID_controller_interactive_header_data:
						memcpy(&judge_sen->custom_info,rxBuf+7, judge_sen->fream_header.data_length);
					break;
				}
			}
		}
		if(rxBuf[frame_length] == JUDGE_FRAME_HEADER)
			judge_update(judge_sen,&rxBuf[frame_length]);
	}
}

//检查裁判系统失联
void check_judge_offline(judge_t *judge_sen)
{
	judge_sen->power_heat_data_offline_cnt++;
	if(judge_sen->power_heat_data_offline_cnt > judge_sen->power_heat_data_offline_max_cnt)
	{
		judge_sen->power_heat_data_offline_cnt = judge_sen->power_heat_data_offline_max_cnt;
		judge_sen->power_heat_data_state = DEV_OFFLINE;
	}
	else if(judge_sen->power_heat_data_state == DEV_OFFLINE)
		judge_sen->power_heat_data_state = DEV_ONLINE;
	
}

/**
* @brief 统计弹速
*
*/
shoot_data_t shoot_statistics;
uint8_t cali_flag = 1; //是否计算平均值和平方差
void Speed_Statistic(void)
{
	shoot_statistics.temperature_LF=rm_motor[B_UP_Fric].rx_info->temperature;
	shoot_statistics.speed_now=Board_Rx_Info.bullet_speed;
    float s_speed =  Board_Rx_Info.bullet_speed;
    shoot_statistics.num++;
	
    // 统计速度区间
    if (s_speed <= 10.0f)
    {
        shoot_statistics.lower_110++;
        shoot_statistics. num--; //弹速太离谱不统计
    }
    else if (s_speed >= 11.0f && s_speed <= 11.1f)
    {
        shoot_statistics.speed_110++;
    }
    else if (s_speed >= 11.1f && s_speed <= 11.2f)
    {
        shoot_statistics.speed_111++;
    }
    else if (s_speed >= 11.2f && s_speed <= 11.3f)
    {
        shoot_statistics.speed_112++;
    }
    else if (s_speed >= 11.3f && s_speed <= 11.4f)
    {
        shoot_statistics.speed_113++;
    }
    else if (s_speed >= 11.4f && s_speed <= 11.5f)
    {
        shoot_statistics.speed_114++;
    }
    else if (s_speed >= 11.5f && s_speed <= 11.6f)
    {
        shoot_statistics.speed_115++;
    }
    else if (s_speed >= 11.6f && s_speed <= 11.7f)
    {
        shoot_statistics.speed_116++;
    }
    else if (s_speed >= 11.7f && s_speed <= 11.8f)
    {
        shoot_statistics.speed_117++;
    }
    else if (s_speed >= 11.8f && s_speed <= 11.9f)
    {
        shoot_statistics.speed_118++;
    }
    else if (s_speed >= 11.9f && s_speed <= 12.0f)
    {
        shoot_statistics.speed_119++;
    }
    else if (s_speed >= 12.0f && s_speed <= 12.1f)
    {
        shoot_statistics.speed_120++;
    }
    else if (s_speed >= 12.1f && s_speed <= 12.2f)
    {
        shoot_statistics.speed_121++;
    }
    else if (s_speed >= 12.2f && s_speed <= 12.3f)
    {
        shoot_statistics.speed_122++;
    }
    else if (s_speed >= 12.3f && s_speed <= 12.4f)
    {
        shoot_statistics.speed_123++;
    }

    else if (s_speed >= 15.0f && s_speed <= 15.1f)
    {
        shoot_statistics.speed_150++;
    }
    else if (s_speed > 15.1f && s_speed <= 15.2f)
    {
        shoot_statistics.speed_151++;
    }
    else if (s_speed > 15.2f && s_speed <= 15.3f)
    {
        shoot_statistics.speed_152++;
    }
    else if (s_speed > 15.3f && s_speed <= 15.4f)
    {
        shoot_statistics.speed_153++;
    }
    else if (s_speed > 15.4f && s_speed <= 15.5f)
    {
        shoot_statistics.speed_154++;
    }
    else if (s_speed > 15.5f && s_speed <= 15.6f)
    {
        shoot_statistics.speed_155++;
    }
    else if (s_speed > 15.6f && s_speed <= 15.7f)
    {
        shoot_statistics.speed_156++;
    }
    else if (s_speed > 15.7f && s_speed <= 15.8f)
    {
        shoot_statistics.speed_157++;
    }
    else if (s_speed > 15.8f && s_speed <= 15.9f)
    {
        shoot_statistics.speed_158++;
    }
    else if (s_speed > 15.9f && s_speed <= 16.0f)
    {
        shoot_statistics.speed_159++;
    }
	else if (s_speed > 16.0f && s_speed <= 16.1f)
    {
        shoot_statistics.speed_160++;
    }
	else if (s_speed > 16.1f && s_speed <= 16.2f)
    {
        shoot_statistics.speed_161++;
    }
	else if (s_speed > 16.2f && s_speed <= 16.3f)
    {
        shoot_statistics.speed_162++;
    }
			else if (s_speed > 16.3f && s_speed <= 16.4f)
    {
        shoot_statistics.speed_163++;
    }
	else if (s_speed > 16.4f && s_speed <= 16.5f)
    {
        shoot_statistics.speed_164++;
    }

    else if (s_speed > 16.5f)
    {
        shoot_statistics.higher_165++;
        shoot_statistics. num--; //弹速太离谱不统计
    }

    // 统计弹速平均值和方差
    if (cali_flag == 1)
    {
        // 计算平均值
        shoot_statistics.mean = (
            shoot_statistics.speed_150 * 15.0f +
            shoot_statistics.speed_151 * 15.1f +
            shoot_statistics.speed_152 * 15.2f +
            shoot_statistics.speed_153 * 15.3f +
            shoot_statistics.speed_154 * 15.4f +
            shoot_statistics.speed_155 * 15.5f +
            shoot_statistics.speed_156 * 15.6f +
            shoot_statistics.speed_157 * 15.7f +
            shoot_statistics.speed_158 * 15.8f +
            shoot_statistics.speed_159 * 15.9f +
			shoot_statistics.speed_160 * 16.0f +
			shoot_statistics.speed_161 * 16.1f +
			shoot_statistics.speed_162 * 16.2f +
			shoot_statistics.speed_163 * 16.3f +
			shoot_statistics.speed_164 * 16.14 ) / (float)shoot_statistics.num;

        // 计算方差
        float sum_of_squares = (
            shoot_statistics.speed_150 * ((15.0f - shoot_statistics.mean) * (15.0f - shoot_statistics.mean)) +
            shoot_statistics.speed_151 * ((15.1f - shoot_statistics.mean) * (15.1f - shoot_statistics.mean)) +
            shoot_statistics.speed_152 * ((15.2f - shoot_statistics.mean) * (15.2f - shoot_statistics.mean)) +
            shoot_statistics.speed_153 * ((15.3f - shoot_statistics.mean) * (15.3f - shoot_statistics.mean)) +
            shoot_statistics.speed_154 * ((15.4f - shoot_statistics.mean) * (15.4f - shoot_statistics.mean)) +
            shoot_statistics.speed_155 * ((15.5f - shoot_statistics.mean) * (15.5f - shoot_statistics.mean)) +
            shoot_statistics.speed_156 * ((15.6f - shoot_statistics.mean) * (15.6f - shoot_statistics.mean)) +
            shoot_statistics.speed_157 * ((15.7f - shoot_statistics.mean) * (15.7f - shoot_statistics.mean)) +
            shoot_statistics.speed_158 * ((15.8f - shoot_statistics.mean) * (15.8f - shoot_statistics.mean)) +
            shoot_statistics.speed_159 * ((15.9f - shoot_statistics.mean) * (15.9f - shoot_statistics.mean)) +
			shoot_statistics.speed_160 * ((16.0f - shoot_statistics.mean) * (16.0f - shoot_statistics.mean)) +
			shoot_statistics.speed_161 * ((16.1f - shoot_statistics.mean) * (16.1f - shoot_statistics.mean)) +
			shoot_statistics.speed_162 * ((16.2f - shoot_statistics.mean) * (16.2f - shoot_statistics.mean)) +
			shoot_statistics.speed_163 * ((16.3f - shoot_statistics.mean) * (16.3f - shoot_statistics.mean)) +
			shoot_statistics.speed_164 * ((16.4f - shoot_statistics.mean) * (16.4f - shoot_statistics.mean)));

        shoot_statistics.variance = sum_of_squares / (float)shoot_statistics.num;
    }
		
  shoot_statistics.last_num = shoot_statistics.num;
}
