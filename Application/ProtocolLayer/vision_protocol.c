/**
 * @file vision_protocol.c
 * @author Isaac
 * @brief 视觉通信协议，负责接受发送和心跳包
 * @version 0.1
 * @date 2023-11-21
 * @copyright Copyright (c) 2023
 * 
 */
#include "car.h"
#include "vision_protocol.h"
#include "communicate.h"
#include "stdbool.h"
#include "string.h"
#include "usbd_cdc_if.h"
#include "usart.h"
#include "crc.h"


void Vision_TxTime_Calculating(void);
void Append_Vision_Timing_Buff(uint32_t* vision_timing_buff, uint8_t size, uint16_t delay);


ElectricalToVisionFrame vision_tx_info = 
{
	.SOF = 0xA5,//帧首字节
	.bullet_id = 0,//初始化时为0
};

VisionToElectricalFrame vision_rx_info;
Vision_Timestamp_Info_t vision_timestamp_info;
Vision_Status_t vision_status = 
{
	.offline_cnt_max = VISION_OFFLINE_CNT_MAX,
	.rx_state = DEV_OFFLINE,
	.tx_state = DEV_OFFLINE,
};
Vision_t vision = 
{
//	.tx_info = &vision_tx_info,
//	.rx_info = &vision_rx_info,
	.EtoV = &vision_tx_info,
	.VtoE = &vision_rx_info,
	.timestamp_info = &vision_timestamp_info,
	.status  = &vision_status,
};

uint8_t vision_txBuf[80];


/**
 * @brief 发送时间间隔计算
 * 
 * @param tx_info 
 */
void Vision_TxTime_Calculating(void)
{
		static uint32_t last_tick = 0; 

		uint32_t tick = HAL_GetTick(); // 记录现在的tick值
		vision.status->send_time = tick - last_tick; // 计算发送间隔
		last_tick = tick;
}

/**
 * @brief 视觉通信收
 * @param rxBuf 
 * @return 收到数据：DEV_ONLINE 没有到数据：DEV_OFFLINE
 * @note 
 */
void Vision_DataRx(uint8_t *rxBuf)
{
	/* 帧首字节是否为0xA5 */
	if(rxBuf[0] == 0xA5)
	{
		/* 帧头CRC8校验*/
		if(Verify_CRC8_Check_Sum(rxBuf, 6) == true)
		{
			/* 帧尾CRC16校验 */
			if(Verify_CRC16_Check_Sum(rxBuf, sizeof(VisionToElectricalFrame)) == true)
			{
				memcpy(&vision_rx_info, rxBuf, sizeof(VisionToElectricalFrame));
				vision.status->offline_cnt = 0;
				vision.status->rx_tick = HAL_GetTick();//记录接受到信息时的时间，好像没用到
				//添加发射时间戳
				if(vision.VtoE->flag_union.bit.is_enable_shootting == 1)
				{
					Append_Vision_Timing_Buff((uint32_t *)vision.timestamp_info->vision_shoot_timing, 
																		sizeof(vision.timestamp_info->vision_shoot_timing)/sizeof(vision_timestamp_info.vision_shoot_timing[0]),
																		vision.VtoE->timing);
				}

			}
		}
	}
}

/**
  * @name    Vision_DataTx
  * @brief   视觉通信发(串口1)*/
void Vision_DataTx(UART_HandleTypeDef *huart)
{
	memcpy(vision_txBuf, &vision_tx_info, sizeof(ElectricalToVisionFrame));//设置发送信息
	Append_CRC8_Check_Sum(vision_txBuf, 6);//添加CRC8校验码
	Append_CRC16_Check_Sum(vision_txBuf, sizeof(ElectricalToVisionFrame));//添加CRC16校验码
	
	if(CDC_Transmit_FS(vision_txBuf,sizeof(ElectricalToVisionFrame)) == USBD_OK)//串口发送
	{
		vision.status->tx_state = DEV_ONLINE;
		Vision_TxTime_Calculating();//发送时间间隔计算
	}
	else
	{
		vision.status->tx_state = DEV_OFFLINE;
	}
}

void USART1_rxDataHandler(uint8_t *rxBuf)//后续换指针
{
  Vision_DataRx(rxBuf);
}
/*视觉上板更新*/
void Vision_Board_Update(void)
{
	vision.EtoV->flag_union.bit.is_ready = Board_Rx_Info.is_ready_shoot;
	vision.EtoV->flag_union.bit.own_color = Board_Rx_Info.my_color;
//	vision.EtoV->flag_union.bit.game_start = ;
	
	switch(Board_Rx_Info.vision_mode)
	{
		case 0:
//			vision.EtoV->flag_union.bit.is_com_vision = 0;
	    vision.EtoV->flag_union.bit.outpost_mode = 0;		
	    vision.EtoV->flag_union.bit.lob_mode = 0;		
	    vision.EtoV->flag_union.bit.engineer_mode = 0;
		break;
		case 1:
//			vision.EtoV->flag_union.bit.is_com_vision = 1;
			vision.EtoV->flag_union.bit.outpost_mode = 0;		
	    vision.EtoV->flag_union.bit.lob_mode = 0;		
	    vision.EtoV->flag_union.bit.engineer_mode = 0;
    break;
		case 2:
//			vision.EtoV->flag_union.bit.is_com_vision = 0;
	    vision.EtoV->flag_union.bit.outpost_mode = 1;		
	    vision.EtoV->flag_union.bit.lob_mode = 0;		
	    vision.EtoV->flag_union.bit.engineer_mode = 0;
		break;
		case 3:
//			vision.EtoV->flag_union.bit.is_com_vision = 0;
	    vision.EtoV->flag_union.bit.outpost_mode = 0;		
	    vision.EtoV->flag_union.bit.lob_mode = 1;		
	    vision.EtoV->flag_union.bit.engineer_mode = 0;
		break;
		case 4:
//			vision.EtoV->flag_union.bit.is_com_vision = 0;
	    vision.EtoV->flag_union.bit.outpost_mode = 0;		
	    vision.EtoV->flag_union.bit.lob_mode = 0;		
	    vision.EtoV->flag_union.bit.engineer_mode = 1;
		break;
    default:
    break;
	}		
	
	if(gimbal.gimbal_ctrl_mode.gimbal_mode != 1)
	{
	  vision.EtoV->yaw = Board_Rx_Info.yaw_mec_imu / PI * 180.f;
		vision.EtoV->pitch = gimbal.base_info.pitch_motor_angle / 4096.f * 180.f;    //360
	}
	else
	{
		vision.EtoV->yaw = Board_Rx_Info.yaw_mec_imu;// / PI * 180.f;
		vision.EtoV->pitch = gimbal.base_info.pitch_imu_angle;
	}
  vision.EtoV->roll = (-imu_sensor.info->base_info.pitch - 0.77);
//	vision.EtoV->pitch_offset = gimbal.offset_info->vision_pitch_offset / 180.f /** 4096.f*/;
	vision.EtoV->pitch_speed = gimbal.base_info.pitch_imu_speed;
	vision.EtoV->yaw_speed = Board_Tx_Info.yaw_v;
//	vision.EtoV->yaw_offset = Board_Rx_Info.yaw_mec_imu / 180.f /* 4096.f*/;
}

/**
 * @brief 在接收到弹速信息后更新已发弹数和弹速
 * 
 */
//void Vison_Interrupt_Update(void)
//{
//  ElectricalToVisionFrame *tx_info = vision.EtoV;
//	if (1)//.status->rx_state == DEV_ONLINE
//	{
//		if (car.car_move_mode == vision_cycle_CAR || car.car_move_mode == vision_gyro_CAR)
//		{
//			 
//			tx_info->bullet_id++;
//		}
//	}
//////	tx_info->bullet_speed = communicate.shoot_data_rx_info->shooting_speed;
//}

///**
// * @brief 打弹命令执行时间计算
// * 
// * @param flag 0；命令开始执行  1：接收到弹速
// */
//void Shooting_Cmd_Excute_Tick_Calculating(uint8_t flag)
//{
//	static uint32_t cmd_start_tick = 0;
//	static uint32_t rx_bullet_tick = 0;
//	static uint8_t rx_bullet_cnt = 0;
//	static uint8_t reset_cnt_flag = 0;
//	
//	const uint8_t buf_length = 100;
//	if (flag == 0)//命令开始执行
//	{
//		cmd_start_tick = HAL_GetTick();
//	}
//	else if (flag == 1)//接收到弹速
//	{
//		rx_bullet_tick = HAL_GetTick();
//		vision.shooting_cmd_excute_tick = rx_bullet_tick - cmd_start_tick;
//		vision.shooting_cmd_excute_tick_buf[rx_bullet_cnt]=vision.shooting_cmd_excute_tick;
//		#if 1
//		//移动指针
//		rx_bullet_cnt++;
//		//回归零点
//		if(rx_bullet_cnt>=buf_length-1)
//		{
//			rx_bullet_cnt=0;
//			reset_cnt_flag=1;
//		}
//		//计算平均数
//		float shooting_cmd_excute_tick_sum;
//		
//		if(reset_cnt_flag==1)//如果回到原点过，直接遍历
//		{
//			
//			for(uint8_t i=0;i<buf_length;i++)
//			{
//				shooting_cmd_excute_tick_sum+=vision.shooting_cmd_excute_tick_buf[i];
//			}
//			vision.shooting_cmd_excute_tick_mean=shooting_cmd_excute_tick_sum/buf_length;
//		}
//		else//多少个就多少个
//		{
//			for(uint8_t i=0;i<rx_bullet_cnt;i++)
//			{
//				shooting_cmd_excute_tick_sum+=vision.shooting_cmd_excute_tick_buf[i];
//			}
//			vision.shooting_cmd_excute_tick_mean=shooting_cmd_excute_tick_sum/rx_bullet_cnt;
//		}
//		#endif
//		
//	}
//}

/**
	* @name    Vision_HearBeat
	* @brief   视觉通信心跳
	* @note    由监控任务调用
	*/
void Vision_HearBeat(void)
{
	vision.status->offline_cnt++;
	if(vision.status->offline_cnt > \
     vision.status->offline_cnt_max)
  {
    vision.status->offline_cnt = \
    vision.status->offline_cnt_max;

    vision.status->rx_state = DEV_OFFLINE;
	
	 
  }
  else if(vision.status->rx_state == DEV_OFFLINE)
  {
    vision.status->rx_state = DEV_ONLINE;
  }	
}
/**
	* @name    Append_Vision_Timing_Buff
	* @brief   将视觉发过来的发射延时转化为SystemTick，并放到数组里
	* @note    Vision_DataRx里调用
	*/
void Append_Vision_Timing_Buff(uint32_t* vision_timing_buff, uint8_t size, uint16_t delay) 
{
  if (size <= 0) 
	{
    return; // 如果数组大小为0或负数，直接返回
  }
	if (delay == 0)
	{
		delay = 1;
	}
  // 将剩余的元素前移一格
  for (uint8_t i = 1; i < size; i++) 
	{
    vision_timing_buff[i - 1] = vision_timing_buff[i];
  }
  // 在数组的最后一个位置添加形参的值
  vision_timing_buff[size - 1] = delay + HAL_GetTick();
}
/**
* @name     Vision_led_work
* @brief    不同视觉状态led不同显示
* @note     红灯失联，绿灯闪烁表示收到信息但是没找到目标，绿灯常亮表示找到目标
*/
void Vision_led_work(void)
{
	if(vision.status->rx_state==DEV_OFFLINE)
	{
		led.colour=LED_colour_red;
		led.state=LED_ON;
	}
	
	else if(vision.VtoE->flag_union.bit.is_find_target==1)
	{
		led.colour=LED_colour_green;
		led.state=LED_ON;
	}
	
	else if(vision.status->rx_state==DEV_ONLINE)
	{
		led.colour=LED_colour_green;
		led.state=LED_BLINK;
	}
	
	else
	{
		led.state=LED_OFF;
	}
}

