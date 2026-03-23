#include "communicate.h"
#include "string.h"
#include "crc.h"
#include "drv_can.h"
#include "usbd_cdc_if.h"

//下主控B开头，上主控发送C开头，数字对应
 Board_Tx_Info_t Board_Tx_Info;
 Board_Rx_Info_t Board_Rx_Info=
 {
	 .is_rc_online = 1,
 };
Board_HeartBeat_t Board_HeartBeat=
{
  .offline_cnt = 0,
	.offline_cnt_max = 50,
};

#ifdef UART_COMMUNICATE

extern UART_HandleTypeDef huart6;

Board_Tx_Info_t Board_C_Tx_Info = 
{
	.SOF = 0xA5,
};
Board_Rx_Info_t Board_C_Rx_Info;
uint8_t Board_C_TxBuf[80];

bool Board_Tx_Send_Data(void)
{
	memcpy(Board_C_TxBuf, &Board_C_Tx_Info, sizeof(Board_Tx_Info_t));
		
	Append_CRC8_Check_Sum(Board_C_TxBuf, 3);
		
	Append_CRC16_Check_Sum(Board_C_TxBuf, sizeof(Board_Tx_Info_t));
	
	if(HAL_UART_Transmit_DMA(&huart6,Board_C_TxBuf,sizeof(Board_Tx_Info_t)) == HAL_OK)
	{
			return true;
	}
	return false;
}

bool Board_C_Recieve_Data(uint8_t *rxBuf)
{
	if(rxBuf[0] == 0xA5)
	{
		if(Verify_CRC8_Check_Sum(rxBuf, 3) == true)
		{
			if(Verify_CRC16_Check_Sum(rxBuf, sizeof(Board_Rx_Info_t)) == true)
			{
				memcpy(&Board_C_Rx_Info, rxBuf, sizeof(Board_Rx_Info_t));
				Board_HeartBeat.offline_cnt = 0;
				return true;
			}
		}
	}
	return false;
}

void Board_Tx_Update(Board_Tx_Info_t *Board_Tx_Info)
{
	Board_Tx_Info->hit_enable = vision.VtoE->is_enable_shootting;
	Board_Tx_Info->is_find_Target  = vision.VtoE->is_find_target;
	Board_Tx_Info->pitch_imu  = gimbal.base_info.pitch_imu_angle;
	Board_Tx_Info->pitch_mec = gimbal.base_info.pitch_motor_angle;
	Board_Tx_Info->pitch_v = gimbal.base_info.pitch_imu_speed;
	Board_Tx_Info->yaw_v = imu_sensor.info->base_info.rate_yaw;
	Board_Tx_Info->yaw_imu = imu_sensor.info->base_info.yaw;
	Board_Tx_Info->vision_yaw_tar = vision.VtoE->yaw;
//	Board_Tx_Info->vision_state = vision.VtoE->flag_union.bit.
//	Board_Tx_Info->vision_pitch_tar = 
//	Board_Tx_Info->launch_timer  = 
//	Board_Tx_Info->is_find_base  = 
//	Board_Tx_Info->is_find_outpost = 

}
void USART6_rxDataHandler(uint8_t *rxBuf)
{
	Board_C_Recieve_Data(rxBuf);
}


#else
extern CAN_HandleTypeDef hcan2;


uint8_t board_tx_buf_1[8];
uint8_t board_tx_buf_2[8];
uint8_t board_tx_buf_3[8];
void Board_Tx_C1(void)
{
	uint16_t pitch_imu_temp,yaw_imu_temp,yaw_v_temp,pitch_mec_temp;
	
	pitch_imu_temp = float_to_uint(Board_Tx_Info.pitch_imu,-360.f,360.f,16);
	yaw_imu_temp = float_to_uint(Board_Tx_Info.yaw_imu,-360.f,360.f,16);
	yaw_v_temp = float_to_uint(Board_Tx_Info.yaw_v,-5000.f,+5000.f,16);
	pitch_mec_temp = float_to_uint(Board_Tx_Info.pitch_mec,-2000.f,2000.f,16);
	
	board_tx_buf_1[0] = (pitch_imu_temp>>8);
	board_tx_buf_1[1] = pitch_imu_temp;
	board_tx_buf_1[2] = (yaw_imu_temp>>8);
	board_tx_buf_1[3] = yaw_imu_temp;
	board_tx_buf_1[4] = (yaw_v_temp>>8);
	board_tx_buf_1[5] = yaw_v_temp;
	board_tx_buf_1[6] = (pitch_mec_temp>>8);
	board_tx_buf_1[7] = pitch_mec_temp;
	
	CAN_SendData(&hcan2,0xC1,board_tx_buf_1);
}

void Board_Tx_C2(void)
{
	uint16_t pitch_tar_temp,yaw_tar_temp;
	pitch_tar_temp = float_to_uint(Board_Tx_Info.vision_pitch_tar,-180.f,180.f,16);
	yaw_tar_temp = float_to_uint(Board_Tx_Info.vision_yaw_tar,-180.f,180.f,16);
	
	uint8_t compressed = 0;
	uint8_t a,b,c,d = 0;
//	compressed |= (Board_Tx_Info.hit_enable << 0);  //  存放在最低位
//	compressed |= (Board_Tx_Info.is_find_base << 1);  //  存放在1
//	compressed |= (Board_Tx_Info.is_find_outpost << 2);  //  存放在2
//	compressed |= (Board_Tx_Info.is_find_Target << 3);  //  存放在3
	if(Board_Tx_Info.hit_enable == 1)
	{
		a = 1;
	}
	else
	{
		a = 0;
	}
	if(Board_Tx_Info.is_find_base == 1)
	{
		b = 1;
	}
	else
	{
		b = 0;
	}
	if(Board_Tx_Info.is_find_outpost == 1)
	{
		c = 1;
	}
	else
	{
		c = 0;
	}
	if(Board_Tx_Info.is_find_Target == 1)
	{
		d = 1;
	}
	else
	{
		d = 0;
	}
	compressed |= (a << 0);  //  存放在最低位
	compressed |= (b << 1);  //  存放在1
	compressed |= (c << 2);  //  存放在2
	compressed |= (d << 3);  //  存放在3
//compressed |= ((Board_Tx_Info.hit_enable ? 1 : 0) << 0);       // bit0
//compressed |= ((Board_Tx_Info.is_find_base ? 1 : 0) << 1);     // bit1
//compressed |= ((Board_Tx_Info.is_find_outpost ? 1 : 0) << 2);  // bit2
//compressed |= ((Board_Tx_Info.is_find_Target ? 1 : 0) << 3);   // bit3
//	
	board_tx_buf_2[0] = (pitch_tar_temp>>8);
	board_tx_buf_2[1] = pitch_tar_temp;
	board_tx_buf_2[2] = (yaw_tar_temp>>8);
	board_tx_buf_2[3] = yaw_tar_temp;
	board_tx_buf_2[4] = Board_Tx_Info.vision_state;
 	board_tx_buf_2[5] = compressed;
	board_tx_buf_2[6] = (Board_Tx_Info.launch_timer>>8);
	board_tx_buf_2[7] = Board_Tx_Info.launch_timer;
	
	CAN_SendData(&hcan2,0xC2,board_tx_buf_2);
}

//void Board_Tx_C3(void)
//{
//	board_tx_buf_3[0] = (Board_Tx_Info.launch_timer>>8);
//	board_tx_buf_3[1] = Board_Tx_Info.launch_timer;
//	board_tx_buf_3[2] = 0;
//	board_tx_buf_3[3] = 0;
//	board_tx_buf_3[4] = 0;
//	board_tx_buf_3[5] = 0;
//	board_tx_buf_3[6] = 0;
//	board_tx_buf_3[7] = 0;
//	
//	CAN_SendData(&hcan2,0xC3,board_tx_buf_3);
//}

void Board_Tx_Update(Board_Tx_Info_t *Board_Tx_Info)
{
	Board_Tx_Info->hit_enable = vision.VtoE->flag_union.bit.is_enable_shootting;
	Board_Tx_Info->is_find_Target  = vision.VtoE->flag_union.bit.is_find_target;
	Board_Tx_Info->pitch_imu  = gimbal.base_info.pitch_imu_angle;
	Board_Tx_Info->pitch_mec = gimbal.base_info.pitch_motor_angle;
	Board_Tx_Info->pitch_v = gimbal.base_info.pitch_imu_speed;
	Board_Tx_Info->yaw_v = imu_sensor.info->base_info.rate_yaw;
	Board_Tx_Info->yaw_imu = imu_sensor.info->base_info.yaw;
	Board_Tx_Info->vision_yaw_tar = -vision.VtoE->yaw;
	if(vision.status->rx_state == DEV_ONLINE)
	{
	  Board_Tx_Info->vision_state = 1;
	}
	else
	{
		Board_Tx_Info->vision_state = 0;
	}
	Board_Tx_Info->vision_pitch_tar = vision.VtoE->pitch;
//	Board_Tx_Info->launch_timer  = 
//	Board_Tx_Info->is_find_base  = 
//	Board_Tx_Info->is_find_outpost = 

}


void Board_Rx_C1(uint8_t *rxbuf)
{
	uint16_t pitch_imu_tar_int,yaw_mec_int;
	
	pitch_imu_tar_int = (rxbuf[0]<<8 | rxbuf[1]);
	yaw_mec_int = (rxbuf[2]<<8 | rxbuf[3]);
	Board_Rx_Info.gimbal_mode = rxbuf[4];
	Board_Rx_Info.my_color = rxbuf[5];//视觉需要
	Board_Rx_Info.video_open = rxbuf[6];
	Board_Rx_Info.vision_mode = rxbuf[7];//视觉模式
	
	Board_Rx_Info.pitch_imu_tar = uint_to_float(pitch_imu_tar_int,-360.f,360.f,16);
//	if(Board_Rx_Info.vision_mode == 5)
//	{
//		Board_Rx_Info.yaw_mec_imu = uint_to_float(yaw_mec_int,-PI,PI,16);//机械yaw
//	}
//	else
//	{
		Board_Rx_Info.yaw_mec_imu = uint_to_float(yaw_mec_int,-360.f,360.f,16);//陀螺仪yaw
//	}
}

void Board_Rx_C2(uint8_t *rxbuf)
{
	uint16_t v_x_int,v_y_int,pitch_mec_temp;
	uint8_t compressed;
	
	compressed = (rxbuf[1]);
	v_x_int = (rxbuf[2]<<8 | rxbuf[3]);
	v_y_int = (rxbuf[4]<<8 | rxbuf[5]);	
  pitch_mec_temp = (rxbuf[6]<<8 | rxbuf[7]);
	
	Board_Rx_Info.gimbal_state = (compressed >> 0) & 0x01;  // 提取第 0 位
	Board_Rx_Info.is_fric_on = (compressed >> 1) & 0x01;  // 提取第 1 位
	Board_Rx_Info.is_ready_shoot = (compressed >> 2) & 0x01;  // 提取第 2 位
	Board_Rx_Info.is_on_lob = (compressed >> 3) & 0x01;  // 提取第 3 位
	Board_Rx_Info.is_handle_shoot = (compressed >> 4) & 0x01;  // 提取第 4 位

	Board_Rx_Info.is_rc_online = (compressed >> 5) & 0x01;  // 提取第 5 位
	Board_Rx_Info.shoot_count = (rxbuf[0]);
	Board_Rx_Info.v_x = uint_to_float(v_x_int,-10.f,10.f,16);
	Board_Rx_Info.v_y = uint_to_float(v_y_int,-10.f,10.f,16);
	Board_Rx_Info.pitch_mec_tar = uint_to_float(pitch_mec_temp,-2000.f,2000.f,16);
}

void Board_Rx_C3(uint8_t *rxbuf)
{
	Board_Rx_Info.blood_0 = rxbuf[0];
	Board_Rx_Info.blood_1 = rxbuf[1];
	Board_Rx_Info.blood_2 = rxbuf[2];
	Board_Rx_Info.blood_3 = rxbuf[3];
	Board_Rx_Info.blood_4 = rxbuf[4];
	Board_Rx_Info.blood_5 = rxbuf[5];
	Board_Rx_Info.blood_6 = rxbuf[6];
	Board_Rx_Info.blood_7 = rxbuf[7];
}

void Board_Rx_C4(uint8_t *rxbuf)
{
	uint16_t bullet;
	
	bullet = (rxbuf[0]<<8|rxbuf[1]);
	
	Board_Rx_Info.bullet_speed = uint_to_float(bullet,-20.f,20.f,16);
}


#endif

void C_Board_HeartBeat(void)
{
	Board_HeartBeat.offline_cnt++;
	if(Board_HeartBeat.offline_cnt > \
     Board_HeartBeat.offline_cnt_max)
  {
    Board_HeartBeat.offline_cnt = \
    Board_HeartBeat.offline_cnt_max;

    Board_HeartBeat.status = DEV_OFFLINE;
	
	 
  }
  else if(Board_HeartBeat.status == DEV_OFFLINE)
  {
    Board_HeartBeat.status = DEV_ONLINE;
  }	
}






/////////////////test////////////////自己加上的
//extern UART_HandleTypeDef huart6;

//Board_Tx_Info_t Board_C_Tx_Info = 
//{
//	.SOF = 0xA5,
//};
//Board_Rx_Info_t Board_C_Rx_Info;
//uint8_t Board_C_TxBuf[80];

//bool Board_Tx_Send_Data(void)
//{
//	memcpy(Board_C_TxBuf, &Board_C_Tx_Info, sizeof(Board_Tx_Info_t));
//		
//	Append_CRC8_Check_Sum(Board_C_TxBuf, 3);
//		
//	Append_CRC16_Check_Sum(Board_C_TxBuf, sizeof(Board_Tx_Info_t));
//	
//	if(HAL_UART_Transmit_DMA(&huart6,Board_C_TxBuf,sizeof(Board_Tx_Info_t)) == HAL_OK)
//	{
//			return true;
//	}
//	return false;
//}


//bool Board_C_Recieve_Data(uint8_t *rxBuf)
//{
//	if(rxBuf[0] == 0xA5)
//	{
//		if(Verify_CRC8_Check_Sum(rxBuf, 3) == true)
//		{
//			if(Verify_CRC16_Check_Sum(rxBuf, sizeof(Board_Rx_Info_t)) == true)
//			{
//				memcpy(&Board_C_Rx_Info, rxBuf, sizeof(Board_Rx_Info_t));
//				Board_HeartBeat.offline_cnt = 0;
//				return true;
//			}
//		}
//	}
//	return false;
//}

//void USART6_rxDataHandler(uint8_t *rxBuf)
//{
//	Board_C_Recieve_Data(rxBuf);
//}
