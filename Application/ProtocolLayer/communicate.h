#ifndef __communicate_H_
#define __communicate_H_
#include "rp_device_config.h"
#include "rc_sensor.h"
#include "drv_can.h"
#include "can_protocol.h"
#include "vision_protocol.h"
#include "shoot.h"
#include "judge_protocol.h"
#include "gimbal.h"
#include "rp_math.h"
#ifdef UART_COMMUNICATE

extern UART_HandleTypeDef huart3;

//#define POWER_HEAT_DATA_RX_ID        (0x100)//power_heat_data发送ID
//#define GAME_ROBOT_STATUS_RX_ID      (0x101)//game_robot_status发送ID
//#define SHOOT_DATA_RX_ID             (0x102)//shoot_data发送ID
//#define GAME_ROBOT_POS_RX_ID    	 (0x103)//game_robot_pos发送ID
//#define CHASSIS_DATA_TX_ID           (0X250)
//#define CAR_DATA_TX_ID               (0X104)
//#define COMMUNICATE_OFFLINE_CNT_MAX  (200)//离线最大计数(ms)


//上板给下板发送的结构体
typedef struct
{
  uint8_t SOF; // 帧头，数据帧的起始标志
  uint8_t CRC8; // 循环冗余校验，用于校验帧头部分的数据完整性

	/*gimbal更新*/
	float pitch_imu;
	float yaw_imu;
	float yaw_v;
	float pitch_v;
	float pitch_mec;//?
	
	/*视觉信息,发射更新*/
	uint8_t vision_state;
	bool hit_enable;
	bool is_find_Target;
	bool is_find_outpost;
	bool is_find_base;
	uint16_t launch_timer;//发射主动等待时间
	float vision_pitch_tar;
	float vision_yaw_tar;
	
  uint16_t CRC16; // 循环冗余校验，用于校验整个数据帧的完整性
}Board_Tx_Info_t;

//下板给上板发送的结构体
typedef struct
{
  uint8_t SOF; // 帧头，数据帧的起始标志
  uint8_t CRC8; // 循环冗余校验，用于校验帧头部分的数据完整性
	
	float pitch_imu_tar;
	float yaw_imu_tar;
	float pitch_mec_tar;//?
	
	bool gimbal_state;
	int8_t gimbal_mode;
	
	bool is_ready_shoot;//拨盘复位
	bool is_on_lob;//吊
	bool is_handle_shoot;//是否操作手手打
	
	bool is_fric_on;
	uint8_t bullet_speed;

	/*视觉信息*/
	uint8_t my_color;
	uint8_t video_open;
	uint8_t vision_mode;
	uint8_t blood_0;
	uint8_t blood_1;
	uint8_t blood_2;
	uint8_t blood_3;
	uint8_t blood_4;
	uint8_t blood_5;
	uint8_t blood_6;
	uint8_t blood_7;
	float v_x;
	float v_y;
  uint16_t CRC16; // 循环冗余校验，用于校验整个数据帧的完整性
}Board_Rx_Info_t;

typedef struct
{
	dev_work_state_t status;						//接受状态
	uint32_t send_time;                   //发送间隔
	uint32_t rx_tick;						//接受到信息时的时间
	uint8_t offline_cnt;									//接受离线计数
	uint8_t offline_cnt_max;							//接受离线最大计数
}Board_HeartBeat_t;


extern Board_Tx_Info_t Board_Tx_Info;
extern Board_Rx_Info_t Board_Rx_Info;
extern Board_HeartBeat_t Board_HeartBeat;

bool Board_Tx_Send_Data(void);
bool Board_C_Recieve_Data(uint8_t *rxBuf);
void C_Board_HeartBeat(void);

#else

typedef struct
{
	/*gimbal更新*/
	float pitch_imu;//**
	float yaw_imu;//**
	float yaw_v;//**
	float pitch_v;
	float pitch_mec;//?//**
	
	/*视觉信息,发射更新*/
	uint8_t vision_state;//**
	bool hit_enable;
	bool is_find_Target;
	bool is_find_outpost;
	bool is_find_base;
	uint16_t launch_timer;//发射主动等待时间//**
	float vision_pitch_tar;//**
	float vision_yaw_tar;//**
	
}Board_Tx_Info_t;

//下板给上板发送的结构体
typedef struct
{
	bool is_rc_online;

	float pitch_imu_tar;//**
	float yaw_mec_imu;//_tar;//*******
	float pitch_mec_tar;//?//**
	int8_t gimbal_mode;//**
	
	bool gimbal_state;//**
	bool is_ready_shoot;//拨盘复位//**
	bool is_on_lob;//吊//**
	bool is_handle_shoot;//是否操作手手打//**
	
	bool is_fric_on;//**
	
	float bullet_speed;//**
  uint8_t shoot_count;
	/*视觉信息*/
	uint8_t my_color;//**
	uint8_t video_open;//**
	uint8_t vision_mode;//**关，开，吊，其他
	uint8_t blood_0;//**
	uint8_t blood_1;//**
	uint8_t blood_2;//**
	uint8_t blood_3;//**
	uint8_t blood_4;//**
	uint8_t blood_5;//**
	uint8_t blood_6;//**
	uint8_t blood_7;//**
	float v_x;//**
	float v_y;//**
}Board_Rx_Info_t;

typedef struct
{
	dev_work_state_t status;						//接受状态
	uint32_t send_time;                   //发送间隔
	uint32_t rx_tick;						//接受到信息时的时间
	uint8_t offline_cnt;									//接受离线计数
	uint8_t offline_cnt_max;							//接受离线最大计数
}Board_HeartBeat_t;

void Board_Tx_Update(Board_Tx_Info_t *Board_Tx_Info);
extern Board_Tx_Info_t Board_Tx_Info;
extern Board_Rx_Info_t Board_Rx_Info;
extern Board_HeartBeat_t Board_HeartBeat;
extern CAN_HandleTypeDef  hcan2;


void Board_Tx_C1(void);
void Board_Tx_C2(void);
void Board_Tx_C3(void);
void Board_Rx_C1(uint8_t *rxbuf);
void Board_Rx_C2(uint8_t *rxBuf);
void Board_Rx_C3(uint8_t *rxBuf);
void Board_Rx_C4(uint8_t *rxBuf);

void C_Board_HeartBeat(void);

#endif

////////////////test///////////自己加上的
////上板给下板发送的结构体
//typedef struct
//{
//  uint8_t SOF; // 帧头，数据帧的起始标志
//  uint8_t CRC8; // 循环冗余校验，用于校验帧头部分的数据完整性

//	/*gimbal更新*/
//	float pitch_imu;
//	float yaw_imu;
//	float yaw_v;
//	float pitch_v;
//	float pitch_mec;//?
//	
//	/*视觉信息,发射更新*/
//	uint8_t vision_state;
//	bool hit_enable;
//	bool is_find_Target;
//	bool is_find_outpost;
//	bool is_find_base;
//	uint16_t launch_timer;//发射主动等待时间
//	float vision_pitch_tar;
//	float vision_yaw_tar;
//	
//  uint16_t CRC16; // 循环冗余校验，用于校验整个数据帧的完整性
//}Board_Tx_Info_t;

////下板给上板发送的结构体
//typedef struct
//{
//  uint8_t SOF; // 帧头，数据帧的起始标志
//  uint8_t CRC8; // 循环冗余校验，用于校验帧头部分的数据完整性
//	
//	float pitch_imu_tar;
//	float yaw_imu_tar;
//	float pitch_mec_tar;//?
//	
//	bool gimbal_state;
//	int8_t gimbal_mode;
//	
//	bool is_ready_shoot;//拨盘复位
//	bool is_on_lob;//吊
//	bool is_handle_shoot;//是否操作手手打
//	
//	bool is_fric_on;
//	uint8_t bullet_speed;

//	/*视觉信息*/
//	uint8_t my_color;
//	uint8_t video_open;
//	uint8_t vision_mode;
//	uint8_t blood_0;
//	uint8_t blood_1;
//	uint8_t blood_2;
//	uint8_t blood_3;
//	uint8_t blood_4;
//	uint8_t blood_5;
//	uint8_t blood_6;
//	uint8_t blood_7;
//	float v_x;
//	float v_y;
//  uint16_t CRC16; // 循环冗余校验，用于校验整个数据帧的完整性
//}Board_Rx_Info_t;

//extern Board_Tx_Info_t Board_Tx_Info;
//extern Board_Rx_Info_t Board_Rx_Info;
//bool Board_Tx_Send_Data(void);
//bool Board_C_Recieve_Data(uint8_t *rxBuf);

#endif