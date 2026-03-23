/**
 * @file vision_protocol.h
 * @author Isaac
 * @brief 视觉通信协议
 * @version 0.1
 * @date 2023-11-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef __VISION_PROTOCOL_H
#define __VISION_PROTOCOL_H
#include "gimbal.h"
#include "rp_config.h"
#include "led.h"
#define  VISION_OFFLINE_CNT_MAX  (80)//离线最大计数(ms)
 /**
 * @brief 电控发给视觉的数据帧结构体
 */
typedef struct  __attribute__((packed)) 
{
 uint8_t SOF; // 帧头，数据帧的起始标志

 __packed union { // 状态标志位联合体（32位）
 uint32_t all_flags; // 整体32位标志值
 __packed struct {
	 uint8_t own_color:1 ; // 位0：己方颜色
 uint8_t game_start:1 ; // 位1：比赛开始
 uint8_t is_ready:1 ; // 位2：是否允许打弹（热量够 && 复位完毕）
 uint8_t outpost_mode:1  ;// 位3：只锁前哨模式
 uint8_t engineer_mode:1 ;// 位4：只锁工程模式
 uint8_t lob_mode:1  ; // 位5：只吊射模式
//	 uint8_t is_com_vision:1;  
 uint32_t reserved :26 ; // 位6-31：可扩展
 } bit; // 按位访问的子结构
 } flag_union; 
	  uint8_t CRC8; // 循环冗余校验，用于校验帧头部分的数据完整性

 float yaw; // 当前yaw角
 float pitch; // 当前pitch角
 float roll; // 当前云台roll角
 float yaw_speed; // yaw轴速度
 float pitch_speed; // pitch轴速度
 int8_t pitch_offset; // pitch轴偏移量（电控退自瞄后清零）
 int8_t yaw_offset; // yaw轴偏移量（电控退自瞄后清零）
// float bullet_speed; // 子弹速度
 
 uint16_t bullet_id;    // 每打出一发加1

 uint32_t user_debug; // 用户调试信息：
 // - 单发模式：接收命令到子弹过测速模块的延时（ms）
 // - 连发模式：发射子弹的间隔时间（ms）
 // - 通用：用于调试目的
 uint16_t CRC16; // 循环冗余校验，用于校验整个数据帧的完整性
} ElectricalToVisionFrame;

/**
 * @brief 视觉发给电控的数据帧结构体
 */
typedef struct  __attribute__((packed)) 
{
 uint8_t SOF; // 帧头，数据帧的起始标志
 __packed union { // 状态标志位联合体（32位）
 uint32_t all_flags; // 整体32位标志值
 __packed struct {
 uint8_t is_find_target:1  ;// 位0：用于决定是否给视觉控pitch、yaw
 uint8_t is_keep_shooting:1  ;// 位1：用于拨盘速度环还是角度环，英雄只角度
 uint8_t is_enable_shootting :1 ;  // 位2：用于是否可以打弹
 uint8_t detect_num :4 ; // 位3-6：锁到几号（占用4位，支持0-15编号
 uint32_t reserved :25 ;  // 位7-31：保留位
 } bit; // 按位访问的子结构
 } flag_union; 

  uint8_t CRC8; // 循环冗余校验，用于校验帧头部分的数据完整性

 float yaw; // 目标yaw角
 float pitch; // 目标pitch角
 
 uint16_t timing;//发射延时
 
 uint32_t user_debug; // 用户调试信息，自定义debug
 uint16_t CRC16; // 循环冗余校验，用于校验整个数据帧的完整性
} VisionToElectricalFrame;


/**
 * @brief 视觉通信 视觉给电控信息结构体
 * 
 */
typedef struct  __attribute__((packed)) 
{
	uint8_t  SOF;
	uint8_t  mode;//视觉状态：1 自瞄   2 识别小陀螺
	uint8_t  CRC8;
  
	float    yaw;			 //云台yaw目标角度
	float    pitch;		 //云台pitch目标角度
	uint8_t  is_find_target;//是否捕获到目标，用于交出云台控制权
	uint8_t  is_shoot_enable;//是否开火，用于更新时间戳，时间戳是视觉控制发射的权力来源
	uint16_t timing;//发射延时
	
	uint16_t UI_x;
	uint16_t UI_y;
 
	uint16_t uix_lt;
	uint16_t uiy_lt;

	uint16_t uix_lb;
	uint16_t uiy_lb;

	uint16_t uix_rb;
	uint16_t uiy_rb;

	uint16_t uix_rt;
	uint16_t uiy_rt;

	uint16_t uix_left;
	uint16_t uiy_left;

	uint16_t uix_right;
	uint16_t uiy_right;

	uint8_t  detect_num; 
	float building_yaw; //前哨和基地的pitch，yaw  
	float building_pitch;
	float distance; //识别到发距离，没识别到发0
	uint16_t CRC16;
}Vision_Rx_Info_t;

/**
 * @brief 视觉通信 电控给视觉信息结构体
 */
typedef struct  __attribute__((packed)) 
{
    uint8_t  SOF;
    uint8_t  mode; // 1：打车 2：打前哨
    uint8_t  CRC8;

    uint8_t  is_ready;     // 是否准备打弹 0：没准备好 1：准备好了
	  uint16_t bullet_id;    // 每打出一发加1

    float    yaw;          // 云台yaw 从上往下看顺时针为负
    float    pitch;        // 云台pitch 向上为负
    float    roll;         // 云台roll
	float    v_yaw;    // 云台yaw speed 从上往下看顺时针为负
	float    v_pitch;  // 云台pitch speed 向上为负

	int8_t   yaw_offset;		//操作手手动发给视觉偏置
	int8_t	 pitch_offset;		//操作手手动发给视觉偏置

	float    bullet_speed; // 子弹速度
    uint8_t  my_color;     // 己方颜色,
	uint8_t  dune;		   // 5s倒计时和比赛开始时为1，其余时间为0

    uint16_t CRC16;
} Vision_Tx_Info_t;

/**
 * @brief 视觉通信 状态结构体
 */
typedef struct __attribute__((packed)) 
{
	dev_work_state_t tx_state;						//发送状态
	dev_work_state_t rx_state;						//接受状态
	uint32_t send_time;                   //发送间隔
	uint32_t rx_tick;						//接受到信息时的时间
	uint8_t offline_cnt;									//接受离线计数
	uint8_t offline_cnt_max;							//接受离线最大计数
}Vision_Status_t;

/**
 * @brief 时间戳信息
 * 
 */

typedef  struct __attribute__((packed)) 
{
	uint32_t vision_shoot_timing[3];
	uint32_t shooting_begin_tick; //开始打弹时用上一帧接受视觉的tick
}Vision_Timestamp_Info_t;


/**
 * @brief 视觉通信 总结构体
 * 
 */
typedef struct __attribute__((packed)) 
{
	/* data */
//	Vision_Tx_Info_t *tx_info;
//	Vision_Rx_Info_t *rx_info;
	VisionToElectricalFrame *VtoE;
	ElectricalToVisionFrame *EtoV;
	Vision_Timestamp_Info_t *timestamp_info;
	Vision_Status_t  *status;
	uint32_t shooting_cmd_excute_tick;
	uint16_t shooting_cmd_excute_tick_buf[100];
	float shooting_cmd_excute_tick_mean;
	float shooting_cmd_excute_tick_variance;
	
}Vision_t;

extern Vision_t vision;

void Vison_Interrupt_Update(void);
void Vision_led_work(void);
void Vision_DataTx(UART_HandleTypeDef *huart);
void Vision_DataRx(uint8_t *rxBuf);
void Vision_Board_Update(void);

void Shooting_Cmd_Excute_Tick_Calculating(uint8_t flag);
void Rearrange_Vision_Timing_Buff(uint32_t* vision_timing_buff, uint8_t size);
void Vision_HearBeat(void);
#endif
