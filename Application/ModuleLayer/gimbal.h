#ifndef __GIMBAL_H
#define __GIMBAL_H


/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "communicate.h"
#include "myrobot_def.h"
#include "motor.h"
#include "bmi.h"

/* Exported macro ------------------------------------------------------------*/

//#define YAW_MOTOR_ANGLE_MIDDLE 		(17463.f-1000.f)  		  //YAW电机中值
#define PITCH_MOTOR_ENCODER_MIDDLE  (1430.f)           //(3400.f+2950.f)      //(2950.f)    //pitch电机编码器中值
#define GIMBAL_LOB_MEC_ANGEL	 (628.f)      //吊射机械角度 15.6弹速606
#define GIMBAL_LOB_LOW_MEC_ANGEL	 (536.f)      //吊射底部机械角度  
#define GIMBAL_MAX_MEC_ANGEL   		(905.f)				//pitch机械角度电控限位最大值 920
#define GIMBAL_MIN_MEC_ANGEL  		(-323.f)			//pitch机械角度电控限位最小值 -338

#define GIMBAL_MAX_GYRO_ANGEL		(gimbal->base_info.pitch_imu_angle + (GIMBAL_MAX_MEC_ANGEL - gimbal->base_info.pitch_motor_angle) / 8192.f * 360.f)
//pitch陀螺仪角度电控限位最小值       
#define GIMBAL_MIN_GYRO_ANGEL		(gimbal->base_info.pitch_imu_angle - (gimbal->base_info.pitch_motor_angle - GIMBAL_MIN_MEC_ANGEL) / 8192.f * 360.f)

/*云台pid计算类型*/
typedef enum
{
	GYRO_PID,
	MEC_PID,
	SPEED_PID,
}gimbal_pid_mode_e;

/*视觉偏置*/
typedef struct __attribute__((packed)) 
{
	float vision_pitch_offset;
	float lob_pitch_gyro_offset;
}gimbal_offset_info_t;

/*吊射包*/
typedef struct __attribute__((packed)) 
{
	uint8_t lob_init_angle_flag;//初始化吊射角度标志位,为了只初始化一次
	float lob_init_mec_yaw_angle;//机械角度 

	float pre_aim_pitch_angle;//吊射预瞄pitch陀螺角
 
	float gyro_init_lob_pitch_angle;//取吊射命令的那一刻的角度
	
	uint8_t last_into_oblique_lob_command_flag;//判断下降沿跳变
	uint16_t out_oblique_head_homing_timeout;
	
	uint8_t into_auto_lob_command_flag; //只有先进命令才能进lob更新，为了先进命令再进吊射更新,持续为1直到退出
	uint8_t into_normal_lob_command_flag;
}gimbal_lob_info_t;

/*重力偏置*/
typedef struct __attribute__((packed))  
{
	float const_k;//mgL
	float center_of_gravity_angle;//水平 与 转轴到整头重心连线的夹角
	float torque_angle;//转矩角 
	float gravity_offset_output;
}gimbal_gravity_offset_info_t;

/*pitch控制类型*/
typedef struct __attribute__((packed))  
{
	uint8_t gimbal_mode;   //2吊射还是1陀螺仪(3自瞄？)
}gimbal_mode_t;

/*云台基础信息包*/
typedef struct __attribute__((packed))  
{
	
	float pitch_imu_angle;					//云台陀螺仪pitch轴角度 初始化后初值是0
	float pitch_imu_speed;          //云台陀螺仪pitch轴速度 rad/s
	float  pitch_imu_angle_target;  //陀螺仪模式目标pitch  世界坐标系 (-90°~90°)   (向上为正)
	float  pitch_motor_angle;       //pitch轴 相对底盘   角度(0~16383)    (向上为正)
	float  pitch_motor_speed;       //pitch轴 相对底盘   速度(dps)           (向上为正)
	float  pitch_mec_angle_target;  //机械模式目标pitch	底盘坐标系	(0~16383)  (向上为正)
	
	float pitch_mec_360_angle;
	
	
	
	int16_t  output_gimbal_p;				//pitch轴电机输出

	uint16_t init_time;//初始化时间
	uint16_t init_time_max;//初始化超时
	uint16_t init_time_max_count;//初始化超时计数
}gimbal_base_info_t;


typedef struct gimbal_class_t
{
	Motor_RM_t 		     *gimbal_p;
	KT_motor_t         *gimbal_y;
	gimbal_gravity_offset_info_t gravity_offset_info;
	gimbal_base_info_t   	base_info;
	gimbal_lob_info_t		lob_info;
	gimbal_offset_info_t  	*offset_info;//偏置信息
 
	
	float (*all_pid_calc)(pid_ctrl_t *out,pid_ctrl_t *inn,float target,float mea_out,float mea_in,float inner_kp,uint8_t err_cal_mode);
	gimbal_pid_mode_e			ptich_pid_mode;//pitch轴pid模式
	
	void              	  		 (*work)(struct gimbal_class_t *gimbal);
	Dev_Reset_State_e			 gimbal_reset_state; //云台初始化状态
	gimbal_mode_t          gimbal_ctrl_mode;
}gimbal_t;

extern gimbal_t gimbal;

void Gimbal_Work(gimbal_t *gimbal);


#endif