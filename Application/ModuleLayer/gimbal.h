#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "rp_config.h"
#include "rp_device_config.h"
#include "DM_Motor.h"
#include "myrobot_def.h"
#include "motor.h"
#include "bmi.h"

/*pitch控制类型*/
typedef struct __attribute__((packed))
{
	uint8_t gimbal_mode;
} gimbal_mode_t;

/*云台基础信息包*/
typedef struct __attribute__((packed))
{

	float pitch_imu_angle;
	float pitch_imu_speed;
	float pitch_motor_angle;
	float pitch_mec_360_angle;
	float output_gimbal_p;
} gimbal_base_info_t;

typedef struct gimbal_class_t
{
	Motor_DM_t *pitch_motor;
	gimbal_base_info_t base_info;
	void (*work)(struct gimbal_class_t *gimbal);
	Dev_Reset_State_e gimbal_reset_state;
	gimbal_mode_t gimbal_ctrl_mode;
} gimbal_t;

extern gimbal_t gimbal;
extern Motor_DM_t Pitch_Motor;

void Gimbal_Work(gimbal_t *gimbal);

#endif