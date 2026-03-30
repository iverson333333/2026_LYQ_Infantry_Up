#include "gimbal.h"
#include "communicate.h"
#include "device.h"
#include "rp_math.h"

gimbal_t gimbal =
	{
		.pitch_motor = &Pitch_Motor,
		.work = Gimbal_Work,
		.gimbal_reset_state = DEV_RESET_NO,
};

static uint8_t pitch_dm_inited = 0;

void Gimbal_Work(gimbal_t *gimbal)
{
	// 仅初始化一次DM电机控制接口
	if (pitch_dm_inited == 0)
	{
		Pitch_Motor.single_init(&Pitch_Motor);
		pitch_dm_inited = 1;
	}

	// 本地姿态信息更新：IMU角度/速度 + DM电机机械角（-PI~PI）
	gimbal->base_info.pitch_imu_angle = -imu_sensor.info->base_info.roll + sgn(imu_sensor.info->base_info.roll) * 180.f;
	gimbal->base_info.pitch_imu_speed = -imu_sensor.info->base_info.ave_rate_roll;

	gimbal->base_info.pitch_motor_angle = Pitch_Motor.rx_info->motor_angle;
	gimbal->base_info.pitch_mec_360_angle = gimbal->base_info.pitch_motor_angle / PI * 180.f;

	gimbal->gimbal_reset_state = DEV_RESET_OK;

	// 开控=1时下发下板给出的pitch_output（直接当力矩）
	if (Board_Rx_Info.flag.bit.is_rc_online == 1)
	{
		gimbal->base_info.output_gimbal_p = constrain(Board_Rx_Info.pitch_output, T_MIN_4310, T_MAX_4310);
		Pitch_Motor.tx_info->torque = gimbal->base_info.output_gimbal_p;
	}
	else
	{
		// 关控=0时卸力
		gimbal->base_info.output_gimbal_p = 0.f;
		Pitch_Motor.single_sleep(&Pitch_Motor);
	}
}
