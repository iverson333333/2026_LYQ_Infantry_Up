/**
 ******************************************************************************
 * File Name          : gimbal.c
 * Description        : 云台控制实现 — 上主控
 ******************************************************************************
 * @attention
 * Copyright (c) 2026 LYQ
 ******************************************************************************
 */

#include "gimbal.h"
#include "communicate.h"
#include "device.h"
#include "rp_math.h"

/* Private function prototypes -----------------------------------------------*/
void Gimbal_Init(gimbal_t *gimbal); // 初始化实现
void Gimbal_Work(gimbal_t *gimbal); // 主处理函数实现

/* Private variables ---------------------------------------------------------*/
static uint8_t pitch_dm_inited = 0; // pitch电机初始化标志

/* Public variables ---------------------------------------------------------*/
gimbal_t gimbal = {
    .pitch_motor = &Pitch_Motor,
    .init = Gimbal_Init,
    .work = Gimbal_Work,
    .gimbal_reset_state = DEV_RESET_NO,
};

/* Public functions --------------------------------------------------------*/

/* Private member functions ---------------------------------------------------*/
/**
 * @brief  初始化实现
 * @param  gimbal: 云台句柄
 */
void Gimbal_Init(gimbal_t *gimbal)
{
    gimbal->init = Gimbal_Init;
    gimbal->work = Gimbal_Work;
    // 初始化DM电机控制接口
    if (pitch_dm_inited == 0)
    {
        Pitch_Motor.single_init(&Pitch_Motor);
        pitch_dm_inited = 1;
    }

    // 设置初始状态为复位
    gimbal->gimbal_reset_state = DEV_RESET_NO;
}

/**
 * @brief  主处理函数实现
 * @param  gimbal: 云台句柄
 */
void Gimbal_Work(gimbal_t *gimbal)
{

    gimbal->base_info.pitch_imu_angle = imu_sensor.info->base_info.pitch;
    gimbal->base_info.pitch_imu_speed = imu_sensor.info->base_info.ave_rate_pitch;
    gimbal->base_info.pitch_motor_angle = Pitch_Motor.rx_info->motor_angle;
    gimbal->base_info.pitch_mec_360_angle = gimbal->base_info.pitch_motor_angle / PI * 180.f;

    // 标记复位完成
    gimbal->gimbal_reset_state = DEV_RESET_OK;

    // 根据遥控器状态决定输出
    if (Board_Rx_Info.flag.bit.is_rc_online == 1)
    {
        // 遥控器在线：输出pitch控制
        gimbal->base_info.output_gimbal_p = constrain(Board_Rx_Info.pitch_output, T_MIN_4310, T_MAX_4310);
        Pitch_Motor.tx_info->torque = Board_Rx_Info.pitch_output;
    }
    else
    {
        // 遥控器离线：输出清零，电机进入睡眠
        gimbal->base_info.output_gimbal_p = 0.f;
        Pitch_Motor.single_sleep(&Pitch_Motor);
    }
}