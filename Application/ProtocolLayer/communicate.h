#ifndef __communicate_H_
#define __communicate_H_
#include "rp_device_config.h"
#include "rc_sensor.h"
#include "drv_can.h"
#include "can_protocol.h"
#include "vision_protocol.h"
#include "judge_protocol.h"
#include "gimbal.h"
#include "rp_math.h"

// 上板给下板发送的结构体 (Up Tx = Down Rx)
typedef struct
{
    float yaw_imu_angle;       // yaw陀螺仪角度
    float yaw_imu_speed;       // yaw陀螺仪角速度
    float pitch_imu_angle;     // pitch陀螺仪角度
    float pitch_imu_speed;     // pitch陀螺仪角速度
    float vision_target_yaw;   // 视觉发过来的yaw
    float vision_target_pitch; // 视觉发过来的pitch
    float pitch_mec_angle;     // pitch机械角度

    __packed union
    {
        uint32_t realtime_flag;
        __packed struct
        {
            uint8_t pitch_motor_online : 1; // pitch在线
            uint8_t L_fric_online : 1;      // 左摩擦轮在线
            uint8_t R_fric_online : 1;      // 右摩擦轮在线
            uint8_t is_find_target : 1;     // 找到人了
            uint8_t hit_enable : 1;         // 开火电平
            uint8_t is_keep_shoot : 1;      // 单发还是连发
            uint8_t is_vision_online : 1;   // 视觉是否在线
        } flag;
    };
} Board_Tx_Info_t;

// 下板给上板发送的结构体 (Up Rx = Down Tx)
typedef struct
{
    float fric_target_speed;
    float pitch_output;

    __packed union
    {
        uint32_t realtime_flag;
        __packed struct
        {
            uint8_t our_color_flag : 1;
            uint8_t is_rc_online : 1;
            uint8_t is_ready_shoot : 1;
            uint8_t is_game_in_progress : 1;
            uint8_t is_energy_engine_mode : 1;
        } bit;
    } flag;
} Board_Rx_Info_t;

typedef struct
{
    dev_work_state_t status;    // 接受状态
    uint32_t send_time;         // 发送间隔
    uint32_t rx_tick;           // 接受到信息时的时间
    uint8_t offline_cnt_pack_1; // 接受离线计数
    uint8_t offline_cnt_pack_2;
    uint8_t offline_cnt_max; // 接受离线最大计数
} Board_HeartBeat_t;

void Board_Tx_Update(Board_Tx_Info_t *Board_Tx_Info);
extern Board_Tx_Info_t Board_Tx_Info;
extern Board_Rx_Info_t Board_Rx_Info;
extern Board_HeartBeat_t Board_HeartBeat;

void Board_Tx_D1(void);
void Board_Tx_D2(void);
void Board_Tx_D3(void);
void Board_Tx_D4(void);
void Board_Rx_D1(uint8_t *rxbuf);
void Board_Rx_D2(uint8_t *rxbuf);

void Send_To_Down_Board(void);
void C_Board_HeartBeat(void);

#endif
