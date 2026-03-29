#include "communicate.h"
#include "string.h"
#include "crc.h"
#include "drv_can.h"
#include "usbd_cdc_if.h"

Board_Tx_Info_t Board_Tx_Info;
Board_Rx_Info_t Board_Rx_Info;
Board_HeartBeat_t Board_HeartBeat = {.offline_cnt_pack_1 = 0, .offline_cnt_pack_2 = 0, .offline_cnt_max = 50};

extern CAN_HandleTypeDef hcan2;

uint8_t board_tx_buf_1[8];
uint8_t board_tx_buf_2[8];
uint8_t board_tx_buf_3[8];
uint8_t board_tx_buf_4[8];

void Board_Tx_D1(void)
{
    memcpy(&board_tx_buf_1[0], &Board_Tx_Info.yaw_imu_angle, 4);
    memcpy(&board_tx_buf_1[4], &Board_Tx_Info.yaw_imu_speed, 4);
    CAN_SendData(&hcan2, 0xD1, board_tx_buf_1);
}

void Board_Tx_D2(void)
{
    memcpy(&board_tx_buf_2[0], &Board_Tx_Info.pitch_imu_angle, 4);
    memcpy(&board_tx_buf_2[4], &Board_Tx_Info.pitch_imu_speed, 4);
    CAN_SendData(&hcan2, 0xD2, board_tx_buf_2);
}

void Board_Tx_D3(void)
{
    memcpy(&board_tx_buf_3[0], &Board_Tx_Info.vision_target_yaw, 4);
    memcpy(&board_tx_buf_3[4], &Board_Tx_Info.vision_target_pitch, 4);
    CAN_SendData(&hcan2, 0xD3, board_tx_buf_3);
}

void Board_Tx_D4(void)
{
    memcpy(&board_tx_buf_4[0], &Board_Tx_Info.pitch_mec_angle, 4);
    memcpy(&board_tx_buf_4[4], &Board_Tx_Info.realtime_flag, 4);
    CAN_SendData(&hcan2, 0xD4, board_tx_buf_4);
}

void Board_Tx_Update(Board_Tx_Info_t *Board_Tx_Info)
{
    Board_Tx_Info->yaw_imu_angle = imu_sensor.info->base_info.yaw;
    Board_Tx_Info->yaw_imu_speed = imu_sensor.info->base_info.rate_yaw;
    Board_Tx_Info->pitch_imu_angle = gimbal.base_info.pitch_imu_angle;
    Board_Tx_Info->pitch_imu_speed = gimbal.base_info.pitch_imu_speed;
    Board_Tx_Info->vision_target_yaw = -vision.VtoE->yaw;
    Board_Tx_Info->vision_target_pitch = vision.VtoE->pitch;
    Board_Tx_Info->pitch_mec_angle = gimbal.base_info.pitch_motor_angle;

    Board_Tx_Info->flag.pitch_motor_online = (rm_motor[gim_pitch].state->status == DEV_ONLINE) ? 1 : 0;
    Board_Tx_Info->flag.L_fric_online = (rm_motor[B_L_Fric].state->status == DEV_ONLINE) ? 1 : 0;
    Board_Tx_Info->flag.R_fric_online = (rm_motor[B_R_Fric].state->status == DEV_ONLINE) ? 1 : 0;
    Board_Tx_Info->flag.is_find_target = vision.VtoE->flag_union.bit.is_find_target;
    Board_Tx_Info->flag.hit_enable = vision.VtoE->flag_union.bit.is_enable_shootting;
    Board_Tx_Info->flag.is_keep_shoot = 0;
    Board_Tx_Info->flag.is_vision_online = (vision.status->rx_state == DEV_ONLINE) ? 1 : 0;
}

void Board_Rx_D1(uint8_t *rxbuf)
{
    memcpy(&Board_Rx_Info.pitch_imu_tar, &rxbuf[0], 4);
    memcpy(&Board_Rx_Info.pitch_mec_tar, &rxbuf[4], 4);
    Board_HeartBeat.offline_cnt_pack_1 = 0;
}

void Board_Rx_D2(uint8_t *rxbuf)
{
    memcpy(&Board_Rx_Info.yaw_mec_imu, &rxbuf[0], 4);
    memcpy(&Board_Rx_Info.bullet_speed, &rxbuf[4], 4);
    Board_HeartBeat.offline_cnt_pack_2 = 0;
}

void Board_Rx_D5(uint8_t *rxbuf)
{
    memcpy(&Board_Rx_Info.flag.realtime_flag, &rxbuf[0], 4);
    Board_Rx_Info.shoot_count = rxbuf[4];
    Board_Rx_Info.vision_mode = rxbuf[5];
    Board_Rx_Info.gimbal_mode = (int8_t)rxbuf[6];
    Board_Rx_Info.video_open = rxbuf[7];
    Board_HeartBeat.offline_cnt_pack_1 = 0;
}

void Send_To_Down_Board(void)
{
    Board_Tx_Update(&Board_Tx_Info);
    Board_Tx_D1();
    Board_Tx_D2();
    Board_Tx_D3();
    Board_Tx_D4();
}

void C_Board_HeartBeat(void)
{
    Board_HeartBeat.offline_cnt_pack_1++;
    Board_HeartBeat.offline_cnt_pack_2++;

    if (Board_HeartBeat.offline_cnt_pack_1 > Board_HeartBeat.offline_cnt_max ||
        Board_HeartBeat.offline_cnt_pack_2 > Board_HeartBeat.offline_cnt_max)
    {
        Board_HeartBeat.offline_cnt_pack_1 = Board_HeartBeat.offline_cnt_max;
        Board_HeartBeat.offline_cnt_pack_2 = Board_HeartBeat.offline_cnt_max;
        Board_HeartBeat.status = DEV_OFFLINE;
    }
    else if (Board_HeartBeat.status == DEV_OFFLINE)
    {
        Board_HeartBeat.status = DEV_ONLINE;
    }
}
