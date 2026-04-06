/**
  ******************************************************************************
  * @file    car.h
  * @brief   整车控制模块头文件（上主控不使用，已精简）
  ******************************************************************************
  */
#ifndef __car_H_
#define __car_H_

#include "communicate.h"
#include "gimbal.h"

/* 仅保留结构体声明，实际控制逻辑在下主控 */
typedef enum {
    offline_CAR,
    init_CAR,
    mec_CAR,
    gyro_CAR,
    cycle_CAR,
    vision_gyro_CAR,
    vision_cycle_CAR,
    lob_CAR,
} Car_Move_Mode_e;

typedef enum {
    RC_CTRL_MODE,
    KEY_CTRL_MODE,
} Car_Ctrl_Mode_e;

typedef struct __attribute__((packed)) car_struct {
    Car_Move_Mode_e car_move_mode;
    Dev_Reset_State_e car_reset_state;
    Car_Ctrl_Mode_e car_ctrl_mode;
    uint8_t unlock_car_flag;
    uint16_t init_cnt;
    uint16_t init_cnt_max;
} car_t;

extern car_t car;

#endif
