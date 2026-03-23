#ifndef __car_H_
#define __car_H_
#include "chassis.h"
#include "communicate.h"
#include "gimbal.h"
#include "shoot.h"
typedef enum 
{
  offline_CAR,        //        0
  init_CAR,           //     1
  mec_CAR,            //      2
  gyro_CAR,           //      3
  cycle_CAR,		  //       4
  vision_gyro_CAR,    //   5
  vision_cycle_CAR,   // 6
  lob_CAR,            //      7

}Car_Move_Mode_e;

/* 3???????????*/
typedef enum 
{
  RC_CTRL_MODE,        //
  KEY_CTRL_MODE,       //   1
}Car_Ctrl_Mode_e;

/**
 * @brief 
 * 
 */
typedef  struct __attribute__((packed))  car_struct
{
  Car_Move_Mode_e car_move_mode;
  Dev_Reset_State_e	 car_reset_state;
  Car_Ctrl_Mode_e  car_ctrl_mode;
  
  uint8_t  unlock_car_flag;
  uint16_t init_cnt;
  uint16_t init_cnt_max;
  
  
	
}car_t;

extern car_t car;
void Car_Ctrl(car_t *car);
void Car_Work(void);

#endif