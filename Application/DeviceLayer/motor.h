#ifndef __MOTOR_H

#define __MOTOR_H

#include "rp_config.h"

#include "can_protocol.h"

#include "rm_motor.h"

#include "KT_motor.h"

#include "HT_motor.h"

#include "DM_motor.h"

#include "motor_def.h"

#include "drv_can.h"

/*电机ID宏定义------------------------------------------------*/

#define ID_FRIC_L 0x201
#define ID_FRIC_R 0x202
#define ID_FRIC_UP 0x203
#define ID_GIMB_P 0x11

extern Motor_RM_Group_t RM_Group_Fric;
extern Motor_RM_t rm_motor[];
/* Exported functions --------------------------------------------------------*/

void rm_motor_list_init(void);

void rm_motor_list_heart_beat(void);

void dm_motor_list_init(void);

uint8_t rm_motor_list_workstate(void);

#endif
