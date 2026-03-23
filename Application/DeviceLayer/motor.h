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
/*电机定义步骤------------------------------------------------*/
//如果要增删改RM电机
//1.rm_motor_driver添加电机ID、CAN类型 
//2.dev_rm_motor_list_e里加电机名字
//3.CAN1_rxDataHandler，CAN2_rxDataHandler里添加获取电机信息的函数
//4.rm_motor_t rm_motor[]数组里加电机总结构体
//5.定义pid结构体以及在rm_motor_list_init里用mo tor_pid_init初始化pid结构体
//如果要驱动电机就在motor_out里赋值，由CAN_Send统一发送
/*电机ID宏定义------------------------------------------------*/
#define ID_GIMB_YAW 	0x142
#define ID_DAIL 		0x207 //0x1FF  45
#define ID_IMAGE		0x206 //0x1FF  23
#define ID_TELESCOPE	0x205 //0x1FF  01


#define ID_FRIC_B_UP 	0x204     //0x201 //0x200  01
#define ID_FRIC_F_UP 	0x201 //0x200  23
#define ID_FRIC_B_R 	0x202//       0x203 //0x200  45
#define ID_FRIC_B_L 	0x203          //  0x204 //0x200  67
#define ID_FRIC_F_R 	0x205 //0x1FF  01
#define ID_GIMB_P 		0x206 //0x1FF  23
#define ID_FRIC_F_L 	0x207 //0x1FF  45
#define ID_FRIC_UP 	0x208 //0x1FF  67
extern  KT_motor_t kt_motor[1];
extern  Motor_HT_t L_Wheel;
extern  Motor_DM_t DAIL;


extern  Motor_RM_Group_t RM_Group_F1;
extern  Motor_RM_Group_t RM_Group_F2;
extern  Motor_RM_Group_t RM_Group_3;
/* Exported functions --------------------------------------------------------*/
void rm_motor_list_init(void);
void rm_motor_list_heart_beat(void);
void kt_motor_list_init(void);
void ht_motor_list_init(void);
void dm_motor_list_init(void);
uint8_t rm_motor_list_workstate(void);

#endif

