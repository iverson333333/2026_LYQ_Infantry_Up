/**
 ******************************************************************************
 * @file    monitor_task.c
 * @brief   监控任务
 *          1. 各模块心跳失联检测
 *          2. 监控遥控器状态，软件复位
 ******************************************************************************
 */
#include "monitor_task.h"
#include "communicate.h"
#include "rc_sensor.h"
int16_t a;
void StartMonitorTask(void const *argument)
{


	for (;;)
	{
		rm_motor_list_heart_beat();
//		DAIL.single_heart_beat(&DAIL);
//		L_Wheel.single_heart_beat(&L_Wheel);
		C_Board_HeartBeat();
		rc_sensor.heart_beat(&rc_sensor);
		Vision_HearBeat();
//		test_rc_lost();
		osDelay(1);
	}
}

