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
#include "led.h"
int16_t a;
void StartMonitorTask(void const *argument)
{

	for (;;)
	{
		rm_motor_list_heart_beat();

		C_Board_Communicate_HeartBeat();
		Vision_HearBeat();
		Vision_Sta_led_work();
		osDelay(1);
	}
}
