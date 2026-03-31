/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   上主控控制任务（云台+发射+视觉）
  *          注意：底盘控制逻辑在下主控
  ******************************************************************************
  */
#include "control_task.h"
#include "vision_protocol.h"
#include "usart.h"

void StartControlTask(void const * argument)
{
    for (;;)
    {
        gimbal.work(&gimbal);
        shoot.work(&shoot);
        Vision_Board_Update();
        Vision_DataTx(&huart1);
        osDelay(1);
    }
}
