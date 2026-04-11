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
static void All_CAN_Send_Here(void);
void StartControlTask(void const *argument)
{

    for (;;)
    {
        gimbal.work(&gimbal);
        fric.work(&fric);
        Vision_Board_Update();
        Vision_DataTx();
        All_CAN_Send_Here();
        osDelay(1);
    }
}
void All_CAN_Send_Here(void)
{
    if (Board_Rx_Info.flag.bit.is_rc_online == 1)
    {
        RM_Group_Fric.group_set_torque(&RM_Group_Fric);
        Pitch_Motor.single_set_torque(&Pitch_Motor);
        Send_To_Down_Board();
    }
    else
    {
        RM_Group_Fric.group_set_torque(&RM_Group_Fric);
        Pitch_Motor.single_sleep(&Pitch_Motor);
        Pitch_Motor.single_set_torque(&Pitch_Motor);
        Send_To_Down_Board();
    }
}