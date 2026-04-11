/* Includes ------------------------------------------------------------------*/
#include "motor.h"

extern CAN_HandleTypeDef hcan1;

/*DM START*/
Motor_DM_Born_Info_t Pitch_Born_Info = {
    .txId = 0x01,
    .hcan = &hcan1,
};

Motor_DM_Rx_Info_t Pitch_Rx_Info;
Motor_DM_Tx_Info_t Pitch_Tx_Info;
Motor_DM_State_t Pitch_State;
Motor_DM_Ctrl_Info_t Pitch_Ctrl;

Motor_DM_t Pitch_Motor = {
    .born_info = &Pitch_Born_Info,
    .rx_info = &Pitch_Rx_Info,
    .tx_info = &Pitch_Tx_Info,
    .state = &Pitch_State,
    .ctrl = &Pitch_Ctrl,
    .single_init = &DM_Single_Motor_Init,
    .type = dm_4310,
};
/*DM END*/

/*RM START*/
Motor_RM_Born_Info_t R_Fric_Born = {
    .rxId = 1,
    .hcan = &hcan1,
    .type = _3508_Single,
    .stdId = 0x200,
};

Motor_RM_Tx_Info_t R_Fric_Tx;
Motor_RM_State_t R_Fric_State;
Motor_RM_Rx_Info_t R_Fric_Rx;

pid_ctrl_t R_Fric_Speed_Ctrl ;

Motor_RM_Ctrl_Info_t R_Fric_Ctrl = {
    .speed_ctrl = &R_Fric_Speed_Ctrl,
};

Motor_RM_Born_Info_t L_Fric_Born = {
    .rxId = 0,
    .hcan = &hcan1,
    .type = _3508_Single,
    .stdId = 0x200,
};

Motor_RM_Tx_Info_t L_Fric_Tx;
Motor_RM_State_t L_Fric_State;
Motor_RM_Rx_Info_t L_Fric_Rx;

pid_ctrl_t L_Fric_Speed_Ctrl ;

Motor_RM_Ctrl_Info_t L_Fric_Ctrl = {
    .speed_ctrl = &L_Fric_Speed_Ctrl,
};

Motor_RM_Born_Info_t UP_Fric_Born = {
    .rxId = 2,
    .hcan = &hcan1,
    .type = _3508_Single,
    .stdId = 0x200,
};

Motor_RM_Tx_Info_t UP_Fric_Tx;
Motor_RM_State_t UP_Fric_State;
Motor_RM_Rx_Info_t UP_Fric_Rx;

pid_ctrl_t UP_Fric_Speed_Ctrl ;

Motor_RM_Ctrl_Info_t UP_Fric_Ctrl = {
    .speed_ctrl = &UP_Fric_Speed_Ctrl,
};

Motor_RM_t rm_motor[] = {
    [R_Fric] = {
        .born_info = &R_Fric_Born,
        .rx_info = &R_Fric_Rx,
        .tx_info = &R_Fric_Tx,
        .state = &R_Fric_State,
        .single_init = RM_Motor_Init,
        .ctrl = &R_Fric_Ctrl,
    },
    [L_Fric] = {
        .born_info = &L_Fric_Born,
        .rx_info = &L_Fric_Rx,
        .tx_info = &L_Fric_Tx,
        .state = &L_Fric_State,
        .single_init = RM_Motor_Init,
        .ctrl = &L_Fric_Ctrl,
    },
    [UP_Fric] = {
        .born_info = &UP_Fric_Born,
        .rx_info = &UP_Fric_Rx,
        .tx_info = &UP_Fric_Tx,
        .state = &UP_Fric_State,
        .single_init = RM_Motor_Init,
        .ctrl = &UP_Fric_Ctrl,
    },
};

Motor_RM_Group_t RM_Group_Fric = {
    .motor[R_Fric] = &rm_motor[R_Fric],
    .motor[L_Fric] = &rm_motor[L_Fric],
    .motor[UP_Fric] = &rm_motor[UP_Fric],
    .motor[3] = NULL,
    .stdId = 0x200,
    .hcan = &hcan1,
    .group_init = RM_Group_Motor_Init,
};

/* Exported functions --------------------------------------------------------*/
void rm_motor_list_init()
{
    RM_Group_Fric.group_init(&RM_Group_Fric);
}

void rm_motor_list_heart_beat()
{
    RM_Group_Fric.group_heartbeat(&RM_Group_Fric);
}