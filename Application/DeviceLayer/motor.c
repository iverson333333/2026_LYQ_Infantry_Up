
/* Includes ------------------------------------------------------------------*/
#include "motor.h"

extern CAN_HandleTypeDef hcan1;

/*DM START*/
Motor_DM_Born_Info_t Pitch_Born_Info =
	{
		.txId = 0x01,
		.hcan = &hcan1,
};

Motor_DM_Rx_Info_t Pitch_Rx_Info;
Motor_DM_Tx_Info_t Pitch_Tx_Info;
Motor_DM_State_t Pitch_State;
Motor_DM_Ctrl_Info_t Pitch_Ctrl;

Motor_DM_t Pitch_Motor =
	{
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
Motor_RM_Born_Info_t R_Fric_Born =
	{
		.rxId = 1,

		.hcan = &hcan1,

		.type = _3508_Single,

		.stdId = 0x200,
};

Motor_RM_Tx_Info_t R_Fric_Tx;

Motor_RM_State_t R_Fric_State;

Motor_RM_Rx_Info_t R_Fric_Rx;

pid_ctrl_t R_Fric_Speed_Ctrl =
	{
		.kp = 27.f, //
		.ki = 0.5f,
		.kd = 0.f,
		.integral_max = 6000.f,
		.out_max = 10000.f, //
};

Motor_RM_Ctrl_Info_t R_Fric_Ctrl =
	{
		.speed_ctrl = &R_Fric_Speed_Ctrl,
};

Motor_RM_Born_Info_t L_Fric_Born =
	{
		.rxId = 0,

		.hcan = &hcan1,

		.type = _3508_Single,

		.stdId = 0x200,
};

Motor_RM_Tx_Info_t L_Fric_Tx;

Motor_RM_State_t L_Fric_State;

Motor_RM_Rx_Info_t L_Fric_Rx;

pid_ctrl_t L_Fric_Speed_Ctrl =
	{
		.kp = 27.f, //
		.ki = 0.5f,
		.kd = 0.f,
		.integral_max = 6000.f,
		.out_max = 10000.f, //
};

Motor_RM_Ctrl_Info_t L_Fric_Ctrl =
	{
		.speed_ctrl = &L_Fric_Speed_Ctrl,
};

Motor_RM_Born_Info_t UP_Fric_Born =
	{
		.rxId = 2,

		.hcan = &hcan1,

		.type = _3508_Single,

		.stdId = 0x200,
};

Motor_RM_Tx_Info_t UP_Fric_Tx;

Motor_RM_State_t UP_Fric_State;

Motor_RM_Rx_Info_t UP_Fric_Rx;

pid_ctrl_t UP_Fric_Speed_Ctrl =
	{
		.kp = 27.f, //
		.ki = 0.51f,
		.kd = 0.f,
		.integral_max = 6000.f,
		.out_max = 10000.f, //
};

Motor_RM_Ctrl_Info_t UP_Fric_Ctrl =
	{
		.speed_ctrl = &UP_Fric_Speed_Ctrl,
};

Motor_RM_t rm_motor[] =
	{
		[R_Fric] =
			{
				.born_info = &R_Fric_Born,

				.rx_info = &R_Fric_Rx,

				.tx_info = &R_Fric_Tx,

				.state = &R_Fric_State,

				.single_init = RM_Motor_Init,

				.ctrl = &R_Fric_Ctrl,
			},
		[L_Fric] =
			{
				.born_info = &L_Fric_Born,

				.rx_info = &L_Fric_Rx,

				.tx_info = &L_Fric_Tx,

				.state = &L_Fric_State,

				.single_init = RM_Motor_Init,

				.ctrl = &L_Fric_Ctrl,
			},
		[UP_Fric] =
			{
				.born_info = &UP_Fric_Born,

				.rx_info = &UP_Fric_Rx,

				.tx_info = &UP_Fric_Tx,

				.state = &UP_Fric_State,

				.single_init = RM_Motor_Init,

				.ctrl = &UP_Fric_Ctrl,
			},

};

Motor_RM_Group_t RM_Group_F1 =
	{
		.motor[0] = NULL,

		.motor[1] = &rm_motor[R_Fric],

		.motor[2] = &rm_motor[L_Fric],

		.motor[3] = &rm_motor[UP_Fric],

		.stdId = 0x200,

		.hcan = &hcan1,

		.group_init = RM_Group_Motor_Init,
};

/*RM END*/

KT_motor_t kt_motor[] = {
	[0] = {
		.KT_motor_info = {
			.tx_info = {
				.angle_single_Control = 0,
				.angle_single_Control_maxSpeed = 0,
				.angle_single_Control_spinDirection = 0,
				.angle_add_Control = 0,
				.angle_add_Control_maxSpeed = 0,
				.angle_sum_Control = 0,
				.angle_sum_Control_maxSpeed = 0,
				.iqControl = 0,
				.speedControl = 0,
			},
			.id = {
				.tx_id = ID_GIMB_YAW,
				.rx_id = 0x88,
				.drive_type = M_CAN1,
				.motor_type = KT9015,
			},
			.rx_info = {
				.accel = 0,
				.encoder = 0,
				.encoderRaw = 0,
				.encoderOffset = 0,
				.temperature = 0,
				.voltage = 0,
				.errorState = 0,
				.current = 0,
				.speed = 0,
				.current_A = 0,
				.current_B = 0,
				.current_C = 0,
				.powerControl = 0,
				.motorAngle = 0,
				.circleAngle = 0,
				.angle_add = 0,
			},
			.state_info = {
				.init_flag = 0,
				.offline_cnt = 0,
				.offline_cnt_max = 0,
				.selfprotect_cnt = 0,
				.selfprotect_cnt_max = 0,
				.selfprotect_flag = 0,
				.work_state = 0,
			},
		},
		.init = KT_motor_class_init,
	},
};

/* Exported functions --------------------------------------------------------*/
void rm_motor_list_init()
{
	RM_Group_F1.group_init(&RM_Group_F1);
}

void rm_motor_list_heart_beat()
{
	RM_Group_F1.group_heartbeat(&RM_Group_F1);
}
