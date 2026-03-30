#include "shoot.h"

#define FRIC_STOP_ENCODER_THRESHOLD 100.0f
#define FRIC_TARGET_EPS 1.0f
#define FRIC_STOP_CONFIRM_TICK 3

typedef enum
{
	FRIC_STATE_RUN = 0,
	FRIC_STATE_SPINDOWN,
	FRIC_STATE_RELEASE,
} fric_ctrl_state_e;

static fric_ctrl_state_e fric_state = FRIC_STATE_RELEASE;
static uint8_t fric_near_stop_cnt = 0;
static uint8_t fric_pid_inited = 0;

static void Shoot_Fric_PidInit(shoot_t *shoot)
{
	if (fric_pid_inited == 1)
	{
		return;
	}

	shoot->fric_b_l->ctrl->speed_ctrl->kp = 22.0f;
	shoot->fric_b_l->ctrl->speed_ctrl->ki = 0.35f;
	shoot->fric_b_l->ctrl->speed_ctrl->kd = 0.0f;
	shoot->fric_b_l->ctrl->speed_ctrl->integral_max = 7000.0f;
	shoot->fric_b_l->ctrl->speed_ctrl->out_max = 12000.0f;

	shoot->fric_b_r->ctrl->speed_ctrl->kp = 22.0f;
	shoot->fric_b_r->ctrl->speed_ctrl->ki = 0.35f;
	shoot->fric_b_r->ctrl->speed_ctrl->kd = 0.0f;
	shoot->fric_b_r->ctrl->speed_ctrl->integral_max = 7000.0f;
	shoot->fric_b_r->ctrl->speed_ctrl->out_max = 12000.0f;

#if SHOOT_CTRL_WITH_UP_FRIC
	shoot->fric_b_up->ctrl->speed_ctrl->kp = 22.0f;
	shoot->fric_b_up->ctrl->speed_ctrl->ki = 0.35f;
	shoot->fric_b_up->ctrl->speed_ctrl->kd = 0.0f;
	shoot->fric_b_up->ctrl->speed_ctrl->integral_max = 7000.0f;
	shoot->fric_b_up->ctrl->speed_ctrl->out_max = 12000.0f;
#endif

	fric_pid_inited = 1;
}

static uint8_t Shoot_Fric_NearStop(shoot_t *shoot)
{
	if (my_abs(shoot->fric_b_l->rx_info->encoder_speed) > FRIC_STOP_ENCODER_THRESHOLD)
	{
		return 0;
	}

	if (my_abs(shoot->fric_b_r->rx_info->encoder_speed) > FRIC_STOP_ENCODER_THRESHOLD)
	{
		return 0;
	}

#if SHOOT_CTRL_WITH_UP_FRIC
	if (my_abs(shoot->fric_b_up->rx_info->encoder_speed) > FRIC_STOP_ENCODER_THRESHOLD)
	{
		return 0;
	}
#endif

	return 1;
}

static void Shoot_Fric_SpeedPid(Motor_RM_t *motor, float target_speed)
{
	motor->ctrl->speed_ctrl->target = target_speed;
	motor->ctrl->speed_ctrl->measure = motor->rx_info->encoder_speed;
	motor->ctrl->speed_ctrl->err = motor->ctrl->speed_ctrl->target - motor->ctrl->speed_ctrl->measure;
	single_pid_ctrl(motor->ctrl->speed_ctrl);
	motor->tx_info->torque = (int16_t)motor->ctrl->speed_ctrl->out;
}

static void Shoot_Fric_ReleaseAll(shoot_t *shoot)
{
	shoot->fric_b_l->tx_info->torque = 0;
	shoot->fric_b_r->tx_info->torque = 0;
#if SHOOT_CTRL_WITH_UP_FRIC
	shoot->fric_b_up->tx_info->torque = 0;
#else
	shoot->fric_b_up->tx_info->torque = 0;
#endif
}

static void Shoot_Fric_UpdateState(shoot_t *shoot)
{
	uint8_t rc_online = Board_Rx_Info.flag.bit.is_rc_online;
	uint8_t has_target = (my_abs(Board_Rx_Info.fric_target_speed) > FRIC_TARGET_EPS);
	uint8_t near_stop = Shoot_Fric_NearStop(shoot);

	if (near_stop == 1)
	{
		if (fric_near_stop_cnt < 255)
		{
			fric_near_stop_cnt++;
		}
	}
	else
	{
		fric_near_stop_cnt = 0;
	}

	if (rc_online == 0)
	{
		if (fric_near_stop_cnt >= FRIC_STOP_CONFIRM_TICK)
		{
			fric_state = FRIC_STATE_RELEASE;
		}
		else
		{
			fric_state = FRIC_STATE_SPINDOWN;
		}
		return;
	}

	if (has_target == 0)
	{
		if (fric_near_stop_cnt >= FRIC_STOP_CONFIRM_TICK)
		{
			fric_state = FRIC_STATE_RELEASE;
		}
		else
		{
			fric_state = FRIC_STATE_SPINDOWN;
		}
		return;
	}

	if (fric_state == FRIC_STATE_RELEASE)
	{
		fric_state = FRIC_STATE_RUN;
	}
	else
	{
		fric_state = FRIC_STATE_RUN;
	}
}
shoot_t shoot =
	{
		.fric_b_l = &rm_motor[L_Fric],
		.fric_b_r = &rm_motor[R_Fric],
		.fric_b_up = &rm_motor[UP_Fric],

		.work = Shoot_Work,

		.target = 0,

};

/*发射pid计算*/
void Shoot_pid_cal(shoot_t *shoot)
{
	Shoot_Fric_UpdateState(shoot);

	if (fric_state == FRIC_STATE_RELEASE)
	{
		Shoot_Fric_ReleaseAll(shoot);
		return;
	}

	if (fric_state == FRIC_STATE_SPINDOWN)
	{
		Shoot_Fric_SpeedPid(shoot->fric_b_l, 0.0f);
		Shoot_Fric_SpeedPid(shoot->fric_b_r, 0.0f);
#if SHOOT_CTRL_WITH_UP_FRIC
		Shoot_Fric_SpeedPid(shoot->fric_b_up, 0.0f);
#else
		shoot->fric_b_up->tx_info->torque = 0;
#endif
		return;
	}

	Shoot_Fric_SpeedPid(shoot->fric_b_l, (float)shoot->base_info.fric_info.target_fric_B_L_speed);
	Shoot_Fric_SpeedPid(shoot->fric_b_r, (float)shoot->base_info.fric_info.target_fric_B_R_speed);
#if SHOOT_CTRL_WITH_UP_FRIC
	Shoot_Fric_SpeedPid(shoot->fric_b_up, (float)shoot->base_info.fric_info.target_fric_B_UP_speed);
#else
	shoot->fric_b_up->tx_info->torque = 0;
#endif
}

/*外部获取*/
void Shoot_extern_get(shoot_t *shoot)
{
	float target_speed_cmd = Board_Rx_Info.fric_target_speed;

	shoot->is_on_fric = (my_abs(target_speed_cmd) > FRIC_TARGET_EPS);
	if (shoot->is_on_fric == 1)
	{
		shoot->base_info.fric_info.target_fric_B_L_speed = (int16_t)target_speed_cmd;
		shoot->base_info.fric_info.target_fric_B_R_speed = (int16_t)(-target_speed_cmd);
#if SHOOT_CTRL_WITH_UP_FRIC
		shoot->base_info.fric_info.target_fric_B_UP_speed = (int16_t)(-target_speed_cmd);
#else
		shoot->base_info.fric_info.target_fric_B_UP_speed = 0;
#endif
	}
	else
	{
		shoot->base_info.fric_info.target_fric_B_L_speed = 0;
		shoot->base_info.fric_info.target_fric_B_R_speed = 0;
		shoot->base_info.fric_info.target_fric_B_UP_speed = 0;
	}
}

/*发射总控*/
void Shoot_Work(shoot_t *shoot)
{
	Shoot_Fric_PidInit(shoot);
	Shoot_extern_get(shoot);
	Shoot_pid_cal(shoot);
}