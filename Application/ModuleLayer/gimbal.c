#include "gimbal.h"
#include "device.h"
#include "rp_math.h"
#include "car.h"
#include "RM_motor.h"

gimbal_offset_info_t offset_info =
{
	.vision_pitch_offset = 0,//吊射偏置
	.lob_pitch_gyro_offset = 0,
};

gimbal_t gimbal = 
{
	.gimbal_p=&rm_motor[gim_pitch],
//	.gimbal_y=&kt_motor[0],
//	.lob_info.pre_aim_yaw_angle = 0,
//	.lob_info.pre_aim_pitch_angle = 35,
	.offset_info=&offset_info,
	.gravity_offset_info.const_k=0,
	.gravity_offset_info.center_of_gravity_angle=43,
	.all_pid_calc=all_pid_calc,
	.work = Gimbal_Work,
	.gimbal_reset_state = DEV_RESET_NO,
	.gimbal_ctrl_mode = 1,//陀螺仪
	.base_info.init_time=0,
	.base_info.init_time_max=1000,	
	.base_info.init_time_max_count=0,	
};

/*云台pitch轴陀螺仪角度限位*/
void Gimbal_Pitch_Gyro_Angle_Limit(gimbal_t *gimbal)
{
	float angle = gimbal->base_info.pitch_imu_angle_target;
	if(angle > GIMBAL_MAX_GYRO_ANGEL)
	{
		angle = GIMBAL_MAX_GYRO_ANGEL;
	}
	if(angle < GIMBAL_MIN_GYRO_ANGEL)
	{
		angle = GIMBAL_MIN_GYRO_ANGEL;
	}
	gimbal->base_info.pitch_imu_angle_target = angle;
}

/*云台pitch轴机械角度限位*/
void Gimbal_Pitch_Mec_Angle_Limit(gimbal_t *gimbal)
{
	float angle = gimbal->base_info.pitch_mec_angle_target;
	if(angle > GIMBAL_MAX_MEC_ANGEL)
	{
		angle = GIMBAL_MAX_MEC_ANGEL;
	}
	if(angle < GIMBAL_MIN_MEC_ANGEL)
	{
		angle = GIMBAL_MIN_MEC_ANGEL;
	}
	gimbal->base_info.pitch_mec_angle_target = angle;
}


/*云台pitch重力补偿*/
void Gimbal_Pitch_Gravity_Offset(gimbal_t *gimbal)
{
	float pitch_angle=gimbal->base_info.pitch_imu_angle;
	float center_of_gravity_angle=gimbal->gravity_offset_info.center_of_gravity_angle;
	gimbal->gravity_offset_info.torque_angle=90.f-center_of_gravity_angle-pitch_angle;
	float output=gimbal->gravity_offset_info.const_k*sin(gimbal->gravity_offset_info.torque_angle);
	gimbal->gravity_offset_info.gravity_offset_output=output;
}

/*云台信息更新*/
void Gimbal_External_Date_Update(gimbal_t *gimbal)
{
	gimbal->base_info.pitch_imu_angle = -imu_sensor.info->base_info.roll+sgn(imu_sensor.info->base_info.roll)*180;
	gimbal->base_info.pitch_imu_speed = -imu_sensor.info->base_info.ave_rate_roll;//后续增加一个service层posture
	
	gimbal->gimbal_reset_state = Board_Rx_Info.gimbal_state;
	gimbal->gimbal_ctrl_mode.gimbal_mode = Board_Rx_Info.gimbal_mode;//优化 找个地方更新自瞄不自瞄？
	gimbal->base_info.pitch_mec_angle_target = Board_Rx_Info.pitch_mec_tar;
	gimbal->base_info.pitch_imu_angle_target = Board_Rx_Info.pitch_imu_tar;
	/*pitch轴电机角度更新*/
	gimbal->base_info.pitch_motor_angle =  (float)gimbal->gimbal_p->rx_info->encoder- PITCH_MOTOR_ENCODER_MIDDLE;
	gimbal->base_info.pitch_motor_angle = motor_half_cycle(gimbal->base_info.pitch_motor_angle, 8192.f);
	gimbal->base_info.pitch_motor_speed = (float)gimbal->gimbal_p->rx_info->speed;

	/*360度标准化角度*/
	gimbal->base_info.pitch_mec_360_angle=gimbal->base_info.pitch_motor_angle/8192.f*360.f;	
}

/*云台自救模式*/
void Gimbal_Save_Update(gimbal_t *gimbal,uint8_t ctrl_mode)
{
	gimbal->base_info.pitch_mec_angle_target = 0;
}

/*云台吊射模式*/
void Gimbal_Lob_Update(gimbal_t *gimbal,uint8_t ctrl_mode)
{
//	//标志位清零
//	gimbal->lob_info.lob_init_angle_flag=0;
//	
//	if(car.car_ctrl_mode==RC_CTRL_MODE)
//	{
//		if(my_abs((float)rc_sensor.info->ch1)>=10)
//		{
//			gimbal->base_info.pitch_imu_angle_target+=rc_sensor.info->ch1*0.001f*0.01;
//		}	
//		if(my_abs((float)rc_sensor.info->ch0)>=10)
//		{
//			gimbal->base_info.yaw_mec_angle_target+=rc_sensor.info->ch0*0.001f*0.2;
//		}	
//	}
//	else
//	{
//		gimbal->base_info.pitch_imu_angle_target+=rc_sensor.info->mouse_y*0.00005f;
//////		gimbal->base_info.yaw_mec_angle_target+=rc_sensor.info->mouse_x*0.0005f;
//	}
//////	  gimbal->base_info.yaw_imu_angle_target=gimbal->base_info.yaw_imu_angle;
	
	if(Board_Rx_Info.vision_mode != 0 && vision.status->rx_state == DEV_ONLINE)
	{
	  gimbal->base_info.pitch_mec_angle_target = vision.VtoE->pitch / 180.f * 4096.f;//Board_Rx_Info.pitch_mec_tar;
	}
	else
	{
	  gimbal->base_info.pitch_mec_angle_target = Board_Rx_Info.pitch_mec_tar;
		gimbal->base_info.pitch_imu_angle_target = gimbal->base_info.pitch_imu_angle;
	}
}

/*云台初始化*/
void Gimbal_init_action(gimbal_t *gimbal)
{
	gimbal->base_info.init_time_max_count++;
	gimbal->lob_info.lob_init_angle_flag=0;
	//设置初始化目标值
	if(Board_Rx_Info.is_rc_online == 0)
	{
		gimbal->base_info.pitch_mec_angle_target = Board_Rx_Info.pitch_mec_tar;
		gimbal->base_info.pitch_imu_angle_target = Board_Rx_Info.pitch_imu_tar;
	}
//	else
//	{
//		gimbal->base_info.pitch_mec_angle_target = gimbal
//		gimbal->base_info.pitch_imu_angle_target = Board_Rx_Info.pitch_imu_tar;
//	}
}

/*云台陀螺仪模式*/
void Gimbal_Gyro_Update(gimbal_t *gimbal,uint8_t ctrl_mode)
{	
	if(Board_Rx_Info.video_open == 1 && vision.status->rx_state == DEV_ONLINE && vision.status->tx_state == DEV_ONLINE
		 && vision.VtoE->flag_union.bit.is_find_target == 1)
	{
		gimbal->base_info.pitch_imu_angle_target = vision.VtoE->pitch;
	}
	else
	{
    gimbal->base_info.pitch_imu_angle_target = Board_Rx_Info.pitch_imu_tar;
	}
	
	gimbal->base_info.pitch_mec_angle_target = gimbal->base_info.pitch_motor_angle;
}

///*自瞄*/
//void Gimbal_Vision_Update(gimbal_t *gimbal,uint8_t ctrl_mode)
//{
//	gimbal->base_info.pitch_imu_angle_target += gimbal->offset_info->vision_pitch_offset;
//}

/*云台pitch轴PID计算*///////////////
void Gimbal_Pitch_Pid_Cal(gimbal_t *gimbal)
{
	float gyro_meas_in,gyro_meas_out,gyro_target,mec_meas_in,mec_meas_out,mec_target,shoot_offset_current;

	switch (gimbal->ptich_pid_mode)
	{
	case GYRO_PID:
		gyro_meas_out = gimbal->base_info.pitch_imu_angle;				//外环
		gyro_meas_in = gimbal->base_info.pitch_imu_speed;			  //内环
		gyro_target = gimbal->base_info.pitch_imu_angle_target;  //目标值
		
//		shoot_offset_current=shoot.shooting_shake_angle.shoot_pitch_offset_current;//发射抖动补偿电流

		gimbal->base_info.output_gimbal_p = feedforward_pid_calc(gimbal->gimbal_p->ctrl->angle_ctrl_outer_gyro,gimbal->gimbal_p->ctrl->angle_ctrl_inner_gyro,gyro_target,gyro_meas_out,gyro_meas_in,-1,0);
		break;

	case MEC_PID:
		mec_meas_out = gimbal->base_info.pitch_motor_angle;		        //外环
		mec_meas_in = gimbal->base_info.pitch_imu_speed;			      //内环  
		mec_target = gimbal->base_info.pitch_mec_angle_target;				//目标值 	
	
//		shoot_offset_current=shoot.shooting_shake_angle.shoot_pitch_offset_current;//发射抖动补偿电流		

		gimbal->base_info.output_gimbal_p = feedforward_pid_calc(gimbal->gimbal_p->ctrl->angle_ctrl_outer,gimbal->gimbal_p->ctrl->angle_ctrl_inner,mec_target,mec_meas_out,mec_meas_in,-1,0);
		break;

	case SPEED_PID:
		gimbal->base_info.output_gimbal_p = gimbal->all_pid_calc( NULL,gimbal->gimbal_p->ctrl->speed_ctrl,0,NULL,gimbal->base_info.pitch_imu_speed,-1,0);
		break;

	default:
		break;
	}
}

///*云台发板间信息更新*/
//void Gimbal_Board_Update(gimbal_t *gimbal)
//{
//	Board_Tx_Info.pitch_imu = gimbal->base_info.pitch_imu_angle;
//	Board_Tx_Info.pitch_mec = gimbal->base_info.pitch_motor_angle;
//	Board_Tx_Info.pitch_v = gimbal->base_info.pitch_imu_speed;
//	Board_Tx_Info.yaw_imu = -imu_sensor.info->base_info.yaw;
//  Board_Tx_Info.yaw_v = imu_sensor.info->base_info.rate_yaw;
//}

/*云台总控*/
void Gimbal_Work(gimbal_t *gimbal)
{
	Gimbal_External_Date_Update(gimbal);
#ifndef TEST
	switch(gimbal->gimbal_reset_state)
	{
		case DEV_RESET_NO:
			Gimbal_init_action(gimbal);
		  gimbal->ptich_pid_mode = MEC_PID;
		break;
		case DEV_RESET_OK:
			switch(gimbal->gimbal_ctrl_mode.gimbal_mode)
			{	
				case 1:
			    Gimbal_Gyro_Update(gimbal,car.car_ctrl_mode);
		      gimbal->ptich_pid_mode = GYRO_PID;
				break;
				case 2:
					Gimbal_Lob_Update(gimbal,car.car_ctrl_mode);
				  gimbal->ptich_pid_mode = MEC_PID;
				break;
				case 3:
					Gimbal_Save_Update(gimbal,car.car_ctrl_mode);
				  gimbal->ptich_pid_mode = MEC_PID;
				break;
				default:
				break;
			}
		break;		
		default:
		break;
	}
  Gimbal_Pitch_Gyro_Angle_Limit(gimbal);
	Gimbal_Pitch_Mec_Angle_Limit(gimbal);

	if(Board_Rx_Info.is_rc_online == 0)//开控
	{
		Gimbal_Pitch_Gravity_Offset(gimbal);
		Gimbal_Pitch_Pid_Cal(gimbal);
		gimbal->gimbal_p->tx_info->torque=gimbal->base_info.output_gimbal_p;			
	}
	else//关控
	{
		gimbal->gimbal_reset_state = DEV_RESET_NO;
		gimbal->gimbal_p->tx_info->torque=0;
	}
	#else
	/*调试*/
	gimbal->ptich_pid_mode = GYRO_PID;
	Gimbal_Pitch_Pid_Cal(gimbal);
	gimbal->gimbal_p->tx_info->torque=gimbal->base_info.output_gimbal_p;
	gimbal->gimbal_p->tx_info->torque=0;
	#endif
}

