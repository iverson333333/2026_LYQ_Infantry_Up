#include "chassis.h"
#include "rc_sensor.h"
#include "myrobot_def.h"
chassis_t chassis={
	.work=Chassis_Work,
};

void Chassis_Mec_Update(chassis_t *chassis, uint8_t ctrl_mode)
{
	 
		if (ctrl_mode == RC_CTRL_MODE) // 遥控器模式   //线性拟合遥控器数据得到正确的目标底盘X、Y、w速度，
	{
		
		chassis->base_info.target_front_speed = (float)rc_sensor.info->ch3 / RC_MAX_CNT * CHASSIS_MAX_SPEED ;
		chassis->base_info.target_right_speed = (float)rc_sensor.info->ch2 / RC_MAX_CNT * CHASSIS_MAX_SPEED ;
		chassis->base_info.target_cycle_speed = (float)rc_sensor.info->ch0 / RC_MAX_CNT * CHASSIS_MAX_SPEED ;
	}
	
		else if (ctrl_mode == KEY_CTRL_MODE) // 键盘模式
	{
		chassis->base_info.target_front_speed = 0;
		chassis->base_info.target_right_speed = 0;
		chassis->base_info.target_front_speed += (float)rc_sensor.info->W.cnt / (float)KEY_W_CNT_MAX * CHASSIS_MAX_SPEED;
		chassis->base_info.target_front_speed -= (float)rc_sensor.info->S.cnt / (float)KEY_S_CNT_MAX * CHASSIS_MAX_SPEED;
		chassis->base_info.target_right_speed += (float)rc_sensor.info->D.cnt / (float)KEY_D_CNT_MAX * CHASSIS_MAX_SPEED;
		chassis->base_info.target_right_speed -= (float)rc_sensor.info->A.cnt / (float)KEY_A_CNT_MAX * CHASSIS_MAX_SPEED;
		chassis->base_info.target_cycle_speed = rc_sensor.info->mouse_vx * 18.f;
		
		}
		if (my_abs(gimbal.base_info.yaw_motor_angle) > 16384) // 如果云台yaw轴电机超过16384就对XY目标速度取反
	{
		chassis->base_info.target_front_speed *= -1;
		chassis->base_info.target_right_speed *= -1;
	}
}

int16_t yaw_angle_err;
float k1_cycle=30;
float k2_cycle=1;
void Chassis_Gyro_Update(chassis_t *chassis, uint8_t ctrl_mode, uint8_t move_mode)
{
	Chassis_Mec_Update(chassis, ctrl_mode);
	
	float front = chassis->base_info.target_front_speed;
	float right = chassis->base_info.target_right_speed;
	if (my_abs(gimbal.base_info.yaw_motor_angle) > 16384) 
	{
		front *= -1;
		right *= -1;
	}
	
	yaw_angle_err = gimbal.base_info.yaw_motor_angle / 32768.f * 4096.f; // yaw轴   相对底盘   角度(-4096~4096)(顺时针为正)
	float yaw_angle_err_rad = (double)yaw_angle_err / 4096.f * 3.14159;	 // yaw轴角度转弧度制（-π~π）

	// front和right值计算
	chassis->base_info.target_front_speed = front * cos(yaw_angle_err_rad) - right * sin(yaw_angle_err_rad);
	chassis->base_info.target_right_speed = right * cos(yaw_angle_err_rad) + front * sin(yaw_angle_err_rad);
	
	
	if (move_mode == 0)
	{
		
		if (my_abs(yaw_angle_err) > 2048) // 头可以朝后
		{
			yaw_angle_err -= 4096 * sgn(yaw_angle_err);
		}
		
			chassis->base_info.target_cycle_speed = yaw_angle_err * yaw_angle_err*sgn(yaw_angle_err) /4096.f*30;
	}
	
}

void Chassis_Work(chassis_t *chassis)
{
	if(RC_ONLINE)
	{
		switch (car.car_move_mode)
		{
//			case init_CAR:
//				lost_protect();
//			break;
			case mec_CAR:
				Chassis_Mec_Update(chassis,car.car_ctrl_mode);
				
			break;
			case gyro_CAR:
				Chassis_Gyro_Update(chassis, car.car_ctrl_mode,0); // 陀螺仪模式
				
						chassis->pid_mode = CHASSIS_SPEED_PID;
					
			break;
			case cycle_CAR:
				Chassis_Gyro_Update(chassis, car.car_ctrl_mode,0); // 陀螺仪模式
			 
						chassis->pid_mode = CHASSIS_SPEED_PID;
					
					chassis->base_info.target_cycle_speed = 500;//CYCLE_SPEED; // 小陀螺速度

				break;
			case lob_CAR:
//				if(my_abs((float)rc_sensor.info->ch0)>=20)
//				{
//		      chassis->base_info.target_cycle_speed = (float)rc_sensor.info->ch0 / RC_MAX_CNT *500 ;
//					chassis->pid_mode = CHASSIS_SPEED_PID;
//				}
//				else
//				{
					chassis->pid_mode = CHASSIS_POSITION_PID;
//				}
			break;
	

			default:
			break;
		}
	}
		else
		{
			chassis->base_info.target_front_speed = 0;
			chassis->base_info.target_right_speed = 0;
			chassis->base_info.target_cycle_speed = 0;
		}
	communicate.chassis_data_tx_info->target_front_speed = chassis->base_info.target_front_speed;
	communicate.chassis_data_tx_info->target_right_speed = chassis->base_info.target_right_speed;
	communicate.chassis_data_tx_info->target_cycle_speed = chassis->base_info.target_cycle_speed;
	communicate.chassis_data_tx_info->pid_mode = chassis->pid_mode;
	communicate.chassis_data_tx_info->max_speed = CHASSIS_MAX_SPEED;

}
