#include "car.h"

//void Car_TxData_Update(void)
//{
//	/*car_data0_tx_info pack update*****************************************************/
//	car_data0_tx_info_t *car_data0_tx_info = communicate.car_data0_tx_info;
//	//压缩chassis_angel取值范围
//	float chassis_angel = (YAW_MOTOR_ANGLE_MIDDLE - gimbal.gimbal_y->KT_motor_info.rx_info.encoder) / 32768.f * 180 / 2;
//	if (chassis_angel < 0)
//	{
//		chassis_angel += 180;
//	}
//	car_data0_tx_info->pitch_angel = gimbal.base_info.pitch_imu_angle * 100;
////	car_data0_tx_info->car_move_mode = car.car_move_mode;

//	if (car.car_ctrl_mode == KEY_CTRL_MODE)//键盘模式
//	{
//		if (rc_sensor.info->V.status== release_to_press)
//		{
//			car_data0_tx_info->car_state.bit.is_on_cap = 1;
//		}
//		else
//		{
//			car_data0_tx_info-> car_state.bit.is_on_cap = 0;
//		}
//	}
//	else//遥控器模式
//	{

//		
//		if(my_abs(chassis.base_info.target_front_speed)>=CHASSIS_MAX_SPEED*0.8)
//		{
//			car_data0_tx_info->car_state.bit.is_on_cap = 1;
//		}
//		else
//		{
//			car_data0_tx_info->car_state.bit.is_on_cap = 0;
//		}


//	}
//	/*car_data1_tx_info pack update******************************/
//	car_data1_tx_info_t *car_data1_tx_info = communicate.car_data1_tx_info;
//	car_data1_tx_info->chassis_angel = chassis_angel;
//	car_data1_tx_info->fric_b_speed = shooting.config->target_B_friction_speed;
//	car_data1_tx_info->fric_f_speed = shooting.config->target_F_friction_speed;
//	car_data1_tx_info->pitch_motor_angle = gimbal.base_info.pitch_mec_360_angle;
//	/*car_data2_tx_info pack update******************************/
//	car_data2_tx_info_t *car_data2_tx_info = communicate.car_data2_tx_info;
//	car_data2_tx_info->detect_num = vision.rx_info->detect_num;
//	car_data2_tx_info->uix_right = vision.rx_info->uix_right / 10;
//	car_data2_tx_info->uiy_right = vision.rx_info->uiy_right / 5;
//	car_data2_tx_info->ui_x = vision.rx_info->UI_x;
//	car_data2_tx_info->ui_y = vision.rx_info->UI_y;
//	/*car_data3 */
//	car_data3_tx_info_t *car_data3_tx_info = communicate.car_data3_tx_info;
//	car_data3_tx_info->uix_lb = vision.rx_info->uix_lb / 10;
//	car_data3_tx_info->uix_rb = vision.rx_info->uix_rb / 10;
//	car_data3_tx_info->uiy_lb = vision.rx_info->uiy_lb / 5;
//	car_data3_tx_info->uiy_rb = vision.rx_info->uiy_rb / 5;

//	car_data3_tx_info->uix_lt = vision.rx_info->uix_lt / 10;
//	car_data3_tx_info->uix_rt = vision.rx_info->uix_rt / 10;
//	car_data3_tx_info->uiy_lt = vision.rx_info->uiy_lt / 5;
//	/*car_data4*/
//	car_data4_tx_info_t *car_data4_tx_info = communicate.car_data4_tx_info;
//	car_data4_tx_info->uiy_rt = vision.rx_info->uiy_rt / 5;
//	car_data4_tx_info->vision_robot_distance = vision.rx_info->distance;
//	

//	car_data4_tx_info->uix_left = vision.rx_info->uix_left / 10;
//	car_data4_tx_info->uiy_left = vision.rx_info->uiy_left / 5;
//}


car_t car;

void Car_Ctrl_Mode_Update(car_t *car)
{
	//右拨杆上：键盘模式  其余都是遥控器模式
	switch(rc_sensor.info->s1.value)
	{
	case 2:	//左拨杆下
		
     car->car_ctrl_mode = KEY_CTRL_MODE;
		break;
	default://左拨杆不是下
		car->car_ctrl_mode = RC_CTRL_MODE;
		break;
	}
}

void Key_Move_Mode_Update(car_t *car)
{
	
	switch (car->car_move_mode)
	{
	case mec_CAR:
		//F键按下，进入小陀螺模式
		if(rc_sensor.info->F.status== release_to_press)
		{
			car->car_move_mode = cycle_CAR;
		}
		//shift按下，进入陀螺仪模式
		if (rc_sensor.info->Shift.status == release_to_press)
		{
			car->car_move_mode = gyro_CAR;
		}
		//R键按下，进入发射模式，多按切换
		if(rc_sensor.info->R.status== release_to_press)
		{
			if(shoot.shoot_status==off_fire)
			{
				shoot.shoot_status=ready_fire;
			}
			else
			{
				shoot.shoot_status=off_fire;
			}
		}
		//发射模式下，单击左单发，长按连发
		if(shoot.shoot_status!=off_fire)
		{
			if(rc_sensor.info->Z.status== release_to_press)//拨盘复位
			{
				shoot.base_info.dail_info.dail_reset_state=DEV_RESET_NO;
			}
			if(rc_sensor.info->mouse_btn_l.status== release_to_press)
			{
				shoot.record_status=record_short;
			}
			if(rc_sensor.info->mouse_btn_l.status== long_press)
			{
				shoot.record_status=record_long;
			}
			if(rc_sensor.info->mouse_btn_l.status== release || rc_sensor.info->mouse_btn_l.status== press_to_release)
			{
				shoot.record_status=record_zero;
			}
			if(shoot.base_info.dail_info.dail_reset_state==DEV_RESET_OK)
			{
				switch(shoot.record_status)
				{
					case record_long:
						shoot.shoot_status=running_fire;
					break;
					case record_short:
						shoot.shoot_status=single_fire;
					break;
					case record_zero:
						shoot.shoot_status=ready_fire;
					break;
					default:
					break;
			 	}
			 }
		}
//		}
		break;
		break;
	case gyro_CAR:
		//C键按下，进入机械模式
		if(rc_sensor.info->C.status== release_to_press)
		{
			car->car_move_mode = mec_CAR;
		}
		//F键按下，进入小陀螺模式
		if(rc_sensor.info->F.status== release_to_press)
		{
			car->car_move_mode = cycle_CAR;
		}
		//R键按下，进入发射模式，多按切换
		if(rc_sensor.info->R.status== release_to_press)
		{
			if(shoot.shoot_status==off_fire)
			{
				shoot.shoot_status=ready_fire;
			}
			else
			{
				shoot.shoot_status=off_fire;
			}
		}
		//发射模式下，单击左单发，长按连发
		if(shoot.shoot_status!=off_fire)
		{
			if(rc_sensor.info->Z.status== release_to_press)//拨盘复位
			{
				shoot.base_info.dail_info.dail_reset_state=DEV_RESET_NO;
			}
			if(rc_sensor.info->mouse_btn_l.status== release_to_press)
			{
				shoot.record_status=record_short;
			}
			if(rc_sensor.info->mouse_btn_l.status== long_press)
			{
				shoot.record_status=record_long;
			}
			if(rc_sensor.info->mouse_btn_l.status== release || rc_sensor.info->mouse_btn_l.status== press_to_release)
			{
				shoot.record_status=record_zero;
			}
			if(shoot.base_info.dail_info.dail_reset_state==DEV_RESET_OK)
			{
				switch(shoot.record_status)
				{
					case record_long:
						shoot.shoot_status=running_fire;
					break;
					case record_short:
						shoot.shoot_status=single_fire;
					break;
					case record_zero:
						shoot.shoot_status=ready_fire;
					break;
					default:
					break;
			 	}
			 }
		}
//		}
		break;
	case cycle_CAR:
		//C键按下，进入机械模式
		if(rc_sensor.info->C.status== release_to_press)
		{
			car->car_move_mode = mec_CAR;
		}
		//F键按下，进入陀螺仪模式
		if(rc_sensor.info->F.status== release_to_press||rc_sensor.info->Shift.status== release_to_press)
		{
			car->car_move_mode = gyro_CAR;
		}
		//R键按下，进入发射模式，多按切换
		if(rc_sensor.info->R.status== release_to_press)
		{
			if(shoot.shoot_status==off_fire)
			{
				shoot.shoot_status=ready_fire;
			}
			else
			{
				shoot.shoot_status=off_fire;
			}
		}
		//发射模式下，单击左单发，长按连发
		if(shoot.shoot_status!=off_fire)
		{
			if(rc_sensor.info->Z.status== release_to_press)//拨盘复位
			{
				shoot.base_info.dail_info.dail_reset_state=DEV_RESET_NO;
			}
			if(rc_sensor.info->mouse_btn_l.status== release_to_press)
			{
				shoot.record_status=record_short;
			}
			if(rc_sensor.info->mouse_btn_l.status== long_press)
			{
				shoot.record_status=record_long;
			}
			if(rc_sensor.info->mouse_btn_l.status== release || rc_sensor.info->mouse_btn_l.status== press_to_release)
			{
				shoot.record_status=record_zero;
			}
			
			if(shoot.base_info.dail_info.dail_reset_state==DEV_RESET_OK)
			{
				switch(shoot.record_status)
				{
					case record_long:
						shoot.shoot_status=running_fire;
					break;
					case record_short:
						shoot.shoot_status=single_fire;
					break;
					case record_zero:
						shoot.shoot_status=ready_fire;
					break;
					default:
					break;
			 	}
			 }
					}
//		}
		break;
	default:
		break;
	}
}


void RC_Move_Mode_Update(car_t *car)
{
	static uint8_t thumbweheel_step,thumbweheel_last_step;
	thumbweheel_last_step = thumbweheel_step;
	thumbweheel_step = rc_sensor.info->thumbwheel.step[RC_TB_UP];
		
	static uint8_t thumbwheel_step,thumbwheel_last_step;
	thumbwheel_last_step = thumbwheel_step;
	thumbwheel_step = rc_sensor.info->thumbwheel.step[RC_TB_MU];
	//////////////加上判断拨杆跳变之后再进入检测拨杆，要是没变就一直陀螺仪模式///////////////
	if(rc_sensor.info->s2.status != 0)//keep_up
	{
		rc_sensor.s2_change_flag=1;
	}
	/////////////////进入拨杆判断////////////////////////////*/*/
	if(rc_sensor.s2_change_flag==1)
	{

		switch(rc_sensor.info->s2.value)
		{
		case 0x01:  //右拨杆up
			if(thumbweheel_step!=thumbweheel_last_step)////////////机械模式控制
			{
				if(car->car_move_mode == mec_CAR)
				{
					car->car_move_mode = cycle_CAR;
				}
				else if(car->car_move_mode == cycle_CAR)
				{
					car->car_move_mode = mec_CAR;
				}
			}
			
				if(	car->car_move_mode != mec_CAR &&
				car->car_move_mode != cycle_CAR)
				{
					car->car_move_mode = mec_CAR;
				}
				
				if(car->car_move_mode==mec_CAR||
					car->car_move_mode == cycle_CAR)
			{
	//			if(rc_sensor.info->s1.value==rc_sensor.info->s1.value_last)
	//			{}
	//			else
	//			{
					switch(rc_sensor.info->s1.value)////////////判断射击模式
					{
						case 0x01:
							shoot.shoot_status=ready_fire;
						//在这里加波轮控制初始化no，static那里加判断no，ok
							if(thumbwheel_step!=thumbwheel_last_step)
								{
									shoot.base_info.dail_info.dail_reset_state=DEV_RESET_NO;
								}
						//
						break;
						default:
							shoot.shoot_status=off_fire;
						break;
					}
					if(shoot.base_info.dail_info.dail_reset_state==DEV_RESET_OK)
					{
						static int16_t thumbwheel_step,thumbwheel_last_step;
			//				thumbweheel_last_step = thumbweheel_step;
							thumbwheel_step = rc_sensor.info->thumbwheel.value;
							if(thumbwheel_step>=550)
							{
								rc_sensor.info->thumbwheel.wheel_count++;
								if(rc_sensor.info->thumbwheel.wheel_count>=1000&&rc_sensor.info->thumbwheel.wheel_count<=2000)
								{
									
									shoot.record_status=record_long;
								
								}
								else if(rc_sensor.info->thumbwheel.wheel_count>=50&&rc_sensor.info->thumbwheel.wheel_count<=1000)
								{
									shoot.record_status=record_short;
								}
								else
								{
									shoot.record_status=record_zero;
								}
							}
							else
							{
								rc_sensor.info->thumbwheel.wheel_count=0;
								if(shoot.shoot_status!=off_fire)
								{
									switch(shoot.record_status)
									{
										case record_long:
											shoot.shoot_status=running_fire;
										break;
										case record_short:
											shoot.shoot_status=single_fire;
										break;
										case record_zero:
											shoot.shoot_status=ready_fire;
										break;
										default:
										break;
									}
								}
							}
						}
				}

			break;
				case 0x02:  //右拨杆down
//			if(thumbweheel_step!=thumbweheel_last_step)////////////机械模式控制
//			{
//				if(car->car_move_mode == lob_CAR)
//				{
//					car->car_move_mode = cycle_CAR;
//				}
//				else if(car->car_move_mode == cycle_CAR)
//				{
//					car->car_move_mode = lob_CAR;
//				}
//			}
			
				if(	car->car_move_mode != lob_CAR)
				{
					car->car_move_mode = lob_CAR;
				}
				
				if(car->car_move_mode==lob_CAR)
			{
	//			if(rc_sensor.info->s1.value==rc_sensor.info->s1.value_last)
	//			{}
	//			else
	//			{
					switch(rc_sensor.info->s1.value)////////////判断射击模式
					{
						case 0x01:
							shoot.shoot_status=ready_fire;
						//在这里加波轮控制初始化no，static那里加判断no，ok
							if(thumbwheel_step!=thumbwheel_last_step)
								{
									shoot.base_info.dail_info.dail_reset_state=DEV_RESET_NO;
								}
						//
						break;
						default:
							shoot.shoot_status=off_fire;
						break;
					}
					if(shoot.base_info.dail_info.dail_reset_state==DEV_RESET_OK)
					{
						static int16_t thumbwheel_step,thumbwheel_last_step;
			//				thumbweheel_last_step = thumbweheel_step;
							thumbwheel_step = rc_sensor.info->thumbwheel.value;
							if(thumbwheel_step>=550)
							{
								rc_sensor.info->thumbwheel.wheel_count++;
								if(rc_sensor.info->thumbwheel.wheel_count>=1000&&rc_sensor.info->thumbwheel.wheel_count<=2000)
								{
									
									shoot.record_status=record_long;
								
								}
								else if(rc_sensor.info->thumbwheel.wheel_count>=50&&rc_sensor.info->thumbwheel.wheel_count<=1000)
								{
									shoot.record_status=record_short;
								}
								else
								{
									shoot.record_status=record_zero;
								}
							}
							else
							{
								rc_sensor.info->thumbwheel.wheel_count=0;
								if(shoot.shoot_status!=off_fire)
								{
									switch(shoot.record_status)
									{
										case record_long:
											shoot.shoot_status=running_fire;
										break;
										case record_short:
											shoot.shoot_status=single_fire;
										break;
										case record_zero:
											shoot.shoot_status=ready_fire;
										break;
										default:
										break;
									}
								}
							}
						}
				}

			break;

		case 0x03:	//右拨杆mid
			//拨轮向上切换陀螺仪和小陀螺
			if(thumbweheel_step!=thumbweheel_last_step)////////////陀螺仪模式控制
			{
				if(car->car_move_mode == gyro_CAR)
				{
					car->car_move_mode = cycle_CAR;
				}
				else if(car->car_move_mode == cycle_CAR)
				{
					car->car_move_mode = gyro_CAR;
				}
			}
			
			//拨杆中间时，切换到陀螺仪模式  
			if (car->car_move_mode != gyro_CAR &&
				car->car_move_mode != cycle_CAR)
			{
				car->car_move_mode = gyro_CAR;
			}
			
			if(car->car_move_mode==gyro_CAR ||
				car->car_move_mode == cycle_CAR)
			{
	//			if(rc_sensor.info->s1.value==rc_sensor.info->s1.value_last)
	//			{}
	//			else
	//			{
					switch(rc_sensor.info->s1.value)
					{
						case 0x01:
							shoot.shoot_status=ready_fire;
							//在这里加波轮控制初始化no，static那里加判断no，ok
							if(thumbwheel_step!=thumbwheel_last_step)
								{
									shoot.base_info.dail_info.dail_reset_state=DEV_RESET_NO;
								}
						//
						break;
						default:
							shoot.shoot_status=off_fire;
						break;
					}
					static int16_t thumbwheel_step,thumbwheel_last_step;
//				thumbweheel_last_step = thumbweheel_step;
					thumbwheel_step = rc_sensor.info->thumbwheel.value;
					if(thumbwheel_step>=550)
					{
						rc_sensor.info->thumbwheel.wheel_count++;
						if(rc_sensor.info->thumbwheel.wheel_count>=1000&&rc_sensor.info->thumbwheel.wheel_count<=2000)
						{
							
								shoot.record_status=record_long;
						
						}
						else if(rc_sensor.info->thumbwheel.wheel_count>=50&&rc_sensor.info->thumbwheel.wheel_count<=1000)
						{
							shoot.record_status=record_short;
						}
						else
						{
							shoot.record_status=record_zero;
						}
					}
					else
					{
						rc_sensor.info->thumbwheel.wheel_count=0;
						if(shoot.shoot_status!=off_fire)
						{
							switch(shoot.record_status)
							{
								case record_long:
									shoot.shoot_status=running_fire;
								break;
								case record_short:
									shoot.shoot_status=single_fire;
								break;
								case record_zero:
									shoot.shoot_status=ready_fire;
								break;
								default:
								break;
							}
						}
					}
	//			}
			}
			break;
		default:
			break;	
		}
	}
	else/////////////上电默认陀螺仪控制
	{
				if(thumbweheel_step!=thumbweheel_last_step)
			{
				if(car->car_move_mode == gyro_CAR)
				{
					car->car_move_mode = cycle_CAR;
				}
				else if(car->car_move_mode == cycle_CAR)
				{
					car->car_move_mode = gyro_CAR;
				}
			}
			
			//拨杆中间时，切换到陀螺仪模式  
			if (car->car_move_mode != gyro_CAR &&
				car->car_move_mode != cycle_CAR)
			{
				car->car_move_mode = gyro_CAR;
			}
			
			if(car->car_move_mode==gyro_CAR ||
				car->car_move_mode == cycle_CAR)
			{
	//			if(rc_sensor.info->s1.value==rc_sensor.info->s1.value_last)
	//			{}
	//			else
	//			{
					switch(rc_sensor.info->s1.value)
					{
						case 0x01:
							shoot.shoot_status=ready_fire;
							//在这里加波轮控制初始化no，static那里加判断no，ok
							if(thumbwheel_step!=thumbwheel_last_step)
								{
									shoot.base_info.dail_info.dail_reset_state=DEV_RESET_NO;
								}
						//

						break;
						default:
							shoot.shoot_status=off_fire;
						break;
					}
					static int16_t thumbwheel_step,thumbwheel_last_step;
	//				thumbweheel_last_step = thumbweheel_step;
					thumbwheel_step = rc_sensor.info->thumbwheel.value;
					if(thumbwheel_step>=550)
					{
						rc_sensor.info->thumbwheel.wheel_count++;
						if(rc_sensor.info->thumbwheel.wheel_count>=1000&&rc_sensor.info->thumbwheel.wheel_count<=2000)
						{
							
								shoot.record_status=record_long;
						
						}
						else if(rc_sensor.info->thumbwheel.wheel_count>=50&&rc_sensor.info->thumbwheel.wheel_count<=1000)
						{
							shoot.record_status=record_short;
						}
						else
						{
							shoot.record_status=record_zero;
						}
					}
					else
					{
						rc_sensor.info->thumbwheel.wheel_count=0;
						if(shoot.shoot_status!=off_fire)
						{
							switch(shoot.record_status)
							{
								case record_long:
									shoot.shoot_status=running_fire;
								break;
								case record_short:
									shoot.shoot_status=single_fire;
								break;
								case record_zero:
									shoot.shoot_status=ready_fire;
								break;
								default:
								break;
							}
						}
					}
	//			}
			}
	
	}
}

void Car_Move_Mode_Update(car_t *car)
{
	if(gimbal.gimbal_reset_state ==DEV_RESET_NO)
	{
		car->car_move_mode=init_CAR;
	}
	else if (car->car_ctrl_mode == RC_CTRL_MODE)//遥控器模式
		{
			RC_Move_Mode_Update(car);//右拨杆控制移动模式
		}
		else{
			Key_Move_Mode_Update(car);
		}
}
uint16_t t;
uint32_t last_tick=0;
void Car_Ctrl(car_t *car) 
{
	/*陀螺仪初始化不控*/
	if(bmi.Kp!=0.125)
	{
		if(HAL_GetTick()-last_tick>=800)
			{bmi.Kp=0.125;}
  }
//	Car_Ctrl_Mode_Update(car);
//  Car_Move_Mode_Update(car);
	/* 整车命令更新 */
//    Command_Update(car);
}

void Car_Work(void)
{
	gimbal.work(&gimbal);
	shoot.work(&shoot);
}


