#ifndef __shoot_H_
#define __shoot_H_
#include "rp_device_config.h"
//#include "RM_motor.h"
#include "DM_motor.h"
#include "motor.h"
#include "communicate.h"
#include "vision_protocol.h"
#include "judge.h"
#include "judge_protocol.h"

#define DAIL_ONESHOT_ANGLE    (31481) //拨盘角度环单发一发要走的角度（正）
#define DAIL_INIT_ANGLE (10000)  //2|?ì3?ê??ˉ213￥???è
#define DAIL_REVERT_PISITION (-31481)  //拨盘反转要走的角度（反转为负)

/*发射总模式*/
typedef enum {
	off_fire,
	ready_fire,
	single_fire,
	running_fire,
	init_fire,
}shoot_status_e;

/*发射状态*/
typedef enum{
	record_zero,
	record_short,
	record_long,
}record_status_e;

/*拨盘pid控制模式*/
typedef enum{
	single_pid,
	double_pid,
}pid_mode_e;

typedef struct __attribute__((packed)) 
{
	uint8_t null;
	uint8_t find_Target;
	uint8_t find_outpost;
	uint8_t find_base;
}hit_target_e;

/*发射消抖包*/
typedef struct __attribute__((packed)) 
{
	float yaw_shake_angle ;		//yaw抖动角度
	float pitch_shake_angle;	//pitch抖动角度
	float const_offset_current; //前馈常数补偿电流
	float shoot_pitch_offset_current;//实际输出补偿电流
	float pitch_a;				//pitch电机角加速度，用来计算补偿电流
	float kd;					//发射时pitch电机pid的kd系数
	uint16_t feedforward_delay_time;//ms
	uint16_t feedforward_continue_time;//ms
	uint8_t feedforward_current_flag;
}shooting_shake_angle_t;

/*拨盘包*/
typedef struct __attribute__((packed)){
	float        	  target_speed;     //拨盘目标速度
	int32_t      	  target_angle_sum;  //拨盘目标位置
	int16_t         now_encoder;       //当前位置
	pid_mode_e pid_mode;
	bool	  stuck_flag;//堵转标志
	bool    reset_flag;//复位完成标志
	bool    is_dail_reset;//拨盘是否就位
	uint8_t count;
	uint8_t count_max;
	uint16_t init_time;
	uint16_t running_shoot_time;
	Dev_Reset_State_e dail_reset_state; //拨盘初始化状态
}dail_info_t;

/*摩擦轮速度*/
typedef struct __attribute__((packed)){
	int16_t target_fric_F_UP_speed;   //目标第二级上摩擦轮速度
	int16_t target_fric_F_L_speed;    //目标第二级左摩擦轮速度
	int16_t target_fric_F_R_speed;    //目标第二级右摩擦轮速度

	int16_t target_fric_B_UP_speed;   //目标第一级上摩擦轮速度
	int16_t target_fric_B_L_speed;    //目标第一级左摩擦轮速度
	int16_t target_fric_B_R_speed;    //目标第一级右摩擦轮速度

}friction_info_t;

/*摩擦轮配置包*/
typedef struct __attribute__((packed)) 
{
	float target_F_friction_speed;      //第二级摩擦轮目标速度
	float target_B_friction_speed;      //第一级摩擦轮目标速度
	float target_bullet_speed; 	//目标弹速
}shooting_config_t;

/*发射基础信息包*/
typedef struct __attribute__((packed)){
		int16_t    	    output_dail;      
		int16_t    	    output_fric_f_up;      
		int16_t    	    output_fric_f_l;    
		int16_t    	    output_fric_f_r;    
		int16_t    	    output_fric_b_up;   
		int16_t    	    output_fric_b_l;     
		int16_t    	    output_fric_b_r;     
	
	uint8_t is_heat_allow;//热量允许打弹
	uint8_t is_enable_shoot;//热量允许打弹
	uint16_t launch_timer;//延时发弹
	
	dail_info_t dail_info;
	friction_info_t fric_info;
}shoot_base_info_t;

typedef struct __attribute__((packed))shooting_struct{
	Motor_RM_t *fric_f_up;
	Motor_RM_t *fric_f_l;
	Motor_RM_t *fric_f_r;
	Motor_RM_t *fric_b_up;
	Motor_RM_t *fric_b_l;
	Motor_RM_t *fric_b_r;
	Motor_DM_t *dail;
	
	uint8_t stuck_count;
	
	shoot_status_e shoot_status;
	record_status_e record_status;
	shoot_base_info_t base_info;
	shooting_config_t 		 config;     
  hit_target_e	        target;
	
	bool is_on_fric;

	shooting_shake_angle_t   shooting_shake_angle;
	
	void     	    (*work)(struct shooting_struct *shoot);  

}shoot_t;

extern Motor_RM_t rm_motor[RM_MOTOR_LIST];
extern shoot_t shoot;

void Shoot_Work(shoot_t *shoot);

#endif


/*
//#include "shoot.h"
////下板收指令+热量限制->发给上板0或1+拨盘复位是否完成->（上板发视觉is_ready->视觉发回上板enable_shoot->）拨盘动标志位1
//shoot_t shoot=
//{
//	.fric_b_l=&rm_motor[B_L_Fric],
//	.fric_b_r=&rm_motor[B_R_Fric],
//	.fric_b_up=&rm_motor[B_UP_Fric],
//	.dail=&DAIL,
//	
//	.base_info.dail_info.pid_mode=double_pid,
//	
//	.base_info.dail_info.dail_reset_state=DEV_RESET_OK,
//	.base_info.dail_info.init_time= 0 ,
//	.base_info.dail_info.count= 0 ,
//	.base_info.dail_info.count_max = 100,
//	.base_info.dail_info.stuck_flag = 0,
//	.base_info.dail_info.reset_flag = 0,
//	.base_info.dail_info.is_dail_reset = 0,
//	.shoot_status=off_fire,//发射标志位
//	.record_status=record_zero,//单连发标志位
//	.work=Shoot_Work,
//	
//	.config.target_bullet_speed=11.7f,	
//	.config.target_B_friction_speed=0,     //4550,4350,4452（21度16.04），4320（22度16.2）,4290,4530,3585
//	
//	.target = 0,
//	
//			//发射消抖
//	.shooting_shake_angle.feedforward_delay_time=50,
//	.shooting_shake_angle.feedforward_continue_time=200,
//	.shooting_shake_angle.const_offset_current=5000,
//	.shooting_shake_angle.kd=1,
//	
//};


//void adapt(void)
//{
//////uint8_t flag;
//////void Shooting_Fri_Speed_Adapt(shoot_t *shoot)
//////{
//////	
///////用户定义参数**********************************************************

//////#define SPEED_SAVE_NUM 2			  // 速度保存个数
//////	const float add_kp = 7.f;		  // 增加增益
//////	const float minus_kp = 7.f;		  // 减少增益
//////	#if HERO_TYPE==2
//////	const float over_blind_err = 0.2; // 超过多少内不调整
//////	#else
//////	const float over_blind_err = 0.2; // 超过多少内不调整
//////	#endif
//////	
//////	const float less_blind_err = 0.1; // 低于多少内不调整
//////	const float max_adapt_range = 100; // 最大单次调整量

//////	/函数变量**************************************************************
//////	static uint8_t normal_speed_flag;	

//////	static float last_speed[SPEED_SAVE_NUM] = {0};					  // 保存上一发速度数组
//////	float now_speed = communicate.shoot_data_rx_info->shooting_speed; // 当前速度

//////	uint8_t over_cnt = 0, less_cnt = 0;								  // 大于目标速度计数，小于目标速度计数
//////	
//////   执行弹速调整的条件***************************************************
//////	#if HERO_TYPE==3
//////		#ifdef Z_CHANGE_FRIC_SPEED
//////			return ;
//////		#endif
//////	#endif
//////	
//////	if(communicate.car_data0_tx_info->car_state.bit.is_open_adapt==0)
//////	{
//////		
//////		return ;
//////	}
////////#ifndef FriSpeedAdaptEnabled
////////	return;
////////#endif
//////	if (shoot->shoot_status==off_fire)		  // 发射未初始化
//////		if (shoot->base_info.fric_info.target_fric_B_L_speed == 0) // 摩擦轮目标速度为0
//////				if (my_abs(communicate.shoot_data_rx_info->shooting_speed - shoot->config.target_bullet_speed) > 3) // 收到数据过于离谱
//////				{
//////					return;
//////				}
//////	//超弹速！！！大量下降
//////	if(now_speed>16.5f)
//////	{
//////		shoot->config.target_B_friction_speed -= 40;
////////		shoot->config.target_F_friction_speed -= 40;
//////		return;
//////	}
//////	
//////	*计算目前存储数组里弹速的情况******************************************
//////	for (uint8_t i = 0; i < SPEED_SAVE_NUM; i++)
//////	{
//////		if (last_speed[i] == 0)
//////		{
//////			// 如果找到一个元素为零，跳出循环
//////			continue;
//////		}
//////		else if (last_speed[i] > shoot->config.target_bullet_speed)
//////		{
//////			over_cnt++;
//////		}
//////		else if (shoot->config.target_bullet_speed > last_speed[i])
//////		{
//////			less_cnt++;
//////		}
//////	}

//////	*根据情况调整摩擦轮速度***********************************************
//////	//施密特触发器
//////	if (now_speed - shoot->config.target_bullet_speed > over_blind_err) // 速度大于目标速度
//////	{
//////		if (over_cnt * minus_kp > max_adapt_range)//限幅
//////			return;
//////		shoot->config.target_B_friction_speed -= over_cnt * minus_kp;
//////		shoot->config.target_F_friction_speed -= over_cnt * minus_kp;
//////	}
////// 
//////	else if (shoot->config.target_bullet_speed - now_speed > less_blind_err) // 速度小于目标速度
//////	{
//////		if (less_cnt * add_kp > max_adapt_range/||normal_speed_flag==1/)
//////			return;
//////		if(less_cnt>=2)//数组里面两个都低于弹速才提高弹速
//////		{
//////			shoot->config.target_B_friction_speed += less_cnt * add_kp;
////////			shoot->config.target_F_friction_speed += less_cnt * add_kp;
//////		}
//////		
//////	}

//////	*保存当前速度到数组*********************************************
//////	for (uint8_t i = 1; i < SPEED_SAVE_NUM; i++)
//////	{
//////		last_speed[i] = last_speed[i - 1];
//////	}
//////	last_speed[0] = now_speed;
//////}
//
//}

//手动拨盘复位
//void Shoot_dail_reset(shoot_t *shoot)
//{
//	shoot->base_info.dail_info.init_time++;
//	shoot->base_info.dail_info.target_speed=-1500;//反转
//	shoot->base_info.dail_info.pid_mode=single_pid;
//		
//	if(my_abs(shoot->dail->ctrl->position_inn->measure)<=15 && my_abs(shoot->dail->ctrl->position_inn->out)>=2000)//堵转判断
//	{
//		shoot->base_info.dail_info.count++;
//		if(shoot->base_info.dail_info.count>=shoot->base_info.dail_info.count_max)//堵转达到时间
//		{
//			shoot->base_info.dail_info.dail_reset_state=DEV_RESET_OK;
//	  	shoot->base_info.dail_info.pid_mode=double_pid;
//			shoot->dail->rx_info->motor_angle_sum=0;
//			shoot->base_info.dail_info.target_angle_sum=0;	
//			shoot->base_info.dail_info.target_angle_sum+=DAIL_INIT_ANGLE;
//			shoot->base_info.dail_info.count=0;
//			shoot->base_info.dail_info.running_shoot_time=0;
//			shoot->record_status=record_zero;	//				shoot->shoot_status=off_fire;
//		}
//	}
//	if(shoot->base_info.dail_info.init_time>=2000)//初始化超时
//	{
//		shoot->base_info.dail_info.dail_reset_state=DEV_RESET_OK;
//		shoot->base_info.dail_info.init_time=0;
//		shoot->base_info.dail_info.pid_mode=double_pid;
//		shoot->dail->rx_info->motor_angle_sum=0;
//		shoot->base_info.dail_info.target_angle_sum=0;	
//	}
//}
//拨盘到位检测
//void Shoot_dail_usable_judge(shoot_t *shoot)
//{
//	static float target;
//	static float measure;
//	target = shoot->base_info.dail_info.target_angle_sum;
//	measure = shoot->dail->rx_info->motor_angle_sum;
//	if(fabsf(target - measure) <= 0.1f)
//	{
//    shoot->base_info.dail_info.reset_flag = 1;
//	}		
//	else
//	{
//    shoot->base_info.dail_info.reset_flag = 0;
//	}
//}
//堵转处理
//void Shoot_stuck_deal(shoot_t *shoot)
//{
//	if(((my_abs(shoot->fric_b_l->ctrl->speed_ctrl->out)>=4000 && my_abs(rm_motor[B_L_Fric].rx_info->speed)<=20)  //堵转判断
//	 ||(my_abs(shoot->fric_b_r->ctrl->speed_ctrl->out)>=4000 && my_abs(rm_motor[B_R_Fric].rx_info->speed)<=20)
//	 ||(my_abs(shoot->fric_b_up->ctrl->speed_ctrl->out)>=4000 && my_abs(rm_motor[B_UP_Fric].rx_info->speed)<=20)
//	 ||(my_abs(shoot->dail->ctrl->position_inn->measure)<=15 && shoot->dail->ctrl->position_inn->out>=12000))
//	 && shoot->shoot_status!=off_fire && shoot->base_info.dail_info.stuck_flag==0)
//	{
//		shoot->stuck_count++;
//		if(shoot->stuck_count>=100)
//		{
//			shoot->base_info.dail_info.target_angle_sum+=DAIL_REVERT_PISITION;
//			shoot->base_info.dail_info.pid_mode=double_pid;
//			shoot->record_status=record_zero;
//		}
//	}
//}

//发射pid计算
//void Shoot_pid_cal(shoot_t *shoot)
//{
//	shoot->fric_b_l->ctrl->speed_ctrl->target=shoot->base_info.fric_info.target_fric_B_L_speed;
//	shoot->fric_b_r->ctrl->speed_ctrl->target=shoot->base_info.fric_info.target_fric_B_R_speed;	
//	shoot->fric_b_up->ctrl->speed_ctrl->target=shoot->base_info.fric_info.target_fric_B_UP_speed;			
//	if(my_abs(rm_motor[B_L_Fric].rx_info->encoder_speed)<=500 &&//不在发射不控摩擦轮
//		 my_abs(rm_motor[B_R_Fric].rx_info->encoder_speed)<=500 &&
//		 my_abs(rm_motor[B_UP_Fric].rx_info->encoder_speed)<=500 &&
//     shoot->shoot_status==off_fire)
//	{
//		shoot->fric_b_l->tx_info->torque=0;
//		shoot->fric_b_r->tx_info->torque=0;
//		shoot->fric_b_up->tx_info->torque=0;
//	}
//	else//////////////////////////////////////////在发射控摩擦轮
//	{
//		rm_motor[B_R_Fric].single_set_speed(&rm_motor[B_R_Fric]);
//		rm_motor[B_L_Fric].single_set_speed(&rm_motor[B_L_Fric]);
//		rm_motor[B_UP_Fric].single_set_speed(&rm_motor[B_UP_Fric]);
//	}
//	switch(shoot->base_info.dail_info.pid_mode)//拨盘pid计算
//	{
//	  case double_pid:
//  		if(shoot->shoot_status == off_fire)
//  		{
//  			shoot->dail->tx_info->torque=0;
//  		}
//  		else
//  		{
//  			shoot->dail->ctrl->position_out->target=shoot->base_info.dail_info.target_angle_sum;//单发位置环
//  			shoot->dail->ctrl->position_out->measure=DAIL.rx_info->motor_angle_sum;
//  			DM_Motor_Set_Angle_Position(&DAIL);
//  		}
//  	break;
//  	case single_pid:
//  		shoot->dail->ctrl->speed_ctrl->target=shoot->base_info.dail_info.target_speed;//连发速度环
//  		DAIL.single_set_speed(&DAIL);
//  	break;
//  	default:  		
//	  break;  	
//	}
//}
//外部获取
//static uint32_t t;
//void Shoot_extern_get(shoot_t *shoot)
//{
//	shoot->config.target_B_friction_speed = Board_Rx_Info.fric_speed_tar;
//	shoot->base_info.dail_info.is_dail_reset = Board_Rx_Info.is_dail_reset;
//	
//}
///发射控制*
//void Shoot_ctrl(shoot_t *shoot)
//{
//  if(shoot->shoot_status!=off_fire)//只要拨杆在上
//	{				
//		shoot->base_info.fric_info.target_fric_B_L_speed=shoot->config.target_B_friction_speed;//摩擦轮转速4350,4550
//		shoot->base_info.fric_info.target_fric_B_R_speed=-(shoot->config.target_B_friction_speed);
//		shoot->base_info.fric_info.target_fric_B_UP_speed=-(shoot->config.target_B_friction_speed);
//		switch(shoot->shoot_status)//判断单发连发
//		{
//			case single_fire:
//		   	shoot->base_info.dail_info.target_angle_sum+=DAIL_ONESHOT_ANGLE;
//				shoot->base_info.dail_info.pid_mode=double_pid;
//			
//				shoot->shoot_status=ready_fire;
//				shoot->record_status=record_zero;
//			break;
//		  case running_fire:
//		 	  if(HAL_GetTick()-t>=800)
//		   	{
//		// 	t++;
//		 	    shoot->base_info.dail_info.target_angle_sum+=DAIL_ONESHOT_ANGLE;
//				  shoot->base_info.dail_info.pid_mode=double_pid;
//				  t=HAL_GetTick();
//			  }	
//			break;
//			default:
//			break;
//		}
//	}
//	
//}

///离线保护*
//void Shoot_offline_detect(shoot_t *shoot)
//{
//	
//
///发射板间更新*
//void Shoot_Board_Update(shoot_t *shoot)
//{
//	Board_Tx_Info.vision_yaw_tar = vision.rx_info->building_yaw;
//	Board_Tx_Info.hit_enable = shoot->base_info.is_enable_shoot;
//  Board_Tx_Info.is_dail_reset = shoot->base_info.dail_info.is_dail_reset;
//	Board_Tx_Info.is_find_Target = shoot->target.find_Target;
//	
//	Board_Tx_Info.launch_timer = shoot->base_info.launch_timer;
//}

//void Shoot_Work_no(shoot_t *shoot)
//{
//	Shoot_dail_usable_judge(shoot);
//	if(RC_ONLINE)
//	{
//		if(shoot->base_info.dail_info.dail_reset_state==DEV_RESET_NO)//先初始化
//		{
//			shoot->base_info.dail_info.init_time++;
//			shoot->base_info.dail_info.target_speed=-1500;//反转
//			shoot->base_info.dail_info.pid_mode=single_pid;
//			
//			if(shoot->base_info.dail_info.init_time>=2000)//初始化超时
//			{
//				shoot->base_info.dail_info.dail_reset_state=DEV_RESET_OK;
//				shoot->base_info.dail_info.init_time=0;
//				shoot->base_info.dail_info.pid_mode=double_pid;
//				shoot->dail->rx_info->motor_angle_sum=0;
//				shoot->base_info.dail_info.target_angle_sum=0;	
//			}
//			
//			if(my_abs(shoot->dail->ctrl->position_inn->measure)<=15 && my_abs(shoot->dail->ctrl->position_inn->out)>=2000)//堵转判断
//			{
//				shoot->base_info.dail_info.count++;
//				if(shoot->base_info.dail_info.count>=shoot->base_info.dail_info.count_max)//堵转达到时间
//				{
//					shoot->base_info.dail_info.dail_reset_state=DEV_RESET_OK;
//					shoot->base_info.dail_info.pid_mode=double_pid;
//					shoot->dail->rx_info->motor_angle_sum=0;
//					shoot->base_info.dail_info.target_angle_sum=0;	
//					shoot->base_info.dail_info.target_angle_sum+=DAIL_INIT_ANGLE;
//					shoot->base_info.dail_info.count=0;
//					shoot->base_info.dail_info.running_shoot_time=0;
//					shoot->record_status=record_zero;
////					shoot->shoot_status=off_fire;

//				}
//			}
//		}			
//	
//		else//初始化完了进主程序
//		{
//			*堵转*
//			if(((my_abs(shoot->fric_b_l->ctrl->speed_ctrl->out)>=4000 && my_abs(rm_motor[B_L_Fric].rx_info->speed)<=20)  //堵转判断
//			 ||(my_abs(shoot->fric_b_r->ctrl->speed_ctrl->out)>=4000 && my_abs(rm_motor[B_R_Fric].rx_info->speed)<=20)
//			 ||(my_abs(shoot->fric_b_up->ctrl->speed_ctrl->out)>=4000 && my_abs(rm_motor[B_UP_Fric].rx_info->speed)<=20)
//			 ||(my_abs(shoot->dail->ctrl->position_inn->measure)<=15 && shoot->dail->ctrl->position_inn->out>=12000))
//			 && shoot->shoot_status!=off_fire && shoot->base_info.dail_info.stuck_flag==0)
//			{
//				shoot->stuck_count++;
//				if(shoot->stuck_count>=100)
//				{
//					shoot->base_info.dail_info.target_angle_sum+=DAIL_REVERT_PISITION;
//					shoot->base_info.dail_info.pid_mode=double_pid;
//					shoot->record_status=record_zero;
//				}
//			}
//				
//				//////////////////////////////不堵转///////////////
//			else
//			{
//			 shoot->stuck_count=0;
//					
//				if(shoot->shoot_status!=off_fire)//只要拨杆在上
//				{				
//					shoot->base_info.fric_info.target_fric_B_L_speed=shoot->config.target_B_friction_speed;//摩擦轮转速4350,4550
//					shoot->base_info.fric_info.target_fric_B_R_speed=-(shoot->config.target_B_friction_speed);
//					shoot->base_info.fric_info.target_fric_B_UP_speed=-(shoot->config.target_B_friction_speed);
//					
//					switch(shoot->shoot_status)//判断单发连发
//					{
//						case single_fire:
//							shoot->base_info.dail_info.target_angle_sum+=DAIL_ONESHOT_ANGLE;
//							shoot->base_info.dail_info.pid_mode=double_pid;
//						
//							shoot->shoot_status=ready_fire;
//							shoot->record_status=record_zero;
//						break;
//						case running_fire:
//							if(HAL_GetTick()-t>=1000)
//							{
//		//						tt++;
//								shoot->base_info.dail_info.target_angle_sum+=DAIL_ONESHOT_ANGLE;
//								shoot->base_info.dail_info.pid_mode=double_pid;
//								t=HAL_GetTick();
//							}	
//						break;
//						default:
//						break;
//					}
//				}
//				else//拨杆不在上
//				{
//					shoot->base_info.fric_info.target_fric_B_L_speed=0;
//					shoot->base_info.fric_info.target_fric_B_R_speed=0;
//					shoot->base_info.fric_info.target_fric_B_UP_speed=0;
//					shoot->base_info.dail_info.now_encoder=shoot->dail->rx_info->motor_angle;/////////////////////
//					
//					shoot->base_info.dail_info.pid_mode=double_pid;
//					shoot->record_status=record_zero;				
//			  }
//		  }
//	  }
//	}
//	else//关控
//	{
//		shoot->base_info.fric_info.target_fric_B_L_speed=0;
//		shoot->base_info.fric_info.target_fric_B_R_speed=0;
//		shoot->base_info.fric_info.target_fric_B_UP_speed=0;
//		shoot->record_status=record_zero;
//		shoot->shoot_status=off_fire;
//		shoot->base_info.dail_info.dail_reset_state=DEV_RESET_NO;
//    shoot->base_info.dail_info.target_angle_sum=0;
//	}			

//  Shoot_pid_cal(shoot);
//	///////////////////////////////////////////////////////////////////////////////////////////////////////////、、、、、、、、、、、、、、、、
//	
//	if(RC_ONLINE)
//	{
//		if(shoot->fric_b_l->state == DEV_ONLINE && shoot->fric_b_r->state == DEV_ONLINE &&         //掉线保护//
//			shoot->fric_b_up->state == DEV_ONLINE && shoot->dail->state == DEV_ONLINE)
//		{
//			if(shoot->shoot_status!=off_fire)
//			{
//				if(shoot->base_info.dail_info.dail_reset_state==DEV_RESET_NO)//判断初始化               //拨盘初始化//
//				{
//					shoot->base_info.dail_info.init_time++;
//					shoot->base_info.dail_info.target_speed=-1500;//反转
//					shoot->base_info.dail_info.pid_mode=single_pid;
//					
//					if(shoot->base_info.dail_info.init_time>=2000)//初始化超时
//					{
//						shoot->base_info.dail_info.dail_reset_state=DEV_RESET_OK;
//						shoot->base_info.dail_info.init_time=0;
//						shoot->base_info.dail_info.pid_mode=double_pid;
//						shoot->dail->rx_info->motor_angle_sum=0;
//						shoot->base_info.dail_info.target_angle_sum=0;	
//					}
//					
//					if(my_abs(shoot->dail->ctrl->position_inn->measure)<=15 && my_abs(shoot->dail->ctrl->position_inn->out)>=2000)//堵转判断
//					{
//						shoot->base_info.dail_info.count++;
//						if(shoot->base_info.dail_info.count>=shoot->base_info.dail_info.count_max)//堵转达到时间
//						{
//							shoot->base_info.dail_info.dail_reset_state=DEV_RESET_OK;
//							shoot->base_info.dail_info.pid_mode=double_pid;
//							shoot->dail->rx_info->motor_angle_sum=0;
//							shoot->base_info.dail_info.target_angle_sum=0;	
//							shoot->base_info.dail_info.target_angle_sum+=DAIL_INIT_ANGLE;
//							shoot->base_info.dail_info.count=0;
//							shoot->base_info.dail_info.running_shoot_time=0;
//							shoot->record_status=record_zero;
//			//				shoot->shoot_status=off_fire;

//						}
//					}
//				}
//			  else                                                                           
//				{
//					if(((my_abs(shoot->fric_b_l->ctrl->speed_ctrl->out)>=4000 && my_abs(rm_motor[B_L_Fric].rx_info->speed)<=20)  //堵转判断///
//			      ||(my_abs(shoot->fric_b_r->ctrl->speed_ctrl->out)>=4000 && my_abs(rm_motor[B_R_Fric].rx_info->speed)<=20)
//		     	  ||(my_abs(shoot->fric_b_up->ctrl->speed_ctrl->out)>=4000 && my_abs(rm_motor[B_UP_Fric].rx_info->speed)<=20)
//		     	  ||(my_abs(shoot->dail->ctrl->position_inn->measure)<=15 && shoot->dail->ctrl->position_inn->out>=12000))
//		      	 && shoot->shoot_status!=off_fire && shoot->base_info.dail_info.stuck_flag==0)
//		     	{
//			     	shoot->stuck_count++;
//		     		if(shoot->stuck_count>=100)
//		     		{
//		     			shoot->base_info.dail_info.target_angle_sum+=DAIL_REVERT_PISITION;
//		     			shoot->base_info.dail_info.pid_mode=double_pid;
//		     			shoot->record_status=record_zero;
//		     		}
//		     	}
//					else
//		     	{
//		     	  shoot->stuck_count=0;                                                                                     //不堵转控///
//					
//		     		shoot->base_info.fric_info.target_fric_B_L_speed=shoot->config.target_B_friction_speed;//摩擦轮转速4350,4550
//		     		shoot->base_info.fric_info.target_fric_B_R_speed=-(shoot->config.target_B_friction_speed);
//		     		shoot->base_info.fric_info.target_fric_B_UP_speed=-(shoot->config.target_B_friction_speed);
//		     			
//		     		switch(shoot->shoot_status)//判断单发连发
//			     	{
//		     			case single_fire:
//		     	    	shoot->base_info.dail_info.target_angle_sum+=DAIL_ONESHOT_ANGLE;
//		     				shoot->base_info.dail_info.pid_mode=double_pid;
//		     			
//		     				shoot->shoot_status=ready_fire;
//		     				shoot->record_status=record_zero;
//		     			break;
//		     			case running_fire:
//		     				if(HAL_GetTick()-t>=1000)
//		     				{
//		//			    	tt++;
//						    	shoot->base_info.dail_info.target_angle_sum+=DAIL_ONESHOT_ANGLE;
//					    		shoot->base_info.dail_info.pid_mode=double_pid;
//					    		t=HAL_GetTick();
//					    	}	
//				    		break;
//			     		default:
//			     		break;
//			     	}
//			     	
//			     
//		       }     
//				}
//					
//			}
//			else//拨杆不在上
//			{
//			  shoot->base_info.fric_info.target_fric_B_L_speed=0;
//			  shoot->base_info.fric_info.target_fric_B_R_speed=0;
//			  shoot->base_info.fric_info.target_fric_B_UP_speed=0;
//			  shoot->base_info.dail_info.now_encoder=shoot->dail->rx_info->motor_angle;/////////////////////
//			     		
//			  shoot->base_info.dail_info.pid_mode=double_pid;
//			  shoot->record_status=record_zero;				
//		  }
//		}
//	}
//	else//关控
//	{
//		
//	}
//}

//void Shoot_Work(shoot_t *shoot)
//{
//	Shoot_offline_detect(shoot);
//	Shoot_dail_usable_judge(shoot);
//	Shoot_extern_get(shoot);
//	if(shoot->base_info.dail_info.is_dail_reset == 1)
//	{
//		Shoot_dail_reset(shoot);
//	}
//	Shoot_ctrl(shoot);
//	Shoot_pid_cal(shoot);
//}
*/