
/* Includes ------------------------------------------------------------------*/
#include "motor.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* Private variables ---------------------------------------------------------*/
// 添加电机时需要初始化的参数：3508还是6020，CAN1还是CAN2，以及电机的接收ID；
// CAN发送数组包含了用CAN1还是CAN2，发送数据下标在电机Motor_SendData判断。
//

//drv_can_t rm_motor_driver[] = {
//	[GIMB_P] = {
//		.can_id = DRV_CAN2,
//		.rx_id = ID_GIMB_P,  //0x204+电机id：2
//	}
//};
////发送ID和接收ID都可以单独设置,它的回馈报文的第一个字节是我们发送给它的ID，它的回馈报文ID可以单独设置
//drv_can_t ht_motor_drive={
//		.rx_id = 0x0B,
//		.tx_id =0x09,
//		.can_id = DRV_CAN1,
//};



/*PID结构体定义------------------------------------------------*/
// 注意定义了之后需要在rm_motor_list_init用rm_motor_pid_init初始化

//motor_pid_t GIMB_P_mec = {
//	.speed.kp = 0,
//	.speed.ki = 0,
//	.speed.kd = 0,
//	.speed.integral_max = 3000,
//	.speed.out_max = 28000,
//	.angle.kp = 0, // 0.45
//	.angle.ki = 0,
//	.angle.kd = 0,
//	.angle.integral_max = 0,
//	.angle.out_max = 500,
//};
///*HT_start*/

//Motor_HT_Born_Info_t L_Wheel_Born_Info = 
//{	
//	.stdId = 0x009,//电机控制报文ID
//	.hcan = &hcan1,//使用的Can总线
//	.order_correction = 0,//电机总角度的正方向为顺时针
//};
//Motor_HT_Rx_Info_t L_Wheel_Rx_Info_t;
//Motor_HT_Tx_Info_t L_Wheel_Tx_Info_t;
//Motor_HT_State_t L_Wheel_State_t;
//Motor_HT_t L_Wheel = 
//{
//	.born_info = &L_Wheel_Born_Info,
//	
//	.rx_info = &L_Wheel_Rx_Info_t,
//	
//	.tx_info = &L_Wheel_Tx_Info_t,
//	
//	.state = &L_Wheel_State_t,
//	
//	.single_init = &HT_Single_Motor_Init,
//};
///*HT_end*/

///*DM_start*/
//Motor_DM_Born_Info_t DAIL_Born_Info =
//{
//	.stdId = 0x001,//电机控制报文ID
//	
//	.hcan = &hcan1,//使用的Can总线

//};

//pid_ctrl_t DAIL_position_Ctrl_out = 
//{
//	.kp = 6.5f,//
//	.ki = 0.f,
//	.kd = 0.f,
//	.integral_max = 0.f,
//	.out_max = 50.f,
//};

//pid_ctrl_t DAIL_position_Ctrl_inn = 
//{
//	.kp = 1.f,//
//	.ki = 0.f,
//	.kd = 0.f,
//	.integral_max = 0.f,
//	.out_max = 7.f,
//};

//Motor_DM_Ctrl_Info_t DAIL_Ctrl = 
//{
//	.position_inn=&DAIL_position_Ctrl_inn,
//	.position_out=&DAIL_position_Ctrl_out,
//};

//Motor_DM_Rx_Info_t DAIL_Rx_Info_t;

//Motor_DM_Tx_Info_t DAIL_Tx_Info_t;

//Motor_DM_State_t DAIL_State_t;

//Motor_DM_t DAIL = 
//{
//	.born_info = &DAIL_Born_Info,
//	
//	.rx_info = &DAIL_Rx_Info_t,
//	
//	.tx_info = &DAIL_Tx_Info_t,
//	
//	.state = &DAIL_State_t,
//	
//	.ctrl = &DAIL_Ctrl,
//	
//	.single_init = &DM_Single_Motor_Init,
//};
///*DM_end*/

/*RM START*/
Motor_RM_Born_Info_t B_R_Fric_Born = 
{
	.rxId = 1,
	
	.hcan = &hcan1,
	
	.type = _3508_Single,
	
	.stdId = 0x200,
};

Motor_RM_Tx_Info_t B_R_Fric_Tx;

Motor_RM_State_t B_R_Fric_State;

Motor_RM_Rx_Info_t B_R_Fric_Rx;

pid_ctrl_t B_R_Fric_Speed_Ctrl = 
{
	.kp = 27.f,//
	.ki = 0.5f,
	.kd = 0.f,
	.integral_max = 6000.f,
	.out_max = 10000.f,//
};

Motor_RM_Ctrl_Info_t B_R_Fric_Ctrl = 
{
	.speed_ctrl = &B_R_Fric_Speed_Ctrl,
};

Motor_RM_Born_Info_t B_L_Fric_Born = 
{
	.rxId = 2,
	
	.hcan = &hcan1,
	
	.type = _3508_Single,
	
	.stdId = 0x200,
};

Motor_RM_Tx_Info_t B_L_Fric_Tx;

Motor_RM_State_t B_L_Fric_State;

Motor_RM_Rx_Info_t B_L_Fric_Rx;

pid_ctrl_t B_L_Fric_Speed_Ctrl = 
{
	.kp = 27.f,//
	.ki = 0.5f,
	.kd = 0.f,
	.integral_max = 6000.f,
	.out_max = 10000.f,//
};

Motor_RM_Ctrl_Info_t B_L_Fric_Ctrl = 
{
	.speed_ctrl = &B_L_Fric_Speed_Ctrl,
};

Motor_RM_Born_Info_t B_UP_Fric_Born = 
{
	.rxId = 3,
	
	.hcan = &hcan1,
	
	.type = _3508_Single,
	
	.stdId = 0x200,
};

Motor_RM_Tx_Info_t B_UP_Fric_Tx;

Motor_RM_State_t B_UP_Fric_State;

Motor_RM_Rx_Info_t B_UP_Fric_Rx;

pid_ctrl_t B_UP_Fric_Speed_Ctrl = 
{
	.kp = 27.f,//
	.ki = 0.51f,
	.kd = 0.f,
	.integral_max = 6000.f,
	.out_max = 10000.f,//
};

Motor_RM_Ctrl_Info_t B_UP_Fric_Ctrl = 
{
	.speed_ctrl = &B_UP_Fric_Speed_Ctrl,
};

//Motor_RM_Born_Info_t F_UP_Fric_Born = 
//{
//	.rxId = 1,
//	
//	.hcan = &hcan1,
//	
//	.type = _3508_Single,
//	
//	.stdId = 0x200,
//};

//Motor_RM_Tx_Info_t F_UP_Fric_Tx;

//Motor_RM_State_t F_UP_Fric_State;

//Motor_RM_Rx_Info_t F_UP_Fric_Rx;

//pid_ctrl_t F_UP_Fric_Speed_Ctrl = 
//{
//	.kp = 27.f,//
//	.ki = 0.5f,
//	.kd = 0.f,
//	.integral_max = 6000.f,
//	.out_max = 10000.f,//
//};

//Motor_RM_Ctrl_Info_t F_UP_Fric_Ctrl = 
//{
//	.speed_ctrl = &F_UP_Fric_Speed_Ctrl,
//};

//Motor_RM_Born_Info_t F_R_Fric_Born = 
//{
//	.rxId = 0,
//	
//	.hcan = &hcan1,
//	
//	.type = _3508_Single,
//	
//	.stdId = 0x1FF,
//};

//Motor_RM_Tx_Info_t F_R_Fric_Tx;

//Motor_RM_State_t F_R_Fric_State;

//Motor_RM_Rx_Info_t F_R_Fric_Rx;

//pid_ctrl_t F_R_Fric_Speed_Ctrl = 
//{
//	.kp = 27.f,//
//	.ki = 0.5f,
//	.kd = 0.f,
//	.integral_max = 6000.f,
//	.out_max = 10000.f,//
//};

//Motor_RM_Ctrl_Info_t F_R_Fric_Ctrl = 
//{
//	.speed_ctrl = &F_R_Fric_Speed_Ctrl,
//};

//Motor_RM_Born_Info_t F_L_Fric_Born = 
//{
//	.rxId = 2,
//	
//	.hcan = &hcan1,
//	
//	.type = _3508_Single,
//	
//	.stdId = 0x1FF,
//};

//Motor_RM_Tx_Info_t F_L_Fric_Tx;

//Motor_RM_State_t F_L_Fric_State;

//Motor_RM_Rx_Info_t F_L_Fric_Rx;

//pid_ctrl_t F_L_Fric_Speed_Ctrl = 
//{
//	.kp = 27.f,//
//	.ki = 0.5f,
//	.kd = 0.f,
//	.integral_max = 6000.f,
//	.out_max = 10000.f,//
//};

//Motor_RM_Ctrl_Info_t F_L_Fric_Ctrl = 
//{
//	.speed_ctrl = &F_L_Fric_Speed_Ctrl,
//};

Motor_RM_Born_Info_t gim_pitch_Born = 
{
	.rxId = 1,
	
	.hcan = &hcan1,
	
	.type = _6020_Single,
	
	.stdId = 0x1FF,	
};
Motor_RM_Born_Info_t UP_Fric_Born = 
{
	.rxId = 3,
	
	.hcan = &hcan1,
	
	.type = _3508_Single,
	
	.stdId = 0x1FF,
};

Motor_RM_Tx_Info_t UP_Fric_Tx;

Motor_RM_State_t UP_Fric_State;

Motor_RM_Rx_Info_t UP_Fric_Rx;

pid_ctrl_t UP_Fric_Speed_Ctrl = 
{
	.kp = 27.f,//
	.ki = 0.5f,
	.kd = 0.f,
	.integral_max = 6000.f,
	.out_max = 10000.f,//
};

Motor_RM_Ctrl_Info_t UP_Fric_Ctrl = 
{
	.speed_ctrl = &UP_Fric_Speed_Ctrl,
};


Motor_RM_Tx_Info_t gim_pitch_Tx;

Motor_RM_State_t gim_pitch_State;

Motor_RM_Rx_Info_t gim_pitch_Rx;

pid_ctrl_t gim_pitch_Speed_Ctrl = 
{
	.kp = 0,     //800.f,//
	.ki = 0,     //5.f,
	.kd = 0.f,
	.integral_max = 3000.f,
	.out_max = 25000.f,
};

pid_ctrl_t gim_pitch_angle_Ctrl_inn = 
{
	.kp = 85.f,//  120.f,     //200.f,//800
	.ki = 0.f,//2
	.kd = 0.f,//5
	.integral_max = 3000.f,
	.out_max = 25000.f,
};

pid_ctrl_t gim_pitch_angle_Ctrl_out = 
{
	.kp = 10.f,//  1.f,//
	.ki = 0.3f,
	.kd = 0.f,
	.integral_max = 5.f,
	.out_max = 200.f,
};

pid_ctrl_t gim_pitch_Speed_Ctrl_gyro = 
{
	.kp = 0,       //800.f,//20
	.ki = 0,       //8.f,//2
	.kd = 0.f,//5
	.integral_max = 3000.f,
	.out_max = 25000.f,
};

pid_ctrl_t gim_pitch_angle_Ctrl_inn_gyro = 
{
	.kp = 280.f,//200.f,       //110.f,//800,700
	.ki = 0.f,       //2.5f,//8,3
	.kd = 0.f,//5
	.integral_max = 3000.f,
	.out_max = 25000.f,
};

pid_ctrl_t gim_pitch_angle_Ctrl_out_gyro = 
{
	.kp = 25,//40会抖       //15.f,//13
	.ki = 1.f,//0.3f,
	.kd = 0.f,
	.integral_max = 5.f,
	.out_max = 5000.f,
};

Motor_RM_Ctrl_Info_t gim_pitch_Ctrl = 
{
	.speed_ctrl = &gim_pitch_Speed_Ctrl,
	.angle_ctrl_inner=&gim_pitch_angle_Ctrl_inn,
	.angle_ctrl_outer=&gim_pitch_angle_Ctrl_out,
	.speed_ctrl_gyro =&gim_pitch_Speed_Ctrl_gyro,
	.angle_ctrl_inner_gyro=&gim_pitch_angle_Ctrl_inn_gyro,
	.angle_ctrl_outer_gyro=&gim_pitch_angle_Ctrl_out_gyro,
};

//Motor_RM_Born_Info_t dail_Born = 
//{
//	.rxId = 2,
//	
//	.hcan = &hcan1,
//	
//	.type = _3508_Single,
//	
//	.stdId = 0x1FF,	
//};

//Motor_RM_Tx_Info_t dail_Tx;

//Motor_RM_State_t dail_State;

//Motor_RM_Rx_Info_t dail_Rx;

//pid_ctrl_t dail_Speed_Ctrl = 
//{
//	.kp = 15.f,
//	.ki = 0.f,
//	.kd = 0.f,
//	.integral_max = 8000.f,
//	.out_max = 25000.f,
//};

//pid_ctrl_t dail_angle_Ctrl_inn = 
//{
//	.kp = 15.f,
//	.ki = 0.f,
//	.kd = 0.f,
//	.integral_max = 3000.f,
//	.out_max = 25000.f,
//};

//pid_ctrl_t dail_angle_Ctrl_out = 
//{
//	.kp = 0.25f,//
//	.ki = 0.f,
//	.kd = 0.f,
//	.integral_max = 1000.f,
//	.out_max = 1000.f,
//};

//pid_ctrl_t dail_position_Ctrl_inn = 
//{
//	.kp = 15.f,//
//	.ki = 0.f,
//	.kd = 0.f,
//	.integral_max = 0.f,
//	.out_max = 20000.f,
//};

//pid_ctrl_t dail_position_Ctrl_out = 
//{
//	.kp = 0.15f,//
//	.ki = 0.f,
//	.kd = 0.f,
//	.integral_max = 1000.f,
//	.out_max = 1000.f,
//};

//Motor_RM_Ctrl_Info_t dail_Ctrl = 
//{
//	.speed_ctrl = &dail_Speed_Ctrl,
//	.angle_ctrl_inner=&dail_angle_Ctrl_inn,
//	.angle_ctrl_outer=&dail_angle_Ctrl_out,
//	.position_inn=&dail_position_Ctrl_inn,
//	.position_out=&dail_position_Ctrl_out,
//};



Motor_RM_t rm_motor[]=
{
	[B_R_Fric]=
	{
		.born_info = &B_R_Fric_Born,
	
	.rx_info = &B_R_Fric_Rx,
	
	.tx_info = &B_R_Fric_Tx,

  .state = &B_R_Fric_State,
	
	.single_init = RM_Motor_Init,
	
	.ctrl = &B_R_Fric_Ctrl,
	},
	[B_L_Fric]=
	{
		.born_info = &B_L_Fric_Born,
	
	.rx_info = &B_L_Fric_Rx,
	
	.tx_info = &B_L_Fric_Tx,

  .state = &B_L_Fric_State,
	
	.single_init = RM_Motor_Init,
	
	.ctrl = &B_L_Fric_Ctrl,
	},
	[B_UP_Fric]=
	{
		.born_info = &B_UP_Fric_Born,
	
	.rx_info = &B_UP_Fric_Rx,
	
	.tx_info = &B_UP_Fric_Tx,

  .state = &B_UP_Fric_State,
	
	.single_init = RM_Motor_Init,
	
	.ctrl = &B_UP_Fric_Ctrl,
	},

	[gim_pitch]=
	{
	.born_info = &gim_pitch_Born,
	
	.rx_info = &gim_pitch_Rx,
	
	.tx_info = &gim_pitch_Tx,

  .state = &gim_pitch_State,
	
	.single_init = RM_Motor_Init,
	
	.ctrl = &gim_pitch_Ctrl,
	},

};

Motor_RM_Group_t RM_Group_F1 =
{
	.motor[0] = NULL,
	
	.motor[1] = &rm_motor[B_R_Fric],
	
	.motor[2] = &rm_motor[B_L_Fric],
	
	.motor[3] = &rm_motor[B_UP_Fric],
	
	.stdId=0x200,
	
	.hcan=&hcan1,
	
	.group_init = RM_Group_Motor_Init,
};

//Motor_RM_Group_t RM_Group_F2 =
//{
//	.motor[0] = &rm_motor[F_R_Fric],
//	
//	.motor[1] = &rm_motor[gim_pitch],
//	
//	.motor[2] = &rm_motor[F_L_Fric],
//	
//	.motor[3] = &rm_motor[F_UP_Fric],
//	
//	.stdId=0x1FF,
//	
//	.hcan=&hcan2,
//	
//	.group_init = RM_Group_Motor_Init,
//};


//Motor_RM_Group_t RM_Group_3 =
//{
//	.motor[0] = NULL,
//	
//	.motor[1] = &rm_motor[IMAGE],
//	
//	.motor[2] = &rm_motor[DAIL],
//	
//	.motor[3] = NULL,
//	
//	.stdId=0x1FF,
//	
//	.hcan=&hcan1,
//	
//	.group_init = RM_Group_Motor_Init,
//};


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
			.rx_info={
				.accel= 0,
				.angle_add= 0,
				.circleAngle= 0,
				.current= 0,
				.current_A= 0,
				.current_B= 0,
				.current_C= 0,
				.encoder= 0,
				.encoderOffset= 0,
				.encoderRaw= 0,
				.errorState= 0,
				.motorAngle= 0,
				.powerControl= 0,
				.speed= 0,
				.temperature= 0,
				.voltage= 0,
			},
//			.pid_info={
//				.init_flag=0,
//				.rx={
//					.angleKi=,
//					.angleKp=,
//					.iqKi=,
//					.iqKp=,
//					.speedKi=,
//					.speedKp=,				
//				},
//				.tx={
//					.angleKi=,
//					.angleKp=,
//					.iqKi=,
//					.iqKp=,
//					.speedKi=,
//					.speedKp=,				
//				},				
//			},
			.state_info={
				.init_flag= 0,
				.offline_cnt= 0,
				.offline_cnt_max= 0,
				.selfprotect_cnt= 0,
				.selfprotect_cnt_max= 0,
				.selfprotect_flag= 0,
				.work_state= 0,
			},
		
		},
		.init = KT_motor_class_init,
	},
};

motor_pid_t GIMB_Y_mec = {
	.speed.kp = 30,//20
	.speed.ki = 1,//0.5
	.speed.kd = 0,
	.speed.integral_max = 1000,
	.speed.out_max = 20000,
	.angle.kp = 6, //7
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.integral_max = 0,
	.angle.out_max = 1680,
}; 
motor_pid_t GIMB_Y_gyro = {
	.speed.kp = 25,//20
	.speed.ki = 1,//0.15
	.speed.kd = 0,
	.speed.integral_max = 1000,
	.speed.out_max = 20000,
	.angle.kp = 6,//7,4
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.integral_max = 0,
	.angle.out_max = 1680,
};

/* Exported functions --------------------------------------------------------*/
void rm_motor_list_init()
{
	/*电机信息初始化*/
//	RM_Group_3.group_init(&RM_Group_3);
	RM_Group_F1.group_init(&RM_Group_F1);
//	RM_Group_F2.group_init(&RM_Group_F2);
	rm_motor[gim_pitch].single_init(&rm_motor[gim_pitch]);

}

//void kt_motor_list_init()
//{
//	kt_motor[0].init(&kt_motor[0]);
//	motor_pid_init(&kt_motor[0].motor_all_pid.mec_pid, GIMB_Y_mec);
//	motor_pid_init(&kt_motor[0].motor_all_pid.gyro_pid, GIMB_Y_gyro);
//}
//void dm_motor_list_init()
//{
//	DAIL.single_init(&DAIL);
//	
//}

//void ht_motor_list_init()
//{
//	L_Wheel.single_init(&L_Wheel);
//	
//}

void rm_motor_list_heart_beat()
{
	RM_Group_F1.group_heartbeat(&RM_Group_F1);
//	RM_Group_F2.group_heartbeat(&RM_Group_F2);
//	RM_Group_3.group_heartbeat(&RM_Group_3);
	rm_motor[gim_pitch].single_heart_beat(&rm_motor[gim_pitch]);
}


