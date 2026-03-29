#include "can_protocol.h"
#include "chassis.h"
#include "motor.h"
#include "communicate.h"
/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
	case 0x11:
		Pitch_Motor.rx(&Pitch_Motor, rxBuf);
		break;
	case ID_FRIC_L:
		rm_motor[L_Fric].rx(&rm_motor[L_Fric], rxBuf);
		break;
	case ID_FRIC_R:
		rm_motor[R_Fric].rx(&rm_motor[R_Fric], rxBuf);
		break;
	case ID_FRIC_UP:
		rm_motor[UP_Fric].rx(&rm_motor[UP_Fric], rxBuf);
		break;
	default:
		break;
	}
}
/**
 *  @brief  CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{

	switch (canId)
	{
	case 0xD1:
		Board_Rx_D1(rxBuf);
		break;
	case 0xD2:
		Board_Rx_D2(rxBuf);
		break;

	default:
		break;
	}
}

void CAN_BOARD_send(void)
{
	if (Board_Rx_Info.flag.bit.is_rc_online == 1)
	{
		RM_Group_F1.group_set_torque(&RM_Group_F1);
		Pitch_Motor.single_set_torque(&Pitch_Motor);
		Send_To_Down_Board();
	}
	else
	{
		RM_Group_F1.group_sleep(&RM_Group_F1);
		RM_Group_F1.group_set_torque(&RM_Group_F1);
		Pitch_Motor.single_sleep(&Pitch_Motor);
		Pitch_Motor.single_set_torque(&Pitch_Motor);
		Send_To_Down_Board();
	}
}
