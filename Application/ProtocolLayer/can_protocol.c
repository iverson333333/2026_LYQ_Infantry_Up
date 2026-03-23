#include "can_protocol.h"
#include "chassis.h"
#include "motor.h"

#ifdef UART_COMMUNICATE

/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case ID_GIMB_P:
			rm_motor[gim_pitch].rx(&rm_motor[gim_pitch],rxBuf);
		break;
		case ID_FRIC_B_L:
			rm_motor[B_L_Fric].rx(&rm_motor[B_L_Fric],rxBuf);
		break;
		case ID_FRIC_B_R:
			rm_motor[B_R_Fric].rx(&rm_motor[B_R_Fric],rxBuf);
		break;
		case ID_FRIC_B_UP:
			rm_motor[B_UP_Fric].rx(&rm_motor[B_UP_Fric],rxBuf);
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

		default:	
		break;
	}
}


void CAN_BOARD_send(void)
{
	if(RC_ONLINE)
	{	  
    RM_Group_F1.group_set_torque(&RM_Group_F1);
		rm_motor[gim_pitch].single_set_torque(&rm_motor[gim_pitch]);
		Board_Tx_Send_Data();
	}
	else if(RC_OFFLINE)
	{
		RM_Group_F1.group_sleep(&RM_Group_F1);
		RM_Group_F1.group_set_torque(&RM_Group_F1);
		rm_motor[gim_pitch].single_sleep(&rm_motor[gim_pitch]);
		rm_motor[gim_pitch].single_set_torque(&rm_motor[gim_pitch]);

	}
	
}

#else

/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case ID_GIMB_P:
			rm_motor[gim_pitch].rx(&rm_motor[gim_pitch],rxBuf);
		break;
		case ID_FRIC_B_L:
			rm_motor[B_L_Fric].rx(&rm_motor[B_L_Fric],rxBuf);
		break;
		case ID_FRIC_B_R:
			rm_motor[B_R_Fric].rx(&rm_motor[B_R_Fric],rxBuf);
		break;
		case ID_FRIC_B_UP:
			rm_motor[B_UP_Fric].rx(&rm_motor[B_UP_Fric],rxBuf);
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
			Board_Rx_C1(rxBuf);
		break;
		case 0xD2:
			Board_Rx_C2(rxBuf);
		break;
		case 0xD3:
			Board_Rx_C3(rxBuf);
		break;
		case 0xD4:
			Board_Rx_C4(rxBuf);
		break;
		
		default:	
		break;
	}
}


void CAN_BOARD_send(void)
{
	if(Board_Rx_Info.is_rc_online == 0)
	{
    RM_Group_F1.group_set_torque(&RM_Group_F1);
		rm_motor[gim_pitch].single_set_torque(&rm_motor[gim_pitch]);
		Board_Tx_Update(&Board_Tx_Info);
		Board_Tx_C1();
		Board_Tx_C2();
	}
	else 
	{
		RM_Group_F1.group_sleep(&RM_Group_F1);
		RM_Group_F1.group_set_torque(&RM_Group_F1);
		rm_motor[gim_pitch].single_sleep(&rm_motor[gim_pitch]);
		rm_motor[gim_pitch].single_set_torque(&rm_motor[gim_pitch]);
		Board_Tx_Update(&Board_Tx_Info);
		Board_Tx_C1();
		Board_Tx_C2();
	}
	
}

#endif
