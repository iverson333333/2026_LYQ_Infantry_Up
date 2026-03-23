/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   
  ******************************************************************************
  */
#include "control_task.h"
#include "vision_protocol.h"
#include "usart.h"

//float t;
void StartControlTask(void const * argument)
{
//RC_ResetData(&rc_sensor);

	for(;;) 
	{
//		rc_sensor.check(&rc_sensor);
		
		Car_Ctrl(&car) ;
		
    Car_Work();
		
		CAN_BOARD_send();
		
		Vision_Board_Update();

		Vision_DataTx(&huart1);

//		/////////////test////////////自己加上的
//		
//		Board_Tx_Send_Data();
//				
		osDelay(1);
	}
}



