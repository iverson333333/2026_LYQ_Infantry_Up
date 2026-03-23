#ifndef __CONTROL_TASK
#define __CONTROL_TASK

#include "cmsis_os.h"
#include "main.h"
#include "device.h"
#include "car.h"
#include "chassis.h"
#include "can_protocol.h"
#include "gimbal.h"


void StartControlTask(void const * argument);


#endif
