#ifndef __CONTROL_TASK
#define __CONTROL_TASK

#include "cmsis_os.h"
#include "main.h"
#include "device.h"
#include "can_protocol.h"
#include "gimbal.h"
#include "shoot.h"

void StartControlTask(void const * argument);

#endif
