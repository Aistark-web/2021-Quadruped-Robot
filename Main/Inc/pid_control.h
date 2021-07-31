#ifndef __PID_CONTROL_H_
#define __PID_CONTROL_H_

#include "stm32f4xx_hal.h"
#include "my_pid.h"
#include "my_motor.h"
#include "csys.h"
#include "imu.h"
#include "ramp.h"
#include "remote.h"
void PID_Standard_Place_Control(void);
void PID_Standard_Place_Speed_Control(void);
void PID_Standard_Control_Place_Speed_Test(void);
void Motor_Auto_Init(void);
_Bool Motor_Check(void);
CSYS_Handle Foot_Angle_Control(CSYS_Handle *CSYS);
#define Motor_Init_Nnoe		0
#define Motor_Init_Finish	1
#endif
