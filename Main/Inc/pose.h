#ifndef __POSE_H_
#define __POSE_H_

#include "stm32f4xx_hal.h"
#include "my_pid.h"
#include "pid_control.h"
#include "my_motor.h"
#include "imu.h"
#include "csys.h"
#include "my_ramp.h"
#include "remote.h"
#include "posture.h"
void Pose_Master(void);
void Foot_Test_Ramp(void);
void Foot_Set(void);
void Foot_Reset(void);
void Fast_Walk(void);
void Stand_Jump(void);
void Front_Jump(void);
void Foot_Init(void);
void Foot_Ramp_Init(void);
void Foot_Whole_Jump(void);
void Foot_Rocker_Fast_Walk(const UART_DataPack *DataPack);
void Foot_Rocker_Slow_Walk(const UART_DataPack *DataPack);
void Foot_Rocker_Run(const UART_DataPack *DataPack);
void Foot_Rocker_Calandira(const UART_DataPack *DataPack);
#define Foot_State_None							0
#define Foot_State_Reset						1
#define Foot_State_Down 						2
#define Foot_State_Up 							3
#define Foot_State_Run_Ready     		4 
#define Foot_State_Jump_Up					5
#define Foot_State_Jump_Down				6
#define Foot_State_Front_Jump_Ready 7
#define Foot_State_Front_Jump_Up		8
#define Foot_State_Front_Jump_Down	9
#define Foot_State_Lengthen					10
#define Foot_State_Lift							11
#define Foot_State_Fall							12
#endif
