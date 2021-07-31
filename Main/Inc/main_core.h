#ifndef __MAIN_CORE_H_
#define __MAIN_CORE_H_


#include "stm32f4xx_hal.h"
#include "my_motor.h"
#include "my_pid.h"
#include "my_ramp.h"
#include "pose.h"
#include "imu.h"
#include "WatchDog.h"
#include "csys.h"
#include "CANDrive.h"
#include "remote.h"
#include "my_kalman.h"
extern uint8_t Input_Data[2][8];
extern uint8_t Return_C620_Data[8][8];
extern uint8_t Imu_Data[30];
void CAN_TX_Init(void);
void CanFilter_Init_Replace(CAN_HandleTypeDef *hcan);
void DMA_IT(void);
void UART_IT(void);
void Software_Reset(void);
#endif
