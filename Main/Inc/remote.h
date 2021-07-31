#ifndef __REMOTE_H
#define __REMOTE_H

#include "stm32f4xx_hal.h"
#include "CRC.h"
#include "string.h"


typedef struct 
{
	uint8_t Remote_Last_State;
	uint8_t Remote_Current_State;
}Remote_State_Handle;


typedef struct
{
  uint16_t Left_Key_Up : 1;
  uint16_t Left_Key_Down : 1;
  uint16_t Left_Key_Left : 1;
  uint16_t Left_Key_Right : 1;
  uint16_t Left_Rocker : 1;
  uint16_t Left_Encoder : 1;
  uint16_t Left_Switch : 2;
  uint16_t Right_Key_Up : 1;
  uint16_t Right_Key_Down : 1;
  uint16_t Right_Key_Left : 1;
  uint16_t Right_Key_Right : 1;
  uint16_t Right_Rocker : 1;
  uint16_t Right_Encoder : 1;
  uint16_t Right_Switch : 2;
} hw_key_t;

#pragma pack(1)
typedef struct {
  uint8_t head;
  uint16_t rocker[4];
  hw_key_t Key;
  int32_t Left_Encoder;
  int32_t Right_Encoder;
  uint16_t crc;
} UART_DataPack;
#pragma pack()


typedef enum 
{
	Remote_Foot_Init_None,				//未初始化
	Remote_Foot_Init_Start,				//开始初始化
	Remote_Foot_Init_Set,					//进入归零位置
	Remote_Foot_None,							//
	Remote_Foot_Stop,							//
	Remote_Foot_Reset,
	Remote_Foot_Emergency_Stop,		//急停
	Remote_Foot_Run_Ready,
	Remote_Foot_Run,
	Remote_Foot_Run_Stop,
	Remote_Foot_Step,
	Remote_Foot_Step_Stop,
	Remote_Foot_Seesaw,
	Remote_Foot_Seesaw_Stop,
	Remote_Foot_Wooden_Bridge,
	Remote_Foot_Rocker_Fast_Walk,
	Remote_Foot_Rocker_Slow_Walk,
	Remote_Foot_Rocker_Run,
	Remote_Foot_Rocker_Calandria,
	Remote_Foot_Wooden_Bridge_Stop,
	Remote_Foot_Calandria,
	Remote_Foot_Calandria_Stop,
	Remote_Foot_Cobble,
	Remote_Foot_Cobble_Stop,
	Remote_Foot_Turn_Left,
	Remote_Foot_Turn_Right,
	Remote_Foot_Walk,
	Remote_Foot_Ready_Jump,
	Remote_Foot_Jump,
	Remote_Foot_Jump_Down,
	Remote_Foot_Front_Foot_Move,
	Remote_Foot_Hind_Foot_Move
}Remote_State_enum_Handle;

extern uint8_t Remote_Receive_Data[42];
void Remote_Receive(void);
void Remote_Deal(void);
void Remote_State_Last_Current_Update(uint8_t Current_State_enmu);
#endif
