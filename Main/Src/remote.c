#include "remote.h"

uint8_t Remote_Receive_Data[42];		//双缓冲接收
Remote_State_Handle Remote_State;		//遥控器状态
UART_DataPack DataPack;
Remote_State_enum_Handle Remote_State_enum;

void Remote_State_Last_Current_Update(uint8_t Current_State_enmu)
{
	Remote_State.Remote_Last_State = Remote_State.Remote_Current_State;
	Remote_State.Remote_Current_State = Current_State_enmu;
}


/**
	*@breif 遥控器数据接收
	*/
void Remote_Receive()
{
	static uint8_t i;
	static uint16_t Need_vertify_crc;
	for(i=0;Remote_Receive_Data[i] != 0x5a && i < 21;i++);
	if(i!=21)
	{
		Need_vertify_crc = Remote_Receive_Data[i+19] | (Remote_Receive_Data[i+20] << 8);
		if(Need_vertify_crc == Verify_CRC16_Check_Sum(&Remote_Receive_Data[i],19))
		{
			memcpy(&DataPack,&Remote_Receive_Data[i],21);
			Remote_Deal();
		}
	}
	memcpy(Remote_Receive_Data,&Remote_Receive_Data[21],21);
	
}

/**
	*@breif 遥控器数据处理
	*/
void Remote_Deal()
{
	/* 保存上一次数据确保动作不重复 */
	static uint8_t Left_Key_Up_Memory;
	static uint8_t Left_Key_Down_Memory;
	static uint8_t Left_Key_Left_Memory;
	static uint8_t Left_Key_Right_Memory;
	static uint8_t Right_Key_Up_Memory;
	static uint8_t Right_Key_Down_Memory;
	static uint8_t Right_Key_Left_Memory;
	static uint8_t Right_Key_Right_Memory;
	static uint32_t time;
	if(DataPack.Key.Left_Key_Up == 1)
	{
		if(Remote_State.Remote_Current_State == Remote_Foot_None)
		{
			Remote_State_Last_Current_Update(Remote_Foot_Seesaw);
			Left_Key_Up_Memory = 1;
		}
		else if(Remote_State.Remote_Current_State == Remote_Foot_Seesaw && Left_Key_Up_Memory == 0)
		{
			Remote_State_Last_Current_Update(Remote_Foot_Seesaw_Stop);
		}
	}
	else if(DataPack.Key.Left_Key_Down == 1)
	{
		//暂不启用
//		if(Remote_State.Remote_Current_State == Remote_Foot_None)
//		{
//			Remote_State_Last_Current_Update(Remote_Foot_Step);
//		}
	}
	else if(DataPack.Key.Left_Key_Left == 1)
	{

		if(Remote_State.Remote_Current_State == Remote_Foot_None)
		{
			Remote_State_Last_Current_Update(Remote_Foot_Rocker_Fast_Walk);
		}
		
	}
	else if(DataPack.Key.Left_Key_Right == 1)
	{
		if(Remote_State.Remote_Current_State == Remote_Foot_None)
		{
			Remote_State_Last_Current_Update(Remote_Foot_Run_Ready);
			Left_Key_Right_Memory = 1;
		}
		else if(Remote_State.Remote_Current_State == Remote_Foot_Run_Ready && Left_Key_Right_Memory == 0)
		{
			Remote_State_Last_Current_Update(Remote_Foot_Cobble);
			Left_Key_Right_Memory = 1;
		}
		else if(Remote_State.Remote_Current_State == Remote_Foot_Cobble && Left_Key_Right_Memory == 0)
		{
			Remote_State_Last_Current_Update(Remote_Foot_Cobble_Stop);
		}
	}
	else if(DataPack.Key.Right_Key_Up == 1)												//初始化/归位 按键
	{
		
		//初始化操作
		if(Remote_State.Remote_Current_State == Remote_Foot_Init_None)
		{
			Remote_State_Last_Current_Update(Remote_Foot_Init_Start);
		}
		if(Remote_State.Remote_Current_State == Remote_Foot_None && Right_Key_Up_Memory == 0)
		{
			Remote_State_Last_Current_Update(Remote_Foot_Reset);
		}
		//急停操作
		switch(Remote_State.Remote_Current_State)
		{
			case Remote_Foot_Run_Ready:
			{
				Remote_State_Last_Current_Update(Remote_Foot_Emergency_Stop);
				Right_Key_Up_Memory = 1;
				break;
			}
			case Remote_Foot_Run:
			{
				Remote_State_Last_Current_Update(Remote_Foot_Emergency_Stop);
				Right_Key_Up_Memory = 1;
				break;
			}
			case Remote_Foot_Calandria:
			{
				Remote_State_Last_Current_Update(Remote_Foot_Emergency_Stop);
				Right_Key_Up_Memory = 1;
				break;
			}
			case Remote_Foot_Seesaw:
			{
				Remote_State_Last_Current_Update(Remote_Foot_Emergency_Stop);
				Right_Key_Up_Memory = 1;
				break;
			}
		}
	}
	else if(DataPack.Key.Right_Key_Down == 1)											//跳跃按键 跑完点后跳跃
	{
		if(Remote_State.Remote_Current_State == Remote_Foot_None)
		{
			Remote_State_Last_Current_Update(Remote_Foot_Jump);
		}
	}
	else if(DataPack.Key.Right_Key_Left == 1)											//跑按键		第一次按
	{
		if(Remote_State.Remote_Current_State == Remote_Foot_None)
		{
			Remote_State_Last_Current_Update(Remote_Foot_Run_Ready);
			Right_Key_Left_Memory = 1;															
		}
		else if(Remote_State.Remote_Current_State == Remote_Foot_Run_Ready && Right_Key_Left_Memory == 0)
		{
			Right_Key_Left_Memory = 1;
			Remote_State_Last_Current_Update(Remote_Foot_Run);
		}
		else if(Remote_State.Remote_Current_State == Remote_Foot_Run && Right_Key_Left_Memory == 0)
		{
			Remote_State_Last_Current_Update(Remote_Foot_Run_Stop);
		}
	}
	else if(DataPack.Key.Right_Key_Right == 1)
	{
		if(Remote_State.Remote_Current_State == Remote_Foot_None)
		{
			Remote_State_Last_Current_Update(Remote_Foot_Calandria);
		}
	}
	else if(DataPack.Key.Left_Switch == 1 && Remote_State.Remote_Current_State == Remote_Foot_None)
	{
		Remote_State_Last_Current_Update(Remote_Foot_Rocker_Fast_Walk);
	}
//	else if(DataPack.Key.Left_Switch == 2 && Remote_State.Remote_Current_State == Remote_Foot_None)
//	{
//		Remote_State_Last_Current_Update(Remote_Foot_Rocker_Run);
//	}
	else if(DataPack.Key.Left_Switch == 0 && (Remote_State.Remote_Current_State == Remote_Foot_Rocker_Fast_Walk || 
					Remote_State.Remote_Current_State == Remote_Foot_Rocker_Run))		
	{
		Remote_State_Last_Current_Update(Remote_Foot_Emergency_Stop);
	}
	else if(DataPack.Key.Right_Switch == 1 && Remote_State.Remote_Current_State == Remote_Foot_None)
	{
		Remote_State_Last_Current_Update(Remote_Foot_Rocker_Calandria);
	}
	else if(DataPack.Key.Right_Switch == 2 && Remote_State.Remote_Current_State == Remote_Foot_None)
	{
		Remote_State_Last_Current_Update(Remote_Foot_Rocker_Slow_Walk);
	}
	else if(DataPack.Key.Right_Switch == 0 && (Remote_State.Remote_Current_State == Remote_Foot_Rocker_Slow_Walk ||
					Remote_State.Remote_Current_State == Remote_Foot_Rocker_Calandria))
	{
		Remote_State_Last_Current_Update(Remote_Foot_Emergency_Stop);
	}
	else									
	{
		Left_Key_Up_Memory 			= 0;
		Left_Key_Down_Memory 		= 0;
		Left_Key_Left_Memory 		= 0;
		Left_Key_Right_Memory 	= 0;
		Right_Key_Up_Memory 		= 0;
		Right_Key_Down_Memory 	= 0;
		Right_Key_Left_Memory 	= 0;
		Right_Key_Right_Memory	= 0;
	}
	
}
