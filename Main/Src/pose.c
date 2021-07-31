#include "pose.h"
/* BEGIN EV */
extern uint8_t Return_Data[8][8];										//!<@brief 八个电调传回数据
extern uint8_t Input_Data[2][8];										//!<@brief CAN1、CAN2发送给电调的数据
extern uint8_t Imu_Data[30];
extern Remote_State_Handle Remote_State;						//!<@brief 遥控器控制四足状态位
extern Remote_State_enum_Handle Remote_State_enum;
extern IMU_Typedef IMU;
extern CSYS_Handle CSYS_Foot[4];
extern UART_DataPack DataPack;

#if 0
extern RM3508_TypeDef Motor_RM3508_Left_Front_Leg_Outside;
extern RM3508_TypeDef Motor_RM3508_Left_Front_Leg_Inside;		
extern RM3508_TypeDef Motor_RM3508_Right_Front_Leg_Outside;	
extern RM3508_TypeDef Motor_RM3508_Right_Front_Leg_Inside;		
extern RM3508_TypeDef Motor_RM3508_Left_Hind_Leg_Outside;		
extern RM3508_TypeDef Motor_RM3508_Left_Hind_Leg_Inside;		
extern RM3508_TypeDef Motor_RM3508_Right_Hind_Leg_Outside;	
extern RM3508_TypeDef Motor_RM3508_Right_Hind_Leg_Inside;
#endif

extern CAN_RxHeaderTypeDef CAN_RX;
extern CAN_TxHeaderTypeDef CAN1_TX;
extern CAN_TxHeaderTypeDef CAN2_TX;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_DataPack DataPack;
Ramp_Typedef Foot_test_ramp[4];
Ramp_Typedef Line_Left_ramp[4];
Ramp_Typedef Line_right_ramp[4];
Ramp_Typedef oval_ramp[4];
Ramp_Typedef Foot_Set_ramp[4];
Ramp_Typedef Foot_line_ramp[4];

Ramp_Typedef Foot_Run_Ready_ramp[4];
Ramp_Typedef Foot_Run_Buffer_ramp;
Ramp_Typedef Foot_Stand_Jump_Ready_ramp[4];
Ramp_Typedef Foot_Stand_Jump_Up_ramp[4];
Ramp_Typedef Foot_Stand_Jump_Down_ramp[4];

Ramp_Typedef Foot_Front_Jump_Ready_ramp[4];
Ramp_Typedef Foot_Front_Jump_Up_ramp[4];
Ramp_Typedef Foot_Front_Jump_Butter_ramp;
Ramp_Typedef Foot_Front_Jump_Down_ramp[4];

Ramp_Typedef Foot_Reset_ramp[4];
Ramp_Typedef Foot_Turn_Oval_ramp[4];
Ramp_Typedef Foot_Turn_Line_ramp[4];

Ramp_Typedef Foot_Lengthen_ramp[4];
Ramp_Typedef Foot_Wooden_Bridge_Up_ramp[4];
Ramp_Typedef Foot_Wooden_Bridge_Oval_ramp[4];
Ramp_Typedef Foot_Wooden_Bridge_Line_ramp[4];

Ramp_Typedef Foot_Step_Walk_Oval_ramp[4];
Ramp_Typedef Foot_Step_Walk_Line_ramp[4];
Ramp_Typedef Foot_Step_Lift_ramp[4];
Ramp_Typedef Foot_Step_Fall_ramp[4];
Ramp_Typedef Foot_Step_Walk_buffer_ramp;

Ramp_Typedef Foot_Calandria_Walk_Oval_ramp[4];
Ramp_Typedef Foot_Calandria_Walk_Line_ramp[4];
Ramp_Typedef Foot_Calandria_Walk_Buffer_ramp;
Ramp_Typedef Foot_Calandria_Lengthen_Buffer_ramp;
Ramp_Typedef Foot_Calandria_Span_Oval_ramp[4];
Ramp_Typedef Foot_Calandria_Span_Line_ramp[4];
Ramp_Typedef Foot_Calandria_Walk_Ready_ramp[4];

Ramp_Typedef Foot_Seesaw_Buffer_ramp;
Ramp_Typedef Foot_Seesaw_Line_Part_1_ramp[4];
Ramp_Typedef Foot_Seesaw_Line_Part_2_ramp[4];
Ramp_Typedef Foot_Seesaw_Line_Part_3_ramp[4];

Ramp_Typedef Foot_Run_Part_ramp[4];
Ramp_Typedef Foot_Rocker_Fast_Walk_Ready_ramp[4];
Ramp_Typedef Foot_Rocker_Fast_Walk_Up_ramp[4];
Ramp_Typedef Foot_Rocker_Fast_Walk_Down_ramp[4];

Ramp_Typedef Foot_Rocker_Stand_Up_ramp[4];
Ramp_Typedef Foot_Rocker_Translation_ramp[4];
Ramp_Typedef Foot_Rocker_Turn_ramp[4];
Ramp_Typedef Foot_Rocker_Slow_Walk_Ready_ramp[4];
Ramp_Typedef Foot_Rocker_Slow_Walk_Up_ramp[4];
Ramp_Typedef Foot_Rocker_Slow_Walk_Down_ramp[4];

Ramp_Typedef Foot_Rocker_Run_Ready_ramp[4];
Ramp_Typedef Foot_Rocker_Run_Up_ramp[4];
Ramp_Typedef Foot_Rocker_Run_Down_ramp[4];

Ramp_Typedef Foot_Rocker_Calandria_Ready_ramp[4];
Ramp_Typedef Foot_Rocker_Calandria_Up_ramp[4];
Ramp_Typedef Foot_Rocker_Calandria_Down_ramp[4];
Ramp_Typedef Foot_Rocker_Calandria_Jump_ramp[4];
Ramp_Typedef Foot_Rocker_Calandria_Turn_ramp[4];
Ramp_Typedef Foot_Rocker_Calandria_Buffer_ramp;
Ramp_Global_Typedef Foot_Rocker_Slow_Walk_Down_Global_ramp[4];

Ramp_Typedef Foot_Walk_Test_ramp[4];
/* END EV */

//Foot_State CSYS_Foot[4];

float x;
static uint8_t i;
uint8_t test_flag;
uint8_t posture_flag[4];

/**
	*@brief 四足动作控制中心 
	*/
void Pose_Master()
{
	static uint8_t first_flag;
	switch(Remote_State.Remote_Current_State)
	{
		case Remote_Foot_Init_Set:
		{
			Foot_Set();
			break;
		}
		case Remote_Foot_Run_Ready:
		{
			if(!first_flag)
			{
				first_flag = 1;
				CSYS_Foot[0].Foot_State = Foot_State_Run_Ready;
				CSYS_Foot[1].Foot_State = Foot_State_Run_Ready;
				CSYS_Foot[2].Foot_State = Foot_State_Run_Ready;
				CSYS_Foot[3].Foot_State = Foot_State_Run_Ready;
			}
			Foot_Run_Ready();
			break;
		}
		case Remote_Foot_Run:
		{
			Foot_Run();
			break;
		}
		case Remote_Foot_Run_Stop:
		{
			if(!first_flag)
				first_flag = 1;
			Foot_Run();
			break;
		}
		case Remote_Foot_Rocker_Fast_Walk:
		{
			Foot_Rocker_Fast_Walk(&DataPack);
			break;
		}
		case Remote_Foot_Rocker_Slow_Walk:
		{
			Foot_Rocker_Slow_Walk(&DataPack);
			break;
		}
//		case Remote_Foot_Rocker_Run:
//		{
//			Foot_Rocker_Run(&DataPack);
//			break;
//		}
		case Remote_Foot_Rocker_Calandria:
		{
			Foot_Rocker_Calandira(&DataPack);
			break;
		}
		case Remote_Foot_Cobble:
		{
			Foot_New_Run();
			break;
		}
		case Remote_Foot_Cobble_Stop:
		{
			Foot_New_Run();
			break;
		}
		case Remote_Foot_Ready_Jump:
		{
			if(!first_flag)
			{
				first_flag = 1;
				CSYS_Foot[0].Foot_State = Foot_State_Front_Jump_Ready;
				CSYS_Foot[1].Foot_State = Foot_State_Front_Jump_Ready;
				CSYS_Foot[2].Foot_State = Foot_State_Front_Jump_Ready;
				CSYS_Foot[3].Foot_State = Foot_State_Front_Jump_Ready;
			}
			Front_Ready_Jump();
			break;
		}
		case Remote_Foot_Jump:
		{
			Front_Jump();
			break;
		}
		case Remote_Foot_Walk:
		{
			if(!first_flag)
			{
				first_flag = 1;
				CSYS_Foot[1].Foot_State = Foot_State_Up;
				CSYS_Foot[2].Foot_State = Foot_State_Up;
				CSYS_Foot[0].Foot_State = Foot_State_Down;
				CSYS_Foot[3].Foot_State = Foot_State_Down;
			}
			Free_Front_Walk();
			break;
		}
		case Remote_Foot_Turn_Left:
		{
			if(!first_flag)
			{
				first_flag = 1;
				CSYS_Foot[0].Foot_State = Foot_State_Up;
				CSYS_Foot[3].Foot_State = Foot_State_Up;
				CSYS_Foot[1].Foot_State = Foot_State_Down;
				CSYS_Foot[2].Foot_State = Foot_State_Down;
			}
			Foot_Turn_Left();
			break;
		}
		case Remote_Foot_Turn_Right:
		{
			if(!first_flag)
			{
				first_flag = 1;
				CSYS_Foot[1].Foot_State = Foot_State_Up;
				CSYS_Foot[2].Foot_State = Foot_State_Up;
				CSYS_Foot[0].Foot_State = Foot_State_Down;
				CSYS_Foot[3].Foot_State = Foot_State_Down;
			}
			Foot_Turn_Right();
			break;
		}
		case Remote_Foot_None:
		{
			first_flag = 0;
			break;
		}
		case Remote_Foot_Reset:
		{
			Foot_Reset();
			break;
		}
		case Remote_Foot_Seesaw:
		{
			Foot_Seesaw();
			break;
		}
		case Remote_Foot_Seesaw_Stop:
		{
			Foot_Seesaw();
			break;
		}
		case Remote_Foot_Calandria:
		{
			if(!first_flag)
			{
				first_flag = 1;
				CSYS_Foot[1].Foot_State = Foot_State_Run_Ready;
				CSYS_Foot[2].Foot_State = Foot_State_Run_Ready;
				CSYS_Foot[0].Foot_State = Foot_State_Run_Ready;
				CSYS_Foot[3].Foot_State = Foot_State_Run_Ready;
			}
			Foot_Calandria();
			break;
		}
		case Remote_Foot_Emergency_Stop:
		{
			switch(Remote_State.Remote_Last_State)
			{
				case Remote_Foot_Calandria:
				{
					Foot_Calandria();
					break;
				}
				case Remote_Foot_Cobble:
				{
					Foot_New_Run();
					break;
				}
				case Remote_Foot_Run_Ready:
				{
					Foot_Run_Ready();
					break;
				}
				case Remote_Foot_Run:
				{
					Foot_Run();
					break;
				}
				case Remote_Foot_Seesaw:
				{
					Foot_Seesaw();
					break;
				}
				case Remote_Foot_Step:
				{
					Foot_Step();
					break;
				}
				case Remote_Foot_Wooden_Bridge:
				{
					Foot_Wooden_Bridge();
					break;
				}
				case Remote_Foot_Rocker_Fast_Walk:
				{
					Foot_Rocker_Fast_Walk(&DataPack);
					break;
				}
				case Remote_Foot_Rocker_Slow_Walk:
				{
					Foot_Rocker_Slow_Walk(&DataPack);
					break;
				}
//				case Remote_Foot_Rocker_Run:
//				{
//					Foot_Rocker_Run(&DataPack);
//					break;
//				}
				case Remote_Foot_Rocker_Calandria:
				{
					Foot_Rocker_Calandira(&DataPack);
					break;
				}
				default:
				{
					break;
				}
			}
			break;
		}
	}
	
}
void Foot_Ramp_Init()
{
	for(i=0;i<4;i++)
	{
		Foot_Set_ramp[i].RampTime = 2000;
		Foot_line_ramp[i].RampTime = 1000;
		oval_ramp[i].RampTime = 2000;
		Foot_Run_Ready_ramp[i].RampTime = 1000;
		Line_Left_ramp[i].RampTime = 200;
		Line_right_ramp[i].RampTime = 200;
		Foot_Stand_Jump_Ready_ramp[i].RampTime = 200;
		Foot_Reset_ramp[i].RampTime = 1000;
		Foot_Front_Jump_Ready_ramp[i].RampTime = 250;
		Foot_Front_Jump_Down_ramp[i].RampTime = 1500;
		Foot_Turn_Oval_ramp[i].RampTime = 200;
		Foot_Turn_Line_ramp[i].RampTime = 200;
		Foot_Lengthen_ramp[i].RampTime = 1000;
		Foot_test_ramp[i].RampTime = 2000;
		Foot_Step_Walk_Oval_ramp[i].RampTime = 300;
		Foot_Step_Walk_Line_ramp[i].RampTime = 300;
		Foot_Step_Lift_ramp[i].RampTime = 800;
		Foot_Step_Fall_ramp[i].RampTime = 800;
		Foot_Calandria_Walk_Oval_ramp[i].RampTime = 2000;
		Foot_Calandria_Walk_Line_ramp[i].RampTime = 1000;
		Foot_Calandria_Span_Oval_ramp[i].RampTime = 3000;
		Foot_Calandria_Span_Line_ramp[i].RampTime = 1500;
		Foot_Calandria_Walk_Ready_ramp[i].RampTime = 1000;
		Foot_Rocker_Slow_Walk_Ready_ramp[i].RampTime = 1000;
		Foot_Rocker_Slow_Walk_Up_ramp[i].RampTime = 250;
		Foot_Rocker_Slow_Walk_Down_ramp[i].RampTime = 750;	//实际上时间为time*3
		Foot_Rocker_Slow_Walk_Down_Global_ramp[i].Ramp = &Foot_Rocker_Slow_Walk_Down_ramp[i];
		Foot_Rocker_Stand_Up_ramp[i].RampTime = 250;
		Foot_Seesaw_Line_Part_1_ramp[i].RampTime = 1000;
		Foot_Seesaw_Line_Part_2_ramp[i].RampTime = 1000;
		Foot_Seesaw_Line_Part_3_ramp[i].RampTime = 1000;
		Foot_Run_Part_ramp[i].RampTime = 4000;
		Foot_Rocker_Turn_ramp[i].RampTime = 200;
		Foot_Rocker_Translation_ramp[i].RampTime = 200;
		Foot_Rocker_Fast_Walk_Ready_ramp[i].RampTime = 1000;
		Foot_Rocker_Fast_Walk_Up_ramp[i].RampTime = 250;				//250
		Foot_Rocker_Fast_Walk_Down_ramp[i].RampTime = 250;
		Foot_Rocker_Run_Ready_ramp[i].RampTime = 1000;
		Foot_Rocker_Run_Up_ramp[i].RampTime = 250;
		Foot_Rocker_Run_Down_ramp[i].RampTime = 125;
		Foot_Rocker_Calandria_Ready_ramp[i].RampTime = 500;
		Foot_Rocker_Calandria_Up_ramp[i].RampTime = 250;
		Foot_Rocker_Calandria_Down_ramp[i].RampTime = 250;
		Foot_Rocker_Calandria_Jump_ramp[i].RampTime = 400;
	}
	Foot_Front_Jump_Butter_ramp.RampTime = 100;
	Foot_Rocker_Calandria_Buffer_ramp.RampTime = 100;
	Foot_Calandria_Lengthen_Buffer_ramp.RampTime = 1000;
	Foot_Seesaw_Buffer_ramp.RampTime = 100;
	Foot_Run_Buffer_ramp.RampTime = 100;
	Foot_Step_Walk_buffer_ramp.RampTime = 100;
	Foot_Calandria_Walk_Buffer_ramp.RampTime = 100;
	
}
#if 0
void Foot_Line()
{
#if 1
//	x = 100.0f;
	CSYS = Foot_Line_RTO_Get_Theta_Ramp(-50.0f,&Line_Left_ramp);
	Motor_RM3508[0].Infer_Real_Angle = 180.0f+(50.0f-CSYS.theta1);
	Motor_RM3508[1].Infer_Real_Angle = (50.0f - CSYS.theta2);
#else
	Motor_RM3508_Left_Front_Leg_Outside.Infer_Real_Angle = 180.0f+(50.0f-CSYS.CSYS1);
	Motor_RM3508_Left_Front_Leg_Inside.Infer_Real_Angle = -(50.0f - CSYS.CSYS2);
#endif
	
}
#endif
/**
	*@brief 初始化位置环初始化
	*/
void Foot_Init()
{
	
#if 1
//	Motor_RM3508[0].Infer_Real_Angle = 180.0f;
//	Motor_RM3508[2].Infer_Real_Angle = -180.0f;
//	Motor_RM3508[5].Infer_Real_Angle = 180.0f;
//	Motor_RM3508[7].Infer_Real_Angle = -180.0f;
#else
	Motor_RM3508_Left_Front_Leg_Outside.Infer_Real_Angle = 180.0f;
	Motor_RM3508_Right_Front_Leg_Outside.Infer_Real_Angle = -180.0f;
	Motor_RM3508_Left_Hind_Leg_Inside.Infer_Real_Angle = 180.0f;
	Motor_RM3508_Right_Hind_Leg_Inside.Infer_Real_Angle = -180.0f;
#endif
//	CSYS_Foot[0].Foot_State = Foot_State_Down;
//	CSYS_Foot[3].Foot_State = Foot_State_Down;
//	CSYS_Foot[1].Foot_State = Foot_State_Down;
//	CSYS_Foot[2].Foot_State = Foot_State_Up;
//	CSYS_Foot[0].Foot_State = Foot_State_Front_Jump_Ready;
//	CSYS_Foot[1].Foot_State = Foot_State_Front_Jump_Ready;
//	CSYS_Foot[2].Foot_State = Foot_State_Front_Jump_Ready;
//	CSYS_Foot[3].Foot_State = Foot_State_Front_Jump_Ready;
//	CSYS_Foot[0].Foot_State = Foot_State_Run_Ready;
//	CSYS_Foot[3].Foot_State = Foot_State_Run_Ready;
//	CSYS_Foot[1].Foot_State = Foot_State_Run_Ready;
//	CSYS_Foot[2].Foot_State = Foot_State_Run_Ready;
}


float Test_Infer_Real_Angle;
void Foot_Test_Ramp()
{
	static uint8_t first_flag;
	static float Init_Angle_1;
	static float Init_Angle_2;
	static float ramp_t;
	if(!first_flag)
	{
		Init_Angle_1 = Motor_RM3508_Front_Right_Outside_Test.Real_Angle;
		Init_Angle_2 = Motor_RM3508_Front_Right_Inside_Test.Real_Angle;
//		Init_Angle_1 = Motor_RM3508[2].Real_Angle;
//		Init_Angle_2 = Motor_RM3508[3].Real_Angle;
		first_flag = 1;
	}
	ramp_t = Slope(&Foot_test_ramp[1]);
	
	Motor_RM3508_Front_Right_Outside_Test.Infer_Real_Angle = ((Test_Infer_Real_Angle - Init_Angle_1) * ramp_t) + Init_Angle_1;
	Motor_RM3508_Front_Right_Inside_Test.Infer_Real_Angle = ((Test_Infer_Real_Angle - Init_Angle_2) * ramp_t) + Init_Angle_2;
//	Motor_RM3508[2].Infer_Real_Angle = ((Test_Infer_Real_Angle - Init_Angle_1) * ramp_t) + Init_Angle_1;
//	Motor_RM3508[3].Infer_Real_Angle = ((Test_Infer_Real_Angle - Init_Angle_2) * ramp_t) + Init_Angle_2;
	if(ramp_t == 1.0f)
	{
//		Motor_RM3508[2].Infer_Real_Angle = ((Test_Infer_Real_Angle - Init_Angle_1) * ramp_t) + Init_Angle_1;
//		Motor_RM3508[3].Infer_Real_Angle = ((Test_Infer_Real_Angle - Init_Angle_2) * ramp_t) + Init_Angle_2;
		Motor_RM3508_Front_Right_Outside_Test.Infer_Real_Angle = ((Test_Infer_Real_Angle - Init_Angle_1) * ramp_t) + Init_Angle_1;
		Motor_RM3508_Front_Right_Inside_Test.Infer_Real_Angle = ((Test_Infer_Real_Angle - Init_Angle_2) * ramp_t) + Init_Angle_2;
		ResetSlope(&Foot_test_ramp[1]);
		test_flag = 1;
		first_flag = 0;
	}
}


void Foot_Set()
{
	Motor_RM3508[0].Infer_Real_Angle = (148.9f)  * Slope(&Foot_Set_ramp[0]);
	Motor_RM3508[1].Infer_Real_Angle = (8.9f)		 * Slope(&Foot_Set_ramp[0]);
	Motor_RM3508[2].Infer_Real_Angle = (-148.9f) * Slope(&Foot_Set_ramp[1]);
	Motor_RM3508[3].Infer_Real_Angle = (-8.9f)	 * Slope(&Foot_Set_ramp[1]);
	Motor_RM3508[4].Infer_Real_Angle = (10.9f)	 * Slope(&Foot_Set_ramp[2]);
	Motor_RM3508[5].Infer_Real_Angle = (146.9f)  * Slope(&Foot_Set_ramp[2]);
	Motor_RM3508[6].Infer_Real_Angle = (-10.9f)	 * Slope(&Foot_Set_ramp[3]);
	Motor_RM3508[7].Infer_Real_Angle = (-146.9f) * Slope(&Foot_Set_ramp[3]);
	if(Slope(&Foot_Set_ramp[0]) == 1.0f && Slope(&Foot_Set_ramp[1]) == 1.0f &&
		 Slope(&Foot_Set_ramp[2]) == 1.0f && Slope(&Foot_Set_ramp[3]) == 1.0f)
	{
		Motor_RM3508[0].Infer_Real_Angle = (148.9f)  * Slope(&Foot_Set_ramp[0]);
		Motor_RM3508[1].Infer_Real_Angle = (8.9f)		 * Slope(&Foot_Set_ramp[0]);
		Motor_RM3508[2].Infer_Real_Angle = (-148.9f) * Slope(&Foot_Set_ramp[1]);
		Motor_RM3508[3].Infer_Real_Angle = (-8.9f)	 * Slope(&Foot_Set_ramp[1]);
		Motor_RM3508[4].Infer_Real_Angle = (10.9f)	 * Slope(&Foot_Set_ramp[2]);
		Motor_RM3508[5].Infer_Real_Angle = (146.9f)  * Slope(&Foot_Set_ramp[2]);
		Motor_RM3508[6].Infer_Real_Angle = (-10.9f)	 * Slope(&Foot_Set_ramp[3]);
		Motor_RM3508[7].Infer_Real_Angle = (-146.9f) * Slope(&Foot_Set_ramp[3]);
		Remote_State_Last_Current_Update(Remote_Foot_None);
		for(i=0;i<4;i++)
			ResetSlope(&Foot_Set_ramp[i]);
	}
//	Motor_RM3508[0].Infer_Real_Angle = (180.0f)  * Slope(&Foot_Set_ramp[0]);
//	Motor_RM3508[2].Infer_Real_Angle = (-180.0f) * Slope(&Foot_Set_ramp[1]);
//	Motor_RM3508[5].Infer_Real_Angle = (180.0f)  * Slope(&Foot_Set_ramp[2]);
//	Motor_RM3508[7].Infer_Real_Angle = (-180.0f) * Slope(&Foot_Set_ramp[3]);
}

/**
	*@brief 归位
	*/
void Foot_Reset()
{
	for(i=0;i<4;i++)
	{
		Foot_Point_RTO_Ramp(0,0,&CSYS_Foot[i],&Foot_Reset_ramp[i]);
		Foot_Angle_Control(&CSYS_Foot[i]);
	}
	if(Slope(&Foot_Reset_ramp[0]) == 1.0f && Slope(&Foot_Reset_ramp[1]) == 1.0f &&
		 Slope(&Foot_Reset_ramp[2]) == 1.0f && Slope(&Foot_Reset_ramp[3]) == 1.0f)
	{
		for(i=0;i<4;i++)
		{
			Foot_Point_RTO_Ramp(0,0,&CSYS_Foot[i],&Foot_Reset_ramp[i]);
			Foot_Angle_Control(&CSYS_Foot[i]);
			ResetSlope(&Foot_Reset_ramp[i]);
			CSYS_Foot[i].Foot_State = Foot_State_None;
		}
		Remote_State_Last_Current_Update(Remote_Foot_None);
	}
}

void Foot_Start_Walk()
{
	for(i=0;i<4;i++)
	{
		Foot_Point_RTO(-60.0f,0,&CSYS_Foot[i]);
		Foot_Angle_Control(&CSYS_Foot[i]);
	}
	
}

/**
	*@brief 跑步准备
	*/
void Foot_Run_Ready()
{
	if(Remote_State.Remote_Current_State != Remote_Foot_Emergency_Stop)
	{
		if(CSYS_Foot[0].Foot_State == Foot_State_Run_Ready && CSYS_Foot[1].Foot_State == Foot_State_Run_Ready &&
			 CSYS_Foot[2].Foot_State == Foot_State_Run_Ready && CSYS_Foot[3].Foot_State == Foot_State_Run_Ready   )
		{
			for(i=0;i<4;i++)
			{
	//			Foot_Line_NRTO_Ramp(-60.0f,0,&CSYS_Foot[i],&Foot_Run_Ready_ramp[i]);
				Foot_Point_RTO_Ramp(-60.0f,0,&CSYS_Foot[i],&Foot_Run_Ready_ramp[i]);
				Foot_Angle_Control(&CSYS_Foot[i]);
			}
			if(Slope(&Foot_Run_Ready_ramp[0]) == 1.0f && Slope(&Foot_Run_Ready_ramp[1]) == 1.0f &&
				 Slope(&Foot_Run_Ready_ramp[2]) == 1.0f && Slope(&Foot_Run_Ready_ramp[3]) == 1.0f   )
			{
				for(i=0;i<4;i++)
				{
					Foot_Line_NRTO_Ramp(-60.0f,0,&CSYS_Foot[i],&Foot_Run_Ready_ramp[i]);
					Foot_Point_RTO_Ramp(-60.0f,0,&CSYS_Foot[i],&Foot_Run_Ready_ramp[i]);
					Foot_Angle_Control(&CSYS_Foot[i]);
					ResetSlope(&Foot_Run_Ready_ramp[i]);
				}
				CSYS_Foot[1].Foot_State = Foot_State_Up;
				CSYS_Foot[2].Foot_State = Foot_State_Up;
				CSYS_Foot[0].Foot_State = Foot_State_Down;
				CSYS_Foot[3].Foot_State = Foot_State_Down;
			}
		}
	}
	else
	{
		for(i=0;i<4;i++)
		{
			CSYS_Foot[i].Foot_State = Foot_State_None;
			ResetSlope(&Foot_Run_Ready_ramp[i]);
		}
		Remote_State_Last_Current_Update(Remote_Foot_None);
	}
}

/**
	*@brief 跑
	*/
void Foot_Run()
{
	
	static uint8_t first_flag;					//第一次左后腿和右前腿动 其余两腿均不动
	static uint8_t buffer_flag;
	static float oval_x = 30.0f;				//椭圆长轴a	//60.f
	static float oval_y = 50.0f;				//椭圆短轴b
	static float oval_y_2 = 70.0f;				//
	static float Line_Back = -60.0f;		//归位路径值
	if(Remote_State.Remote_Current_State != Remote_Foot_Emergency_Stop)
	{
		if(	CSYS_Foot[2].Foot_State == Foot_State_Up && CSYS_Foot[1].Foot_State == Foot_State_Up &&
				CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
		{
			if(!buffer_flag)
			{
				Foot_Front_Up_Oval_RTO_Ramp(-oval_x,0,oval_x,oval_y,&CSYS_Foot[1],&oval_ramp[1]);
	//			Foot_Walk_Posture(&CSYS_Global[1],&CSYS_Foot[1],&IMU,&posture_flag[1]);
				Foot_Angle_Control(&CSYS_Foot[1]);
				Foot_Front_Up_Oval_RTO_Ramp(-oval_x,0,oval_x,oval_y,&CSYS_Foot[2],&oval_ramp[2]);
	//			Foot_Walk_Posture(&CSYS_Global[2],&CSYS_Foot[2],&IMU,&posture_flag[2]);
				Foot_Angle_Control(&CSYS_Foot[2]);
	//			Foot_Front_Up_Oval_RTO_Ramp(-oval_x,0,oval_x,oval_y,&CSYS_Foot[1],&oval_ramp[1]);
	//			Foot_Angle_Control(&CSYS_Foot[1]);
	//			Foot_Front_Up_Oval_RTO_Ramp(-oval_x,0,oval_x,oval_y,&CSYS_Foot[2],&oval_ramp[2]);
	//			Foot_Angle_Control(&CSYS_Foot[2]);
				if(first_flag == 1)
				{
					Foot_Round_X_NRTO_ramp(Line_Back,&CSYS_Foot[0],&Foot_line_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_Round_X_NRTO_ramp(Line_Back,&CSYS_Foot[3],&Foot_line_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
	//					Foot_Line_NRTO_Ramp(Line_Back,0,&CSYS_Foot[0],&Foot_line_ramp[0]);
	//					Foot_Angle_Control(&CSYS_Foot[0]);
	//					Foot_Line_NRTO_Ramp(Line_Back,0,&CSYS_Foot[3],&Foot_line_ramp[3]);
	//					Foot_Angle_Control(&CSYS_Foot[3]);
				}
				if(Slope(&oval_ramp[1]) == 1.0f && Slope(&oval_ramp[2]) == 1.0f)
				{
					Foot_Front_Up_Oval_RTO_Ramp(-oval_x,0,oval_x,oval_y,&CSYS_Foot[1],&oval_ramp[1]);
	//				Foot_Walk_Posture(&CSYS_Global[1],&CSYS_Foot[1],&IMU,&posture_flag[1]);
					Foot_Angle_Control(&CSYS_Foot[1]);
					Foot_Front_Up_Oval_RTO_Ramp(-oval_x,0,oval_x,oval_y,&CSYS_Foot[2],&oval_ramp[2]);
	//				Foot_Walk_Posture(&CSYS_Global[2],&CSYS_Foot[2],&IMU,&posture_flag[2]);
					Foot_Angle_Control(&CSYS_Foot[2]);
					if(first_flag == 1)
					{
						Foot_Round_X_NRTO_ramp(Line_Back,&CSYS_Foot[0],&Foot_line_ramp[0]);
						Foot_Angle_Control(&CSYS_Foot[0]);
						Foot_Round_X_NRTO_ramp(Line_Back,&CSYS_Foot[3],&Foot_line_ramp[3]);
						Foot_Angle_Control(&CSYS_Foot[3]);
						
	//					Foot_Line_NRTO_Ramp(Line_Back,0,&CSYS_Foot[0],&Foot_line_ramp[0]);
	//					Foot_Angle_Control(&CSYS_Foot[0]);
	//					Foot_Line_NRTO_Ramp(Line_Back,0,&CSYS_Foot[3],&Foot_line_ramp[3]);
	//					Foot_Angle_Control(&CSYS_Foot[3]);
					}
					ResetFlag(&posture_flag[1]);
					ResetFlag(&posture_flag[2]);
					buffer_flag = 1;
					ResetSlope(&Foot_line_ramp[0]);
					ResetSlope(&Foot_line_ramp[3]);
					ResetSlope(&oval_ramp[1]);
					ResetSlope(&oval_ramp[2]);
				}
			}
			else
			{
				if(Slope(&Foot_Run_Buffer_ramp) == 1.0f)
				{
					buffer_flag = 0;
					CSYS_Foot[0].Foot_State = Foot_State_Up;
					CSYS_Foot[3].Foot_State = Foot_State_Up;
					CSYS_Foot[1].Foot_State = Foot_State_Down;
					CSYS_Foot[2].Foot_State = Foot_State_Down;
					ResetSlope(&Foot_Run_Buffer_ramp);
				}
			}
		}
		else if(CSYS_Foot[2].Foot_State == Foot_State_Down && CSYS_Foot[1].Foot_State == Foot_State_Down 
					&& CSYS_Foot[0].Foot_State == Foot_State_Up && CSYS_Foot[3].Foot_State == Foot_State_Up)
		{
			if(!buffer_flag)				//初始为buffer_flag
			{
				if(!first_flag)
					first_flag = 1;
				Foot_Front_Up_Oval_RTO_Ramp(-oval_x,0,oval_x,oval_y,&CSYS_Foot[0],&oval_ramp[0]);
	//			Foot_Walk_Posture(&CSYS_Global[0],&CSYS_Foot[0],&IMU,&posture_flag[0]);
				Foot_Angle_Control(&CSYS_Foot[0]);
				Foot_Front_Up_Oval_RTO_Ramp(-oval_x,0,oval_x,oval_y,&CSYS_Foot[3],&oval_ramp[3]);
	//			Foot_Walk_Posture(&CSYS_Global[3],&CSYS_Foot[3],&IMU,&posture_flag[3]);
				Foot_Angle_Control(&CSYS_Foot[3]);
				
	//			Foot_Front_Up_Oval_RTO_Ramp(-oval_x,0,oval_x,oval_y,&CSYS_Foot[0],&oval_ramp[0]);
	//			Foot_Angle_Control(&CSYS_Foot[0]);
	//			Foot_Front_Up_Oval_RTO_Ramp(-oval_x,0,oval_x,oval_y,&CSYS_Foot[3],&oval_ramp[3]);
	//			Foot_Angle_Control(&CSYS_Foot[3]);
				
				Foot_Round_X_NRTO_ramp(Line_Back,&CSYS_Foot[1],&Foot_line_ramp[1]);
				Foot_Angle_Control(&CSYS_Foot[1]);
				Foot_Round_X_NRTO_ramp(Line_Back,&CSYS_Foot[2],&Foot_line_ramp[2]);
				Foot_Angle_Control(&CSYS_Foot[2]);
	//			Foot_Line_NRTO_Ramp(Line_Back,0,&CSYS_Foot[1],&Foot_line_ramp[1]);
	//			Foot_Angle_Control(&CSYS_Foot[1]);
	//			Foot_Line_NRTO_Ramp(Line_Back,0,&CSYS_Foot[2],&Foot_line_ramp[2]);
	//			Foot_Angle_Control(&CSYS_Foot[2]);

				if(Slope(&oval_ramp[0]) == 1.0f && Slope(&oval_ramp[3]) == 1.0f 
						&& Slope(&Foot_line_ramp[1]) == 1.0f && Slope(&Foot_line_ramp[2]) == 1.0f)
				{
					buffer_flag = 1;
					Foot_Front_Up_Oval_RTO_Ramp(-oval_x,0,oval_x,oval_y,&CSYS_Foot[0],&oval_ramp[0]);
	//				Foot_Walk_Posture(&CSYS_Global[0],&CSYS_Foot[0],&IMU,&posture_flag[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_Front_Up_Oval_RTO_Ramp(-oval_x,0,oval_x,oval_y,&CSYS_Foot[3],&oval_ramp[3]);
	//				Foot_Walk_Posture(&CSYS_Global[3],&CSYS_Foot[3],&IMU,&posture_flag[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
					
	//				Foot_Front_Up_Oval_RTO_Ramp(-oval_x,0,oval_x,oval_y,&CSYS_Foot[0],&oval_ramp[0]);
	//				Foot_Angle_Control(&CSYS_Foot[0]);
	//				Foot_Front_Up_Oval_RTO_Ramp(-oval_x,0,oval_x,oval_y,&CSYS_Foot[3],&oval_ramp[3]);
	//				Foot_Angle_Control(&CSYS_Foot[3]);
					
					Foot_Round_X_NRTO_ramp(Line_Back,&CSYS_Foot[1],&Foot_line_ramp[1]);
					Foot_Angle_Control(&CSYS_Foot[1]);
					Foot_Round_X_NRTO_ramp(Line_Back,&CSYS_Foot[2],&Foot_line_ramp[2]);
					Foot_Angle_Control(&CSYS_Foot[2]);
	//			Foot_Line_NRTO_Ramp(Line_Back,0,&CSYS_Foot[1],&Foot_line_ramp[1]);
	//			Foot_Angle_Control(&CSYS_Foot[1]);
	//			Foot_Line_NRTO_Ramp(Line_Back,0,&CSYS_Foot[2],&Foot_line_ramp[2]);
	//			Foot_Angle_Control(&CSYS_Foot[2]);
					ResetFlag(&posture_flag[0]);
					ResetFlag(&posture_flag[3]);
					ResetSlope(&Foot_line_ramp[1]);
					ResetSlope(&Foot_line_ramp[2]);
					ResetSlope(&oval_ramp[0]);
					ResetSlope(&oval_ramp[3]);
				}
			}
			else
			{
				if(Slope(&Foot_Run_Buffer_ramp) == 1.0f)
				{
					ResetFlag(&buffer_flag);
					ResetSlope(&Foot_Run_Buffer_ramp);
					if(Remote_State.Remote_Current_State == Remote_Foot_Run_Stop &&
						 Remote_State.Remote_Last_State		 == Remote_Foot_Run				)
					{
						ResetFlag(&first_flag);
						Remote_State_Last_Current_Update(Remote_Foot_None);
						CSYS_Foot[0].Foot_State = Foot_State_None;
						CSYS_Foot[1].Foot_State = Foot_State_None;
						CSYS_Foot[2].Foot_State = Foot_State_None;
						CSYS_Foot[3].Foot_State = Foot_State_None;
					}
					else
					{
						CSYS_Foot[1].Foot_State = Foot_State_Up;
						CSYS_Foot[2].Foot_State = Foot_State_Up;
						CSYS_Foot[0].Foot_State = Foot_State_Down;
						CSYS_Foot[3].Foot_State = Foot_State_Down;
					}
				}
			}
		}
	}
	else
	{
		ResetFlag(&first_flag);
		ResetFlag(&buffer_flag);
		for(i=0;i<4;i++)
		{
			CSYS_Foot[i].Foot_State = Foot_State_None;
			ResetSlope(&oval_ramp[i]);
			ResetSlope(&Foot_line_ramp[i]);
		}
		ResetSlope(&Foot_Run_Buffer_ramp);
		Remote_State_Last_Current_Update(Remote_Foot_None);
	}
}

void Foot_Rocker_Fast_Walk(const UART_DataPack *DataPack)
{
	static uint16_t initial_Rocker_L_X;
	static uint16_t initial_Rocker_L_Y;
	static uint16_t initial_Rocker_R_X;
	static uint16_t initial_Rocker_R_Y;
	static uint8_t circle_flag;
	static uint8_t Part_State;
	static uint8_t start_flag;
	static uint8_t Current_flag;
	static uint8_t state;
	if(Remote_State.Remote_Current_State != Remote_Foot_Emergency_Stop)
	{
		static float initial_x = -60.0f;			
		static float initial_y = 0;
		if(!start_flag)
		{
			for(i=0;i<4;i++)
				CSYS_Foot[i].Foot_State = Foot_State_Run_Ready;
			start_flag = 1;
		}
		if(Part_State == 0)
		{
			for(i=0;i<4;i++)
			{
				Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[i],&Foot_Rocker_Fast_Walk_Ready_ramp[i]);
				Foot_Angle_Control(&CSYS_Foot[i]);
			}
			if(Slope(&Foot_Rocker_Fast_Walk_Ready_ramp[0]) == 1.0f && Slope(&Foot_Rocker_Fast_Walk_Ready_ramp[1]) == 1.0f &&
				 Slope(&Foot_Rocker_Fast_Walk_Ready_ramp[2]) == 1.0f && Slope(&Foot_Rocker_Fast_Walk_Ready_ramp[3]) == 1.0f)
			{
				for(i=0;i<4;i++)
					ResetSlope(&Foot_Rocker_Fast_Walk_Ready_ramp[i]);
				Part_State = 1;
				CSYS_Foot[1].Foot_State = Foot_State_Up;
				CSYS_Foot[2].Foot_State = Foot_State_Up;
				CSYS_Foot[0].Foot_State = Foot_State_Down;
				CSYS_Foot[3].Foot_State = Foot_State_Down;
			}
		}
		else if(Part_State == 1)
		{
			if(!circle_flag)
			{
				initial_Rocker_L_X = DataPack->rocker[0];
				initial_Rocker_L_Y = DataPack->rocker[1];
				initial_Rocker_R_X = DataPack->rocker[2];
				initial_Rocker_R_Y = DataPack->rocker[3];
				circle_flag = 1;
			}
			//行走
			if(initial_Rocker_L_Y < 1000)
			{
				static float Short_S = 40.0f;		//40
				static float Long_S	 = 80.0f;	//80
				static float S = 80.0f;
				static float L_S,R_S;
				static float H = 80.0f;					//40.0f;
				L_S = initial_Rocker_R_X > 3000 ? Short_S : initial_Rocker_R_X < 1000 ? Long_S : S;
				R_S = initial_Rocker_R_X < 1000 ? Short_S : initial_Rocker_R_X > 3000 ? Long_S : S;
				if(CSYS_Foot[1].Foot_State == Foot_State_Up 	&& CSYS_Foot[2].Foot_State == Foot_State_Up &&
					 CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
				{
					
					Foot_New_Walk(R_S,H,&CSYS_Foot[1],&Foot_Rocker_Fast_Walk_Up_ramp[1]);
					Foot_New_Walk(L_S,H,&CSYS_Foot[2],&Foot_Rocker_Fast_Walk_Up_ramp[2]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[0],&Foot_Rocker_Fast_Walk_Down_ramp[0]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[3],&Foot_Rocker_Fast_Walk_Down_ramp[3]);
					for(i=0;i<4;i++)
						Foot_Angle_Control(&CSYS_Foot[i]);
					if(Slope(&Foot_Rocker_Fast_Walk_Up_ramp[1]) == 1.0f && Slope(&Foot_Rocker_Fast_Walk_Up_ramp[2]) == 1.0f &&
						 Slope(&Foot_Rocker_Fast_Walk_Down_ramp[0]) == 1.0f && Slope(&Foot_Rocker_Fast_Walk_Down_ramp[3]) == 1.0f)
					{
						CSYS_Foot[0].Foot_State = Foot_State_Up;
						CSYS_Foot[3].Foot_State = Foot_State_Up;
						CSYS_Foot[1].Foot_State = Foot_State_Down;
						CSYS_Foot[2].Foot_State = Foot_State_Down;
						ResetSlope(&Foot_Rocker_Fast_Walk_Up_ramp[1]);
						ResetSlope(&Foot_Rocker_Fast_Walk_Up_ramp[2]);
						ResetSlope(&Foot_Rocker_Fast_Walk_Down_ramp[0]);
						ResetSlope(&Foot_Rocker_Fast_Walk_Down_ramp[3]);
					}
				}
				else if(CSYS_Foot[0].Foot_State == Foot_State_Up 	 && CSYS_Foot[3].Foot_State == Foot_State_Up &&
								CSYS_Foot[1].Foot_State == Foot_State_Down && CSYS_Foot[2].Foot_State == Foot_State_Down)
				{
					Foot_New_Walk(L_S,H,&CSYS_Foot[0],&Foot_Rocker_Fast_Walk_Up_ramp[0]);
					Foot_New_Walk(R_S,H,&CSYS_Foot[3],&Foot_Rocker_Fast_Walk_Up_ramp[3]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[1],&Foot_Rocker_Fast_Walk_Down_ramp[1]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[2],&Foot_Rocker_Fast_Walk_Down_ramp[2]);
					for(i=0;i<4;i++)
						Foot_Angle_Control(&CSYS_Foot[i]);
					if(Slope(&Foot_Rocker_Fast_Walk_Up_ramp[0]) == 1.0f && Slope(&Foot_Rocker_Fast_Walk_Up_ramp[3]) == 1.0f &&
						 Slope(&Foot_Rocker_Fast_Walk_Down_ramp[1]) == 1.0f && Slope(&Foot_Rocker_Fast_Walk_Down_ramp[2]) == 1.0f)
					{
						for(i=0;i<4;i++)
							Foot_Angle_Control(&CSYS_Foot[i]);
						ResetSlope(&Foot_Rocker_Fast_Walk_Up_ramp[0]);
						ResetSlope(&Foot_Rocker_Fast_Walk_Up_ramp[3]);
						ResetSlope(&Foot_Rocker_Fast_Walk_Down_ramp[1]);
						ResetSlope(&Foot_Rocker_Fast_Walk_Down_ramp[2]);
						if(DataPack->rocker[1] > 1000 && DataPack->rocker[1] < 3000)
						{
							CSYS_Foot[0].Foot_State = Foot_State_Down;
							CSYS_Foot[3].Foot_State = Foot_State_Down;
						}
						else
						{
							CSYS_Foot[1].Foot_State = Foot_State_Up;
							CSYS_Foot[2].Foot_State = Foot_State_Up;
							CSYS_Foot[0].Foot_State = Foot_State_Down;
							CSYS_Foot[3].Foot_State = Foot_State_Down;
							ResetFlag(&circle_flag);
						}
					}
				}
				else if(CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[1].Foot_State == Foot_State_Down &&
								CSYS_Foot[2].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
				{
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[0],&Foot_Rocker_Fast_Walk_Down_ramp[0]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[3],&Foot_Rocker_Fast_Walk_Down_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_Angle_Control(&CSYS_Foot[3]);
					if(Slope(&Foot_Rocker_Fast_Walk_Down_ramp[0]) == 1.0f && Slope(&Foot_Rocker_Fast_Walk_Down_ramp[3]) == 1.0f)
					{
						ResetSlope(&Foot_Rocker_Fast_Walk_Down_ramp[0]);
						ResetSlope(&Foot_Rocker_Fast_Walk_Down_ramp[3]);
						CSYS_Foot[0].Foot_State = Foot_State_Down;
						CSYS_Foot[1].Foot_State = Foot_State_Up;
						CSYS_Foot[2].Foot_State = Foot_State_Up;
						CSYS_Foot[3].Foot_State = Foot_State_Down;
						ResetFlag(&circle_flag);
					}
				}
			}
			//左右平移
			else if(initial_Rocker_L_Y > 1000 && initial_Rocker_L_Y < 3000 && (initial_Rocker_R_X < 1000 || initial_Rocker_R_X > 3000))		
			{
				static float Low_H = 60.0f;
				static float High_H = 80.0f;
				static float L_H,R_H;
				static float Remember_x[4];
				static float Remember_y[4];
				L_H = initial_Rocker_R_X > 3000 ? Low_H : High_H;
				R_H = initial_Rocker_R_X < 1000 ? Low_H : High_H;
				if(!Current_flag)
				{
					for(i=0;i<4;i++)
					{
						Remember_x[i] = CSYS_Foot[i].x;
						Remember_y[i] = CSYS_Foot[i].y;
					}
					Current_flag = 1;
				}					
				if(CSYS_Foot[1].Foot_State == Foot_State_Up 	&& CSYS_Foot[2].Foot_State == Foot_State_Up &&
					 CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
				{
					Foot_New_Walk_RTO(Remember_x[1],Remember_y[1],0,R_H,&CSYS_Foot[1],&Foot_Rocker_Translation_ramp[1]);
					Foot_New_Walk_RTO(Remember_x[2],Remember_y[2],0,L_H,&CSYS_Foot[2],&Foot_Rocker_Translation_ramp[2]);
//					Foot_New_Walk(0,R_H,&CSYS_Foot[1],&Foot_Rocker_Translation_ramp[1]);
//					Foot_New_Walk(0,L_H,&CSYS_Foot[2],&Foot_Rocker_Translation_ramp[2]);
					Foot_Angle_Control(&CSYS_Foot[1]);
					Foot_Angle_Control(&CSYS_Foot[2]);
					if(Slope(&Foot_Rocker_Translation_ramp[1]) == 1.0f && Slope(&Foot_Rocker_Translation_ramp[2]) == 1.0f)
					{
						ResetSlope(&Foot_Rocker_Translation_ramp[1]);
						ResetSlope(&Foot_Rocker_Translation_ramp[2]);
						CSYS_Foot[0].Foot_State = Foot_State_Up;
						CSYS_Foot[3].Foot_State = Foot_State_Up;
						CSYS_Foot[1].Foot_State = Foot_State_Down;
						CSYS_Foot[2].Foot_State = Foot_State_Down;
					}
				}
				else if(CSYS_Foot[0].Foot_State == Foot_State_Up && CSYS_Foot[3].Foot_State == Foot_State_Up &&
								CSYS_Foot[1].Foot_State == Foot_State_Down && CSYS_Foot[2].Foot_State == Foot_State_Down)
				{
					Foot_New_Walk_RTO(Remember_x[0],Remember_y[0],0,R_H,&CSYS_Foot[0],&Foot_Rocker_Translation_ramp[0]);
					Foot_New_Walk_RTO(Remember_x[3],Remember_y[3],0,L_H,&CSYS_Foot[3],&Foot_Rocker_Translation_ramp[3]);
//					Foot_New_Walk(0,R_H,&CSYS_Foot[3],&Foot_Rocker_Translation_ramp[3]);
//					Foot_New_Walk(0,L_H,&CSYS_Foot[0],&Foot_Rocker_Translation_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[3]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					if(Slope(&Foot_Rocker_Translation_ramp[3]) == 1.0f && Slope(&Foot_Rocker_Translation_ramp[0]) == 1.0f)
					{
						ResetSlope(&Foot_Rocker_Translation_ramp[3]);
						ResetSlope(&Foot_Rocker_Translation_ramp[0]);
						CSYS_Foot[1].Foot_State = Foot_State_Up;
						CSYS_Foot[2].Foot_State = Foot_State_Up;
						CSYS_Foot[0].Foot_State = Foot_State_Down;
						CSYS_Foot[3].Foot_State = Foot_State_Down;
						ResetFlag(&circle_flag);
					}
				}
			}
			//原地旋转
			else if(initial_Rocker_L_X > 3000 || initial_Rocker_L_X < 1000)
			{
				static float H = 80.0f;
				static float Positive_S = 40.0f;
				static float Negative_S = -40.0f;		//两个数值应保持一致
				static float L_S,R_S;
				L_S = initial_Rocker_L_X > 3000 ? Negative_S : Positive_S;
				R_S = initial_Rocker_L_X < 1000 ? Negative_S : Positive_S;
				if(CSYS_Foot[1].Foot_State == Foot_State_Up && CSYS_Foot[2].Foot_State == Foot_State_Up &&
					 CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
				{
					Foot_New_Walk_Target_RTO(initial_x+R_S,H,&CSYS_Foot[1],&Foot_Rocker_Turn_ramp[1]);
					Foot_New_Walk_Target_RTO(initial_x+L_S,H,&CSYS_Foot[2],&Foot_Rocker_Turn_ramp[2]);
//					Foot_New_Walk(R_S,H,&CSYS_Foot[1],&Foot_Rocker_Turn_ramp[1]);
//					Foot_New_Walk(L_S,H,&CSYS_Foot[2],&Foot_Rocker_Turn_ramp[2]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[0],&Foot_Rocker_Turn_ramp[0]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[3],&Foot_Rocker_Turn_ramp[3]);
					for(i=0;i<4;i++)
						Foot_Angle_Control(&CSYS_Foot[i]);
					if(Slope(&Foot_Rocker_Turn_ramp[1]) == 1.0f && Slope(&Foot_Rocker_Turn_ramp[2]) == 1.0f &&
						 Slope(&Foot_Rocker_Turn_ramp[0]) == 1.0f && Slope(&Foot_Rocker_Turn_ramp[3]) == 1.0f )
					{
						for(i=0;i<4;i++)
							ResetSlope(&Foot_Rocker_Turn_ramp[i]);
						CSYS_Foot[0].Foot_State = Foot_State_Up;
						CSYS_Foot[3].Foot_State = Foot_State_Up;
						CSYS_Foot[1].Foot_State = Foot_State_Down;
						CSYS_Foot[2].Foot_State = Foot_State_Down;
					}
				}
				else if(CSYS_Foot[0].Foot_State == Foot_State_Up && CSYS_Foot[3].Foot_State == Foot_State_Up &&
								CSYS_Foot[1].Foot_State == Foot_State_Down && CSYS_Foot[2].Foot_State == Foot_State_Down)
				{
					Foot_New_Walk_Target_RTO(initial_x+R_S,H,&CSYS_Foot[3],&Foot_Rocker_Turn_ramp[3]);
					Foot_New_Walk_Target_RTO(initial_x+L_S,H,&CSYS_Foot[0],&Foot_Rocker_Turn_ramp[0]);
//					Foot_New_Walk(R_S,H,&CSYS_Foot[3],&Foot_Rocker_Turn_ramp[3]);
//					Foot_New_Walk(L_S,H,&CSYS_Foot[0],&Foot_Rocker_Turn_ramp[0]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[1],&Foot_Rocker_Turn_ramp[1]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[2],&Foot_Rocker_Turn_ramp[2]);
					for(i=0;i<4;i++)
						Foot_Angle_Control(&CSYS_Foot[i]);
					if(Slope(&Foot_Rocker_Turn_ramp[0]) == 1.0f && Slope(&Foot_Rocker_Turn_ramp[1]) == 1.0f &&
						 Slope(&Foot_Rocker_Turn_ramp[2]) == 1.0f && Slope(&Foot_Rocker_Turn_ramp[3]) == 1.0f)
					{
						for(i=0;i<4;i++)
							ResetSlope(&Foot_Rocker_Turn_ramp[i]);
						if(DataPack->rocker[0] > 1000 && DataPack->rocker[0] < 3000)
						{
							CSYS_Foot[0].Foot_State = Foot_State_Down;
							CSYS_Foot[3].Foot_State = Foot_State_Down;
						}
						else 
						{
							CSYS_Foot[1].Foot_State = Foot_State_Up;
							CSYS_Foot[2].Foot_State = Foot_State_Up;
							CSYS_Foot[0].Foot_State = Foot_State_Down;
							CSYS_Foot[3].Foot_State = Foot_State_Down;
							ResetFlag(&circle_flag);
						}
					}
				}
				else if(CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[1].Foot_State == Foot_State_Down &&
								CSYS_Foot[2].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
				{
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[0],&Foot_Rocker_Turn_ramp[0]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[3],&Foot_Rocker_Turn_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_Angle_Control(&CSYS_Foot[3]);
					if(Slope(&Foot_Rocker_Turn_ramp[0]) == 1.0f && Slope(&Foot_Rocker_Turn_ramp[3]) == 1.0f)
					{
						ResetSlope(&Foot_Rocker_Turn_ramp[0]);
						ResetSlope(&Foot_Rocker_Turn_ramp[3]);
						CSYS_Foot[0].Foot_State = Foot_State_Down;
						CSYS_Foot[1].Foot_State = Foot_State_Up;
						CSYS_Foot[2].Foot_State = Foot_State_Up;
						CSYS_Foot[3].Foot_State = Foot_State_Down;
						ResetFlag(&circle_flag);
					}
				}
			}
			
			else			//若不在范围内，继续获取摇杆数据从而做出判断是否做出动作
			{
				ResetFlag(&Current_flag);
				ResetFlag(&circle_flag);
			}
		}
	}
	else
	{
		ResetFlag(&circle_flag);
		ResetFlag(&Part_State);
		ResetFlag(&start_flag);
		for(i=0;i<4;i++)
		{
			ResetSlope(&Foot_Rocker_Fast_Walk_Down_ramp[i]);
			ResetSlope(&Foot_Rocker_Fast_Walk_Up_ramp[i]);
			ResetSlope(&Foot_Rocker_Translation_ramp[i]);
			ResetSlope(&Foot_Rocker_Turn_ramp[i]);
			CSYS_Foot[i].Foot_State = Foot_State_None;
		}
		Remote_State_Last_Current_Update(Remote_Foot_None);
	}
}


void Foot_Rocker_Slow_Walk(const UART_DataPack *DataPack)
{
	static uint16_t initial_L_X;
	static uint16_t initial_L_Y;
	static uint16_t initial_R_X;
	static uint16_t initial_R_Y;
	static uint8_t circle_flag;
	static uint8_t buffer_flag;
	static uint8_t Part_State;
	static uint8_t start_flag;
	static uint8_t awake_flag;
	static uint8_t first_flag;
	static uint8_t state;
	if(Remote_State.Remote_Current_State != Remote_Foot_Emergency_Stop)
	{
		static float initial_x = -40.0f;
		static float initial_y = 0;
		if(!start_flag)
		{
			for(i=0;i<4;i++)
				CSYS_Foot[i].Foot_State = Foot_State_Run_Ready;
			start_flag = 1;
		}
		if(Part_State == 0)
		{
			if(CSYS_Foot[0].Foot_State == Foot_State_Run_Ready && CSYS_Foot[1].Foot_State == Foot_State_Run_Ready &&
				 CSYS_Foot[2].Foot_State == Foot_State_Run_Ready && CSYS_Foot[3].Foot_State == Foot_State_Run_Ready)
			{
				
				for(i=0;i<4;i++)
				{
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[i],&Foot_Rocker_Slow_Walk_Ready_ramp[i]);
					Foot_Angle_Control(&CSYS_Foot[i]);
				}
				if(Slope(&Foot_Rocker_Slow_Walk_Ready_ramp[0]) == 1.0f && Slope(&Foot_Rocker_Slow_Walk_Ready_ramp[1]) == 1.0f &&
					 Slope(&Foot_Rocker_Slow_Walk_Ready_ramp[2]) == 1.0f && Slope(&Foot_Rocker_Slow_Walk_Ready_ramp[3]) == 1.0f)
				{
					for(i=0;i<4;i++)
						ResetSlope(&Foot_Rocker_Slow_Walk_Ready_ramp[i]);
					Part_State = 1;
					CSYS_Foot[2].Foot_State = Foot_State_Up;
					CSYS_Foot[0].Foot_State = Foot_State_Down;
					CSYS_Foot[3].Foot_State = Foot_State_Down;
					CSYS_Foot[1].Foot_State = Foot_State_Down;
					buffer_flag = 1;
				}
			}
		}
		else if(Part_State == 1)
		{
			if(!circle_flag)
			{
				initial_L_X = DataPack->rocker[0];
				initial_L_Y = DataPack->rocker[1];
				initial_R_X = DataPack->rocker[2];
				initial_R_Y = DataPack->rocker[3];
				circle_flag = 1;
			}
			if(initial_L_Y < 1000 && initial_R_X < 3000 && initial_R_X > 1000)
			{
				static float s = 80.0f;
				static float h = 80.0f;
				if(CSYS_Foot[2].Foot_State == Foot_State_Up 	&& CSYS_Foot[0].Foot_State == Foot_State_Down &&
					 CSYS_Foot[3].Foot_State == Foot_State_Down && CSYS_Foot[1].Foot_State == Foot_State_Down)
				{
					if(awake_flag && first_flag)
					{
						Awake_Slope(&Foot_Rocker_Slow_Walk_Down_Global_ramp[0]);
						Awake_Slope(&Foot_Rocker_Slow_Walk_Down_Global_ramp[3]);
						awake_flag = 0;
					}
					Foot_New_Walk(s,h,&CSYS_Foot[2],&Foot_Rocker_Slow_Walk_Up_ramp[2]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Down_ramp[0]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Down_ramp[3]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Down_ramp[1]);
					for(i=0;i<4;i++)
						Foot_Angle_Control(&CSYS_Foot[i]);
					if(Slope(&Foot_Rocker_Slow_Walk_Up_ramp[2]) == 1.0f)
					{
						CSYS_Foot[2].Foot_State = Foot_State_Down;
						CSYS_Foot[0].Foot_State = Foot_State_Up;
						ResetSlope(&Foot_Rocker_Slow_Walk_Up_ramp[2]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[0]);
					}
				}
				else if(CSYS_Foot[0].Foot_State == Foot_State_Up 	 && CSYS_Foot[3].Foot_State == Foot_State_Down &&
								CSYS_Foot[1].Foot_State == Foot_State_Down && CSYS_Foot[2].Foot_State == Foot_State_Down)
				{
					Foot_New_Walk(s,h,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Up_ramp[0]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Down_ramp[3]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Down_ramp[1]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[2],&Foot_Rocker_Slow_Walk_Down_ramp[2]);
					for(i=0;i<4;i++)
						Foot_Angle_Control(&CSYS_Foot[i]);
					if(Slope(&Foot_Rocker_Slow_Walk_Up_ramp[0]) == 1.0f)
					{
						CSYS_Foot[0].Foot_State = Foot_State_Down;
						CSYS_Foot[3].Foot_State = Foot_State_Up;
						ResetSlope(&Foot_Rocker_Slow_Walk_Up_ramp[0]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[3]);
					}
				}
				else if(CSYS_Foot[3].Foot_State == Foot_State_Up 	 && CSYS_Foot[1].Foot_State == Foot_State_Down &&
								CSYS_Foot[2].Foot_State == Foot_State_Down && CSYS_Foot[0].Foot_State == Foot_State_Down)
				
				{
					Foot_New_Walk(s,h,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Up_ramp[3]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Down_ramp[1]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[2],&Foot_Rocker_Slow_Walk_Down_ramp[2]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Down_ramp[0]);
					for(i=0;i<4;i++)
						Foot_Angle_Control(&CSYS_Foot[i]);
					if(Slope(&Foot_Rocker_Slow_Walk_Up_ramp[3]) == 1.0f)
					{
						CSYS_Foot[3].Foot_State = Foot_State_Down;
						CSYS_Foot[1].Foot_State = Foot_State_Up;
						ResetSlope(&Foot_Rocker_Slow_Walk_Up_ramp[3]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[1]);
					}
				}
				else if(CSYS_Foot[1].Foot_State == Foot_State_Up 	 && CSYS_Foot[2].Foot_State == Foot_State_Down &&
								CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
				{
					Foot_New_Walk(s,h,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Up_ramp[1]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[2],&Foot_Rocker_Slow_Walk_Down_ramp[2]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Down_ramp[0]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Down_ramp[3]);
					for(i=0;i<4;i++)
						Foot_Angle_Control(&CSYS_Foot[i]);
					if(Slope(&Foot_Rocker_Slow_Walk_Up_ramp[1]) == 1.0f)
					{
						CSYS_Foot[1].Foot_State = Foot_State_Down;
						CSYS_Foot[2].Foot_State = Foot_State_Up;
						ResetSlope(&Foot_Rocker_Slow_Walk_Up_ramp[1]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[2]);
						ResetFlag(&circle_flag);
						first_flag = 1;
					}
				}
			}
			else if(initial_L_Y < 1000 && initial_R_X < 1000)
			{
				static float L_S = 80.0f;
				static float L_H = 80.0f;
				static float R_S = 40.0f;
				static float R_H = 80.0f;
				if(CSYS_Foot[2].Foot_State == Foot_State_Up 	&& CSYS_Foot[0].Foot_State == Foot_State_Down &&
					 CSYS_Foot[3].Foot_State == Foot_State_Down && CSYS_Foot[1].Foot_State == Foot_State_Down)
				{
					if(awake_flag && first_flag)
					{
						Awake_Slope(&Foot_Rocker_Slow_Walk_Down_Global_ramp[0]);
						Awake_Slope(&Foot_Rocker_Slow_Walk_Down_Global_ramp[3]);
						awake_flag = 0;
					}
					Foot_New_Walk(L_S,L_H,&CSYS_Foot[2],&Foot_Rocker_Slow_Walk_Up_ramp[2]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Down_ramp[0]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Down_ramp[3]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Down_ramp[1]);
					for(i=0;i<4;i++)
						Foot_Angle_Control(&CSYS_Foot[i]);
					if(Slope(&Foot_Rocker_Slow_Walk_Up_ramp[2]) == 1.0f)
					{
						CSYS_Foot[2].Foot_State = Foot_State_Down;
						CSYS_Foot[0].Foot_State = Foot_State_Up;
						ResetSlope(&Foot_Rocker_Slow_Walk_Up_ramp[2]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[0]);
					}
				}
				else if(CSYS_Foot[0].Foot_State == Foot_State_Up 	 && CSYS_Foot[3].Foot_State == Foot_State_Down &&
								CSYS_Foot[1].Foot_State == Foot_State_Down && CSYS_Foot[2].Foot_State == Foot_State_Down)
				{
					Foot_New_Walk(L_S,L_H,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Up_ramp[0]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Down_ramp[3]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Down_ramp[1]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[2],&Foot_Rocker_Slow_Walk_Down_ramp[2]);
					for(i=0;i<4;i++)
						Foot_Angle_Control(&CSYS_Foot[i]);
					if(Slope(&Foot_Rocker_Slow_Walk_Up_ramp[0]) == 1.0f)
					{
						CSYS_Foot[0].Foot_State = Foot_State_Down;
						CSYS_Foot[3].Foot_State = Foot_State_Up;
						ResetSlope(&Foot_Rocker_Slow_Walk_Up_ramp[0]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[3]);
					}
				}
				else if(CSYS_Foot[3].Foot_State == Foot_State_Up 	 && CSYS_Foot[1].Foot_State == Foot_State_Down &&
								CSYS_Foot[2].Foot_State == Foot_State_Down && CSYS_Foot[0].Foot_State == Foot_State_Down)
				
				{
					Foot_New_Walk(R_S,R_H,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Up_ramp[3]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Down_ramp[1]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[2],&Foot_Rocker_Slow_Walk_Down_ramp[2]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Down_ramp[0]);
					for(i=0;i<4;i++)
						Foot_Angle_Control(&CSYS_Foot[i]);
					if(Slope(&Foot_Rocker_Slow_Walk_Up_ramp[3]) == 1.0f)
					{
						CSYS_Foot[3].Foot_State = Foot_State_Down;
						CSYS_Foot[1].Foot_State = Foot_State_Up;
						ResetSlope(&Foot_Rocker_Slow_Walk_Up_ramp[3]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[1]);
					}
				}
				else if(CSYS_Foot[1].Foot_State == Foot_State_Up 	 && CSYS_Foot[2].Foot_State == Foot_State_Down &&
								CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
				{
					Foot_New_Walk(R_S,R_H,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Up_ramp[1]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[2],&Foot_Rocker_Slow_Walk_Down_ramp[2]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Down_ramp[0]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Down_ramp[3]);
					for(i=0;i<4;i++)
						Foot_Angle_Control(&CSYS_Foot[i]);
					if(Slope(&Foot_Rocker_Slow_Walk_Up_ramp[1]) == 1.0f)
					{
						CSYS_Foot[1].Foot_State = Foot_State_Down;
						CSYS_Foot[2].Foot_State = Foot_State_Up;
						ResetSlope(&Foot_Rocker_Slow_Walk_Up_ramp[1]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[2]);
						ResetFlag(&circle_flag);
						first_flag = 1;
					}
				}
			}
			else if(initial_L_Y < 1000 && initial_R_X > 3000)
			{
				static float L_S = 40.0f;
				static float L_H = 80.0f;
				static float R_S = 80.0f;
				static float R_H = 80.0f;
				if(CSYS_Foot[2].Foot_State == Foot_State_Up 	&& CSYS_Foot[0].Foot_State == Foot_State_Down &&
					 CSYS_Foot[3].Foot_State == Foot_State_Down && CSYS_Foot[1].Foot_State == Foot_State_Down)
				{
					if(awake_flag && first_flag)
					{
						Awake_Slope(&Foot_Rocker_Slow_Walk_Down_Global_ramp[0]);
						Awake_Slope(&Foot_Rocker_Slow_Walk_Down_Global_ramp[3]);
						awake_flag = 0;
					}
					Foot_New_Walk(L_S,L_H,&CSYS_Foot[2],&Foot_Rocker_Slow_Walk_Up_ramp[2]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Down_ramp[0]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Down_ramp[3]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Down_ramp[1]);
					for(i=0;i<4;i++)
						Foot_Angle_Control(&CSYS_Foot[i]);
					if(Slope(&Foot_Rocker_Slow_Walk_Up_ramp[2]) == 1.0f)
					{
						CSYS_Foot[2].Foot_State = Foot_State_Down;
						CSYS_Foot[0].Foot_State = Foot_State_Up;
						ResetSlope(&Foot_Rocker_Slow_Walk_Up_ramp[2]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[0]);
					}
				}
				else if(CSYS_Foot[0].Foot_State == Foot_State_Up 	 && CSYS_Foot[3].Foot_State == Foot_State_Down &&
								CSYS_Foot[1].Foot_State == Foot_State_Down && CSYS_Foot[2].Foot_State == Foot_State_Down)
				{
					Foot_New_Walk(L_S,L_H,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Up_ramp[0]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Down_ramp[3]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Down_ramp[1]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[2],&Foot_Rocker_Slow_Walk_Down_ramp[2]);
					for(i=0;i<4;i++)
						Foot_Angle_Control(&CSYS_Foot[i]);
					if(Slope(&Foot_Rocker_Slow_Walk_Up_ramp[0]) == 1.0f)
					{
						CSYS_Foot[0].Foot_State = Foot_State_Down;
						CSYS_Foot[3].Foot_State = Foot_State_Up;
						ResetSlope(&Foot_Rocker_Slow_Walk_Up_ramp[0]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[3]);
					}
				}
				else if(CSYS_Foot[3].Foot_State == Foot_State_Up 	 && CSYS_Foot[1].Foot_State == Foot_State_Down &&
								CSYS_Foot[2].Foot_State == Foot_State_Down && CSYS_Foot[0].Foot_State == Foot_State_Down)
				
				{
					Foot_New_Walk(R_S,R_H,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Up_ramp[3]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Down_ramp[1]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[2],&Foot_Rocker_Slow_Walk_Down_ramp[2]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Down_ramp[0]);
					for(i=0;i<4;i++)
						Foot_Angle_Control(&CSYS_Foot[i]);
					if(Slope(&Foot_Rocker_Slow_Walk_Up_ramp[3]) == 1.0f)
					{
						CSYS_Foot[3].Foot_State = Foot_State_Down;
						CSYS_Foot[1].Foot_State = Foot_State_Up;
						ResetSlope(&Foot_Rocker_Slow_Walk_Up_ramp[3]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[1]);
					}
				}
				else if(CSYS_Foot[1].Foot_State == Foot_State_Up 	 && CSYS_Foot[2].Foot_State == Foot_State_Down &&
								CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
				{
					Foot_New_Walk(R_S,R_H,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Up_ramp[1]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[2],&Foot_Rocker_Slow_Walk_Down_ramp[2]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Down_ramp[0]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Down_ramp[3]);
					for(i=0;i<4;i++)
						Foot_Angle_Control(&CSYS_Foot[i]);
					if(Slope(&Foot_Rocker_Slow_Walk_Up_ramp[1]) == 1.0f)
					{
						CSYS_Foot[1].Foot_State = Foot_State_Down;
						CSYS_Foot[2].Foot_State = Foot_State_Up;
						ResetSlope(&Foot_Rocker_Slow_Walk_Up_ramp[1]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[2]);
						ResetFlag(&circle_flag);
						first_flag = 1;
					}
				}
			}
			else if(initial_L_Y > 1000 && initial_L_Y < 3000 && (initial_R_X > 3000 || initial_R_X < 1000))		
			{
				static float Low_H = 80.0f;		
				static float High_H = 80.0f;
				static float L_H,R_H;
				L_H = initial_R_X > 3000 ? Low_H : High_H;		//原地踏步
				R_H = initial_R_X > 3000 ? High_H : Low_H;
				if(CSYS_Foot[2].Foot_State == Foot_State_Up 	&& CSYS_Foot[0].Foot_State == Foot_State_Down &&
					 CSYS_Foot[3].Foot_State == Foot_State_Down && CSYS_Foot[1].Foot_State == Foot_State_Down)
				{
					Foot_New_Walk_Target_RTO(CSYS_Foot[2].x,L_H,&CSYS_Foot[2],&Foot_Rocker_Stand_Up_ramp[2]);
					Foot_Angle_Control(&CSYS_Foot[2]);
					if(Slope(&Foot_Rocker_Stand_Up_ramp[2]) == 1.0f)
					{
						ResetSlope(&Foot_Rocker_Stand_Up_ramp[2]);
						CSYS_Foot[2].Foot_State = Foot_State_Down;
						CSYS_Foot[0].Foot_State = Foot_State_Up;
					}
				}
				else if(CSYS_Foot[0].Foot_State == Foot_State_Up 		&& CSYS_Foot[3].Foot_State == Foot_State_Down &&
								CSYS_Foot[1].Foot_State == Foot_State_Down	&& CSYS_Foot[2].Foot_State == Foot_State_Down)
				{
					Foot_New_Walk_Target_RTO(CSYS_Foot[0].x,L_H,&CSYS_Foot[0],&Foot_Rocker_Stand_Up_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					if(Slope(&Foot_Rocker_Stand_Up_ramp[0]) == 1.0f)
					{
						ResetSlope(&Foot_Rocker_Stand_Up_ramp[0]);
						CSYS_Foot[0].Foot_State = Foot_State_Down;
						CSYS_Foot[3].Foot_State = Foot_State_Up;
					}
				}
				else if(CSYS_Foot[3].Foot_State == Foot_State_Up && CSYS_Foot[1].Foot_State == Foot_State_Down &&
								CSYS_Foot[2].Foot_State == Foot_State_Down && CSYS_Foot[0].Foot_State == Foot_State_Down)
				{
					Foot_New_Walk_Target_RTO(CSYS_Foot[3].x,R_H,&CSYS_Foot[3],&Foot_Rocker_Stand_Up_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
					if(Slope(&Foot_Rocker_Stand_Up_ramp[3]) == 1.0f)
					{
						ResetSlope(&Foot_Rocker_Stand_Up_ramp[3]);
						CSYS_Foot[3].Foot_State = Foot_State_Down;
						CSYS_Foot[1].Foot_State = Foot_State_Up;
					}
				}
				else if(CSYS_Foot[1].Foot_State == Foot_State_Up && CSYS_Foot[2].Foot_State == Foot_State_Down &&
								CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
				{
					Foot_New_Walk_Target_RTO(CSYS_Foot[1].x,R_H,&CSYS_Foot[1],&Foot_Rocker_Stand_Up_ramp[1]);
					Foot_Angle_Control(&CSYS_Foot[1]);
					if(Slope(&Foot_Rocker_Stand_Up_ramp[1]) == 1.0f)
					{
						ResetSlope(&Foot_Rocker_Stand_Up_ramp[1]);
						CSYS_Foot[1].Foot_State = Foot_State_Down;
						CSYS_Foot[2].Foot_State = Foot_State_Up;
						ResetFlag(&circle_flag);
					}
				}
			}
			else
			{
				Sleep_Slope(&Foot_Rocker_Slow_Walk_Down_Global_ramp[0]);
				Sleep_Slope(&Foot_Rocker_Slow_Walk_Down_Global_ramp[3]);
				ResetFlag(&circle_flag);
				awake_flag = 1;
			}
		}
	}
	else
	{
		ResetFlag(&circle_flag);
		ResetFlag(&Part_State);
		ResetFlag(&state);
		ResetFlag(&start_flag);
		ResetFlag(&awake_flag);
		ResetFlag(&first_flag);
		for(i=0;i<4;i++)
		{
			CSYS_Foot[i].Foot_State = Foot_State_None;
			ResetSlope(&Foot_Rocker_Slow_Walk_Up_ramp[i]);
			ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[i]);
			ResetSlope(&Foot_Rocker_Slow_Walk_Ready_ramp[i]);
			ResetSlope(&Foot_Rocker_Stand_Up_ramp[i]);
			Reset_Global_Slope(&Foot_Rocker_Slow_Walk_Down_Global_ramp[i]);
		}
		Remote_State_Last_Current_Update(Remote_Foot_None);
	} 
}


void Foot_Rocker_Run(const UART_DataPack *DataPack)
{
	static uint16_t initial_L_X;
	static uint16_t initial_L_Y;
	static uint16_t initial_R_X;
	static uint16_t initial_R_Y;
	static uint8_t circle_flag;
	static uint8_t Part_State;
	static uint8_t first_flag;
	static uint8_t start_flag;
	if(Remote_State.Remote_Current_State != Remote_Foot_Emergency_Stop)
	{
		static float initial_x = -40.0f;
		static float initial_y = 0;
		if(Part_State == 0)
		{
			if(!start_flag)
			{
				for(i=0;i<4;i++) 
					CSYS_Foot[i].Foot_State = Foot_State_Run_Ready;
				start_flag = 1;
			}
			if(CSYS_Foot[0].Foot_State == Foot_State_Run_Ready && CSYS_Foot[1].Foot_State == Foot_State_Run_Ready &&
				 CSYS_Foot[2].Foot_State == Foot_State_Run_Ready && CSYS_Foot[3].Foot_State == Foot_State_Run_Ready)
			{
				for(i=0;i<4;i++)
				{
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[i],&Foot_Rocker_Run_Ready_ramp[i]);
					Foot_Angle_Control(&CSYS_Foot[i]);
				}
				if(Slope(&Foot_Rocker_Run_Ready_ramp[0]) == 1.0f && Slope(&Foot_Rocker_Run_Ready_ramp[1]) == 1.0f &&
					 Slope(&Foot_Rocker_Run_Ready_ramp[2]) == 1.0f && Slope(&Foot_Rocker_Run_Ready_ramp[3]) == 1.0f)
				{
					for(i=0;i<4;i++)
						ResetSlope(&Foot_Rocker_Run_Ready_ramp[i]);
					CSYS_Foot[0].Foot_State = Foot_State_Down;
					CSYS_Foot[1].Foot_State = Foot_State_Up;
					CSYS_Foot[2].Foot_State = Foot_State_Up;
					CSYS_Foot[3].Foot_State = Foot_State_Up;
					Part_State = 1;
				}
			}
		}
		else if(Part_State == 1)
		{
			
			if(!circle_flag)
			{
				initial_L_X = DataPack->rocker[0];
				initial_L_Y = DataPack->rocker[1];
				initial_R_X = DataPack->rocker[2];
				initial_R_Y = DataPack->rocker[3];
				circle_flag = 1;
			}
			if(initial_L_Y < 1000)
			{
				static float H = 80.0f;
				static float Short_S = 80.0f;
				static float Long_S = 60.0f;
				static float S = 80.0f;
				static float L_S,R_S;
				L_S = initial_R_X > 3000 ? Short_S : initial_R_X < 1000 ? Long_S : S;
				R_S = initial_R_X < 1000 ? Short_S : initial_R_X > 3000 ? Long_S : S; 
				if(CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[1].Foot_State == Foot_State_Up &&
					 CSYS_Foot[2].Foot_State == Foot_State_Up	 && CSYS_Foot[3].Foot_State == Foot_State_Up)
				{
					if(first_flag)
					{
						Foot_New_Walk(R_S,H,&CSYS_Foot[3],&Foot_Rocker_Run_Up_ramp[3]);
					}
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[0],&Foot_Rocker_Run_Down_ramp[0]);
					Foot_New_Walk(R_S,H,&CSYS_Foot[1],&Foot_Rocker_Run_Up_ramp[1]);
					Foot_New_Walk(L_S,H,&CSYS_Foot[2],&Foot_Rocker_Run_Up_ramp[2]);
					for(i=0;i<4;i++)
						Foot_Angle_Control(&CSYS_Foot[i]);
					if(Slope(&Foot_Rocker_Run_Down_ramp[0]) == 1.0f)
					{
						first_flag = 1;
						ResetSlope(&Foot_Rocker_Run_Down_ramp[0]);
						ResetSlope(&Foot_Rocker_Run_Up_ramp[3]);
						CSYS_Foot[0].Foot_State = Foot_State_Up;
						CSYS_Foot[1].Foot_State = Foot_State_Up;
						CSYS_Foot[2].Foot_State = Foot_State_Up;
						CSYS_Foot[3].Foot_State = Foot_State_Down;
					}
				}
				else if(CSYS_Foot[0].Foot_State == Foot_State_Up 	 && CSYS_Foot[1].Foot_State == Foot_State_Up &&
								CSYS_Foot[2].Foot_State == Foot_State_Up	 && CSYS_Foot[3].Foot_State == Foot_State_Down)
				{
					Foot_New_Walk(L_S,H,&CSYS_Foot[0],&Foot_Rocker_Run_Up_ramp[0]);
					Foot_New_Walk(R_S,H,&CSYS_Foot[1],&Foot_Rocker_Run_Up_ramp[1]);
					Foot_New_Walk(L_S,H,&CSYS_Foot[2],&Foot_Rocker_Run_Up_ramp[2]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[3],&Foot_Rocker_Run_Down_ramp[3]);
					for(i=0;i<4;i++)
						Foot_Angle_Control(&CSYS_Foot[i]);
					if(Slope(&Foot_Rocker_Run_Down_ramp[3]) == 1.0f)
					{
						ResetSlope(&Foot_Rocker_Run_Down_ramp[3]);
						ResetSlope(&Foot_Rocker_Run_Up_ramp[1]);
						ResetSlope(&Foot_Rocker_Run_Up_ramp[2]);
						CSYS_Foot[0].Foot_State = Foot_State_Up;
						CSYS_Foot[1].Foot_State = Foot_State_Down;
						CSYS_Foot[2].Foot_State = Foot_State_Down;
						CSYS_Foot[3].Foot_State = Foot_State_Up;
					}
				}
				else if(CSYS_Foot[0].Foot_State == Foot_State_Up 	 && CSYS_Foot[1].Foot_State == Foot_State_Down &&		
								CSYS_Foot[2].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Up)
				{
					Foot_New_Walk(L_S,H,&CSYS_Foot[0],&Foot_Rocker_Run_Up_ramp[0]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[1],&Foot_Rocker_Run_Down_ramp[1]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[2],&Foot_Rocker_Run_Down_ramp[2]);
					Foot_New_Walk(R_S,H,&CSYS_Foot[3],&Foot_Rocker_Run_Up_ramp[3]);
					for(i=0;i<4;i++)
						Foot_Angle_Control(&CSYS_Foot[i]);
					if(Slope(&Foot_Rocker_Run_Down_ramp[1]) == 1.0f && Slope(&Foot_Rocker_Run_Down_ramp[2]) == 1.0f )
					{
						
						ResetSlope(&Foot_Rocker_Run_Up_ramp[0]);
						ResetSlope(&Foot_Rocker_Run_Down_ramp[1]);
						ResetSlope(&Foot_Rocker_Run_Down_ramp[2]);
						if(DataPack->rocker[1] > 1000 && DataPack->rocker[1] < 3000)		//检查此时摇杆数据是否放下
						{
							CSYS_Foot[0].Foot_State = Foot_State_Down;
							CSYS_Foot[1].Foot_State = Foot_State_Down;
							CSYS_Foot[2].Foot_State = Foot_State_Down;
							CSYS_Foot[3].Foot_State = Foot_State_Up;
							
						}
						else
						{
							CSYS_Foot[0].Foot_State = Foot_State_Down;
							CSYS_Foot[1].Foot_State = Foot_State_Up;
							CSYS_Foot[2].Foot_State = Foot_State_Up;
							CSYS_Foot[3].Foot_State = Foot_State_Up;
							ResetFlag(&circle_flag);
						}
					}
				}
				//以下为停止运动命令  走完循环
				else if(CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[1].Foot_State == Foot_State_Down &&
								CSYS_Foot[2].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Up)
				{
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[0],&Foot_Rocker_Run_Down_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_New_Walk(R_S,H,&CSYS_Foot[3],&Foot_Rocker_Run_Up_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
					if(Slope(&Foot_Rocker_Run_Down_ramp[0]) == 1.0f && Slope(&Foot_Rocker_Run_Up_ramp[3]) == 1.0f)
					{
						ResetSlope(&Foot_Rocker_Run_Down_ramp[0]);
						ResetSlope(&Foot_Rocker_Run_Up_ramp[3]);
						CSYS_Foot[3].Foot_State = Foot_State_Down;
					}
				}
				else if(CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[1].Foot_State == Foot_State_Down &&
								CSYS_Foot[2].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
				{
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[3],&Foot_Rocker_Run_Down_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
					if(Slope(&Foot_Rocker_Run_Down_ramp[3]) == 1.0f)
					{
						ResetSlope(&Foot_Rocker_Run_Down_ramp[3]);
						CSYS_Foot[0].Foot_State = Foot_State_Down;
						CSYS_Foot[1].Foot_State = Foot_State_Up;
						CSYS_Foot[2].Foot_State = Foot_State_Up;
						CSYS_Foot[3].Foot_State = Foot_State_Up;
						ResetFlag(&circle_flag);
						ResetFlag(&first_flag);
					}
				}
			}
			else
				ResetFlag(&circle_flag);
		}
	}
	else
	{
		ResetFlag(&circle_flag);
		ResetFlag(&Part_State);
		ResetFlag(&first_flag);
		ResetFlag(&start_flag);
		for(i=0;i<4;i++)
		{
			CSYS_Foot[i].Foot_State = Foot_State_None;
			ResetSlope(&Foot_Rocker_Run_Ready_ramp[i]);
			ResetSlope(&Foot_Rocker_Run_Up_ramp[i]);
			ResetSlope(&Foot_Rocker_Run_Down_ramp[i]);
		}
		Remote_State_Last_Current_Update(Remote_Foot_None);
	}
}

void Foot_Rocker_Calandira(const UART_DataPack *DataPack)
{
	static uint16_t initial_L_X;
	static uint16_t initial_L_Y;
	static uint16_t initial_R_X;
	static uint16_t initial_R_Y;
	static uint8_t Reset_State;
	static uint8_t Part_State;
	static uint8_t circle_flag;
	static uint8_t Key_State;
	static uint8_t Jump_State;
	static uint8_t buffer_flag;
	static uint8_t init_flag;
	static uint8_t first_flag;
	static uint8_t State;
	static float initial_x = -40.0f;
	static float initial_y = 0;
//	static float Change_x[2];
//	static float Change_y[2];
//	if(!init_flag)
//	{
//		Change_x[0] = initial_x;
//		Change_x[1] = initial_x;
//		Change_y[0] = initial_y;
//		Change_y[1] = initial_y;
//		init_flag = 1;
//	}
	if(Remote_State.Remote_Current_State != Remote_Foot_Emergency_Stop)
	{
		if(Part_State == 0)
		{
			for(i=0;i<4;i++)
			{
				Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[i],&Foot_Rocker_Calandria_Ready_ramp[i]);
				Foot_Angle_Control(&CSYS_Foot[i]);
			}
			if(Slope(&Foot_Rocker_Calandria_Ready_ramp[0]) == 1.0f && Slope(&Foot_Rocker_Calandria_Ready_ramp[1]) == 1.0f &&
				 Slope(&Foot_Rocker_Calandria_Ready_ramp[2]) == 1.0f && Slope(&Foot_Rocker_Calandria_Ready_ramp[3]) == 1.0f)
			{
				for(i=0;i<4;i++)
				{
					ResetSlope(&Foot_Rocker_Calandria_Ready_ramp[i]);
				}
				Part_State = 1;
			}
		}
		else if(Part_State == 1)
		{
			if(!circle_flag)
			{
				initial_L_X = DataPack->rocker[0];
				initial_L_Y = DataPack->rocker[1];
				initial_R_X = DataPack->rocker[2];
				initial_R_Y = DataPack->rocker[3];
				Reset_State = DataPack->Key.Right_Key_Up;
				Jump_State = DataPack->Key.Right_Key_Down;
				circle_flag = 1;
			}
			if(initial_L_Y < 1000)
			{
				Key_State = 1;
				static float High_S = 80.0f;
				static float Short_S = 50.0f;
				static float S = 80.0f;
				static float H = 80.0f;
				static float L_S,R_S;
				L_S = initial_R_X > 3000 ? Short_S : initial_R_X < 1000 ? High_S : S;
				R_S = initial_R_X < 1000 ? Short_S : initial_R_X > 3000 ? High_S : S;
				if(!first_flag)
				{
					CSYS_Foot[1].Foot_State = Foot_State_Up;
					CSYS_Foot[2].Foot_State = Foot_State_Up;
					CSYS_Foot[0].Foot_State = Foot_State_Down;
					CSYS_Foot[3].Foot_State = Foot_State_Down;
					first_flag = 1;
				}
				if(CSYS_Foot[1].Foot_State == Foot_State_Up 	&& CSYS_Foot[2].Foot_State == Foot_State_Up &&
					 CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
				{
					Foot_New_Walk(R_S,H,&CSYS_Foot[1],&Foot_Rocker_Calandria_Up_ramp[1]);
					Foot_New_Walk(L_S,H,&CSYS_Foot[2],&Foot_Rocker_Calandria_Up_ramp[2]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[0],&Foot_Rocker_Calandria_Down_ramp[0]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[3],&Foot_Rocker_Calandria_Down_ramp[3]);
					for(i=0;i<4;i++)
						Foot_Angle_Control(&CSYS_Foot[i]);
					if(Slope(&Foot_Rocker_Calandria_Up_ramp[1]) == 1.0f && Slope(&Foot_Rocker_Calandria_Up_ramp[2]) == 1.0f &&
						 Slope(&Foot_Rocker_Calandria_Down_ramp[0]) == 1.0f && Slope(&Foot_Rocker_Calandria_Down_ramp[3]) == 1.0f)
					{
						ResetSlope(&Foot_Rocker_Calandria_Up_ramp[1]);
						ResetSlope(&Foot_Rocker_Calandria_Up_ramp[2]);
						ResetSlope(&Foot_Rocker_Calandria_Down_ramp[0]);
						ResetSlope(&Foot_Rocker_Calandria_Down_ramp[3]);
						CSYS_Foot[0].Foot_State = Foot_State_Up;
						CSYS_Foot[3].Foot_State = Foot_State_Up;
						CSYS_Foot[1].Foot_State = Foot_State_Down;
						CSYS_Foot[2].Foot_State = Foot_State_Down;
					}
				}
				else if(CSYS_Foot[0].Foot_State == Foot_State_Up 	 && CSYS_Foot[3].Foot_State == Foot_State_Up &&
								CSYS_Foot[1].Foot_State == Foot_State_Down && CSYS_Foot[2].Foot_State == Foot_State_Down)
				{
					Foot_New_Walk(L_S,H,&CSYS_Foot[0],&Foot_Rocker_Calandria_Up_ramp[0]);
					Foot_New_Walk(R_S,H,&CSYS_Foot[3],&Foot_Rocker_Calandria_Up_ramp[3]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[1],&Foot_Rocker_Calandria_Down_ramp[1]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[2],&Foot_Rocker_Calandria_Down_ramp[2]);
					for(i=0;i<4;i++)
						Foot_Angle_Control(&CSYS_Foot[i]);
					if(Slope(&Foot_Rocker_Calandria_Up_ramp[0]) == 1.0f 	&& Slope(&Foot_Rocker_Calandria_Up_ramp[3]) == 1.0f &&
						 Slope(&Foot_Rocker_Calandria_Down_ramp[1]) == 1.0f && Slope(&Foot_Rocker_Calandria_Down_ramp[2]) == 1.0f)
					{
						ResetSlope(&Foot_Rocker_Calandria_Up_ramp[0]);
						ResetSlope(&Foot_Rocker_Calandria_Up_ramp[3]);
						ResetSlope(&Foot_Rocker_Calandria_Down_ramp[1]);
						ResetSlope(&Foot_Rocker_Calandria_Down_ramp[2]);
						if(DataPack->rocker[1] > 1000 && DataPack->rocker[1] < 3000)
						{
							CSYS_Foot[0].Foot_State = Foot_State_Down;
							CSYS_Foot[3].Foot_State = Foot_State_Down;
						}
						else
						{
							CSYS_Foot[1].Foot_State = Foot_State_Up;
							CSYS_Foot[2].Foot_State = Foot_State_Up;
							CSYS_Foot[0].Foot_State = Foot_State_Down;
							CSYS_Foot[3].Foot_State = Foot_State_Down;
							ResetFlag(&circle_flag);
						}
					}
				}
				else if(CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[1].Foot_State == Foot_State_Down &&
								CSYS_Foot[2].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
				{
					Foot_New_Walk_Target_RTO(initial_x,initial_y,&CSYS_Foot[0],&Foot_Rocker_Calandria_Down_ramp[0]);
					Foot_New_Walk_Target_RTO(initial_x,initial_y,&CSYS_Foot[3],&Foot_Rocker_Calandria_Down_ramp[3]);
//					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[0],&Foot_Rocker_Calandria_Down_ramp[0]);
//					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[3],&Foot_Rocker_Calandria_Down_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_Angle_Control(&CSYS_Foot[3]);
					if(Slope(&Foot_Rocker_Calandria_Down_ramp[0]) == 1.0f && Slope(&Foot_Rocker_Calandria_Down_ramp[3]) == 1.0f)
					{
						ResetSlope(&Foot_Rocker_Calandria_Down_ramp[0]);
						ResetSlope(&Foot_Rocker_Calandria_Down_ramp[3]);
						ResetFlag(&circle_flag);
						ResetFlag(&first_flag);
						ResetFlag(&Key_State);
					}
				}
			}
			else if(initial_L_X < 1000 || initial_L_X > 3000)
			{
				Key_State = 1;
				static float S = 40.0f;
				static float H = 40.0f;
				static float L_S,R_S;
				L_S = initial_L_X > 3000 ? -S : S;
				R_S = initial_L_X < 1000 ? -S : S;
				if(State == 0)
				{
					Foot_New_Walk(R_S,H,&CSYS_Foot[1],&Foot_Rocker_Turn_ramp[1]);
					Foot_New_Walk(L_S,H,&CSYS_Foot[2],&Foot_Rocker_Turn_ramp[2]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[0],&Foot_Rocker_Turn_ramp[0]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[3],&Foot_Rocker_Turn_ramp[3]);
					for(i=0;i<4;i++)
						Foot_Angle_Control(&CSYS_Foot[i]);
					if(Slope(&Foot_Rocker_Turn_ramp[0]) == 1.0f && Slope(&Foot_Rocker_Turn_ramp[1]) == 1.0f &&
						 Slope(&Foot_Rocker_Turn_ramp[2]) == 1.0f && Slope(&Foot_Rocker_Turn_ramp[3]) == 1.0f	)
					{
						for(i=0;i<4;i++)
							ResetSlope(&Foot_Rocker_Turn_ramp[i]);
						State = 1;
					}
				}
				else if(State == 1)
				{
					Foot_New_Walk(L_S,H,&CSYS_Foot[0],&Foot_Rocker_Turn_ramp[0]);
					Foot_New_Walk(R_S,H,&CSYS_Foot[3],&Foot_Rocker_Turn_ramp[3]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[1],&Foot_Rocker_Turn_ramp[1]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[2],&Foot_Rocker_Turn_ramp[2]);
					for(i=0;i<4;i++)
						Foot_Angle_Control(&CSYS_Foot[i]);
					if(Slope(&Foot_Rocker_Turn_ramp[0]) == 1.0f && Slope(&Foot_Rocker_Turn_ramp[1]) == 1.0f &&
						 Slope(&Foot_Rocker_Turn_ramp[2]) == 1.0f && Slope(&Foot_Rocker_Turn_ramp[3]) == 1.0f)
					{
						for(i=0;i<4;i++)
							ResetSlope(&Foot_Rocker_Turn_ramp[i]);
						if(DataPack->rocker[0] > 1000 && DataPack->rocker[0] < 3000)
						{
							State = 2;
						}
						else
						{
							ResetFlag(&State);
							ResetFlag(&circle_flag);
						}
					}
				}
				else if(State == 2)
				{
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[0],&Foot_Rocker_Turn_ramp[0]);
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[3],&Foot_Rocker_Turn_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_Angle_Control(&CSYS_Foot[3]);
					if(Slope(&Foot_Rocker_Turn_ramp[0]) == 1.0f && Slope(&Foot_Rocker_Turn_ramp[3]) == 1.0f)
					{
						ResetSlope(&Foot_Rocker_Turn_ramp[0]);
						ResetSlope(&Foot_Rocker_Turn_ramp[3]);
						ResetFlag(&State);
						ResetFlag(&circle_flag);
						ResetFlag(&Key_State);
					}
				}
			}
			else if(!Key_State && Jump_State)
			{
				if(!first_flag)
				{
					for(i=0;i<4;i++)
						CSYS_Foot[i].Foot_State = Foot_State_Front_Jump_Ready;
					first_flag = 1;
				}
				if(CSYS_Foot[0].Foot_State == Foot_State_Front_Jump_Ready && CSYS_Foot[1].Foot_State == Foot_State_Front_Jump_Ready &&
					 CSYS_Foot[2].Foot_State == Foot_State_Front_Jump_Ready && CSYS_Foot[3].Foot_State == Foot_State_Front_Jump_Ready)
				{
					static float x = -40.0f;
					static float y = 100.0f;
					if(!buffer_flag)
					{
						for(i=0;i<4;i++)
						{
							Foot_Point_RTO_Ramp(x,y,&CSYS_Foot[i],&Foot_Front_Jump_Ready_ramp[i]);
							Foot_Angle_Control(&CSYS_Foot[i]);
						}
						if(Slope(&Foot_Front_Jump_Ready_ramp[0]) == 1.0f && Slope(&Foot_Front_Jump_Ready_ramp[1]) == 1.0f &&
							 Slope(&Foot_Front_Jump_Ready_ramp[2]) == 1.0f && Slope(&Foot_Front_Jump_Ready_ramp[3]) == 1.0f)
						{
							for(i=0;i<4;i++)
								ResetSlope(&Foot_Front_Jump_Ready_ramp[i]);
							buffer_flag = 1;
						}
					}
					else
					{
						if(Slope(&Foot_Front_Jump_Butter_ramp) == 1.0f)
						{
							ResetSlope(&Foot_Front_Jump_Butter_ramp);
							for(i=0;i<4;i++)
								CSYS_Foot[i].Foot_State = Foot_State_Front_Jump_Up;
							ResetFlag(&buffer_flag);
						}
					}
				}
				else if(CSYS_Foot[0].Foot_State == Foot_State_Front_Jump_Up && CSYS_Foot[1].Foot_State == Foot_State_Front_Jump_Up &&
								CSYS_Foot[2].Foot_State == Foot_State_Front_Jump_Up && CSYS_Foot[3].Foot_State == Foot_State_Front_Jump_Up)
				{
					static float x = -120.0f;
					static float y = -50.0f;
					static uint8_t flag;
					if(!flag)
					{
						for(i=0;i<4;i++)
						{
							Foot_Point_RTO(x,y,&CSYS_Foot[i]);
							Foot_Angle_Control(&CSYS_Foot[i]);
						}
						flag = 1;
					}
					if(Slope(&Foot_Front_Jump_Butter_ramp) == 1.0f)
					{
						for(i=0;i<4;i++)
						{
							CSYS_Foot[i].Foot_State = Foot_State_Front_Jump_Down;
						}
						ResetSlope(&Foot_Front_Jump_Butter_ramp);
						flag = 0;
					}
				}
				else if(CSYS_Foot[0].Foot_State == Foot_State_Front_Jump_Down && CSYS_Foot[1].Foot_State == Foot_State_Front_Jump_Down &&
								CSYS_Foot[2].Foot_State == Foot_State_Front_Jump_Down && CSYS_Foot[3].Foot_State == Foot_State_Front_Jump_Down)
				{
					static uint8_t flag;
					for(i=0;i<4;i++)
					{
						Foot_Point_RTO(initial_x,100.0f,&CSYS_Foot[i]);
						Foot_Angle_Control(&CSYS_Foot[i]);
					}
					if(!flag)
					{
						if(Slope(&Foot_Front_Jump_Butter_ramp) == 1.0f)
						{
							for(i=0;i<4;i++)
							{
								CSYS_Foot[i].Foot_State = Foot_State_Front_Jump_Down;
							}
							ResetSlope(&Foot_Front_Jump_Butter_ramp);
							flag = 1;
						}
					}
					else
					{
						for(i=0;i<4;i++)
						{
							Foot_Point_RTO_Ramp(40.0f,initial_y,&CSYS_Foot[i],&Foot_Front_Jump_Down_ramp[i]);
							Foot_Angle_Control(&CSYS_Foot[i]);
						}
						if(Slope(&Foot_Front_Jump_Down_ramp[0]) == 1.0f && Slope(&Foot_Front_Jump_Down_ramp[1]) == 1.0f && 
							 Slope(&Foot_Front_Jump_Down_ramp[2]) == 1.0f && Slope(&Foot_Front_Jump_Down_ramp[3]) == 1.0f)
						{
							for(i=0;i<4;i++)
							{
								ResetSlope(&Foot_Front_Jump_Down_ramp[i]);
								CSYS_Foot[i].Foot_State = Foot_State_None;
							}
							ResetFlag(&flag);
							ResetFlag(&first_flag);
							ResetFlag(&circle_flag);
						}
					}
				}
			}
			else if(!Key_State && Reset_State)
			{
				for(i=0;i<4;i++)
				{
					Foot_Point_RTO_Ramp(initial_x,initial_y,&CSYS_Foot[i],&Foot_Rocker_Calandria_Ready_ramp[i]);
					Foot_Angle_Control(&CSYS_Foot[i]);
				}
				if(Slope(&Foot_Rocker_Calandria_Ready_ramp[0]) == 1.0f &&	Slope(&Foot_Rocker_Calandria_Ready_ramp[1]) == 1.0f &&
					 Slope(&Foot_Rocker_Calandria_Ready_ramp[2]) == 1.0f && Slope(&Foot_Rocker_Calandria_Ready_ramp[3]) == 1.0f)
				{
					for(i=0;i<4;i++)
						ResetSlope(&Foot_Rocker_Calandria_Ready_ramp[i]);
					ResetFlag(&Reset_State);
					ResetFlag(&circle_flag);
				}
			}
			else
			{
				ResetFlag(&circle_flag);
			}
		}
	}
	else
	{
		ResetFlag(&Reset_State);
		ResetFlag(&Part_State);
		ResetFlag(&circle_flag);
		ResetFlag(&Key_State);
		ResetFlag(&buffer_flag);
		ResetFlag(&init_flag);
		ResetFlag(&first_flag);
		for(i=0;i<4;i++)
		{
			ResetSlope(&Foot_Rocker_Calandria_Down_ramp[i]);
			ResetSlope(&Foot_Rocker_Calandria_Ready_ramp[i]);
			ResetSlope(&Foot_Rocker_Calandria_Jump_ramp[i]);
			ResetSlope(&Foot_Rocker_Calandria_Up_ramp[i]);
		}
		ResetSlope(&Foot_Rocker_Calandria_Buffer_ramp);
		Remote_State_Last_Current_Update(Remote_Foot_None);
	}
}


void Foot_New_Run()
{
	static uint8_t first_flag;					//第一次左后腿和右前腿动 其余两腿均不动
	static uint8_t buffer_flag;
	static uint8_t Calibration_flag;
	static uint8_t record_flag;
	static float Record_Direction;
	static float Line_x = 60.0f;				//椭圆长轴a	//60.f
	static float oval_y = 50.0f;				//椭圆短轴b
	static float Variation_x = 30.0f;
	static float oval_y_2 = 70.0f;				//
	static float Line_Back = -60.0f;		//归位路径值
	if(Remote_State.Remote_Current_State != Remote_Foot_Emergency_Stop)
	{
		if(	CSYS_Foot[2].Foot_State == Foot_State_Up && CSYS_Foot[1].Foot_State == Foot_State_Up &&
				CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
		{
			
			if(!buffer_flag)
			{
				if(!record_flag)
				{
					record_flag = 1;
					Record_Direction = Foot_Orient_Calibration_Direction(&IMU,&Calibration_flag);
				}
				if(Record_Direction > 1.0f)
				{
					Foot_IMU_New_Walk_NRTO(Line_x+Variation_x,oval_y,&CSYS_Foot[2],&IMU,&CSYS_Global[2],&oval_ramp[2]);
//					Foot_New_Walk_RTO(-Line_x,0,Line_x+Variation_x,oval_y,&CSYS_Foot[2],&oval_ramp[2]);
					Foot_Angle_Control(&CSYS_Foot[2]);
					Foot_IMU_New_Walk_NRTO(Line_x,oval_y,&CSYS_Foot[1],&IMU,&CSYS_Global[1],&oval_ramp[1]);
//					Foot_New_Walk_RTO(-Line_x,0,Line_x,oval_y,&CSYS_Foot[1],&oval_ramp[1]);
					Foot_Angle_Control(&CSYS_Foot[1]);
				}
				else if(Record_Direction < -1.0f)
				{
					Foot_IMU_New_Walk_NRTO(Line_x,oval_y,&CSYS_Foot[2],&IMU,&CSYS_Global[2],&oval_ramp[2]);
//					Foot_New_Walk_RTO(-Line_x,0,Line_x,oval_y,&CSYS_Foot[2],&oval_ramp[2]);
					Foot_Angle_Control(&CSYS_Foot[2]);
					Foot_IMU_New_Walk_NRTO(Line_x+Variation_x,oval_y,&CSYS_Foot[1],&IMU,&CSYS_Global[1],&oval_ramp[1]);
//					Foot_New_Walk_RTO(-Line_x,0,Line_x+Variation_x,oval_y,&CSYS_Foot[1],&oval_ramp[1]);
					Foot_Angle_Control(&CSYS_Foot[1]);
				}
				else 
				{
					Foot_IMU_New_Walk_NRTO(Line_x,oval_y,&CSYS_Foot[2],&IMU,&CSYS_Global[2],&oval_ramp[2]);
//					Foot_New_Walk_RTO(-Line_x,0,Line_x,oval_y,&CSYS_Foot[2],&oval_ramp[2]);
					Foot_Angle_Control(&CSYS_Foot[2]);
					Foot_IMU_New_Walk_NRTO(Line_x,oval_y,&CSYS_Foot[1],&IMU,&CSYS_Global[1],&oval_ramp[1]);
//					Foot_New_Walk_RTO(-Line_x,0,Line_x,oval_y,&CSYS_Foot[1],&oval_ramp[1]);
					Foot_Angle_Control(&CSYS_Foot[1]);
				}
				if(first_flag)
				{
					Foot_Point_RTO_Ramp(-Line_x,0,&CSYS_Foot[0],&Foot_line_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_Point_RTO_Ramp(-Line_x,0,&CSYS_Foot[3],&Foot_line_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
				}
				if(Slope(&oval_ramp[1]) == 1.0f && Slope(&oval_ramp[2]) == 1.0f)
				{
					if(Record_Direction > 1.0f)
					{
						Foot_IMU_New_Walk_NRTO(Line_x+Variation_x,oval_y,&CSYS_Foot[2],&IMU,&CSYS_Global[2],&oval_ramp[2]);
	//					Foot_New_Walk_RTO(-Line_x,0,Line_x+Variation_x,oval_y,&CSYS_Foot[2],&oval_ramp[2]);
						Foot_Angle_Control(&CSYS_Foot[2]);
						Foot_IMU_New_Walk_NRTO(Line_x,oval_y,&CSYS_Foot[1],&IMU,&CSYS_Global[1],&oval_ramp[1]);
	//					Foot_New_Walk_RTO(-Line_x,0,Line_x,oval_y,&CSYS_Foot[1],&oval_ramp[1]);
						Foot_Angle_Control(&CSYS_Foot[1]);
					}
					else if(Record_Direction < -1.0f)
					{
						Foot_IMU_New_Walk_NRTO(Line_x,oval_y,&CSYS_Foot[2],&IMU,&CSYS_Global[2],&oval_ramp[2]);
	//					Foot_New_Walk_RTO(-Line_x,0,Line_x,oval_y,&CSYS_Foot[2],&oval_ramp[2]);
						Foot_Angle_Control(&CSYS_Foot[2]);
						Foot_IMU_New_Walk_NRTO(Line_x+Variation_x,oval_y,&CSYS_Foot[1],&IMU,&CSYS_Global[1],&oval_ramp[1]);
	//					Foot_New_Walk_RTO(-Line_x,0,Line_x+Variation_x,oval_y,&CSYS_Foot[1],&oval_ramp[1]);
						Foot_Angle_Control(&CSYS_Foot[1]);
					}
					else 
					{
						Foot_IMU_New_Walk_NRTO(Line_x,oval_y,&CSYS_Foot[2],&IMU,&CSYS_Global[2],&oval_ramp[2]);
	//					Foot_New_Walk_RTO(-Line_x,0,Line_x,oval_y,&CSYS_Foot[2],&oval_ramp[2]);
						Foot_Angle_Control(&CSYS_Foot[2]);
						Foot_IMU_New_Walk_NRTO(Line_x,oval_y,&CSYS_Foot[1],&IMU,&CSYS_Global[1],&oval_ramp[1]);
	//					Foot_New_Walk_RTO(-Line_x,0,Line_x,oval_y,&CSYS_Foot[1],&oval_ramp[1]);
						Foot_Angle_Control(&CSYS_Foot[1]);
					}
					if(first_flag)
					{
						Foot_Point_RTO_Ramp(-Line_x,0,&CSYS_Foot[0],&Foot_line_ramp[0]);
						Foot_Angle_Control(&CSYS_Foot[0]);
						Foot_Point_RTO_Ramp(-Line_x,0,&CSYS_Foot[3],&Foot_line_ramp[3]);
						Foot_Angle_Control(&CSYS_Foot[3]);
					}
					ResetSlope(&oval_ramp[2]);
					ResetSlope(&oval_ramp[1]);
					ResetSlope(&Foot_line_ramp[0]);
					ResetSlope(&Foot_line_ramp[3]);
					buffer_flag = 1;
					ResetFlag(&record_flag);
				}
			}
			else
			{
				if(Slope(&Foot_Run_Buffer_ramp) == 1.0f)
				{
					buffer_flag = 0;
					CSYS_Foot[0].Foot_State = Foot_State_Up;
					CSYS_Foot[3].Foot_State = Foot_State_Up;
					CSYS_Foot[1].Foot_State = Foot_State_Down;
					CSYS_Foot[2].Foot_State = Foot_State_Down;
					ResetSlope(&Foot_Run_Buffer_ramp);
				}
			}
		}
		else if(CSYS_Foot[2].Foot_State == Foot_State_Down && CSYS_Foot[1].Foot_State == Foot_State_Down 
					&& CSYS_Foot[0].Foot_State == Foot_State_Up && CSYS_Foot[3].Foot_State == Foot_State_Up)
		{
			if(!buffer_flag)				//初始为buffer_flag
			{
				if(!first_flag)
					first_flag = 1;
				if(!record_flag)
				{
					Record_Direction = Foot_Orient_Calibration_Direction(&IMU,&Calibration_flag);
					record_flag = 1;
				}
				if(Record_Direction > 1.0f)
				{
					Foot_IMU_New_Walk_NRTO(Line_x+Variation_x,oval_y,&CSYS_Foot[0],&IMU,&CSYS_Global[0],&oval_ramp[0]);
					
//					Foot_New_Walk_RTO(-Line_x,0,Line_x+Variation_x,oval_y,&CSYS_Foot[0],&oval_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_IMU_New_Walk_NRTO(Line_x,oval_y,&CSYS_Foot[3],&IMU,&CSYS_Global[3],&oval_ramp[3]);
//					Foot_New_Walk_RTO(-Line_x,0,Line_x,oval_y,&CSYS_Foot[3],&oval_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
				}
				else if(Record_Direction < -1.0f)
				{
					Foot_IMU_New_Walk_NRTO(Line_x,oval_y,&CSYS_Foot[0],&IMU,&CSYS_Global[0],&oval_ramp[0]);
//					Foot_New_Walk_RTO(-Line_x,0,Line_x,oval_y,&CSYS_Foot[0],&oval_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_IMU_New_Walk_NRTO(Line_x+Variation_x,oval_y,&CSYS_Foot[3],&IMU,&CSYS_Global[3],&oval_ramp[3]);
//					Foot_New_Walk_RTO(-Line_x,0,Line_x+Variation_x,oval_y,&CSYS_Foot[3],&oval_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
				}
				else 
				{
					Foot_IMU_New_Walk_NRTO(Line_x,oval_y,&CSYS_Foot[0],&IMU,&CSYS_Global[0],&oval_ramp[0]);
//					Foot_New_Walk_RTO(-Line_x,0,Line_x,oval_y,&CSYS_Foot[0],&oval_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_IMU_New_Walk_NRTO(Line_x,oval_y,&CSYS_Foot[3],&IMU,&CSYS_Global[3],&oval_ramp[3]);
//					Foot_New_Walk_RTO(-Line_x,0,Line_x,oval_y,&CSYS_Foot[3],&oval_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
				}
				Foot_Point_RTO_Ramp(-Line_x,0,&CSYS_Foot[1],&Foot_line_ramp[1]);
				Foot_Angle_Control(&CSYS_Foot[1]);
				Foot_Point_RTO_Ramp(-Line_x,0,&CSYS_Foot[2],&Foot_line_ramp[2]);
				Foot_Angle_Control(&CSYS_Foot[2]);
				
				if(Slope(&oval_ramp[0]) == 1.0f && Slope(&oval_ramp[3]) == 1.0f 
						&& Slope(&Foot_line_ramp[1]) == 1.0f && Slope(&Foot_line_ramp[2]) == 1.0f)
				{
					buffer_flag = 1;
					if(Record_Direction > 1.0f)
					{
						Foot_IMU_New_Walk_NRTO(Line_x+Variation_x,oval_y,&CSYS_Foot[0],&IMU,&CSYS_Global[0],&oval_ramp[0]);
						
	//					Foot_New_Walk_RTO(-Line_x,0,Line_x+Variation_x,oval_y,&CSYS_Foot[0],&oval_ramp[0]);
						Foot_Angle_Control(&CSYS_Foot[0]);
						Foot_IMU_New_Walk_NRTO(Line_x,oval_y,&CSYS_Foot[3],&IMU,&CSYS_Global[3],&oval_ramp[3]);
	//					Foot_New_Walk_RTO(-Line_x,0,Line_x,oval_y,&CSYS_Foot[3],&oval_ramp[3]);
						Foot_Angle_Control(&CSYS_Foot[3]);
					}
					else if(Record_Direction < -1.0f)
					{
						Foot_IMU_New_Walk_NRTO(Line_x,oval_y,&CSYS_Foot[0],&IMU,&CSYS_Global[0],&oval_ramp[0]);
	//					Foot_New_Walk_RTO(-Line_x,0,Line_x,oval_y,&CSYS_Foot[0],&oval_ramp[0]);
						Foot_Angle_Control(&CSYS_Foot[0]);
						Foot_IMU_New_Walk_NRTO(Line_x+Variation_x,oval_y,&CSYS_Foot[3],&IMU,&CSYS_Global[3],&oval_ramp[3]);
	//					Foot_New_Walk_RTO(-Line_x,0,Line_x+Variation_x,oval_y,&CSYS_Foot[3],&oval_ramp[3]);
						Foot_Angle_Control(&CSYS_Foot[3]);
					}
					else 
					{
						Foot_IMU_New_Walk_NRTO(Line_x,oval_y,&CSYS_Foot[0],&IMU,&CSYS_Global[0],&oval_ramp[0]);
	//					Foot_New_Walk_RTO(-Line_x,0,Line_x,oval_y,&CSYS_Foot[0],&oval_ramp[0]);
						Foot_Angle_Control(&CSYS_Foot[0]);
						Foot_IMU_New_Walk_NRTO(Line_x,oval_y,&CSYS_Foot[3],&IMU,&CSYS_Global[3],&oval_ramp[3]);
	//					Foot_New_Walk_RTO(-Line_x,0,Line_x,oval_y,&CSYS_Foot[3],&oval_ramp[3]);
						Foot_Angle_Control(&CSYS_Foot[3]);
					}
					Foot_Point_RTO_Ramp(-Line_x,0,&CSYS_Foot[1],&Foot_line_ramp[1]);
					Foot_Angle_Control(&CSYS_Foot[1]);
					Foot_Point_RTO_Ramp(-Line_x,0,&CSYS_Foot[2],&Foot_line_ramp[2]);
					Foot_Angle_Control(&CSYS_Foot[2]);
					ResetFlag(&record_flag);
					ResetSlope(&Foot_line_ramp[1]);
					ResetSlope(&Foot_line_ramp[2]);
					ResetSlope(&oval_ramp[0]);
					ResetSlope(&oval_ramp[3]);
				}
			}
			else
			{
				if(Slope(&Foot_Run_Buffer_ramp) == 1.0f)
				{
					buffer_flag = 0;
					ResetSlope(&Foot_Run_Buffer_ramp);
					if(Remote_State.Remote_Current_State == Remote_Foot_Cobble_Stop &&
						 Remote_State.Remote_Last_State		 == Remote_Foot_Cobble				)
					{
						ResetFlag(&Calibration_flag);
						Remote_State_Last_Current_Update(Remote_Foot_None);
						CSYS_Foot[0].Foot_State = Foot_State_None;
						CSYS_Foot[1].Foot_State = Foot_State_None;
						CSYS_Foot[2].Foot_State = Foot_State_None;
						CSYS_Foot[3].Foot_State = Foot_State_None;
					}
					else
					{
						CSYS_Foot[1].Foot_State = Foot_State_Up;
						CSYS_Foot[2].Foot_State = Foot_State_Up;
						CSYS_Foot[0].Foot_State = Foot_State_Down;
						CSYS_Foot[3].Foot_State = Foot_State_Down;
					}
				}
			}
		}
	}
	else
	{
		ResetFlag(&first_flag);
		ResetFlag(&buffer_flag);
		ResetFlag(&Calibration_flag);
		ResetFlag(&record_flag);
		for(i=0;i<4;i++)
		{
			CSYS_Foot[i].Foot_State = Foot_State_None;
			ResetSlope(&oval_ramp[i]);
			ResetSlope(&Foot_line_ramp[i]);
		}
		ResetSlope(&Foot_Run_Buffer_ramp);
		Remote_State_Last_Current_Update(Remote_Foot_None);
	}
}

/**
	*@brief 向前走
	*/
void Free_Front_Walk()
{
	static float oval_x = 30.0f;
	static float oval_y = 20.0f;
	static float Line_back = -60.0f;
	if(	CSYS_Foot[2].Foot_State == Foot_State_Up && CSYS_Foot[1].Foot_State == Foot_State_Up &&
			CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
	{
		Foot_Front_Up_Oval_RTO_Ramp(CSYS_Foot[1].x+oval_x,CSYS_Foot[1].y,oval_x,oval_y,&CSYS_Foot[1],&oval_ramp[1]);
		Foot_Angle_Control(&CSYS_Foot[1]);
		Foot_Front_Up_Oval_RTO_Ramp(CSYS_Foot[2].x+oval_x,CSYS_Foot[2].y,oval_x,oval_y,&CSYS_Foot[2],&oval_ramp[2]);
		Foot_Angle_Control(&CSYS_Foot[2]);
		if(Slope(&oval_ramp[1]) == 1.0f && Slope(&oval_ramp[2]) == 1.0f)
		{
			Foot_Front_Up_Oval_RTO_Ramp(CSYS_Foot[1].x+oval_x,CSYS_Foot[1].y,oval_x,oval_y,&CSYS_Foot[1],&oval_ramp[1]);
			Foot_Angle_Control(&CSYS_Foot[1]);
			Foot_Front_Up_Oval_RTO_Ramp(CSYS_Foot[2].x+oval_x,CSYS_Foot[2].y,oval_x,oval_y,&CSYS_Foot[2],&oval_ramp[2]);
			Foot_Angle_Control(&CSYS_Foot[2]);
			CSYS_Foot[0].Foot_State = Foot_State_Up;
			CSYS_Foot[3].Foot_State = Foot_State_Up;
			CSYS_Foot[1].Foot_State = Foot_State_Down;
			CSYS_Foot[2].Foot_State = Foot_State_Down;
			ResetSlope(&oval_ramp[1]);
			ResetSlope(&oval_ramp[2]);
		}
	}
	else if(CSYS_Foot[2].Foot_State == Foot_State_Down && CSYS_Foot[1].Foot_State == Foot_State_Down 
				&& CSYS_Foot[0].Foot_State == Foot_State_Up && CSYS_Foot[3].Foot_State == Foot_State_Up)
	{
		Foot_Front_Up_Oval_RTO_Ramp(CSYS_Foot[0].x+oval_x,CSYS_Foot[0].y,oval_x,oval_y,&CSYS_Foot[0],&oval_ramp[0]);
		Foot_Angle_Control(&CSYS_Foot[0]);
		Foot_Front_Up_Oval_RTO_Ramp(CSYS_Foot[0].x+oval_x,CSYS_Foot[0].y,oval_x,oval_y,&CSYS_Foot[3],&oval_ramp[3]);
		Foot_Angle_Control(&CSYS_Foot[3]);
		Foot_Line_NRTO_Ramp(Line_back,0,&CSYS_Foot[1],&Foot_line_ramp[1]);
		Foot_Angle_Control(&CSYS_Foot[1]);
		Foot_Line_NRTO_Ramp(Line_back,0,&CSYS_Foot[2],&Foot_line_ramp[2]);
		Foot_Angle_Control(&CSYS_Foot[2]);
		if(Slope(&oval_ramp[0]) == 1.0f && Slope(&oval_ramp[3]) == 1.0f 
				&& Slope(&Foot_line_ramp[1]) == 1.0f && Slope(&Foot_line_ramp[2]) == 1.0f)
		{
			Foot_Front_Up_Oval_RTO_Ramp(CSYS_Foot[0].x+oval_x,CSYS_Foot[0].y,oval_x,oval_y,&CSYS_Foot[0],&oval_ramp[0]);
			Foot_Angle_Control(&CSYS_Foot[0]);
			Foot_Front_Up_Oval_RTO_Ramp(CSYS_Foot[0].x+oval_x,CSYS_Foot[0].y,oval_x,oval_y,&CSYS_Foot[3],&oval_ramp[3]);
			Foot_Angle_Control(&CSYS_Foot[3]);
			Foot_Line_NRTO_Ramp(Line_back,0,&CSYS_Foot[1],&Foot_line_ramp[1]);
			Foot_Angle_Control(&CSYS_Foot[1]);
			Foot_Line_NRTO_Ramp(Line_back,0,&CSYS_Foot[2],&Foot_line_ramp[2]);
			Foot_Angle_Control(&CSYS_Foot[2]);
			CSYS_Foot[1].Foot_State = Foot_State_None;
			CSYS_Foot[2].Foot_State = Foot_State_None;
			CSYS_Foot[0].Foot_State = Foot_State_Down;
			CSYS_Foot[3].Foot_State = Foot_State_Down;
			ResetSlope(&Foot_line_ramp[1]);
			ResetSlope(&Foot_line_ramp[2]);
			ResetSlope(&oval_ramp[0]);
			ResetSlope(&oval_ramp[3]);
		}
	}
	else if(CSYS_Foot[2].Foot_State == Foot_State_Down && CSYS_Foot[1].Foot_State == Foot_State_Down 
			&& CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
	{
		Foot_Line_NRTO_Ramp(Line_back,0,&CSYS_Foot[0],&Foot_line_ramp[0]);
		Foot_Angle_Control(&CSYS_Foot[1]);
		Foot_Line_NRTO_Ramp(Line_back,0,&CSYS_Foot[3],&Foot_line_ramp[3]);
		Foot_Angle_Control(&CSYS_Foot[3]);
		if(Slope(&Foot_line_ramp[0]) == 1.0f && Slope(&Foot_line_ramp[3]) == 1.0f)
		{
			Foot_Line_NRTO_Ramp(Line_back,0,&CSYS_Foot[0],&Foot_line_ramp[0]);
			Foot_Angle_Control(&CSYS_Foot[1]);
			Foot_Line_NRTO_Ramp(Line_back,0,&CSYS_Foot[3],&Foot_line_ramp[3]);
			Foot_Angle_Control(&CSYS_Foot[3]);
			ResetSlope(&Foot_line_ramp[0]);
			ResetSlope(&Foot_line_ramp[3]);
			CSYS_Foot[0].Foot_State = Foot_State_None;
			CSYS_Foot[3].Foot_State = Foot_State_None;
			Remote_State_Last_Current_Update(Remote_Foot_None);
		}
	}
}


/**
	*@breif 向后走
	*/
void Free_Back_Walk()
{
	static float oval_x = 30.0f;
	static float oval_y = 20.0f;
	static float Line_back = 60.0f;
	if(	CSYS_Foot[2].Foot_State == Foot_State_Up && CSYS_Foot[1].Foot_State == Foot_State_Up &&
			CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
	{
		Foot_Back_Up_Oval_RTO_Ramp(CSYS_Foot[1].x-oval_x,CSYS_Foot[1].y,oval_x,oval_y,&CSYS_Foot[1],&oval_ramp[1]);
		Foot_Angle_Control(&CSYS_Foot[1]);
		Foot_Back_Up_Oval_RTO_Ramp(CSYS_Foot[2].x-oval_x,CSYS_Foot[2].y,oval_x,oval_y,&CSYS_Foot[2],&oval_ramp[2]);
		Foot_Angle_Control(&CSYS_Foot[2]);
		if(Slope(&oval_ramp[1]) == 1.0f && Slope(&oval_ramp[2]) == 1.0f)
		{
			Foot_Back_Up_Oval_RTO_Ramp(CSYS_Foot[1].x-oval_x,CSYS_Foot[1].y,oval_x,oval_y,&CSYS_Foot[1],&oval_ramp[1]);
			Foot_Angle_Control(&CSYS_Foot[1]);
			Foot_Back_Up_Oval_RTO_Ramp(CSYS_Foot[2].x-oval_x,CSYS_Foot[2].y,oval_x,oval_y,&CSYS_Foot[2],&oval_ramp[2]);
			Foot_Angle_Control(&CSYS_Foot[2]);
			CSYS_Foot[0].Foot_State = Foot_State_Up;
			CSYS_Foot[3].Foot_State = Foot_State_Up;
			CSYS_Foot[1].Foot_State = Foot_State_Down;
			CSYS_Foot[2].Foot_State = Foot_State_Down;
			ResetSlope(&oval_ramp[1]);
			ResetSlope(&oval_ramp[2]);
		}
	}
	else if(CSYS_Foot[2].Foot_State == Foot_State_Down && CSYS_Foot[1].Foot_State == Foot_State_Down 
				&& CSYS_Foot[0].Foot_State == Foot_State_Up && CSYS_Foot[3].Foot_State == Foot_State_Up)
	{
		Foot_Back_Up_Oval_RTO_Ramp(CSYS_Foot[0].x-oval_x,CSYS_Foot[0].y,oval_x,oval_y,&CSYS_Foot[0],&oval_ramp[0]);
		Foot_Angle_Control(&CSYS_Foot[0]);
		Foot_Back_Up_Oval_RTO_Ramp(CSYS_Foot[0].x-oval_x,CSYS_Foot[0].y,oval_x,oval_y,&CSYS_Foot[3],&oval_ramp[3]);
		Foot_Angle_Control(&CSYS_Foot[3]);
		Foot_Line_NRTO_Ramp(Line_back,0,&CSYS_Foot[1],&Foot_line_ramp[1]);
		Foot_Angle_Control(&CSYS_Foot[1]);
		Foot_Line_NRTO_Ramp(Line_back,0,&CSYS_Foot[2],&Foot_line_ramp[2]);
		Foot_Angle_Control(&CSYS_Foot[2]);
		if(Slope(&oval_ramp[0]) == 1.0f && Slope(&oval_ramp[3]) == 1.0f 
				&& Slope(&Foot_line_ramp[1]) == 1.0f && Slope(&Foot_line_ramp[2]) == 1.0f)
		{
			Foot_Back_Up_Oval_RTO_Ramp(CSYS_Foot[0].x-oval_x,CSYS_Foot[0].y,oval_x,oval_y,&CSYS_Foot[0],&oval_ramp[0]);
			Foot_Angle_Control(&CSYS_Foot[0]);
			Foot_Back_Up_Oval_RTO_Ramp(CSYS_Foot[0].x-oval_x,CSYS_Foot[0].y,oval_x,oval_y,&CSYS_Foot[3],&oval_ramp[3]);
			Foot_Angle_Control(&CSYS_Foot[3]);
			Foot_Line_NRTO_Ramp(Line_back,0,&CSYS_Foot[1],&Foot_line_ramp[1]);
			Foot_Angle_Control(&CSYS_Foot[1]);
			Foot_Line_NRTO_Ramp(Line_back,0,&CSYS_Foot[2],&Foot_line_ramp[2]);
			Foot_Angle_Control(&CSYS_Foot[2]);
			CSYS_Foot[1].Foot_State = Foot_State_None;
			CSYS_Foot[2].Foot_State = Foot_State_None;
			CSYS_Foot[0].Foot_State = Foot_State_Down;
			CSYS_Foot[3].Foot_State = Foot_State_Down;
			ResetSlope(&Foot_line_ramp[1]);
			ResetSlope(&Foot_line_ramp[2]);
			ResetSlope(&oval_ramp[0]);
			ResetSlope(&oval_ramp[3]);
		}
	}
	else if(CSYS_Foot[2].Foot_State == Foot_State_Down && CSYS_Foot[1].Foot_State == Foot_State_Down 
			&& CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
	{
		Foot_Line_NRTO_Ramp(Line_back,0,&CSYS_Foot[0],&Foot_line_ramp[0]);
		Foot_Angle_Control(&CSYS_Foot[1]);
		Foot_Line_NRTO_Ramp(Line_back,0,&CSYS_Foot[3],&Foot_line_ramp[3]);
		Foot_Angle_Control(&CSYS_Foot[3]);
		if(Slope(&Foot_line_ramp[0]) == 1.0f && Slope(&Foot_line_ramp[3]) == 1.0f)
		{
			Foot_Line_NRTO_Ramp(Line_back,0,&CSYS_Foot[0],&Foot_line_ramp[0]);
			Foot_Angle_Control(&CSYS_Foot[1]);
			Foot_Line_NRTO_Ramp(Line_back,0,&CSYS_Foot[3],&Foot_line_ramp[3]);
			Foot_Angle_Control(&CSYS_Foot[3]);
			ResetSlope(&Foot_line_ramp[0]);
			ResetSlope(&Foot_line_ramp[3]);
			CSYS_Foot[0].Foot_State = Foot_State_None;
			CSYS_Foot[3].Foot_State = Foot_State_None;
			Remote_State_Last_Current_Update(Remote_Foot_None);
		}
	}
}

/**
	*@brief 向左转
	*/
void Foot_Turn_Left()
{
	static float oval_x = 20.0f;
	static float oval_y = 20.0f;
	static float line		= 40.0f;
	if(CSYS_Foot[0].Foot_State == Foot_State_Up && CSYS_Foot[3].Foot_State == Foot_State_Up &&
		 CSYS_Foot[1].Foot_State == Foot_State_Down && CSYS_Foot[2].Foot_State == Foot_State_Down)
	{
		Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Turn_Oval_ramp[0]);
		Foot_Angle_Control(&CSYS_Foot[0]);
		Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Turn_Oval_ramp[3]);
		Foot_Angle_Control(&CSYS_Foot[3]);
		if(Slope(&Foot_Turn_Oval_ramp[0]) == 1.0f && Slope(&Foot_Turn_Oval_ramp[3]) == 1.0f)
		{
			Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Turn_Oval_ramp[0]);
			Foot_Angle_Control(&CSYS_Foot[0]);
			Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Turn_Oval_ramp[3]);
			Foot_Angle_Control(&CSYS_Foot[3]);
			ResetSlope(&Foot_Turn_Oval_ramp[0]);
			ResetSlope(&Foot_Turn_Oval_ramp[3]);
			CSYS_Foot[1].Foot_State = Foot_State_Up;
			CSYS_Foot[2].Foot_State = Foot_State_Up;
		}
	}
	else if(CSYS_Foot[0].Foot_State == Foot_State_Up && CSYS_Foot[3].Foot_State == Foot_State_Up &&
					CSYS_Foot[1].Foot_State == Foot_State_Up && CSYS_Foot[2].Foot_State == Foot_State_Up )
	{
		Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[2],&Foot_Turn_Oval_ramp[2]);
		Foot_Angle_Control(&CSYS_Foot[2]);
		Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[1],&Foot_Turn_Oval_ramp[1]);
		Foot_Angle_Control(&CSYS_Foot[1]);
		if(Slope(&Foot_Turn_Oval_ramp[2]) == 1.0f && Slope(&Foot_Turn_Oval_ramp[1]) == 1.0f)
		{
			Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[2],&Foot_Turn_Oval_ramp[2]);
			Foot_Angle_Control(&CSYS_Foot[2]);
			Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[1],&Foot_Turn_Oval_ramp[1]);
			Foot_Angle_Control(&CSYS_Foot[1]);
			for(i=0;i<4;i++)
			{
				CSYS_Foot[i].Foot_State = Foot_State_Down;
				ResetSlope(&Foot_Turn_Oval_ramp[i]);
			}
		}
	}
	else if(CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[1].Foot_State == Foot_State_Down &&
					CSYS_Foot[2].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
	{
		Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[0],&Foot_Turn_Line_ramp[0]);
		Foot_Angle_Control(&CSYS_Foot[0]);
		Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[2],&Foot_Turn_Line_ramp[2]);
		Foot_Angle_Control(&CSYS_Foot[2]);
		Foot_Line_NRTO_Ramp(-line,0,&CSYS_Foot[1],&Foot_Turn_Line_ramp[1]);
		Foot_Angle_Control(&CSYS_Foot[1]);
		Foot_Line_NRTO_Ramp(-line,0,&CSYS_Foot[3],&Foot_Turn_Line_ramp[3]);
		Foot_Angle_Control(&CSYS_Foot[3]);
		if(Slope(&Foot_Turn_Oval_ramp[1]) == 1.0f && Slope(&Foot_Turn_Oval_ramp[2]) == 1.0f)
		{
			Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[0],&Foot_Turn_Line_ramp[0]);
			Foot_Angle_Control(&CSYS_Foot[0]);
			Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[2],&Foot_Turn_Line_ramp[2]);
			Foot_Angle_Control(&CSYS_Foot[2]);
			Foot_Line_NRTO_Ramp(-line,0,&CSYS_Foot[1],&Foot_Turn_Line_ramp[1]);
			Foot_Angle_Control(&CSYS_Foot[1]);
			Foot_Line_NRTO_Ramp(-line,0,&CSYS_Foot[3],&Foot_Turn_Line_ramp[3]);
			Foot_Angle_Control(&CSYS_Foot[3]);
			for(i=0;i<4;i++)
			{
				CSYS_Foot[i].Foot_State = Foot_State_None;
				ResetSlope(&Foot_Turn_Line_ramp[i]);
			}
			Remote_State_Last_Current_Update(Remote_Foot_None);
		}
	}
}

/**
	*@brief 向右转
	*/
void Foot_Turn_Right()
{
	static float oval_x = 20.0f;
	static float oval_y = 20.0f;
	static float line		= 40.0f;
	if(CSYS_Foot[1].Foot_State == Foot_State_Up && CSYS_Foot[2].Foot_State == Foot_State_Up &&
		 CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
	{
		Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[1],&Foot_Turn_Oval_ramp[1]);
		Foot_Angle_Control(&CSYS_Foot[1]);
		Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[2],&Foot_Turn_Oval_ramp[2]);
		Foot_Angle_Control(&CSYS_Foot[2]);
		if(Slope(&Foot_Turn_Oval_ramp[1]) == 1.0f && Slope(&Foot_Turn_Oval_ramp[2]) == 1.0f)
		{
			Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[1],&Foot_Turn_Oval_ramp[1]);
			Foot_Angle_Control(&CSYS_Foot[1]);
			Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[2],&Foot_Turn_Oval_ramp[2]);
			Foot_Angle_Control(&CSYS_Foot[2]);
			ResetSlope(&Foot_Turn_Oval_ramp[1]);
			ResetSlope(&Foot_Turn_Oval_ramp[2]);
			CSYS_Foot[0].Foot_State = Foot_State_Up;
			CSYS_Foot[3].Foot_State = Foot_State_Up;
		}
	}
	else if(CSYS_Foot[0].Foot_State == Foot_State_Up && CSYS_Foot[3].Foot_State == Foot_State_Up &&
					CSYS_Foot[1].Foot_State == Foot_State_Up && CSYS_Foot[2].Foot_State == Foot_State_Up )
	{
		Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Turn_Oval_ramp[3]);
		Foot_Angle_Control(&CSYS_Foot[3]);
		Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Turn_Oval_ramp[0]);
		Foot_Angle_Control(&CSYS_Foot[0]);
		if(Slope(&Foot_Turn_Oval_ramp[3]) == 1.0f && Slope(&Foot_Turn_Oval_ramp[0]) == 1.0f)
		{
			Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Turn_Oval_ramp[3]);
			Foot_Angle_Control(&CSYS_Foot[3]);
			Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Turn_Oval_ramp[0]);
			Foot_Angle_Control(&CSYS_Foot[0]);
			for(i=0;i<4;i++)
			{
				CSYS_Foot[i].Foot_State = Foot_State_Down;
				ResetSlope(&Foot_Turn_Oval_ramp[i]);
			}
		}
	}
	else if(CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[1].Foot_State == Foot_State_Down &&
					CSYS_Foot[2].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
	{
		Foot_Line_NRTO_Ramp(-line,0,&CSYS_Foot[0],&Foot_Turn_Line_ramp[0]);
		Foot_Angle_Control(&CSYS_Foot[0]);
		Foot_Line_NRTO_Ramp(-line,0,&CSYS_Foot[2],&Foot_Turn_Line_ramp[2]);
		Foot_Angle_Control(&CSYS_Foot[2]);
		Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[1],&Foot_Turn_Line_ramp[1]);
		Foot_Angle_Control(&CSYS_Foot[1]);
		Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[3],&Foot_Turn_Line_ramp[3]);
		Foot_Angle_Control(&CSYS_Foot[3]);
		if(Slope(&Foot_Turn_Oval_ramp[1]) == 1.0f && Slope(&Foot_Turn_Oval_ramp[2]) == 1.0f)
		{
			Foot_Line_NRTO_Ramp(-line,0,&CSYS_Foot[0],&Foot_Turn_Line_ramp[0]);
			Foot_Angle_Control(&CSYS_Foot[0]);
			Foot_Line_NRTO_Ramp(-line,0,&CSYS_Foot[2],&Foot_Turn_Line_ramp[2]);
			Foot_Angle_Control(&CSYS_Foot[2]);
			Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[1],&Foot_Turn_Line_ramp[1]);
			Foot_Angle_Control(&CSYS_Foot[1]);
			Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[3],&Foot_Turn_Line_ramp[3]);
			Foot_Angle_Control(&CSYS_Foot[3]);
			for(i=0;i<4;i++)
			{
				CSYS_Foot[i].Foot_State = Foot_State_None;
				ResetSlope(&Foot_Turn_Line_ramp[i]);
			}
			Remote_State_Last_Current_Update(Remote_Foot_None);
		}
	}
}

/**
	*@brief 腿伸长
	*/
void Foot_Lengthen()
{
	static float lengthen = 100.0f;
	if(CSYS_Foot[0].Foot_State == Foot_State_Lengthen && CSYS_Foot[1].Foot_State == Foot_State_Lengthen &&
		 CSYS_Foot[2].Foot_State == Foot_State_Lengthen && CSYS_Foot[3].Foot_State == Foot_State_Lengthen)
	{
		for(i=0;i<4;i++)
		{
			Foot_Point_RTO_Ramp(0,lengthen,&CSYS_Foot[i],&Foot_Lengthen_ramp[i]);
			Foot_Angle_Control(&CSYS_Foot[i]);
		}
		if(Slope(&Foot_Lengthen_ramp[0]) == 1.0f && Slope(&Foot_Lengthen_ramp[1]) == 1.0f &&
			 Slope(&Foot_Lengthen_ramp[2]) == 1.0f && Slope(&Foot_Lengthen_ramp[3]) == 1.0f)
		{
			for(i=0;i<4;i++)
			{
				Foot_Point_RTO_Ramp(0,lengthen,&CSYS_Foot[i],&Foot_Lengthen_ramp[i]);
				Foot_Angle_Control(&CSYS_Foot[i]);
				ResetSlope(&Foot_Lengthen_ramp[i]);
				CSYS_Foot[i].Foot_State = Foot_State_None;
			}
			
		}
	}
	
}

void Foot_Front_Wood_Up()
{
	
}

/**
	*@breif 前跳准备
	*/
void Front_Ready_Jump()
{
	if(CSYS_Foot[0].Foot_State == Foot_State_Front_Jump_Ready && CSYS_Foot[1].Foot_State == Foot_State_Front_Jump_Ready &&
		 CSYS_Foot[2].Foot_State == Foot_State_Front_Jump_Ready && CSYS_Foot[3].Foot_State == Foot_State_Front_Jump_Ready)
	{
		
		static float line_x = 0;
		static float line_y = 100.0f;
		for(i=0;i<4;i++)
		{
			Foot_Point_RTO_Ramp(line_x,line_y,&CSYS_Foot[i],&Foot_Front_Jump_Ready_ramp[i]);
			Foot_Angle_Control(&CSYS_Foot[i]);
		}
//		#ifdef Remote
		if(Slope(&Foot_Front_Jump_Ready_ramp[0]) == 1.0f && Slope(&Foot_Front_Jump_Ready_ramp[1]) == 1.0f &&
			 Slope(&Foot_Front_Jump_Ready_ramp[2]) == 1.0f && Slope(&Foot_Front_Jump_Ready_ramp[3]) == 1.0f)
		{
			for(i=0;i<4;i++)
			{
				Foot_Point_RTO_Ramp(line_x,line_y,&CSYS_Foot[i],&Foot_Front_Jump_Ready_ramp[i]);
				Foot_Angle_Control(&CSYS_Foot[i]);
				ResetSlope(&Foot_Front_Jump_Ready_ramp[i]);
				CSYS_Foot[i].Foot_State = Foot_State_Front_Jump_Up;
			}
		}
//		#endif
	}
}

/**
	*@breif 前跳
	*/
void Front_Jump()
{
	static uint8_t first_flag;
	static uint8_t buffer_flag;
	if(!first_flag)
	{
		for(i=0;i<4;i++)
			CSYS_Foot[i].Foot_State = Foot_State_Front_Jump_Ready;
		first_flag = 1;
	}
	if(CSYS_Foot[0].Foot_State == Foot_State_Front_Jump_Ready && CSYS_Foot[1].Foot_State == Foot_State_Front_Jump_Ready &&
		 CSYS_Foot[2].Foot_State == Foot_State_Front_Jump_Ready && CSYS_Foot[3].Foot_State == Foot_State_Front_Jump_Ready)
	{
		static float x = -40.0f;
		static float y = 100.0f;
		if(!buffer_flag)
		{
			for(i=0;i<4;i++)
			{
				Foot_Point_RTO_Ramp(x,y,&CSYS_Foot[i],&Foot_Front_Jump_Ready_ramp[i]);
				Foot_Angle_Control(&CSYS_Foot[i]);
			}
			if(Slope(&Foot_Front_Jump_Ready_ramp[0]) == 1.0f && Slope(&Foot_Front_Jump_Ready_ramp[1]) == 1.0f &&
				 Slope(&Foot_Front_Jump_Ready_ramp[2]) == 1.0f && Slope(&Foot_Front_Jump_Ready_ramp[3]) == 1.0f)
			{
				for(i=0;i<4;i++)
					ResetSlope(&Foot_Front_Jump_Ready_ramp[i]);
				buffer_flag = 1;
			}
		}
		else
		{
			if(Slope(&Foot_Front_Jump_Butter_ramp) == 1.0f)
			{
				ResetSlope(&Foot_Front_Jump_Butter_ramp);
				for(i=0;i<4;i++)
					CSYS_Foot[i].Foot_State = Foot_State_Front_Jump_Up;
				ResetFlag(&buffer_flag);
			}
		}
	}
	else if(CSYS_Foot[0].Foot_State == Foot_State_Front_Jump_Up && CSYS_Foot[1].Foot_State == Foot_State_Front_Jump_Up &&
					CSYS_Foot[2].Foot_State == Foot_State_Front_Jump_Up && CSYS_Foot[3].Foot_State == Foot_State_Front_Jump_Up)
	{
		static float x = -80.0f;
		static float y = -50.0f;
		static uint8_t flag;
		if(!flag)
		{
			for(i=0;i<4;i++)
			{
				Foot_Point_RTO(x,y,&CSYS_Foot[i]);
				Foot_Angle_Control(&CSYS_Foot[i]);
			}
			flag = 1;
		}
//		#if 0
		if(Slope(&Foot_Front_Jump_Butter_ramp) == 1.0f)
		{
			for(i=0;i<4;i++)
			{
				CSYS_Foot[i].Foot_State = Foot_State_Front_Jump_Down;
			}
			ResetSlope(&Foot_Front_Jump_Butter_ramp);
			flag = 0;
		}
//		#endif
	}
//	#if 0
	else if(CSYS_Foot[0].Foot_State == Foot_State_Front_Jump_Down && CSYS_Foot[1].Foot_State == Foot_State_Front_Jump_Down &&
					CSYS_Foot[2].Foot_State == Foot_State_Front_Jump_Down && CSYS_Foot[3].Foot_State == Foot_State_Front_Jump_Down)
	{
		static float x = 0;
		static float y = 0;
		static uint8_t flag;
		
		for(i=0;i<4;i++)
		{
			Foot_Point_RTO(-40.0f,100.0f,&CSYS_Foot[i]);
			Foot_Angle_Control(&CSYS_Foot[i]);
		}
		
		if(!flag)
		{
			if(Slope(&Foot_Front_Jump_Butter_ramp) == 1.0f)
			{
				for(i=0;i<4;i++)
				{
					CSYS_Foot[i].Foot_State = Foot_State_Front_Jump_Down;
				}
				ResetSlope(&Foot_Front_Jump_Butter_ramp);
				flag = 1;
			}
		}
		else
		{
			for(i=0;i<4;i++)
			{
				Foot_Point_RTO_Ramp(x,y,&CSYS_Foot[i],&Foot_Front_Jump_Down_ramp[i]);
				Foot_Angle_Control(&CSYS_Foot[i]);
			}
			if(Slope(&Foot_Front_Jump_Down_ramp[0]) == 1.0f && Slope(&Foot_Front_Jump_Down_ramp[1]) == 1.0f && 
				 Slope(&Foot_Front_Jump_Down_ramp[2]) == 1.0f && Slope(&Foot_Front_Jump_Down_ramp[3]) == 1.0f)
			{
				for(i=0;i<4;i++)
				{
					ResetSlope(&Foot_Front_Jump_Down_ramp[i]);
					CSYS_Foot[i].Foot_State = Foot_State_None;
				}
				ResetFlag(&flag);
				ResetFlag(&first_flag);
				Remote_State_Last_Current_Update(Remote_Foot_None);
			}
		}
	}
//	#endif
}  



/**
	*@breif 排管
	*/
void Foot_Calandria()
{
	static uint8_t Part_State;
	static uint8_t Start_Walk_flag;
	static uint8_t buffer_flag;
	static uint8_t t;
	static uint8_t stop_flag;
	if(Remote_State.Remote_Current_State != Remote_Foot_Emergency_Stop)
	{
		if(Part_State == 0)
		{
			static float line = -50.0f;
			if(!buffer_flag)
			{
				if(CSYS_Foot[0].Foot_State == Foot_State_Run_Ready && CSYS_Foot[1].Foot_State == Foot_State_Run_Ready &&
					 CSYS_Foot[2].Foot_State == Foot_State_Run_Ready && CSYS_Foot[3].Foot_State == Foot_State_Run_Ready)
				{
					for(i=0;i<4;i++)
					{
						Foot_Point_RTO_Ramp(line,0,&CSYS_Foot[i],&Foot_Calandria_Walk_Ready_ramp[i]);
						Foot_Angle_Control(&CSYS_Foot[i]);
					}
					if(Slope(&Foot_Calandria_Walk_Ready_ramp[0]) == 1.0f && Slope(&Foot_Calandria_Walk_Ready_ramp[1]) == 1.0f &&
						 Slope(&Foot_Calandria_Walk_Ready_ramp[2]) == 1.0f && Slope(&Foot_Calandria_Walk_Ready_ramp[3]) == 1.0f)
					{
						for(i=0;i<4;i++)
						{
							buffer_flag = 1;
							Foot_Point_RTO_Ramp(line,0,&CSYS_Foot[i],&Foot_Calandria_Walk_Ready_ramp[i]);
							Foot_Angle_Control(&CSYS_Foot[i]);
							ResetSlope(&Foot_Calandria_Walk_Ready_ramp[i]);
						}
					}
				}
			}
			else
			{
				if(Slope(&Foot_Calandria_Lengthen_Buffer_ramp) == 1.0f)
				{
					CSYS_Foot[1].Foot_State = Foot_State_Up;
					CSYS_Foot[2].Foot_State = Foot_State_Up;
					CSYS_Foot[0].Foot_State = Foot_State_Down;
					CSYS_Foot[3].Foot_State = Foot_State_Down;
					ResetFlag(&buffer_flag);
					ResetSlope(&Foot_Calandria_Lengthen_Buffer_ramp);
					Part_State = 1;
				}
			}
		}
		else if(Part_State == 1)
		{
			static float oval_x = 25.0f;
			static float oval_y = 50.0f;
			static float line		= -50.0f;
			static float s = 50.0f;
			static float h = 50.0f;
			if(CSYS_Foot[1].Foot_State == Foot_State_Up && CSYS_Foot[2].Foot_State == Foot_State_Up &&
				 CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
			{
				if(!buffer_flag)
				{
					if(Start_Walk_flag)			//开始行走标志位
					{
						Foot_Point_RTO_Ramp(line,0,&CSYS_Foot[0],&Foot_Calandria_Walk_Line_ramp[0]);
						Foot_Angle_Control(&CSYS_Foot[0]);
						Foot_Point_RTO_Ramp(line,0,&CSYS_Foot[3],&Foot_Calandria_Walk_Line_ramp[3]);
						Foot_Angle_Control(&CSYS_Foot[3]);
					}
					Foot_IMU_New_Walk_NRTO(s,h,&CSYS_Foot[1],&IMU,&CSYS_Global[1],&Foot_Calandria_Walk_Oval_ramp[1]);
//					Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[1],&Foot_Calandria_Walk_Oval_ramp[1]);
					Foot_Angle_Control(&CSYS_Foot[1]);
					Foot_IMU_New_Walk_NRTO(s,h,&CSYS_Foot[2],&IMU,&CSYS_Global[2],&Foot_Calandria_Walk_Oval_ramp[2]);
//					Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[2],&Foot_Calandria_Walk_Oval_ramp[2]);
					Foot_Angle_Control(&CSYS_Foot[2]);
					if(Slope(&Foot_Calandria_Walk_Oval_ramp[1]) == 1.0f && Slope(&Foot_Calandria_Walk_Oval_ramp[2]) == 1.0f)
					{
						if(Start_Walk_flag)			//开始行走标志位
						{
							Foot_Point_RTO_Ramp(line,0,&CSYS_Foot[0],&Foot_Calandria_Walk_Line_ramp[0]);
							Foot_Angle_Control(&CSYS_Foot[0]);
							Foot_Point_RTO_Ramp(line,0,&CSYS_Foot[3],&Foot_Calandria_Walk_Line_ramp[3]);
							Foot_Angle_Control(&CSYS_Foot[3]);
							ResetSlope(&Foot_Calandria_Walk_Line_ramp[0]);
							ResetSlope(&Foot_Calandria_Walk_Line_ramp[3]);
						}
						Foot_IMU_New_Walk_NRTO(s,h,&CSYS_Foot[1],&IMU,&CSYS_Global[1],&Foot_Calandria_Walk_Oval_ramp[1]);
//						Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[1],&Foot_Calandria_Walk_Oval_ramp[1]);
						Foot_Angle_Control(&CSYS_Foot[1]);
						Foot_IMU_New_Walk_NRTO(s,h,&CSYS_Foot[2],&IMU,&CSYS_Global[2],&Foot_Calandria_Walk_Oval_ramp[2]);
//						Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[2],&Foot_Calandria_Walk_Oval_ramp[2]);
						Foot_Angle_Control(&CSYS_Foot[2]);
						ResetSlope(&Foot_Calandria_Walk_Oval_ramp[1]);
						ResetSlope(&Foot_Calandria_Walk_Oval_ramp[2]);
						buffer_flag = 1;
					}
				}
				else
				{
					if(Slope(&Foot_Calandria_Walk_Buffer_ramp) == 1.0f)
					{
						ResetSlope(&Foot_Calandria_Walk_Buffer_ramp);
						buffer_flag = 0;
						CSYS_Foot[0].Foot_State = Foot_State_Up;
						CSYS_Foot[3].Foot_State = Foot_State_Up;
						CSYS_Foot[1].Foot_State = Foot_State_Down;
						CSYS_Foot[2].Foot_State = Foot_State_Down;
					}
				}
			}
			else if(CSYS_Foot[0].Foot_State == Foot_State_Up 	 && CSYS_Foot[3].Foot_State == Foot_State_Up &&
							CSYS_Foot[1].Foot_State == Foot_State_Down && CSYS_Foot[2].Foot_State == Foot_State_Down)
			{
				if(!buffer_flag)
				{
					if(!Start_Walk_flag)	Start_Walk_flag = 1;
					Foot_IMU_New_Walk_NRTO(s,h,&CSYS_Foot[0],&IMU,&CSYS_Global[0],&Foot_Calandria_Walk_Oval_ramp[0]);					
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_IMU_New_Walk_NRTO(s,h,&CSYS_Foot[3],&IMU,&CSYS_Global[3],&Foot_Calandria_Walk_Oval_ramp[3]);					
					Foot_Angle_Control(&CSYS_Foot[3]);
					Foot_Point_RTO_Ramp(line,0,&CSYS_Foot[1],&Foot_Calandria_Walk_Line_ramp[1]);
					Foot_Angle_Control(&CSYS_Foot[1]);
					Foot_Point_RTO_Ramp(line,0,&CSYS_Foot[2],&Foot_Calandria_Walk_Line_ramp[2]);
					Foot_Angle_Control(&CSYS_Foot[2]);
//					Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[0],&Foot_Calandria_Walk_Oval_ramp[0]);
//					Foot_Angle_Control(&CSYS_Foot[0]);
//					Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[3],&Foot_Calandria_Walk_Oval_ramp[3]);
//					Foot_Angle_Control(&CSYS_Foot[3]);
//					Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[1],&Foot_Calandria_Walk_Line_ramp[1]);
//					Foot_Angle_Control(&CSYS_Foot[1]);
//					Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[2],&Foot_Calandria_Walk_Line_ramp[2]);
//					Foot_Angle_Control(&CSYS_Foot[2]);
					if(Slope(&Foot_Calandria_Walk_Oval_ramp[0]) == 1.0f && Slope(&Foot_Calandria_Walk_Oval_ramp[3]) == 1.0f &&
						 Slope(&Foot_Calandria_Walk_Line_ramp[1]) == 1.0f && Slope(&Foot_Calandria_Walk_Line_ramp[2]) == 1.0f)
					{
						Foot_IMU_New_Walk_NRTO(s,h,&CSYS_Foot[0],&IMU,&CSYS_Global[0],&Foot_Calandria_Walk_Oval_ramp[0]);					
						Foot_Angle_Control(&CSYS_Foot[0]);
						Foot_IMU_New_Walk_NRTO(s,h,&CSYS_Foot[3],&IMU,&CSYS_Global[3],&Foot_Calandria_Walk_Oval_ramp[3]);
						Foot_Angle_Control(&CSYS_Foot[3]);
						Foot_Point_RTO_Ramp(line,0,&CSYS_Foot[1],&Foot_Calandria_Walk_Line_ramp[1]);
						Foot_Angle_Control(&CSYS_Foot[1]);
						Foot_Point_RTO_Ramp(line,0,&CSYS_Foot[2],&Foot_Calandria_Walk_Line_ramp[2]);
						Foot_Angle_Control(&CSYS_Foot[2]);
//						Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[0],&Foot_Calandria_Walk_Oval_ramp[0]);
//						Foot_Angle_Control(&CSYS_Foot[0]);
//						Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[3],&Foot_Calandria_Walk_Oval_ramp[3]);
//						Foot_Angle_Control(&CSYS_Foot[3]);
//						Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[1],&Foot_Calandria_Walk_Line_ramp[1]);
//						Foot_Angle_Control(&CSYS_Foot[1]);
//						Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[2],&Foot_Calandria_Walk_Line_ramp[2]);
//						Foot_Angle_Control(&CSYS_Foot[2]);
						ResetSlope(&Foot_Calandria_Walk_Oval_ramp[0]);
						ResetSlope(&Foot_Calandria_Walk_Oval_ramp[3]);
						ResetSlope(&Foot_Calandria_Walk_Line_ramp[1]);
						ResetSlope(&Foot_Calandria_Walk_Line_ramp[2]);
						buffer_flag = 1;
					}
				}
				else
				{
					if(!stop_flag)
					{
						if(Slope(&Foot_Calandria_Walk_Buffer_ramp) == 1.0f)
						{
							ResetSlope(&Foot_Calandria_Walk_Buffer_ramp);
							if(++t == 7)
							{
								stop_flag = 1;
								t=0;
							}
							else
							{
								buffer_flag = 0;
								CSYS_Foot[0].Foot_State = Foot_State_Down;
								CSYS_Foot[3].Foot_State = Foot_State_Down;
								CSYS_Foot[1].Foot_State = Foot_State_Up;
								CSYS_Foot[2].Foot_State = Foot_State_Up;
							}
						}
					}
					else
					{
						Foot_New_Walk(-s,h,&CSYS_Foot[0],&Foot_Calandria_Walk_Oval_ramp[0]);
//						Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Calandria_Walk_Oval_ramp[0]);
						Foot_Angle_Control(&CSYS_Foot[0]);
						Foot_New_Walk(-s,h,&CSYS_Foot[3],&Foot_Calandria_Walk_Oval_ramp[3]);
//						Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Calandria_Walk_Oval_ramp[3]);
						Foot_Angle_Control(&CSYS_Foot[3]);
						if(Slope(&Foot_Calandria_Walk_Oval_ramp[0]) == 1.0f && Slope(&Foot_Calandria_Walk_Oval_ramp[3]) == 1.0f)
						{
							Foot_New_Walk(-s,h,&CSYS_Foot[0],&Foot_Calandria_Walk_Oval_ramp[0]);
//							Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Calandria_Walk_Oval_ramp[0]);
							Foot_Angle_Control(&CSYS_Foot[0]);
							Foot_New_Walk(-s,h,&CSYS_Foot[3],&Foot_Calandria_Walk_Oval_ramp[3]);
//							Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Calandria_Walk_Oval_ramp[3]);
							Foot_Angle_Control(&CSYS_Foot[3]);
							ResetSlope(&Foot_Calandria_Walk_Oval_ramp[0]);
							ResetSlope(&Foot_Calandria_Walk_Oval_ramp[3]);
							Part_State = 2;
							stop_flag = 0;
							buffer_flag = 0;
							Start_Walk_flag = 0;
							for(i=0;i<4;i++)
								CSYS_Foot[i].Foot_State = Foot_State_Lengthen;
						}
					}
				}
			}
		}
	//	#if 0
		else if(Part_State == 2)
		{
			static uint8_t Len = 100.0f;
			static float line = -50.0f;
			if(CSYS_Foot[0].Foot_State == Foot_State_Lengthen && CSYS_Foot[1].Foot_State == Foot_State_Lengthen &&
				 CSYS_Foot[2].Foot_State == Foot_State_Lengthen && CSYS_Foot[3].Foot_State == Foot_State_Lengthen)
			{
				if(!buffer_flag)
				{
					if(Slope(&Foot_Calandria_Lengthen_Buffer_ramp) == 1.0f)
					{
						buffer_flag = 1;
						ResetSlope(&Foot_Calandria_Lengthen_Buffer_ramp);
					}
				}
				else if(buffer_flag == 1)
				{
					for(i=0;i<4;i++)
					{
						Foot_Point_RTO_Ramp(line,-Len,&CSYS_Foot[i],&Foot_Lengthen_ramp[i]);
						Foot_Angle_Control(&CSYS_Foot[i]);
					}
					if(Slope(&Foot_Lengthen_ramp[0]) == 1.0f && Slope(&Foot_Lengthen_ramp[1]) == 1.0f &&
						 Slope(&Foot_Lengthen_ramp[2]) == 1.0f && Slope(&Foot_Lengthen_ramp[3]) == 1.0f)
					{
						for(i=0;i<4;i++)
						{
							Foot_Point_RTO_Ramp(line,-Len,&CSYS_Foot[i],&Foot_Lengthen_ramp[i]);
							Foot_Angle_Control(&CSYS_Foot[i]);
							ResetSlope(&Foot_Lengthen_ramp[i]);
						}
						buffer_flag = 2;
						
					}
				}
				else if(buffer_flag == 2)
				{
					if(Slope(&Foot_Calandria_Lengthen_Buffer_ramp) == 1.0f)
					{
						ResetSlope(&Foot_Calandria_Lengthen_Buffer_ramp);
						CSYS_Foot[1].Foot_State = Foot_State_Up;
						CSYS_Foot[2].Foot_State = Foot_State_Up;
						CSYS_Foot[0].Foot_State = Foot_State_Down;
						CSYS_Foot[3].Foot_State = Foot_State_Down;
						Part_State = 3;
						buffer_flag = 0;
					}
				}
			}
		}
	//	#if 0
		else if(Part_State == 3)
		{
			static uint8_t t;
			static float oval_x = 50.f;
			static float oval_y = 100.f;
			static float line 	=-100.f;
			static float line_1 = -50.0f;
			static float len		= 100.0f;
			static float s = 100.0f;
			static float h = 100.0f;
			if(CSYS_Foot[1].Foot_State == Foot_State_Up && CSYS_Foot[2].Foot_State == Foot_State_Up &&
				 CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
			{
				if(!buffer_flag)
				{
//					if(Start_Walk_flag)			//开始行走标志位
//					{
//						Foot_Point_RTO_Ramp(line_1,-len,&CSYS_Foot[0],&Foot_Calandria_Span_Line_ramp[0]);
////						Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[0],&Foot_Calandria_Span_Line_ramp[0]);
//						Foot_Angle_Control(&CSYS_Foot[0]);
//						Foot_Point_RTO_Ramp(line_1,-len,&CSYS_Foot[3],&Foot_Calandria_Span_Line_ramp[3]);
////						Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[3],&Foot_Calandria_Span_Line_ramp[3]);
//						Foot_Angle_Control(&CSYS_Foot[3]);
//					}
					Foot_New_Walk(s,h,&CSYS_Foot[1],&Foot_Calandria_Span_Oval_ramp[1]);
//					Foot_Front_Up_Oval_RTO_Ramp(oval_x,-len,oval_x,oval_y,&CSYS_Foot[1],&Foot_Calandria_Span_Oval_ramp[1]);
					Foot_Angle_Control(&CSYS_Foot[1]);
					Foot_New_Walk(s,h,&CSYS_Foot[2],&Foot_Calandria_Span_Oval_ramp[2]);
//					Foot_Front_Up_Oval_RTO_Ramp(oval_x,-len,oval_x,oval_y,&CSYS_Foot[2],&Foot_Calandria_Span_Oval_ramp[2]);
					Foot_Angle_Control(&CSYS_Foot[2]);
					if(Slope(&Foot_Calandria_Span_Oval_ramp[1]) == 1.0f && Slope(&Foot_Calandria_Span_Oval_ramp[2]) == 1.0f)
					{
//						if(Start_Walk_flag)			//开始行走标志位
//						{
//							Foot_Point_RTO_Ramp(line_1,-len,&CSYS_Foot[0],&Foot_Calandria_Span_Line_ramp[0]);
////							Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[0],&Foot_Calandria_Span_Line_ramp[0]);
//							Foot_Angle_Control(&CSYS_Foot[0]);
//							Foot_Point_RTO_Ramp(line_1,-len,&CSYS_Foot[3],&Foot_Calandria_Span_Line_ramp[3]);
////							Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[3],&Foot_Calandria_Span_Line_ramp[3]);
//							Foot_Angle_Control(&CSYS_Foot[3]);
//							ResetSlope(&Foot_Calandria_Span_Line_ramp[0]);
//							ResetSlope(&Foot_Calandria_Span_Line_ramp[3]);
//						}
						Foot_New_Walk(s,h,&CSYS_Foot[1],&Foot_Calandria_Span_Oval_ramp[1]);
//						Foot_Front_Up_Oval_RTO_Ramp(oval_x,-len,oval_x,oval_y,&CSYS_Foot[1],&Foot_Calandria_Span_Oval_ramp[1]);
						Foot_Angle_Control(&CSYS_Foot[1]);
						Foot_New_Walk(s,h,&CSYS_Foot[2],&Foot_Calandria_Span_Oval_ramp[2]);
//						Foot_Front_Up_Oval_RTO_Ramp(oval_x,-len,oval_x,oval_y,&CSYS_Foot[2],&Foot_Calandria_Span_Oval_ramp[2]);
						Foot_Angle_Control(&CSYS_Foot[2]);
						ResetSlope(&Foot_Calandria_Span_Oval_ramp[1]);
						ResetSlope(&Foot_Calandria_Span_Oval_ramp[2]);
						buffer_flag = 1;
					}
				}
				else
				{
					if(Slope(&Foot_Calandria_Walk_Buffer_ramp) == 1.0f)
					{
						ResetSlope(&Foot_Calandria_Walk_Buffer_ramp);
						buffer_flag = 0;
						CSYS_Foot[0].Foot_State = Foot_State_Up;
						CSYS_Foot[3].Foot_State = Foot_State_Up;
						CSYS_Foot[1].Foot_State = Foot_State_Down;
						CSYS_Foot[2].Foot_State = Foot_State_Down;
					}
				}
			}
			else if(CSYS_Foot[0].Foot_State == Foot_State_Up 	 && CSYS_Foot[3].Foot_State == Foot_State_Up &&
							CSYS_Foot[1].Foot_State == Foot_State_Down && CSYS_Foot[2].Foot_State == Foot_State_Down)
			{
				if(!buffer_flag)
				{
//					if(!Start_Walk_flag)	Start_Walk_flag = 1;
					Foot_New_Walk(s,h,&CSYS_Foot[0],&Foot_Calandria_Span_Oval_ramp[0]);
//					Foot_Front_Up_Oval_RTO_Ramp(oval_x,-len,oval_x,oval_y,&CSYS_Foot[0],&Foot_Calandria_Span_Oval_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_New_Walk(s,h,&CSYS_Foot[3],&Foot_Calandria_Span_Oval_ramp[3]);
//					Foot_Front_Up_Oval_RTO_Ramp(oval_x,-len,oval_x,oval_y,&CSYS_Foot[3],&Foot_Calandria_Span_Oval_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
//					Foot_Point_RTO_Ramp(line_1,-len,&CSYS_Foot[1],&Foot_Calandria_Span_Line_ramp[1]);
////					Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[1],&Foot_Calandria_Span_Line_ramp[1]);
//					Foot_Angle_Control(&CSYS_Foot[1]);
//					Foot_Point_RTO_Ramp(line_1,-len,&CSYS_Foot[2],&Foot_Calandria_Span_Line_ramp[2]);
////					Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[2],&Foot_Calandria_Span_Line_ramp[2]);
//					Foot_Angle_Control(&CSYS_Foot[2]);
					if(Slope(&Foot_Calandria_Span_Oval_ramp[0]) == 1.0f && Slope(&Foot_Calandria_Span_Oval_ramp[3]) == 1.0f)
					{
						Foot_New_Walk(s,h,&CSYS_Foot[0],&Foot_Calandria_Span_Oval_ramp[0]);
//						Foot_Front_Up_Oval_RTO_Ramp(oval_x,-len,oval_x,oval_y,&CSYS_Foot[0],&Foot_Calandria_Span_Oval_ramp[0]);
						Foot_Angle_Control(&CSYS_Foot[0]);
						Foot_New_Walk(s,h,&CSYS_Foot[3],&Foot_Calandria_Span_Oval_ramp[3]);
//						Foot_Front_Up_Oval_RTO_Ramp(oval_x,-len,oval_x,oval_y,&CSYS_Foot[3],&Foot_Calandria_Span_Oval_ramp[3]);
						Foot_Angle_Control(&CSYS_Foot[3]);
//						Foot_Point_RTO_Ramp(line_1,-len,&CSYS_Foot[1],&Foot_Calandria_Span_Line_ramp[1]);
////						Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[1],&Foot_Calandria_Span_Line_ramp[1]);
//						Foot_Angle_Control(&CSYS_Foot[1]);
//						Foot_Point_RTO_Ramp(line_1,-len,&CSYS_Foot[2],&Foot_Calandria_Span_Line_ramp[2]);
////						Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[2],&Foot_Calandria_Span_Line_ramp[2]);
//						Foot_Angle_Control(&CSYS_Foot[2]);
						ResetSlope(&Foot_Calandria_Span_Oval_ramp[0]);
						ResetSlope(&Foot_Calandria_Span_Oval_ramp[3]);
//						ResetSlope(&Foot_Calandria_Span_Line_ramp[1]);
//						ResetSlope(&Foot_Calandria_Span_Line_ramp[2]);
						buffer_flag = 1;
					}
				}
				else
				{
					if(Slope(&Foot_Calandria_Walk_Buffer_ramp) == 1.0f)
					{
						ResetFlag(&buffer_flag);
						ResetSlope(&Foot_Calandria_Walk_Buffer_ramp);
						for(i=0;i<4;i++)
							CSYS_Foot[i].Foot_State = Foot_State_Down;
					}
				}
			}
			else if(CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[1].Foot_State == Foot_State_Down &&
							CSYS_Foot[2].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
			{
				if(!buffer_flag)
				{
					for(i=0;i<4;i++)
					{
						Foot_Point_RTO_Ramp(line_1,-len,&CSYS_Foot[i],&Foot_Calandria_Walk_Line_ramp[i]);
						Foot_Angle_Control(&CSYS_Foot[i]);
					}
					if(Slope(&Foot_Calandria_Walk_Line_ramp[0]) == 1.0f && Slope(&Foot_Calandria_Walk_Line_ramp[1]) == 1.0f &&
						 Slope(&Foot_Calandria_Walk_Line_ramp[2]) == 1.0f && Slope(&Foot_Calandria_Walk_Line_ramp[3]) == 1.0f)
					{
						for(i=0;i<4;i++)
						{
							Foot_Point_RTO_Ramp(line_1,-len,&CSYS_Foot[i],&Foot_Calandria_Walk_Line_ramp[i]);
							Foot_Angle_Control(&CSYS_Foot[i]);
							ResetSlope(&Foot_Calandria_Walk_Line_ramp[i]);
						}
						buffer_flag = 1;
					}
				}
				else
				{
					if(!stop_flag)
					{
						if(Slope(&Foot_Calandria_Walk_Buffer_ramp) == 1.0f)
						{
							ResetSlope(&Foot_Calandria_Walk_Buffer_ramp);  
							if(++t == 10)
							{
								stop_flag = 1;
								t = 0;
							}
							else
							{
								buffer_flag = 0;
								CSYS_Foot[0].Foot_State = Foot_State_Down;
								CSYS_Foot[3].Foot_State = Foot_State_Down;
								CSYS_Foot[1].Foot_State = Foot_State_Up;
								CSYS_Foot[2].Foot_State = Foot_State_Up;
							}
						}
					}
					else
					{
						Foot_New_Walk(-s,h,&CSYS_Foot[0],&Foot_Calandria_Span_Oval_ramp[0]);
//						Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Calandria_Span_Oval_ramp[0]);
						Foot_Angle_Control(&CSYS_Foot[0]);
						Foot_New_Walk(-s,h,&CSYS_Foot[3],&Foot_Calandria_Span_Oval_ramp[3]);
//						Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Calandria_Span_Oval_ramp[3]);
						Foot_Angle_Control(&CSYS_Foot[3]);
						if(Slope(&Foot_Calandria_Span_Oval_ramp[0]) == 1.0f && Slope(&Foot_Calandria_Span_Oval_ramp[3]) == 1.0f)
						{
							Foot_New_Walk(-s,h,&CSYS_Foot[0],&Foot_Calandria_Span_Oval_ramp[0]);
//							Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Calandria_Span_Oval_ramp[0]);
							Foot_Angle_Control(&CSYS_Foot[0]);
							Foot_New_Walk(-s,h,&CSYS_Foot[3],&Foot_Calandria_Span_Oval_ramp[3]);
//							Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Calandria_Span_Oval_ramp[3]);
							Foot_Angle_Control(&CSYS_Foot[3]);
							ResetSlope(&Foot_Calandria_Span_Oval_ramp[0]);
							ResetSlope(&Foot_Calandria_Span_Oval_ramp[3]);
							Part_State = 0;
							stop_flag = 0;
							buffer_flag = 0;
							Start_Walk_flag = 0;
							for(i=0;i<4;i++)
								CSYS_Foot[i].Foot_State = Foot_State_None;
						}
					}
				}
			}
		}
	}
	else
	{
		for(i=0;i<4;i++)
		{
			ResetSlope(&Foot_Calandria_Span_Line_ramp[i]);
			ResetSlope(&Foot_Calandria_Span_Oval_ramp[i]);
			ResetSlope(&Foot_Calandria_Walk_Line_ramp[i]);
			ResetSlope(&Foot_Calandria_Walk_Oval_ramp[i]);
			ResetSlope(&Foot_Lengthen_ramp[i]);
		}
		ResetSlope(&Foot_Calandria_Lengthen_Buffer_ramp);
		ResetSlope(&Foot_Calandria_Walk_Buffer_ramp);
		ResetFlag(&Part_State);
		ResetFlag(&Start_Walk_flag);
		ResetFlag(&buffer_flag);
		ResetFlag(&t);
		ResetFlag(&stop_flag);
		Remote_State_Last_Current_Update(Remote_Foot_None);
	}
//	#endif
}

/**
	*@brief 阶梯
	*/
void Foot_Step()
{
	static uint8_t Part_State;				/*	Value				0						1				2	
																				State	  走至阶梯前		上坡		 下坡 
																		*/
	static float oval_x = 25.0f;
	static float oval_y = 20.0f;
	static float line = -50.f;
	static uint8_t Start_Walk_flag;
	if(Part_State == 0)
	{
		static uint8_t t;
		static uint8_t buffer_flag;
		if(CSYS_Foot[1].Foot_State == Foot_State_Up 	&& CSYS_Foot[2].Foot_State == Foot_State_Up &&
			 CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
		{
			if(!buffer_flag)
			{
				if(Start_Walk_flag)
				{
					Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[0],&Foot_Step_Walk_Line_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[3],&Foot_Step_Walk_Line_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
				}
				Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[1],&Foot_Step_Walk_Oval_ramp[1]);
				Foot_Angle_Control(&CSYS_Foot[1]);
				Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[2],&Foot_Step_Walk_Oval_ramp[2]);
				Foot_Angle_Control(&CSYS_Foot[2]);
				if(Slope(&Foot_Step_Walk_Oval_ramp[1]) == 1.0f && Slope(&Foot_Step_Walk_Oval_ramp[2]) == 1.0f)
				{
					if(Start_Walk_flag)
					{
						Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[0],&Foot_Step_Walk_Line_ramp[0]);
						Foot_Angle_Control(&CSYS_Foot[0]);
						Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[3],&Foot_Step_Walk_Line_ramp[3]);
						Foot_Angle_Control(&CSYS_Foot[3]);
						ResetSlope(&Foot_Step_Walk_Line_ramp[0]);
						ResetSlope(&Foot_Step_Walk_Line_ramp[3]);
					}
					Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[1],&Foot_Step_Walk_Oval_ramp[1]);
					Foot_Angle_Control(&CSYS_Foot[1]);
					Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[2],&Foot_Step_Walk_Oval_ramp[2]);
					Foot_Angle_Control(&CSYS_Foot[2]);
					ResetSlope(&Foot_Step_Walk_Oval_ramp[1]);
					ResetSlope(&Foot_Step_Walk_Oval_ramp[2]);
					buffer_flag = 1;
				}
			}
			else
			{
				if(Slope(&Foot_Step_Walk_buffer_ramp) == 1.0f)
				{
					buffer_flag = 0;
					ResetSlope(&Foot_Step_Walk_buffer_ramp);
					CSYS_Foot[0].Foot_State = Foot_State_Up;
					CSYS_Foot[3].Foot_State = Foot_State_Up;
					CSYS_Foot[1].Foot_State = Foot_State_Down;
					CSYS_Foot[2].Foot_State = Foot_State_Down;
					
				}
			}
		}
		else if(CSYS_Foot[0].Foot_State == Foot_State_Up 	 && CSYS_Foot[3].Foot_State == Foot_State_Up &&
						CSYS_Foot[1].Foot_State == Foot_State_Down && CSYS_Foot[2].Foot_State == Foot_State_Down)
		{
			if(!buffer_flag)
			{
				if(!Start_Walk_flag) Start_Walk_flag = 1;
				Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Step_Walk_Oval_ramp[0]);
				Foot_Angle_Control(&CSYS_Foot[0]);
				Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Step_Walk_Oval_ramp[3]);
				Foot_Angle_Control(&CSYS_Foot[3]);
				Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[1],&Foot_Step_Walk_Line_ramp[1]);
				Foot_Angle_Control(&CSYS_Foot[1]);
				Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[2],&Foot_Step_Walk_Line_ramp[2]);
				Foot_Angle_Control(&CSYS_Foot[2]);
				if(Slope(&Foot_Step_Walk_Oval_ramp[0]) == 1.0f && Slope(&Foot_Step_Walk_Oval_ramp[3]) == 1.0f &&
					 Slope(&Foot_Step_Walk_Line_ramp[1]) == 1.0f && Slope(&Foot_Step_Walk_Line_ramp[2]) == 1.0f)
				{
					Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Step_Walk_Oval_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Step_Walk_Oval_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
					Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[1],&Foot_Step_Walk_Line_ramp[1]);
					Foot_Angle_Control(&CSYS_Foot[1]);
					Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[2],&Foot_Step_Walk_Line_ramp[2]);
					Foot_Angle_Control(&CSYS_Foot[2]);
					ResetSlope(&Foot_Step_Walk_Oval_ramp[0]);
					ResetSlope(&Foot_Step_Walk_Oval_ramp[3]);
					ResetSlope(&Foot_Step_Walk_Line_ramp[1]);
					ResetSlope(&Foot_Step_Walk_Line_ramp[2]);
					buffer_flag = 1;
				}
			}
			else
			{
				static uint8_t stop_flag;
				if(!stop_flag)							
				{
					if(Slope(&Foot_Step_Walk_buffer_ramp) == 1.0f)
					{
						buffer_flag = 0;
						ResetSlope(&Foot_Step_Walk_buffer_ramp);
						if(++t == 5)
							stop_flag = 1;
						else
						{
							CSYS_Foot[0].Foot_State = Foot_State_Down;
							CSYS_Foot[3].Foot_State = Foot_State_Down;
							CSYS_Foot[1].Foot_State = Foot_State_Up;
							CSYS_Foot[2].Foot_State = Foot_State_Up;
						}
					}
				}
				else			//走完循环
				{
					Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Step_Walk_Oval_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Step_Walk_Oval_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
					if(Slope(&Foot_Step_Walk_Oval_ramp[0]) == 1.0f && Slope(&Foot_Step_Walk_Oval_ramp[3]) == 1.0f)
					{
						Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Step_Walk_Oval_ramp[0]);
						Foot_Angle_Control(&CSYS_Foot[0]);
						Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Step_Walk_Oval_ramp[3]);
						Foot_Angle_Control(&CSYS_Foot[3]);
						ResetSlope(&Foot_Step_Walk_Oval_ramp[0]);
						ResetSlope(&Foot_Step_Walk_Oval_ramp[3]);
						Part_State = 1;
						stop_flag = 0;
					}
				}
			}
		}
	}
	else if(Part_State == 1)
	{
		 
	}
	else if(Part_State == 2)
	{
		
	}
}

/**
	*@brief 双木桥
	*/
void Foot_Wooden_Bridge()
{
	static uint8_t Part_State;				/*	
																				Value				0						1				2				3	
																				State	  走至双木桥前	上坡		往前走  下坡
																		*/
	static float oval_x = 25.0f;
	static float oval_y = 20.0f;
	static float line = -50.f;
	static uint8_t Start_Walk_flag;
	if(Part_State == 0)
	{
		static uint8_t t;
		static float buffer_flag;
		if(CSYS_Foot[1].Foot_State == Foot_State_Up 	&& CSYS_Foot[2].Foot_State == Foot_State_Up &&
			 CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
		{
			if(!buffer_flag)
			{
				if(Start_Walk_flag)
				{
					Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[0],&Foot_Step_Walk_Line_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[3],&Foot_Step_Walk_Line_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
//					Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[0],&Foot_Step_Walk_Line_ramp[0]);
//					Foot_Angle_Control(&CSYS_Foot[0]);
//					Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[3],&Foot_Step_Walk_Line_ramp[3]);
//					Foot_Angle_Control(&CSYS_Foot[3]);
				}
				Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[1],&Foot_Step_Walk_Oval_ramp[1]);
				Foot_Angle_Control(&CSYS_Foot[1]);
				Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[2],&Foot_Step_Walk_Oval_ramp[2]);
				Foot_Angle_Control(&CSYS_Foot[2]);
				if(Slope(&Foot_Step_Walk_Oval_ramp[1]) == 1.0f && Slope(&Foot_Step_Walk_Oval_ramp[2]) == 1.0f)
				{
					if(Start_Walk_flag)
					{
						Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[0],&Foot_Step_Walk_Line_ramp[0]);
						Foot_Angle_Control(&CSYS_Foot[0]);
						Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[3],&Foot_Step_Walk_Line_ramp[3]);
						Foot_Angle_Control(&CSYS_Foot[3]);
//						Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[0],&Foot_Step_Walk_Line_ramp[0]);
//						Foot_Angle_Control(&CSYS_Foot[0]);
//						Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[3],&Foot_Step_Walk_Line_ramp[3]);
//						Foot_Angle_Control(&CSYS_Foot[3]);
						ResetSlope(&Foot_Step_Walk_Line_ramp[0]);
						ResetSlope(&Foot_Step_Walk_Line_ramp[3]);
					}
					Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[1],&Foot_Step_Walk_Oval_ramp[1]);
					Foot_Angle_Control(&CSYS_Foot[1]);
					Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[2],&Foot_Step_Walk_Oval_ramp[2]);
					Foot_Angle_Control(&CSYS_Foot[2]);
					ResetSlope(&Foot_Step_Walk_Oval_ramp[1]);
					ResetSlope(&Foot_Step_Walk_Oval_ramp[2]);
					buffer_flag = 1;
				}
			}
			else
			{
				if(Slope(&Foot_Step_Walk_buffer_ramp) == 1.0f)
				{
					buffer_flag = 0;
					ResetSlope(&Foot_Step_Walk_buffer_ramp);
					CSYS_Foot[0].Foot_State = Foot_State_Up;
					CSYS_Foot[3].Foot_State = Foot_State_Up;
					CSYS_Foot[1].Foot_State = Foot_State_Down;
					CSYS_Foot[2].Foot_State = Foot_State_Down;
					
				}
			}
		}
		else if(CSYS_Foot[0].Foot_State == Foot_State_Up 	 && CSYS_Foot[3].Foot_State == Foot_State_Up &&
						CSYS_Foot[1].Foot_State == Foot_State_Down && CSYS_Foot[2].Foot_State == Foot_State_Down)
		{
			if(!buffer_flag)
			{
				if(!Start_Walk_flag) Start_Walk_flag = 1;
				Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Step_Walk_Oval_ramp[0]);
				Foot_Angle_Control(&CSYS_Foot[0]);
				Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Step_Walk_Oval_ramp[3]);
				Foot_Angle_Control(&CSYS_Foot[3]);
				Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[0],&Foot_Step_Walk_Line_ramp[0]);
				Foot_Angle_Control(&CSYS_Foot[0]);
				Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[3],&Foot_Step_Walk_Line_ramp[3]);
				Foot_Angle_Control(&CSYS_Foot[3]);
//				Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[1],&Foot_Step_Walk_Line_ramp[1]);
//				Foot_Angle_Control(&CSYS_Foot[1]);
//				Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[2],&Foot_Step_Walk_Line_ramp[2]);
//				Foot_Angle_Control(&CSYS_Foot[2]);
				if(Slope(&Foot_Step_Walk_Oval_ramp[0]) == 1.0f && Slope(&Foot_Step_Walk_Oval_ramp[3]) == 1.0f &&
					 Slope(&Foot_Step_Walk_Line_ramp[1]) == 1.0f && Slope(&Foot_Step_Walk_Line_ramp[2]) == 1.0f)
				{
					Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Step_Walk_Oval_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Step_Walk_Oval_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
					Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[0],&Foot_Step_Walk_Line_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[3],&Foot_Step_Walk_Line_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
//					Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[1],&Foot_Step_Walk_Line_ramp[1]);
//					Foot_Angle_Control(&CSYS_Foot[1]);
//					Foot_Line_NRTO_Ramp(line,0,&CSYS_Foot[2],&Foot_Step_Walk_Line_ramp[2]);
//					Foot_Angle_Control(&CSYS_Foot[2]);
					ResetSlope(&Foot_Step_Walk_Oval_ramp[0]);
					ResetSlope(&Foot_Step_Walk_Oval_ramp[3]);
					ResetSlope(&Foot_Step_Walk_Line_ramp[1]);
					ResetSlope(&Foot_Step_Walk_Line_ramp[2]);
					buffer_flag = 1;
				}
			}
			else
			{
				static uint8_t stop_flag;
				if(!stop_flag)							
				{
					if(Slope(&Foot_Step_Walk_buffer_ramp) == 1.0f)
					{
						buffer_flag = 0;
						ResetSlope(&Foot_Step_Walk_buffer_ramp);
						if(++t == 5)
							stop_flag = 1;
						else
						{
							CSYS_Foot[0].Foot_State = Foot_State_Down;
							CSYS_Foot[3].Foot_State = Foot_State_Down;
							CSYS_Foot[1].Foot_State = Foot_State_Up;
							CSYS_Foot[2].Foot_State = Foot_State_Up;
						}
					}
				}
				else			//走完循环
				{
					Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Step_Walk_Oval_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Step_Walk_Oval_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
					if(Slope(&Foot_Step_Walk_Oval_ramp[0]) == 1.0f && Slope(&Foot_Step_Walk_Oval_ramp[3]) == 1.0f)
					{
						Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Step_Walk_Oval_ramp[0]);
						Foot_Angle_Control(&CSYS_Foot[0]);
						Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Step_Walk_Oval_ramp[3]);
						Foot_Angle_Control(&CSYS_Foot[3]);
						ResetSlope(&Foot_Step_Walk_Oval_ramp[0]);
						ResetSlope(&Foot_Step_Walk_Oval_ramp[3]);
						Part_State = 1;
						stop_flag = 0;
						t = 0;
						for(i=0;i<4;i++)
							CSYS_Foot[i].Foot_State = Foot_State_Lengthen;
					}
				} 
			}
		}
	}
	else if(Part_State == 1)
	{
		static float Lengthen = 100.0f;
		if(CSYS_Foot[0].Foot_State == Foot_State_Lengthen && CSYS_Foot[1].Foot_State == Foot_State_Lengthen &&
			 CSYS_Foot[2].Foot_State == Foot_State_Lengthen && CSYS_Foot[3].Foot_State == Foot_State_Lengthen)
		{
			for(i=0;i<4;i++)
			{
				Foot_Point_RTO_Ramp(0,-Lengthen,&CSYS_Foot[i],&Foot_Lengthen_ramp[i]);
				Foot_Angle_Control(&CSYS_Foot[i]);
			}
			if(Slope(&Foot_Lengthen_ramp[0]) == 1.0f && Slope(&Foot_Lengthen_ramp[1]) == 1.0f &&
				 Slope(&Foot_Lengthen_ramp[2]) == 1.0f && Slope(&Foot_Lengthen_ramp[3]) == 1.0f)
			{
				for(i=0;i<4;i++)
				{
					Foot_Point_RTO_Ramp(0,-Lengthen,&CSYS_Foot[i],&Foot_Lengthen_ramp[i]);
					Foot_Angle_Control(&CSYS_Foot[i]);
					ResetSlope(&Foot_Lengthen_ramp[i]);
					CSYS_Foot[i].Foot_State = Foot_State_Lift;
				}
				
			}
		}
		else if(CSYS_Foot[0].Foot_State == Foot_State_Lift && CSYS_Foot[1].Foot_State == Foot_State_Lift &&
						CSYS_Foot[2].Foot_State == Foot_State_Lift && CSYS_Foot[3].Foot_State == Foot_State_Lift)
		{
			static uint8_t climb_state;				/*
																						Value			0								1			 			2								3								4						5
																						State	前左腿抬起		左前腿迈上木板	右前腿抬起	右前腿向迈上木板		腿摆正				
																				*/
			static float oval_x = 30.0f;
			static float oval_y = 20.0f;
			static float back	=	-60.0f;
			if(climb_state == 0)				//
			{
				Foot_Line_NRTO_Ramp(0,Lengthen,&CSYS_Foot[0],&Foot_Wooden_Bridge_Up_ramp[0]);
				Foot_Angle_Control(&CSYS_Foot[0]);
				if(Slope(&Foot_Wooden_Bridge_Up_ramp[0]) == 1.0f)
				{
					Foot_Line_NRTO_Ramp(0,Lengthen,&CSYS_Foot[0],&Foot_Wooden_Bridge_Up_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					ResetSlope(&Foot_Wooden_Bridge_Up_ramp[0]);
					climb_state = 1;
				}
			}
			else if(climb_state == 1)
			{
				Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Wooden_Bridge_Oval_ramp[0]);
				Foot_Angle_Control(&CSYS_Foot[0]);
				if(Slope(&Foot_Wooden_Bridge_Oval_ramp[0]) == 1.0f)
				{
					Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Wooden_Bridge_Oval_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					ResetSlope(&Foot_Wooden_Bridge_Oval_ramp[0]);
					climb_state = 2;
				}
			}
			else if(climb_state == 2)
			{
				Foot_Line_NRTO_Ramp(0,Lengthen,&CSYS_Foot[1],&Foot_Wooden_Bridge_Up_ramp[1]);
				Foot_Angle_Control(&CSYS_Foot[1]);
				if(Slope(&Foot_Wooden_Bridge_Up_ramp[1]) == 1.0f)
				{
					Foot_Line_NRTO_Ramp(0,Lengthen,&CSYS_Foot[1],&Foot_Wooden_Bridge_Up_ramp[1]);
					Foot_Angle_Control(&CSYS_Foot[1]);
					ResetSlope(&Foot_Wooden_Bridge_Up_ramp[1]);
					climb_state = 3;
				}
			}
			else if(climb_state == 3)
			{
				Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[1],&Foot_Wooden_Bridge_Oval_ramp[1]);
				Foot_Angle_Control(&CSYS_Foot[1]);
				if(Slope(&Foot_Wooden_Bridge_Oval_ramp[1]) == 1.0f)
				{
					Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[1],&Foot_Wooden_Bridge_Oval_ramp[1]);
					Foot_Angle_Control(&CSYS_Foot[1]);
					ResetSlope(&Foot_Wooden_Bridge_Oval_ramp[1]);
					climb_state = 4;
				}
			}
			else if(climb_state == 4)
			{
				Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[2],&Foot_Wooden_Bridge_Oval_ramp[2]);
				Foot_Angle_Control(&CSYS_Foot[2]);
				if(Slope(&Foot_Wooden_Bridge_Oval_ramp[2]) == 1.0f)
				{
					Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[2],&Foot_Wooden_Bridge_Oval_ramp[2]);
					Foot_Angle_Control(&CSYS_Foot[2]);
					ResetSlope(&Foot_Wooden_Bridge_Oval_ramp[2]);
					climb_state = 5;
				}
			}
			else if(climb_state == 5)			
			{
				Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Wooden_Bridge_Oval_ramp[3]);
				Foot_Angle_Control(&CSYS_Foot[3]);
				if(Slope(&Foot_Wooden_Bridge_Oval_ramp[3]) == 1.0f)
				{
					Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Wooden_Bridge_Oval_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
					ResetSlope(&Foot_Wooden_Bridge_Oval_ramp[3]);
					climb_state = 6;
				}
			}
			else if(climb_state == 6)
			{
				for(i=0;i<4;i++)
				{
					Foot_Line_NRTO_Ramp(back,0,&CSYS_Foot[i],&Foot_Wooden_Bridge_Line_ramp[i]);
					Foot_Angle_Control(&CSYS_Foot[i]);
				}
				if(Slope(&Foot_Wooden_Bridge_Line_ramp[0]) == 1.0f && Slope(&Foot_Wooden_Bridge_Line_ramp[1]) == 1.0f &&
					 Slope(&Foot_Wooden_Bridge_Line_ramp[2]) == 1.0f && Slope(&Foot_Wooden_Bridge_Line_ramp[3]) == 1.0f)
				{
					for(i=0;i<4;i++)
					{
						Foot_Line_NRTO_Ramp(back,0,&CSYS_Foot[i],&Foot_Wooden_Bridge_Line_ramp[i]);
						Foot_Angle_Control(&CSYS_Foot[i]);
						ResetSlope(&Foot_Wooden_Bridge_Line_ramp[i]);
					}
					climb_state = 7;
				}
			}
			else if(climb_state == 7)				//向前走
			{
				static uint8_t repeat;
				if(repeat == 5)
				{
					repeat = 0;
					climb_state = 8;
				}
				else
				{
					static uint8_t state;
					static float part_oval_x;
					static float part_oval_y;
					switch(state)
					{
						case 0:
						{
							Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Wooden_Bridge_Oval_ramp[0]);
							Foot_Angle_Control(&CSYS_Foot[0]);
							if(Slope(&Foot_Wooden_Bridge_Oval_ramp[0]) == 1.0f)
							{
								Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Wooden_Bridge_Oval_ramp[0]);
								Foot_Angle_Control(&CSYS_Foot[0]);
								ResetSlope(&Foot_Wooden_Bridge_Oval_ramp[0]);
								state = 1;
							}
							break;
						}
						case 1:
						{
							Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[1],&Foot_Wooden_Bridge_Oval_ramp[1]);
							Foot_Angle_Control(&CSYS_Foot[1]);
							if(Slope(&Foot_Wooden_Bridge_Oval_ramp[1]) == 1.0f)
							{
								Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[1],&Foot_Wooden_Bridge_Oval_ramp[1]);
								Foot_Angle_Control(&CSYS_Foot[1]);
								ResetSlope(&Foot_Wooden_Bridge_Oval_ramp[1]);
								state = 2;
							}
							break;
						}
						case 2:
						{
							Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[2],&Foot_Wooden_Bridge_Oval_ramp[2]);
							Foot_Angle_Control(&CSYS_Foot[2]);
							if(Slope(&Foot_Wooden_Bridge_Oval_ramp[2]) == 1.0f)
							{
								Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[2],&Foot_Wooden_Bridge_Oval_ramp[2]);
								Foot_Angle_Control(&CSYS_Foot[2]);
								ResetSlope(&Foot_Wooden_Bridge_Oval_ramp[2]);
								state = 3;
							}
							break;
						}
						case 3:
						{
							Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Wooden_Bridge_Oval_ramp[3]);
							Foot_Angle_Control(&CSYS_Foot[3]);
							if(Slope(&Foot_Wooden_Bridge_Oval_ramp[3]) == 1.0f)
							{
								Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Wooden_Bridge_Oval_ramp[3]);
								Foot_Angle_Control(&CSYS_Foot[3]);
								ResetSlope(&Foot_Wooden_Bridge_Oval_ramp[3]);
								state = 4;
							}
							break;
						}
						case 4:
						{
							for(i=0;i<4;i++)
							{
								Foot_Round_X_NRTO_ramp(back,&CSYS_Foot[i],&Foot_Wooden_Bridge_Line_ramp[i]);
								Foot_Angle_Control(&CSYS_Foot[i]);
//								Foot_Line_NRTO_Ramp(back,0,&CSYS_Foot[i],&Foot_Wooden_Bridge_Line_ramp[i]);
//								Foot_Angle_Control(&CSYS_Foot[i]);
							}
							if(Slope(&Foot_Wooden_Bridge_Line_ramp[0]) == 1.0f && Slope(&Foot_Wooden_Bridge_Line_ramp[1]) == 1.0f &&
								 Slope(&Foot_Wooden_Bridge_Line_ramp[2]) == 1.0f && Slope(&Foot_Wooden_Bridge_Line_ramp[3]) == 1.0f)
							{
								for(i=0;i<4;i++)
								{
									Foot_Round_X_NRTO_ramp(back,&CSYS_Foot[i],&Foot_Wooden_Bridge_Line_ramp[i]);
									Foot_Angle_Control(&CSYS_Foot[i]);
//									Foot_Line_NRTO_Ramp(back,0,&CSYS_Foot[i],&Foot_Wooden_Bridge_Line_ramp[i]);
//									Foot_Angle_Control(&CSYS_Foot[i]);
									ResetSlope(&Foot_Wooden_Bridge_Line_ramp[i]);
								}
								state = 0;
								repeat++;
							}
							break;
						}
					}
				}
			}
			else if(climb_state == 8)
			{
				static uint8_t state;
				switch(state)
				{
					case 0:
					{
						Foot_Point_RTO_Ramp(0,Lengthen,&CSYS_Foot[2],&Foot_Lengthen_ramp[2]);
						Foot_Angle_Control(&CSYS_Foot[2]);
						if(Slope(&Foot_Lengthen_ramp[2]) == 1.0f)
						{
							ResetSlope(&Foot_Lengthen_ramp[2]);
							state = 1;
						}
						break;
					}
					case 1:
					{
						Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[2],&Foot_Wooden_Bridge_Oval_ramp[2]);
						Foot_Angle_Control(&CSYS_Foot[2]);
						if(Slope(&Foot_Wooden_Bridge_Oval_ramp[2]) == 1.0f)
						{
							ResetSlope(&Foot_Wooden_Bridge_Oval_ramp[2]);
							state = 2;
						}
						break;
					}
					case 2:
					{
						Foot_Point_RTO_Ramp(0,Lengthen,&CSYS_Foot[3],&Foot_Lengthen_ramp[3]);
						Foot_Angle_Control(&CSYS_Foot[3]);
						if(Slope(&Foot_Lengthen_ramp[3]) == 1.0f)
						{
							ResetSlope(&Foot_Lengthen_ramp[3]);
							state = 3;
						}
						break;
					}
					case 3:
					{
						Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Wooden_Bridge_Oval_ramp[3]);
						Foot_Angle_Control(&CSYS_Foot[3]);
						if(Slope(&Foot_Wooden_Bridge_Oval_ramp[3]) == 1.0f)
						{
							ResetSlope(&Foot_Wooden_Bridge_Oval_ramp[3]);
							climb_state = 0;
							state = 0;
							Part_State = 2;
						}
						break;
					}
				}
			}
		}
	}
	else if(Part_State == 2)
	{
		static float oval_x = 25.0f;
		static float oval_y = 40.0f;
		static float line		= -50.0f;
		static float buffer_flag;
		static uint8_t t;
		if(CSYS_Foot[1].Foot_State == Foot_State_Up && CSYS_Foot[2].Foot_State == Foot_State_Up &&
			 CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
		{
			if(!buffer_flag)
			{
				if(Start_Walk_flag)			//开始行走标志位
				{
					Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[0],&Foot_Calandria_Walk_Line_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[3],&Foot_Calandria_Walk_Line_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
				}
				Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[1],&Foot_Calandria_Walk_Oval_ramp[1]);
				Foot_Angle_Control(&CSYS_Foot[1]);
				Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[2],&Foot_Calandria_Walk_Oval_ramp[2]);
				Foot_Angle_Control(&CSYS_Foot[2]);
				if(Slope(&Foot_Calandria_Walk_Oval_ramp[1]) == 1.0f && Slope(&Foot_Calandria_Walk_Oval_ramp[2]) == 1.0f)
				{
					if(Start_Walk_flag)			//开始行走标志位
					{
						Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[0],&Foot_Calandria_Walk_Line_ramp[0]);
						Foot_Angle_Control(&CSYS_Foot[0]);
						Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[3],&Foot_Calandria_Walk_Line_ramp[3]);
						Foot_Angle_Control(&CSYS_Foot[3]);
						ResetSlope(&Foot_Calandria_Walk_Line_ramp[0]);
						ResetSlope(&Foot_Calandria_Walk_Line_ramp[3]);
					}
					Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[1],&Foot_Calandria_Walk_Oval_ramp[1]);
					Foot_Angle_Control(&CSYS_Foot[1]);
					Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[2],&Foot_Calandria_Walk_Oval_ramp[2]);
					Foot_Angle_Control(&CSYS_Foot[2]);
					ResetSlope(&Foot_Calandria_Walk_Oval_ramp[1]);
					ResetSlope(&Foot_Calandria_Walk_Oval_ramp[2]);
					buffer_flag = 1;
				}
			}
			else
			{
				if(Slope(&Foot_Calandria_Walk_Buffer_ramp) == 1.0f)
				{
					ResetSlope(&Foot_Calandria_Walk_Buffer_ramp);
					buffer_flag = 0;
					CSYS_Foot[0].Foot_State = Foot_State_Up;
					CSYS_Foot[3].Foot_State = Foot_State_Up;
					CSYS_Foot[1].Foot_State = Foot_State_Down;
					CSYS_Foot[2].Foot_State = Foot_State_Down;
				}
			}
		}
		else if(CSYS_Foot[0].Foot_State == Foot_State_Up 	 && CSYS_Foot[3].Foot_State == Foot_State_Up &&
						CSYS_Foot[1].Foot_State == Foot_State_Down && CSYS_Foot[2].Foot_State == Foot_State_Down)
		{
			if(!buffer_flag)
			{
				if(!Start_Walk_flag)	Start_Walk_flag = 1;
				Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[0],&Foot_Calandria_Walk_Oval_ramp[0]);
				Foot_Angle_Control(&CSYS_Foot[0]);
				Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[3],&Foot_Calandria_Walk_Oval_ramp[3]);
				Foot_Angle_Control(&CSYS_Foot[3]);
				Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[1],&Foot_Calandria_Walk_Line_ramp[1]);
				Foot_Angle_Control(&CSYS_Foot[1]);
				Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[2],&Foot_Calandria_Walk_Line_ramp[2]);
				Foot_Angle_Control(&CSYS_Foot[2]);
				if(Slope(&Foot_Calandria_Walk_Oval_ramp[0]) == 1.0f && Slope(&Foot_Calandria_Walk_Oval_ramp[3]) == 1.0f &&
					 Slope(&Foot_Calandria_Walk_Line_ramp[1]) == 1.0f && Slope(&Foot_Calandria_Walk_Line_ramp[2]) == 1.0f)
				{
					Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[0],&Foot_Calandria_Walk_Oval_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[3],&Foot_Calandria_Walk_Oval_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
					Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[1],&Foot_Calandria_Walk_Line_ramp[1]);
					Foot_Angle_Control(&CSYS_Foot[1]);
					Foot_Round_X_NRTO_ramp(line,&CSYS_Foot[2],&Foot_Calandria_Walk_Line_ramp[2]);
					Foot_Angle_Control(&CSYS_Foot[2]);
					ResetSlope(&Foot_Calandria_Walk_Oval_ramp[0]);
					ResetSlope(&Foot_Calandria_Walk_Oval_ramp[3]);
					ResetSlope(&Foot_Calandria_Walk_Line_ramp[1]);
					ResetSlope(&Foot_Calandria_Walk_Line_ramp[2]);
					buffer_flag = 1;
				}
			}
			else
			{
				static uint8_t stop_flag;
				if(!stop_flag)
				{
					if(Slope(&Foot_Calandria_Walk_Buffer_ramp) == 1.0f)
					{
						ResetSlope(&Foot_Calandria_Walk_Buffer_ramp);
						if(++t == 5)
						{
							stop_flag = 1;
							t=0;
						}
						else
						{
							buffer_flag = 0;
							CSYS_Foot[0].Foot_State = Foot_State_Down;
							CSYS_Foot[3].Foot_State = Foot_State_Down;
							CSYS_Foot[1].Foot_State = Foot_State_Up;
							CSYS_Foot[2].Foot_State = Foot_State_Up;
						}
					}
				}
				else
				{
					Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Calandria_Walk_Oval_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Calandria_Walk_Oval_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
					if(Slope(&Foot_Calandria_Walk_Oval_ramp[0]) == 1.0f && Slope(&Foot_Calandria_Walk_Oval_ramp[3]) == 1.0f)
					{
						Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Calandria_Walk_Oval_ramp[0]);
						Foot_Angle_Control(&CSYS_Foot[0]);
						Foot_Back_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Calandria_Walk_Oval_ramp[3]);
						Foot_Angle_Control(&CSYS_Foot[3]);
						ResetSlope(&Foot_Calandria_Walk_Oval_ramp[0]);
						ResetSlope(&Foot_Calandria_Walk_Oval_ramp[3]);
						Part_State = 3;
						stop_flag = 0;
						buffer_flag = 0;
						Start_Walk_flag = 0;
						CSYS_Foot[0].Foot_State = Foot_State_Fall;
						CSYS_Foot[1].Foot_State = Foot_State_Fall;
						CSYS_Foot[2].Foot_State = Foot_State_Up;
						CSYS_Foot[3].Foot_State = Foot_State_Up;
					}
				}
			}
		}
	}
	else if(Part_State == 3)
	{
		static uint8_t climb_state;				 /*
																					Value			0								1			 			2								3								4						5
																					State	前左腿抬起		左前腿迈上木板	右前腿抬起	右前腿向迈上木板		腿摆正				
																			*/
		static float oval_x = 50.0f;
		static float oval_y = 50.0f;
		static float back	=	-100.0f;
		static float len = 100.f;
		if(climb_state == 0)				//
		{
			Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Wooden_Bridge_Oval_ramp[0]);
			Foot_Angle_Control(&CSYS_Foot[0]);
			if(Slope(&Foot_Wooden_Bridge_Oval_ramp[0]) == 1.0f)
			{
				Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Wooden_Bridge_Oval_ramp[0]);
				Foot_Angle_Control(&CSYS_Foot[0]);
				ResetSlope(&Foot_Wooden_Bridge_Oval_ramp[0]);
				climb_state = 1;
			}
		}
		else if(climb_state == 1)
		{
			Foot_Line_NRTO_Ramp(0,-len,&CSYS_Foot[0],&Foot_Wooden_Bridge_Line_ramp[0]);
			Foot_Angle_Control(&CSYS_Foot[0]);
			if(Slope(&Foot_Wooden_Bridge_Line_ramp[0]) == 1.0f)
			{
				Foot_Line_NRTO_Ramp(0,-len,&CSYS_Foot[0],&Foot_Wooden_Bridge_Line_ramp[0]);
				Foot_Angle_Control(&CSYS_Foot[0]);
				ResetSlope(&Foot_Wooden_Bridge_Line_ramp[0]);
				climb_state = 2;
			}
		}
		else if(climb_state == 2)
		{
			Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[1],&Foot_Wooden_Bridge_Oval_ramp[1]);
			Foot_Angle_Control(&CSYS_Foot[1]);
			if(Slope(&Foot_Wooden_Bridge_Oval_ramp[1]) == 1.0f)
			{
				Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[1],&Foot_Wooden_Bridge_Oval_ramp[1]);
				Foot_Angle_Control(&CSYS_Foot[1]);
				ResetSlope(&Foot_Wooden_Bridge_Oval_ramp[1]);
				climb_state = 3;
			}
		}
		else if(climb_state == 3)
		{
			Foot_Line_NRTO_Ramp(0,-len,&CSYS_Foot[1],&Foot_Wooden_Bridge_Line_ramp[1]);
			Foot_Angle_Control(&CSYS_Foot[1]);
			if(Slope(&Foot_Wooden_Bridge_Line_ramp[1]) == 1.0f)
			{
				Foot_Line_NRTO_Ramp(0,-len,&CSYS_Foot[1],&Foot_Wooden_Bridge_Line_ramp[1]);
				Foot_Angle_Control(&CSYS_Foot[1]);
				ResetSlope(&Foot_Wooden_Bridge_Line_ramp[1]);
				climb_state = 2;
			}
		}
		else if(climb_state == 4)
		{
			Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[2],&Foot_Wooden_Bridge_Oval_ramp[2]);
			Foot_Angle_Control(&CSYS_Foot[2]);
			if(Slope(&Foot_Wooden_Bridge_Oval_ramp[2]) == 1.0f)
			{
				Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[2],&Foot_Wooden_Bridge_Oval_ramp[2]);
				Foot_Angle_Control(&CSYS_Foot[2]);
				ResetSlope(&Foot_Wooden_Bridge_Oval_ramp[2]);
				climb_state = 5;
			}
		}
		else if(climb_state == 5)			
		{
			Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Wooden_Bridge_Oval_ramp[3]);
			Foot_Angle_Control(&CSYS_Foot[3]);
			if(Slope(&Foot_Wooden_Bridge_Oval_ramp[3]) == 1.0f)
			{
				Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Wooden_Bridge_Oval_ramp[3]);
				Foot_Angle_Control(&CSYS_Foot[3]);
				ResetSlope(&Foot_Wooden_Bridge_Oval_ramp[3]);
				climb_state = 6;
			}
		}
		else if(climb_state == 6)
		{
			
			for(i=0;i<4;i++)
			{
				Foot_Round_X_NRTO_ramp(back,&CSYS_Foot[i],&Foot_Wooden_Bridge_Line_ramp[i]);
				Foot_Angle_Control(&CSYS_Foot[i]);
			}
			if(Slope(&Foot_Wooden_Bridge_Line_ramp[0]) == 1.0f && Slope(&Foot_Wooden_Bridge_Line_ramp[1]) == 1.0f &&
				 Slope(&Foot_Wooden_Bridge_Line_ramp[2]) == 1.0f && Slope(&Foot_Wooden_Bridge_Line_ramp[3]) == 1.0f)
			{
				for(i=0;i<4;i++)
				{
					Foot_Round_X_NRTO_ramp(back,&CSYS_Foot[i],&Foot_Wooden_Bridge_Line_ramp[i]);
					Foot_Angle_Control(&CSYS_Foot[i]);
					ResetSlope(&Foot_Wooden_Bridge_Line_ramp[i]);
				}
				climb_state = 7;
			}
		}
		else if(climb_state == 7)				//向前走
		{
			static uint8_t repeat;
			if(repeat == 5)
			{
				repeat = 0;
				climb_state = 8;
			}
			else
			{
				static uint8_t state;
				static float part_oval_x;
				static float part_oval_y;
				switch(state)
				{
					case 0:
					{
						Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Wooden_Bridge_Oval_ramp[0]);
						Foot_Angle_Control(&CSYS_Foot[0]);
						if(Slope(&Foot_Wooden_Bridge_Oval_ramp[0]) == 1.0f)
						{
							Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Wooden_Bridge_Oval_ramp[0]);
							Foot_Angle_Control(&CSYS_Foot[0]);
							ResetSlope(&Foot_Wooden_Bridge_Oval_ramp[0]);
							state = 1;
						}
						break;
					}
					case 1:
					{
						Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[1],&Foot_Wooden_Bridge_Oval_ramp[1]);
						Foot_Angle_Control(&CSYS_Foot[1]);
						if(Slope(&Foot_Wooden_Bridge_Oval_ramp[1]) == 1.0f)
						{
							Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[1],&Foot_Wooden_Bridge_Oval_ramp[1]);
							Foot_Angle_Control(&CSYS_Foot[1]);
							ResetSlope(&Foot_Wooden_Bridge_Oval_ramp[1]);
							state = 2;
						}
						break;
					}
					case 2:
					{
						Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[2],&Foot_Wooden_Bridge_Oval_ramp[2]);
						Foot_Angle_Control(&CSYS_Foot[2]);
						if(Slope(&Foot_Wooden_Bridge_Oval_ramp[2]) == 1.0f)
						{
							Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[2],&Foot_Wooden_Bridge_Oval_ramp[2]);
							Foot_Angle_Control(&CSYS_Foot[2]);
							ResetSlope(&Foot_Wooden_Bridge_Oval_ramp[2]);
							state = 3;
						}
						break;
					}
					case 3:
					{
						Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Wooden_Bridge_Oval_ramp[3]);
						Foot_Angle_Control(&CSYS_Foot[3]);
						if(Slope(&Foot_Wooden_Bridge_Oval_ramp[3]) == 1.0f)
						{
							Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Wooden_Bridge_Oval_ramp[3]);
							Foot_Angle_Control(&CSYS_Foot[3]);
							ResetSlope(&Foot_Wooden_Bridge_Oval_ramp[3]);
							state = 4;
						}
						break;
					}
					case 4:
					{
						for(i=0;i<4;i++)
						{
							Foot_Round_X_NRTO_ramp(back,&CSYS_Foot[i],&Foot_Wooden_Bridge_Line_ramp[i]);
							Foot_Angle_Control(&CSYS_Foot[i]);
//								Foot_Line_NRTO_Ramp(back,0,&CSYS_Foot[i],&Foot_Wooden_Bridge_Line_ramp[i]);
//								Foot_Angle_Control(&CSYS_Foot[i]);
						}
						if(Slope(&Foot_Wooden_Bridge_Line_ramp[0]) == 1.0f && Slope(&Foot_Wooden_Bridge_Line_ramp[1]) == 1.0f &&
							 Slope(&Foot_Wooden_Bridge_Line_ramp[2]) == 1.0f && Slope(&Foot_Wooden_Bridge_Line_ramp[3]) == 1.0f)
						{
							for(i=0;i<4;i++)
							{
								Foot_Round_X_NRTO_ramp(back,&CSYS_Foot[i],&Foot_Wooden_Bridge_Line_ramp[i]);
								Foot_Angle_Control(&CSYS_Foot[i]);
//									Foot_Line_NRTO_Ramp(back,0,&CSYS_Foot[i],&Foot_Wooden_Bridge_Line_ramp[i]);
//									Foot_Angle_Control(&CSYS_Foot[i]);
								ResetSlope(&Foot_Wooden_Bridge_Line_ramp[i]);
							}
							state = 0;
							repeat++;
						}
						break;
					}
				}
			}
		}
		else if(climb_state == 8)
		{
			static uint8_t state;
			switch(state)
			{
				case 0:
				{
					Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[2],&Foot_Wooden_Bridge_Oval_ramp[2]);
					Foot_Angle_Control(&CSYS_Foot[2]);
					if(Slope(&Foot_Wooden_Bridge_Oval_ramp[2]) == 1.0f)
					{
						Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[2],&Foot_Wooden_Bridge_Oval_ramp[2]);
						Foot_Angle_Control(&CSYS_Foot[2]);
						ResetSlope(&Foot_Wooden_Bridge_Oval_ramp[2]);
						state = 1;
					}
					break;
				}
				case 1:
				{
					Foot_Line_NRTO_Ramp(0,-len,&CSYS_Foot[2],&Foot_Wooden_Bridge_Line_ramp[2]);
					Foot_Angle_Control(&CSYS_Foot[2]);
					if(Slope(&Foot_Wooden_Bridge_Line_ramp[2]) == 1.0f)
					{
						Foot_Line_NRTO_Ramp(0,-len,&CSYS_Foot[2],&Foot_Wooden_Bridge_Line_ramp[2]);
						Foot_Angle_Control(&CSYS_Foot[2]);
						ResetSlope(&Foot_Wooden_Bridge_Line_ramp[2]);
						state = 2;
					}
					break;
				}
				case 2:
				{
					Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Wooden_Bridge_Oval_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
					if(Slope(&Foot_Wooden_Bridge_Oval_ramp[3]) == 1.0f)
					{
						Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Wooden_Bridge_Oval_ramp[3]);
						Foot_Angle_Control(&CSYS_Foot[3]);
						ResetSlope(&Foot_Wooden_Bridge_Oval_ramp[3]);
						state = 3;
					}
					break;
				}
				case 3:
				{
					Foot_Line_NRTO_Ramp(0,-len,&CSYS_Foot[3],&Foot_Wooden_Bridge_Line_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
					if(Slope(&Foot_Wooden_Bridge_Line_ramp[3]) == 1.0f)
					{
						Foot_Line_NRTO_Ramp(0,-len,&CSYS_Foot[3],&Foot_Wooden_Bridge_Line_ramp[3]);
						Foot_Angle_Control(&CSYS_Foot[3]);
						ResetSlope(&Foot_Wooden_Bridge_Line_ramp[3]);
						state = 2;
						Part_State = 0;
					}
					break;
				}
			}
		}
	}
}

/**
	*@brief 跷跷板
	*/
void Foot_Seesaw()
{
	static float oval_x = 50.0f;
	static float oval_y = 80.0f;
	static float part_back = -100.0f*(1.0f/3.0f);
	static uint8_t Part_State;
	static uint8_t buffer_flag_1;
	static uint8_t p1_state;
	static uint8_t buffer_flag_2;
	if(Remote_State.Remote_Current_State != Remote_Foot_Emergency_Stop)
	{
		if(Part_State == 0)
		{
			if(!buffer_flag_1)
			{
				for(i=0;i<4;i++)
				{
					Foot_Line_NRTO_Ramp(-oval_x,0,&CSYS_Foot[i],&Foot_Rocker_Slow_Walk_Down_ramp[i]);
					Foot_Angle_Control(&CSYS_Foot[i]);
				}
				if(Slope(&Foot_Rocker_Slow_Walk_Down_ramp[0]) == 1.0f && Slope(&Foot_Rocker_Slow_Walk_Down_ramp[1]) == 1.0f &&
					 Slope(&Foot_Rocker_Slow_Walk_Down_ramp[2]) == 1.0f && Slope(&Foot_Rocker_Slow_Walk_Down_ramp[3]) == 1.0f)
				{
					for(i=0;i<4;i++)
					{
						Foot_Line_NRTO_Ramp(-oval_x,0,&CSYS_Foot[i],&Foot_Rocker_Slow_Walk_Down_ramp[i]);
						Foot_Angle_Control(&CSYS_Foot[i]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[i]);
					}
					buffer_flag_1 = 1;
				}
			}
			else
			{
				if(Slope(&Foot_Seesaw_Buffer_ramp) == 1.0f)
				{
					ResetSlope(&Foot_Seesaw_Buffer_ramp);
					buffer_flag_1 = 0;
					Part_State = 1;
				}
			}
		}
		else if(Part_State == 1)		//准备行走
		{
			if(p1_state == 0)
			{
				static float part_oval_x = 50.0f*(1.0f/3.0f);
				static float part_oval_y = 80.0f*(1.0f/3.0f);
				Foot_New_Walk(2*part_oval_x,oval_y,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Up_ramp[0]);
	//			Foot_Front_Up_Oval_RTO_Ramp(part_oval_x,0,part_oval_x,part_oval_y,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Up_ramp[0]);
				Foot_Angle_Control(&CSYS_Foot[0]);
	//			Foot_Front_Up_Oval_NRTO_Ramp(part_oval_x,part_oval_y,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Up_ramp[0]);
	//			Foot_Angle_Control(&CSYS_Foot[0]);
				if(Slope(&Foot_Rocker_Slow_Walk_Up_ramp[0]) == 1.0f)
				{
					Foot_New_Walk(2*part_oval_x,oval_y,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Up_ramp[0]);
	//				Foot_Front_Up_Oval_RTO_Ramp(part_oval_x,0,part_oval_x,part_oval_y,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Up_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
	//				Foot_Front_Up_Oval_NRTO_Ramp(part_oval_x,part_oval_y,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Up_ramp[0]);
	//				Foot_Angle_Control(&CSYS_Foot[0]);
					ResetSlope(&Foot_Rocker_Slow_Walk_Up_ramp[0]);
					p1_state = 1;
				}
			}
			else if(p1_state == 1)
			{
				static float part_oval_x = 50.0f*(2.0f/3.0f);
				static float part_oval_y = 80.0f*(2.0f/3.0f);
				Foot_New_Walk(2*part_oval_x,oval_y,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Up_ramp[3]);
	//			Foot_Front_Up_Oval_RTO_Ramp(part_oval_x,0,part_oval_x,part_oval_y,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Up_ramp[3]);
				Foot_Angle_Control(&CSYS_Foot[3]);
	//			Foot_Front_Up_Oval_NRTO_Ramp(part_oval_x,part_oval_y,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Up_ramp[3]);
	//			Foot_Angle_Control(&CSYS_Foot[3]);
				if(Slope(&Foot_Rocker_Slow_Walk_Up_ramp[3]) == 1.0f)
				{
					Foot_New_Walk(2*part_oval_x,oval_y,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Up_ramp[3]);
	//				Foot_Front_Up_Oval_RTO_Ramp(part_oval_x,0,part_oval_x,part_oval_y,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Up_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
	//				Foot_Front_Up_Oval_NRTO_Ramp(part_oval_x,part_oval_y,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Up_ramp[3]);
	//				Foot_Angle_Control(&CSYS_Foot[3]);
					ResetSlope(&Foot_Rocker_Slow_Walk_Up_ramp[3]);
					p1_state = 2;
				}
			}
			else if(p1_state == 2)
			{
				static float part_oval_x = 50.0f;
				static float part_oval_y = 80.0f;
				Foot_New_Walk(2*part_oval_x,oval_y,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Up_ramp[1]);
	//			Foot_Front_Up_Oval_RTO_Ramp(part_oval_x,0,part_oval_x,part_oval_y,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Up_ramp[1]);
				Foot_Angle_Control(&CSYS_Foot[1]);
	//			Foot_Front_Up_Oval_NRTO_Ramp(part_oval_x,part_oval_y,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Up_ramp[1]);
	//			Foot_Angle_Control(&CSYS_Foot[1]);
				if(Slope(&Foot_Rocker_Slow_Walk_Up_ramp[1]) == 1.0f)
				{
					Foot_New_Walk(2*part_oval_x,oval_y,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Up_ramp[1]);
	//				Foot_Front_Up_Oval_RTO_Ramp(part_oval_x,0,part_oval_x,part_oval_y,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Up_ramp[1]);
					Foot_Angle_Control(&CSYS_Foot[1]);
	//				Foot_Front_Up_Oval_NRTO_Ramp(part_oval_x,part_oval_y,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Up_ramp[1]);
	//				Foot_Angle_Control(&CSYS_Foot[1]);
					ResetSlope(&Foot_Rocker_Slow_Walk_Up_ramp[1]);
					p1_state = 3;
				}
			}
			else if(p1_state == 3)
			{
				if(Slope(&Foot_Run_Buffer_ramp) == 1.0f)
				{
					ResetSlope(&Foot_Run_Buffer_ramp);
					p1_state = 0;
					Part_State = 2;
					CSYS_Foot[0].Foot_State = Foot_State_Down;
					CSYS_Foot[1].Foot_State = Foot_State_Down;
					CSYS_Foot[2].Foot_State = Foot_State_Up;
					CSYS_Foot[3].Foot_State = Foot_State_Down;
				}
			}
		}
	//	#if 0
		else if(Part_State == 2)		//行走
		{
			if(CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[1].Foot_State == Foot_State_Down &&
				 CSYS_Foot[2].Foot_State == Foot_State_Up 	&& CSYS_Foot[3].Foot_State == Foot_State_Down)
			{
				if(!buffer_flag_2)
				{
					Foot_New_Walk(2*oval_x,oval_y,&CSYS_Foot[2],&Foot_Rocker_Slow_Walk_Up_ramp[2]);
	//				Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[2],&Foot_Rocker_Slow_Walk_Up_ramp[2]);
	//				Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[2],&Foot_Rocker_Slow_Walk_Up_ramp[2]);
					Foot_Angle_Control(&CSYS_Foot[2]);
					Foot_Line_NRTO_Ramp(part_back,0,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Down_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_Line_NRTO_Ramp(part_back,0,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Down_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
					Foot_Line_NRTO_Ramp(part_back,0,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Down_ramp[1]);
					Foot_Angle_Control(&CSYS_Foot[1]);
					if(Slope(&Foot_Rocker_Slow_Walk_Up_ramp[2]) == 1.0f && Slope(&Foot_Rocker_Slow_Walk_Down_ramp[0]) == 1.0f &&
						 Slope(&Foot_Rocker_Slow_Walk_Down_ramp[3]) == 1.0f	&& Slope(&Foot_Rocker_Slow_Walk_Down_ramp[1]) == 1.0f		)
					{
						Foot_New_Walk(2*oval_x,oval_y,&CSYS_Foot[2],&Foot_Rocker_Slow_Walk_Up_ramp[2]);
	//					Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[2],&Foot_Rocker_Slow_Walk_Up_ramp[2]);
	//					Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[2],&Foot_Rocker_Slow_Walk_Up_ramp[2]);
						Foot_Angle_Control(&CSYS_Foot[2]);
						Foot_Line_NRTO_Ramp(part_back,0,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Down_ramp[0]);
						Foot_Angle_Control(&CSYS_Foot[0]);
						buffer_flag_2 = 1;
						ResetSlope(&Foot_Rocker_Slow_Walk_Up_ramp[2]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[0]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[3]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[1]);
					}
				}
				else
				{
					if(Slope(&Foot_Seesaw_Buffer_ramp) == 1.0f)
					{
						buffer_flag_2 = 0;
						ResetSlope(&Foot_Seesaw_Buffer_ramp);
						CSYS_Foot[0].Foot_State = Foot_State_Up;
						CSYS_Foot[2].Foot_State = Foot_State_Down;
					}
				}
			}
			else if(CSYS_Foot[0].Foot_State == Foot_State_Up 	 && CSYS_Foot[1].Foot_State == Foot_State_Down &&
							CSYS_Foot[2].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down	 )
			{
				if(!buffer_flag_2)
				{
					Foot_New_Walk(2*oval_x,oval_y,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Up_ramp[0]);
	//				Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Up_ramp[0]);
	//				Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Up_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_Line_NRTO_Ramp(part_back,0,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Down_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
					Foot_Line_NRTO_Ramp(part_back,0,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Down_ramp[1]);
					Foot_Angle_Control(&CSYS_Foot[1]);
					Foot_Line_NRTO_Ramp(part_back,0,&CSYS_Foot[2],&Foot_Rocker_Slow_Walk_Down_ramp[2]);
					Foot_Angle_Control(&CSYS_Foot[2]);
					if(Slope(&Foot_Rocker_Slow_Walk_Up_ramp[0]) == 1.0f && Slope(&Foot_Rocker_Slow_Walk_Down_ramp[3]) == 1.0f &&
						 Slope(&Foot_Rocker_Slow_Walk_Down_ramp[1]) == 1.0f	&& Slope(&Foot_Rocker_Slow_Walk_Down_ramp[2]) == 1.0f		)
					{
						Foot_New_Walk(2*oval_x,oval_y,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Up_ramp[0]);
	//					Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Up_ramp[0]);
	//					Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Up_ramp[0]);
						Foot_Angle_Control(&CSYS_Foot[0]);
						Foot_Line_NRTO_Ramp(part_back,0,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Down_ramp[3]);
						Foot_Angle_Control(&CSYS_Foot[3]);
						buffer_flag_2 = 1;
						ResetSlope(&Foot_Rocker_Slow_Walk_Up_ramp[0]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[3]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[1]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[2]);
					}
				}
				else
				{
					if(Slope(&Foot_Seesaw_Buffer_ramp) == 1.0f)
					{
						buffer_flag_2 = 0;
						ResetSlope(&Foot_Seesaw_Buffer_ramp);
						CSYS_Foot[0].Foot_State = Foot_State_Down;
						CSYS_Foot[3].Foot_State = Foot_State_Up;
					}
				}
			}
			else if(CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[1].Foot_State == Foot_State_Down &&
							CSYS_Foot[2].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Up)
			{
				if(!buffer_flag_2)
				{
					Foot_New_Walk(2*oval_x,oval_y,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Up_ramp[3]);
	//				Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Up_ramp[3]);
	//				Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Up_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
					Foot_Line_NRTO_Ramp(part_back,0,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Down_ramp[1]);
					Foot_Angle_Control(&CSYS_Foot[1]);
					Foot_Line_NRTO_Ramp(part_back,0,&CSYS_Foot[2],&Foot_Rocker_Slow_Walk_Down_ramp[2]);
					Foot_Angle_Control(&CSYS_Foot[2]);
					Foot_Line_NRTO_Ramp(part_back,0,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Down_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					if(Slope(&Foot_Rocker_Slow_Walk_Up_ramp[3]) == 1.0f && Slope(&Foot_Rocker_Slow_Walk_Down_ramp[1]) == 1.0f &&
						 Slope(&Foot_Rocker_Slow_Walk_Down_ramp[2]) == 1.0f	&& Slope(&Foot_Rocker_Slow_Walk_Down_ramp[0]) == 1.0f		)
					{
						Foot_New_Walk(2*oval_x,oval_y,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Up_ramp[3]);
	//					Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Up_ramp[3]);
	//					Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Up_ramp[3]);
						Foot_Angle_Control(&CSYS_Foot[3]);
						Foot_Line_NRTO_Ramp(part_back,0,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Down_ramp[1]);
						Foot_Angle_Control(&CSYS_Foot[1]);
						buffer_flag_2 = 1;
						ResetSlope(&Foot_Rocker_Slow_Walk_Up_ramp[3]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[1]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[2]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[0]);
					}
				}
				else
				{
					if(Slope(&Foot_Seesaw_Buffer_ramp) == 1.0f)
					{
						buffer_flag_2 = 0;
						ResetSlope(&Foot_Seesaw_Buffer_ramp);
						CSYS_Foot[3].Foot_State = Foot_State_Down;
						CSYS_Foot[1].Foot_State = Foot_State_Up;
					}
				}
			}
	//		#if 0 
			else if(CSYS_Foot[0].Foot_State == Foot_State_Down && CSYS_Foot[1].Foot_State == Foot_State_Up && 
							CSYS_Foot[2].Foot_State == Foot_State_Down && CSYS_Foot[3].Foot_State == Foot_State_Down)
			{
				if(!buffer_flag_2)
				{
					Foot_New_Walk(2*oval_x,oval_y,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Up_ramp[1]);
	//				Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Up_ramp[1]);
	//				Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Up_ramp[1]);
					Foot_Angle_Control(&CSYS_Foot[1]);
					Foot_Line_NRTO_Ramp(part_back,0,&CSYS_Foot[2],&Foot_Rocker_Slow_Walk_Down_ramp[2]);
					Foot_Angle_Control(&CSYS_Foot[2]);
					Foot_Line_NRTO_Ramp(part_back,0,&CSYS_Foot[0],&Foot_Rocker_Slow_Walk_Down_ramp[0]);
					Foot_Angle_Control(&CSYS_Foot[0]);
					Foot_Line_NRTO_Ramp(part_back,0,&CSYS_Foot[3],&Foot_Rocker_Slow_Walk_Down_ramp[3]);
					Foot_Angle_Control(&CSYS_Foot[3]);
					if(Slope(&Foot_Rocker_Slow_Walk_Up_ramp[1]) == 1.0f && Slope(&Foot_Rocker_Slow_Walk_Down_ramp[2]) == 1.0f &&
						 Slope(&Foot_Rocker_Slow_Walk_Down_ramp[0]) == 1.0f	&& Slope(&Foot_Rocker_Slow_Walk_Down_ramp[3]) == 1.0f		)
					{
						Foot_New_Walk(2*oval_x,oval_y,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Up_ramp[1]);
	//					Foot_Front_Up_Oval_RTO_Ramp(oval_x,0,oval_x,oval_y,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Up_ramp[1]);
	//					Foot_Front_Up_Oval_NRTO_Ramp(oval_x,oval_y,&CSYS_Foot[1],&Foot_Rocker_Slow_Walk_Up_ramp[1]);
						Foot_Angle_Control(&CSYS_Foot[1]);
						Foot_Line_NRTO_Ramp(part_back,0,&CSYS_Foot[2],&Foot_Rocker_Slow_Walk_Down_ramp[2]);
						Foot_Angle_Control(&CSYS_Foot[2]);
						buffer_flag_2 = 1;
						ResetSlope(&Foot_Rocker_Slow_Walk_Up_ramp[1]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[2]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[0]);
						ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[3]);
					}
				}
				else
				{
					if(Slope(&Foot_Seesaw_Buffer_ramp) == 1.0f)
					{
						buffer_flag_2 = 0;
						ResetSlope(&Foot_Seesaw_Buffer_ramp);
						if(Remote_State.Remote_Current_State == Remote_Foot_Seesaw_Stop)
						{
							for(i=0;i<4;i++)
							{
								CSYS_Foot[i].Foot_State = Foot_State_None;
							}
							Remote_State_Last_Current_Update(Remote_Foot_None);
						}
						else
						{
							CSYS_Foot[1].Foot_State = Foot_State_Down;
							CSYS_Foot[2].Foot_State = Foot_State_Up;
						}
					}
				}
			}
//		#endif
		}
	}
	else
	{
		ResetFlag(&Part_State);
		ResetFlag(&buffer_flag_1);
		ResetFlag(&p1_state);
		ResetFlag(&buffer_flag_2);
		for(i=0;i<4;i++)
		{
			ResetSlope(&Foot_Rocker_Slow_Walk_Down_ramp[i]);
			ResetSlope(&Foot_Rocker_Slow_Walk_Up_ramp[i]);
		}
		ResetSlope(&Foot_Seesaw_Buffer_ramp);
		Remote_State_Last_Current_Update(Remote_Foot_None);
	}
//	#endif
}

void Foot_Run_Test()
{
	static uint8_t state;
	static float s = 100.0f;
	static float h = 50.0f;
	if(state == 0)
	{
		for(i=0;i<4;i++)
		{
//			Foot_Line_NRTO_Ramp(-60.0f,0,&CSYS_Foot[i],&Foot_line_ramp[i]);
			Foot_New_Walk(s,h,&CSYS_Foot[i],&oval_ramp[i]);
			Foot_Angle_Control(&CSYS_Foot[i]);
		}
		if(Slope(&oval_ramp[0]) == 1.0f && Slope(&oval_ramp[1]) == 1.0f &&
			 Slope(&oval_ramp[2]) == 1.0f && Slope(&oval_ramp[3]) == 1.0f)
		{
			for(i=0;i<4;i++)
			{
//				Foot_Line_NRTO_Ramp(-60.0f,0,&CSYS_Foot[i],&Foot_line_ramp[i]);
				Foot_New_Walk(s,h,&CSYS_Foot[i],&oval_ramp[i]);
				Foot_Angle_Control(&CSYS_Foot[i]);
				ResetSlope(&oval_ramp[i]);
			}
			state = 1;
		}
	}
	else if(state == 1)
	{
		if(Slope(&Foot_Run_Buffer_ramp) == 1.0f)
		{
			ResetSlope(&Foot_Run_Buffer_ramp);
			state = 2;
		}
	}
	else if(state == 2)
	{
		for(i=0;i<4;i++)
		{
			Foot_Line_NRTO_Ramp(-60.0f,0,&CSYS_Foot[i],&Foot_line_ramp[i]);
//			Foot_Round_X_NRTO_ramp(-2*s,&CSYS_Foot[i],&Foot_line_ramp[i]);
			Foot_Angle_Control(&CSYS_Foot[i]);
		}
		if(Slope(&Foot_line_ramp[0]) == 1.0f && Slope(&Foot_line_ramp[1]) == 1.0f &&
			 Slope(&Foot_line_ramp[2]) == 1.0f && Slope(&Foot_line_ramp[3]) == 1.0f)
		{
			for(i=0;i<4;i++)
			{
				Foot_Line_NRTO_Ramp(-60.0f,0,&CSYS_Foot[i],&Foot_line_ramp[i]);
//				Foot_Round_X_NRTO_ramp(-s,&CSYS_Foot[i],&Foot_line_ramp[i]);
				Foot_Angle_Control(&CSYS_Foot[i]);
				ResetSlope(&Foot_line_ramp[i]);
			}
			state = 3;
		}
	}
	else if(state == 3)
	{
		if(Slope(&Foot_Run_Buffer_ramp) == 1.0f)
		{
			ResetSlope(&Foot_Run_Buffer_ramp);
			state = 0;
		}
	}
}



