#include "pid.h"
#include "motor.h"
/**
	*@brief 声明PID句柄
	*/
/**
	*@brief 数组8个PID位置环句柄分别对应八个电机
					数组元素下标=电机所接的电调CAN_ID-200-1
	*/
PID_Standard PID_Place_RM3508[8];
PID_Standard PID_Speed_RM3508[8];
PID_Standard PID_Place_RM3508_Left_Front_Leg_Outside;	
PID_Standard PID_Place_RM3508_Left_Front_Leg_Inside;
PID_Standard PID_Speed_RM3508_Left_Front_Leg_Outside;	
PID_Standard PID_Speed_RM3508_Left_Front_Leg_Inside;

PID_Standard PID_Place_RM3508_Right_Front_Leg_Outside;	
PID_Standard PID_Place_RM3508_Right_Front_Leg_Inside;	
PID_Standard PID_Speed_RM3508_Right_Front_Leg_Outside;	
PID_Standard PID_Speed_RM3508_Right_Front_Leg_Inside;	

PID_Standard PID_Place_RM3508_Left_Hind_Leg_Outside;	
PID_Standard PID_Place_RM3508_Left_Hind_Leg_Inside;
PID_Standard PID_Speed_RM3508_Left_Hind_Leg_Outside;	
PID_Standard PID_Speed_RM3508_Left_Hind_Leg_Inside;

PID_Standard PID_Place_RM3508_Right_Hind_Leg_Outside;		
PID_Standard PID_Place_RM3508_Right_Hind_Leg_Inside;
PID_Standard PID_Speed_RM3508_Right_Hind_Leg_Outside;		
PID_Standard PID_Speed_RM3508_Right_Hind_Leg_Inside;

/**
	* @brief 标准式PID位置环控制 
	*/

void PID_Control_Place(float Current_value,float Infer_value,PID_Standard *PID)
{
	PID->Error_Last = PID->Error_Current;
	PID->Error_Current = Infer_value-Current_value;
	PID->Error_Infer += PID->Error_Current;
	
	if(PID->Error_Infer > PID->Limit)
		PID->Error_Infer = PID->Limit;
	if(PID->Error_Infer < -PID->Limit)
		PID->Error_Infer = -PID->Limit;
	
	PID->PID_Out 	= PID->Kp * PID->Error_Current
								+	PID->Ki *	PID->Error_Infer
								+	PID->Kd *	(PID->Error_Current - PID->Error_Last);
	if(PID->PID_Out > RM3508_LIMIT)
	{
		PID->PID_Out = RM3508_LIMIT;
	}
	else if(PID->PID_Out < -RM3508_LIMIT)
	{
		PID->PID_Out = -RM3508_LIMIT;
	}
}

/**
	* @brief 标准式PID速度环控制 
	*/

void PID_Control_Speed(float Current_value,float Infer_value,PID_Standard *PID)
{
	PID->Error_Last = PID->Error_Current;
	PID->Error_Current = Infer_value - Current_value;
	PID->Error_Infer += PID->Error_Infer;
	
	if((PID->Error_Infer > PID->Limit))
		PID->Error_Infer = PID->Limit;
	if((PID->Error_Infer < -PID->Limit))
		PID->Error_Infer = -PID->Limit;
	PID->PID_Out 	= PID->Kp * PID->Error_Current
								+	PID->Ki * PID->Error_Infer
								+ PID->Kd * (PID->Error_Current - PID->Error_Last);
	if(PID->PID_Out > RM3508_LIMIT)
	{
		PID->PID_Out = RM3508_LIMIT;
	}
	else if(PID->PID_Out < -RM3508_LIMIT)
	{
		PID->PID_Out = -RM3508_LIMIT;
	}
}

/**
	* @brief 标准式PID位置环初始化
	*/

void PID_Place_Init(PID_Standard *PID)
{
	PID->Kp 						= 5.0f;
	PID->Ki 						= 0;
	PID->Kd							=	50.0f;
	PID->Error_Current 	= 0.0f;
	PID->Error_Last 		= 0.0f;
	PID->Limit					=	3000.0f;
	PID->Error_Infer		=	0.0f;
	PID->PID_Out				= 0.0f;
}

/**
	* @brief 标准式PID速度环初始化
	*/

void PID_Speed_Init(PID_Standard *PID)
{
	PID->Kp							=	3.0f;
	PID->Ki							=	0.0003f;
	PID->Kd							=	0.0f;
	PID->Error_Current	= 0.0f;
	PID->Error_Last			= 0.0f;
	PID->Error_Infer		= 0.0f;
	PID->Limit					= 1000.0f;
	PID->PID_Out				= 0.0f;
}

