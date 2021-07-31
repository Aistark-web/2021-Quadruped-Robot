#ifndef __PID_H_
#define __PID_H_

#include "stm32f4xx_hal.h"


/**
	* @brief 标准式PID结构体
	*/

typedef struct{
	float Kp;
	float Ki;
	float Kd;
	float Limit;
	float Error_Current;
	float Error_Last;
	float Error_Infer;
	float PID_Out;
}PID_Standard;

extern PID_Standard PID_Place_RM3508[8];
extern PID_Standard PID_Speed_RM3508[8];
#if 0
extern PID_Standard PID_Place_RM3508_Left_Front_Leg_Outside;	
extern PID_Standard PID_Place_RM3508_Left_Front_Leg_Inside;
extern PID_Standard PID_Place_RM3508_Right_Front_Leg_Outside;	
extern PID_Standard PID_Place_RM3508_Right_Front_Leg_Inside;	
extern PID_Standard PID_Place_RM3508_Left_Hind_Leg_Outside;	
extern PID_Standard PID_Place_RM3508_Left_Hind_Leg_Inside;
extern PID_Standard PID_Place_RM3508_Right_Hind_Leg_Outside;		
extern PID_Standard PID_Place_RM3508_Right_Hind_Leg_Inside;

extern PID_Standard PID_Speed_RM3508_Left_Front_Leg_Outside;	
extern PID_Standard PID_Speed_RM3508_Left_Front_Leg_Inside;
extern PID_Standard PID_Speed_RM3508_Right_Front_Leg_Outside;	
extern PID_Standard PID_Speed_RM3508_Right_Front_Leg_Inside;
extern PID_Standard PID_Speed_RM3508_Left_Hind_Leg_Outside;	
extern PID_Standard PID_Speed_RM3508_Left_Hind_Leg_Inside;
extern PID_Standard PID_Speed_RM3508_Right_Hind_Leg_Outside;		
extern PID_Standard PID_Speed_RM3508_Right_Hind_Leg_Inside;
#endif
void PID_Control_Place(float Current_value,float Infer_value,PID_Standard *PID);
void PID_Control_Speed(float Current_value,float Infer_value,PID_Standard *PID);
void PID_Place_Init(PID_Standard *PID);
void PID_Speed_Init(PID_Standard *PID);

#endif
