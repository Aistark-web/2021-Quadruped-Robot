#include "my_pid.h"

PID PID_Place_RM3508[8];
PID PID_Speed_RM3508[8];

void PID_Control_Place_Limit(float Current_Value,float Infer_Value,PID *PID_Place,float PID_LIMIT)
{
	PID_Control(Current_Value,Infer_Value,PID_Place);
	limit(PID_Place->pid_out,PID_LIMIT,-PID_LIMIT);
}

void PID_Control_Speed_Limit(float Current_Value,float Infer_Value,PID *PID_Speed,float PID_LIMIT)
{
	PID_Control(Current_Value,Infer_Value,PID_Speed);
	limit(PID_Speed->pid_out,PID_LIMIT,-PID_LIMIT);
}

void PID_Place_Init(PID *PID_Place)
{
	PID_Place->Kp = 6.0f;						//		6.0		7.2f			
	PID_Place->Ki = 0.05;							//		0.05	0.01f	
	PID_Place->Kd = 15.0f;					//		15		50.0f
	PID_Place->limit =	2000.0f;		
	PID_Place->error_inter = 0;
	PID_Place->error_last = 0;
	PID_Place->error_now = 0;
	PID_Place->pid_out = 0;
}

void PID_Speed_Init(PID *PID_Speed)
{
	PID_Speed->Kp = 8.0f;							//4.0		5.4f
	PID_Speed->Ki = 0.01f;						//0.01	
	PID_Speed->Kd = 15.0f;							//4.0		5.0f
	PID_Speed->limit = 2000.0f;
	PID_Speed->error_inter = 0;
	PID_Speed->error_last = 0;
	PID_Speed->error_now = 0;
	PID_Speed->pid_out = 0;
}

