#ifndef __MY_PID_H
#define __MY_PID_H

#include "PID.h"

void PID_Control_Place_Limit(float Current_Value,float Infer_Value,PID *PID,float PID_LIMIT);
void PID_Control_Speed_Limit(float Current_Value,float Infer_Value,PID *PID,float PID_LIMIT);
void PID_Place_Init(PID *PID_Place);
void PID_Speed_Init(PID *PID_Speed);

extern PID PID_Place_RM3508[8];
extern PID PID_Speed_RM3508[8];

#endif
