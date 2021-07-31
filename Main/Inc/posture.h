#ifndef __POSTURE_H_
#define __POSTURE_H_

#include "imu.h"
#include "csys.h"
#include "pid_control.h"
typedef struct
{
	uint8_t State;
	IMU_Typedef *imu;
	float Infer_Pitch;
	float Infer_Roll;
	float Infer_Yaw;
	float Rotate_Yaw;
	float Infer_x;
	float Infer_y;
	float Infer_z;
}Posture_Handle;


void Posture_Handle_Init(Posture_Handle *Posture);
void Posture_One_Foot_Change(CSYS_Global_Handle *CSYS,Posture_Handle *Posture,float Roll,float Pitch,float Yaw);
void Posture_Steady_State(void);
Posture_Handle Posture_Steep_Steady_State(void);
Posture_Handle Foot_Walk_Posture(CSYS_Global_Handle *CSYS_Global,CSYS_Handle *CSYS,IMU_Typedef *imu,uint8_t *flag);
float Foot_Orient_Calibration_Direction(IMU_Typedef *imu,uint8_t *flag);
void Posture_Walk(void);
void ResetFlag(uint8_t *flag);
#endif
