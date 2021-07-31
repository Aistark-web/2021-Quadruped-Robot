#ifndef __CSYS_H
#define __CSYS_H
/**
	*@file ����ϵͷ�ļ�
	*/

#include "my_motor.h"
#include "imu.h"
#include "arm_math.h"

#ifdef __ramp_H


#else

//#include "ramp.h"
#include "my_ramp.h"
#endif
typedef struct 
{
		uint8_t ID;
		uint8_t Foot_State;
    float theta1;
    float theta2;
		float Psi;
		float Phi;
		float x;
		float y;
		float z;
		float L;
		MY_RM3508_Handle *RM3508_1;
		MY_RM3508_Handle *RM3508_2;
		float Infer_theta1;							
		float Infer_theta2;
		float Infer_x;								//�������ϵ������xֵ
		float Infer_y;								//�������ϵ������yֵ
		float Infer_L;								//�����ȳ�ֵ
		
}CSYS_Handle;


typedef struct
{
	float x;
	float y;
	float z;
}CSYS_Global_Foot_Handle;

typedef struct
{
	float x;
	float y;
	float z;
}CSYS_Global_Motor_Handle;

typedef struct
{
	float x;
	float y;
	float z;
}CSYS_Global_Body_Center_Handle;

typedef struct
{
	float Vector_x;
	float Vector_y;
	float Vector_z;
}CSYS_Global_Vector_Motor_To_Foot_Handle;

typedef struct
{
	uint8_t ID;
	CSYS_Global_Body_Center_Handle *CSYS_Global_Body_Center;
	CSYS_Global_Motor_Handle *CSYS_Global_Motor;
	CSYS_Global_Foot_Handle *CSYS_Global_Foot;
	CSYS_Global_Vector_Motor_To_Foot_Handle *CSYS_Global_Vector_Motor_To_Foot;
	float CSYS_Infer_x;																	//����������ϵx����ֵ
	float CSYS_Infer_y;																	//����������ϵy����ֵ
}CSYS_Global_Handle;
				
#define L1 										160.0f														//С�ȳ�
#define L2 										320.0f														//���ȳ�
#define EXCHANGE_RAD_TO_ANGLE 57.2957795130823208768f						//������ת���Ƕ�ֵ
#define Motor_y								306.9063326673223f								//���������ϵ��y��ֵ
#define Motor_x  							0																	//���������ϵ��x��ֵ
#define Body_Center_To_Motor_Distance_x									202.5f
#define Body_Center_To_Motor_Distance_y									280.0f
#define Body_Center_To_Motor_Distance_z									10.0f

extern CSYS_Handle CSYS_Foot[4];
extern CSYS_Global_Handle CSYS_Global[4];
extern CSYS_Global_Body_Center_Handle CSYS_Global_Body_Center[4];
extern CSYS_Global_Foot_Handle CSYS_Global_Foot[4];
extern CSYS_Global_Motor_Handle CSYS_Global_Motor[4];
extern CSYS_Global_Vector_Motor_To_Foot_Handle CSYS_Global_Vector_Motor_To_Foot[4];

void CSYS_Init(void);
CSYS_Global_Handle CSYS_Global_Vector_Analysis(CSYS_Global_Handle *CSYS_Global);
CSYS_Handle Foot_Get_CSYS(CSYS_Handle *CSYS);
CSYS_Handle Foot_Line_RTO_Get_Theta_Ramp(float x,Ramp_Typedef *Ramp);
CSYS_Handle Foot_Line_RTO_Right_To_Left_Theta_Ramp(float x,Ramp_Typedef *Ramp);			//��ʱʹ��
CSYS_Handle Foot_Line_RTO_Get_Theta(float x);
CSYS_Handle Foot_Oval_RTO_Get_Theta_Ramp(float a,float b,Ramp_Typedef *Ramp);

CSYS_Handle Foot_Point_RTO(float x,float y,CSYS_Handle *CSYS);
CSYS_Handle Foot_Point_RTO_Ramp(float x,float y,CSYS_Handle *CSYS,Ramp_Typedef *Ramp);
CSYS_Handle Foot_Line_NRTO_Ramp(float x,float y,CSYS_Handle *CSYS,Ramp_Typedef *Ramp);
CSYS_Handle Foot_Line_NRTO_Get_Theta_Ramp(float x,Ramp_Typedef *Ramp,CSYS_Handle *CSYS);
CSYS_Handle Foot_Front_Up_Oval_RTO_Ramp(float x,float y,float a,float b,CSYS_Handle *CSYS,Ramp_Typedef *Ramp);
CSYS_Handle Foot_Back_Up_Oval_RTO_Ramp(float x,float y,float a,float b,CSYS_Handle *CSYS,Ramp_Typedef *Ramp);
CSYS_Handle Foot_Back_Down_Oval_RTO_Ramp(float x,float y,float a,float b,CSYS_Handle *CSYS,Ramp_Typedef *Ramp);
CSYS_Handle	Foot_Front_Up_Oval_NRTO_Ramp(float a,float b,CSYS_Handle *CSYS,Ramp_Typedef *Ramp);
CSYS_Handle Foot_Back_Up_Oval_NRTO_Ramp(float a,float b,CSYS_Handle *CSYS,Ramp_Typedef *Ramp);
CSYS_Handle Foot_Back_Down_Oval_NRTO_Ramp(float a,float b,CSYS_Handle *CSYS,Ramp_Typedef *Ramp);
CSYS_Handle Foot_Round_X_NRTO_ramp(float x,CSYS_Handle *CSYS,Ramp_Typedef *Ramp);
CSYS_Handle *Foot_X_New_Walk(float x,float y,CSYS_Handle CSYS[],Ramp_Global_Typedef *Ramp_global);
CSYS_Handle Foot_New_Walk(float s,float h,CSYS_Handle *CSYS,Ramp_Typedef *Ramp);
CSYS_Handle Foot_New_Walk_RTO(float x,float y,float s,float h,CSYS_Handle *CSYS,Ramp_Typedef *Ramp);
CSYS_Handle Foot_New_Walk_Target_RTO(float x,float y,CSYS_Handle *CSYS,Ramp_Typedef *Ramp);
CSYS_Global_Handle Posture_Pitch(float Pitch,CSYS_Handle *CSYS,CSYS_Global_Handle *CSYS_global,float x,float y);
CSYS_Handle Foot_IMU_New_Walk_NRTO(float s,float h,CSYS_Handle *CSYS,IMU_Typedef *imu,CSYS_Global_Handle *CSYS_global,Ramp_Typedef *Ramp);
#endif
