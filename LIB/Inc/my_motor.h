#ifndef __MY_MOTOR_H
#define __MY_MOTOR_H

#include "motor.h"

//#define RM3508_Place_Limit 8000.0f
#define EXCHANGE_ANGLE_DEG_TO_INFER_REAL_ANGLE (187.0f/3591.0f)		//�������Ƕ��ƽǶ� ת���� С���������Ƕ�
#define EXCHANGE_REAL_ANGLE_TO_ANGLE 436.9796791443f							//С���������Ƕ� ת���� �������Ƕ�

typedef struct 
{
	uint8_t ESC_ID;													//!<@brief ���ID
	uint16_t InitAngle;											//!<@brief ��ʼ��е�Ƕ�				/* ����
	int16_t Init_r;													//!<@brief ��ʼȦ��						 	 ���	*/
	float Real_Angle;												//!<@brief С���������Ƕ�			(��λ����)
	float Infer_Real_Angle;									//!<@brief С�������������Ƕ�	(��λ����)
	int32_t Infer_Angle;										//!<@brief ������J-Scope�۲�����
	uint8_t get_flag;												//!<@breif ��ȡ����
	
	RM3508_TypeDef Senior_RM3508_Handle;		//!<@brief ���ÿ��RM3508���
}MY_RM3508_Handle;

void RM3508_Receive_Get_More(MY_RM3508_Handle *Motor_RM3508, uint8_t *Data,uint8_t *flag);
extern MY_RM3508_Handle Motor_RM3508[8];
extern MY_RM3508_Handle Motor_RM3508_Front_Right_Outside_Test;
extern MY_RM3508_Handle Motor_RM3508_Front_Right_Inside_Test;
#endif
