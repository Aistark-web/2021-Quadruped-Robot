#ifndef __MY_MOTOR_H
#define __MY_MOTOR_H

#include "motor.h"

//#define RM3508_Place_Limit 8000.0f
#define EXCHANGE_ANGLE_DEG_TO_INFER_REAL_ANGLE (187.0f/3591.0f)		//连续化角度制角度 转换成 小轴连续化角度
#define EXCHANGE_REAL_ANGLE_TO_ANGLE 436.9796791443f							//小轴连续化角度 转换成 连续化角度

typedef struct 
{
	uint8_t ESC_ID;													//!<@brief 电调ID
	uint16_t InitAngle;											//!<@brief 初始机械角度				/* 消除
	int16_t Init_r;													//!<@brief 初始圈数						 	 误差	*/
	float Real_Angle;												//!<@brief 小轴连续化角度			(单位：度)
	float Infer_Real_Angle;									//!<@brief 小轴连续化期望角度	(单位：度)
	int32_t Infer_Angle;										//!<@brief 仅用于J-Scope观测数据
	uint8_t get_flag;												//!<@breif 获取数据
	
	RM3508_TypeDef Senior_RM3508_Handle;		//!<@brief 引用库的RM3508句柄
}MY_RM3508_Handle;

void RM3508_Receive_Get_More(MY_RM3508_Handle *Motor_RM3508, uint8_t *Data,uint8_t *flag);
extern MY_RM3508_Handle Motor_RM3508[8];
extern MY_RM3508_Handle Motor_RM3508_Front_Right_Outside_Test;
extern MY_RM3508_Handle Motor_RM3508_Front_Right_Inside_Test;
#endif
