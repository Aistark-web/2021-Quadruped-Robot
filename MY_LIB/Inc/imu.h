#ifndef __IMU_H_
#define __IMU_H_

#include <stdint.h>

/**
	*@brief 陀螺仪结构体 
	*/
typedef struct{
	int16_t a_x;		//<!@brief x轴加速度
	int16_t a_y;		//<!@brief y轴加速度
	int16_t a_z;		//<!@brief z轴加速度
	float Pitch;		//<!@brief 俯仰角
	float Yaw;			//<!@brief 航向角
	float Roll;			//<!@brief 翻滚角
}IMU_Handle;


void IMU_Receive(IMU_Handle *Imu,uint8_t *data);
void CRC_Check(void);
#endif
