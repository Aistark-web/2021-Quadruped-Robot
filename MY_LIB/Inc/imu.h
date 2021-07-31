#ifndef __IMU_H_
#define __IMU_H_

#include <stdint.h>

/**
	*@brief �����ǽṹ�� 
	*/
typedef struct{
	int16_t a_x;		//<!@brief x����ٶ�
	int16_t a_y;		//<!@brief y����ٶ�
	int16_t a_z;		//<!@brief z����ٶ�
	float Pitch;		//<!@brief ������
	float Yaw;			//<!@brief �����
	float Roll;			//<!@brief ������
}IMU_Handle;


void IMU_Receive(IMU_Handle *Imu,uint8_t *data);
void CRC_Check(void);
#endif
