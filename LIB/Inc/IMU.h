#ifndef _IMU_H_
#define _IMU_H_

#include "stm32f4xx_hal.h"


#define Time_EN 0//ʱ��
#define Acceleration_EN 0//���ٶ�
#define AngularVelocity_EN 1//���ٶ�
#define EulerAngle_EN 1//ŷ����
#define MagneticFieldIntensity_EN 0//�ų�ǿ��
#define Pressure_Height_EN 0//��ѹ���߶�
#define Quaternions_EN 0//��Ԫ��
#define USART6_BUFLEN 30		//�������ݳ���

typedef struct{

#if Time_EN == 1
	struct {
		int16_t MS;
	}Time;//ʱ��
#endif
	
#if Acceleration_EN == 1
	struct {
		int16_t X;
		int16_t Y;
		int16_t Z;
	}Acceleration;//���ٶ�
#endif
	
#if AngularVelocity_EN == 1
struct {
	float X;
	float Y;
	float Z;
	}AngularVelocity;//���ٶ�
#endif

#if EulerAngle_EN == 1
struct {
		float Pitch;
		float Roll;
		float Yaw;
		int16_t r;
		float LsatAngle;
		float ContinuousYaw;
		float Yawoffset;
	}EulerAngler;//ŷ����
#endif	
	
#if MagneticFieldIntensity_EN == 1
	struct {
		int16_t X;
		int16_t Y;
		int16_t Z;
	}MagneticFieldIntensity;//�ų�ǿ��
#endif
	
#if Pressure_Height_EN == 1
	struct {
		int16_t P;
		int16_t H;
	}Pressure_Height;//��ѹ���߶�
#endif
	
#if Quaternions_EN == 1
	struct {
		float W;
		float X;
		float Y;
		float Z;
	}Quaternions;//��Ԫ��
#endif
	
}IMU_Typedef;

typedef enum 
{
	kItemTime = 					0x50,	// ʱ�����ݰ�ͷ
    kItemAcceleration =         	0x51,   // ���ٶ����ݰ�ͷ
    kItemAngularVelocity =      	0x52,	// ���ٶ����ݰ�ͷ
	kItemEulerAngler =        		0x53,   // ŷ�������ݰ�ͷ
	kItemMagneticFieldIntensity =   0x54,	// �ų�ǿ�����ݰ�ͷ
	kItemPressure_Height = 			0x56,	// ��ѹ���߶ȴų�ǿ��
    kItemQuaternions =         		0x59,   // ��Ԫ���ų�ǿ��  
}ItemID_t;

extern void IMU_Receive(IMU_Typedef* Dst, uint8_t* Data);

#endif
