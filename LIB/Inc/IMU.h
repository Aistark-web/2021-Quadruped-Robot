#ifndef _IMU_H_
#define _IMU_H_

#include "stm32f4xx_hal.h"


#define Time_EN 0//时间
#define Acceleration_EN 0//加速度
#define AngularVelocity_EN 1//角速度
#define EulerAngle_EN 1//欧拉角
#define MagneticFieldIntensity_EN 0//磁场强度
#define Pressure_Height_EN 0//气压、高度
#define Quaternions_EN 0//四元数
#define USART6_BUFLEN 30		//串口数据长度

typedef struct{

#if Time_EN == 1
	struct {
		int16_t MS;
	}Time;//时间
#endif
	
#if Acceleration_EN == 1
	struct {
		int16_t X;
		int16_t Y;
		int16_t Z;
	}Acceleration;//加速度
#endif
	
#if AngularVelocity_EN == 1
struct {
	float X;
	float Y;
	float Z;
	}AngularVelocity;//角速度
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
	}EulerAngler;//欧拉角
#endif	
	
#if MagneticFieldIntensity_EN == 1
	struct {
		int16_t X;
		int16_t Y;
		int16_t Z;
	}MagneticFieldIntensity;//磁场强度
#endif
	
#if Pressure_Height_EN == 1
	struct {
		int16_t P;
		int16_t H;
	}Pressure_Height;//气压、高度
#endif
	
#if Quaternions_EN == 1
	struct {
		float W;
		float X;
		float Y;
		float Z;
	}Quaternions;//四元数
#endif
	
}IMU_Typedef;

typedef enum 
{
	kItemTime = 					0x50,	// 时间数据包头
    kItemAcceleration =         	0x51,   // 加速度数据包头
    kItemAngularVelocity =      	0x52,	// 角速度数据包头
	kItemEulerAngler =        		0x53,   // 欧拉角数据包头
	kItemMagneticFieldIntensity =   0x54,	// 磁场强度数据包头
	kItemPressure_Height = 			0x56,	// 气压、高度磁场强度
    kItemQuaternions =         		0x59,   // 四元数磁场强度  
}ItemID_t;

extern void IMU_Receive(IMU_Typedef* Dst, uint8_t* Data);

#endif
