#include "my_kalman.h"

kalman_filter_t Imu_Pitch_kalman_filter;
kalman_filter_t Imu_Roll_kalman_filter;
kalman_filter_t Imu_Yaw_kalman_filter;

void Kalman_Filter_Init()
{
	kalman_Init(&Imu_Pitch_kalman_filter,1,200);
	kalman_Init(&Imu_Roll_kalman_filter,1,200);
	kalman_Init(&Imu_Yaw_kalman_filter,1,200);
}
