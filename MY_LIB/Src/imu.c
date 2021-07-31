#include "imu.h"
#include "crc.h"
#include "my_kalman.h"
//IMU_Handle IMU;
IMU_Handle IMU;
uint16_t len;
extern kalman_filter_t Imu_Pitch_kalman_filter;
extern kalman_filter_t Imu_Roll_kalman_filter;
extern kalman_filter_t Imu_Yaw_kalman_filter;
/**
	* @brief 	HI219���Բ���ģ�����
						���ݰ���ʽ��[PRE TYPE LEN CRC ID1 DATA1 ID2 DATA2]
												[PRE] ��ǰ���룩: [0x5A]
												[TYPE]��֡���ͣ�: [0xA5]
												[LEN]	��֡���ȣ���[0x__ 0x__]
												[CRC]	��CRC16У����):	[0x__ 0x__]
												[ID1]	�����ٶ�ID�룩��
												[DATA1](���ٶ�X��Y��Z�����ݰ�)
												[ID2]	(ŷ���Ǹ��������ID��)
												[DATA2](ŷ���Ǹ��������ݰ�)
	*/
void IMU_Receive(IMU_Handle *Imu,uint8_t *data)
{
	static int16_t mid_int16_t;
	static int32_t mid_float;
	static uint16_t Need_Vertify_CRC ;
	
	mid_int16_t = data[7]  | data[8]  << 8;
	Imu->a_x = *(int16_t *)&mid_int16_t;
	mid_int16_t = data[9]  | data[10] << 8;
	Imu->a_y = *(int16_t *)&mid_int16_t;
	mid_int16_t = data[11] | data[12] << 8;
	Imu->a_z = *(int16_t *)&mid_int16_t;
	mid_float 	= (data[14] | data[15] << 8 | data[16] << 16 | data[17] << 24);
	Imu->Pitch 	= *(float *)&mid_float;
	mid_float 	= (data[18] | data[19] << 8 | data[20] << 16 | data[21] << 24);
	Imu->Roll		= *(float *)&mid_float;
	mid_float		= (data[22] | data[23] << 8 | data[24] << 16 | data[25] << 24);
	Imu->Yaw		= *(float *)&mid_float;
}

