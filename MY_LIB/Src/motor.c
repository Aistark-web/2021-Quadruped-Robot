#include "motor.h"


/**
	* @brief 初始化RM3508结构体
	*/

#if 1
RM3508_TypeDef Motor_RM3508[8];
#else
RM3508_TypeDef Motor_RM3508_Left_Front_Leg_Outside 	= {1};		
RM3508_TypeDef Motor_RM3508_Left_Front_Leg_Inside		= {2};

RM3508_TypeDef Motor_RM3508_Right_Front_Leg_Outside	=	{3};		
RM3508_TypeDef Motor_RM3508_Right_Front_Leg_Inside	=	{4};		

RM3508_TypeDef Motor_RM3508_Left_Hind_Leg_Outside		=	{5};		
RM3508_TypeDef Motor_RM3508_Left_Hind_Leg_Inside		=	{6};

RM3508_TypeDef Motor_RM3508_Right_Hind_Leg_Outside	=	{7};		
RM3508_TypeDef Motor_RM3508_Right_Hind_Leg_Inside		=	{8};		
#endif
/**
	* @brief 声明CAN结构体
	*/
CAN_RxHeaderTypeDef CAN_RX;
CAN_TxHeaderTypeDef CAN1_TX;
CAN_TxHeaderTypeDef CAN2_TX;

/**
 * @brief 用转矩电流计算得到功率值
 * @param[in] speed 电机速度
 * @param[in] current 转矩电流
 * @param[in] pcof 参数
 * @return 电机功率
 */


static inline float GetChassisMotorPower(int speed, int current, struct PowerCOF_s *pcof) {
    return (pcof->ss * speed * speed +
            pcof->sc * speed * current +
            pcof->cc * current * current +
            pcof->constant);
}

void RM3508_Receive(RM3508_TypeDef *Dst, uint8_t *Data,uint8_t *flag) {
		if(!(*flag))																									//电机初始位置校准
		{
			Dst->InitAngle = (uint16_t)(Data[0] << 8 | Data[1]);
			int16_t diff = Dst->InitAngle;

			if (diff > 4000)
					Dst->Init_r--;
			if (diff < -4000)
					Dst->Init_r++;
			*flag = 1;
			return;
		}
			Dst->MchanicalAngle = (uint16_t)(Data[0] << 8 | Data[1]);
			Dst->Speed = (int16_t)(Data[2] << 8 | Data[3]);
			Dst->TorqueCurrent = (uint16_t)(Data[4] << 8 | Data[5]);
			Dst->temp = Data[6];

			int16_t diff = Dst->MchanicalAngle - Dst->LsatAngle;

			if (diff > 4000)
					Dst->r--;
			if (diff < -4000)
					Dst->r++;
			
			Dst->Angle = Dst->r * 8192 + Dst->MchanicalAngle - Dst->InitAngle - Dst->Init_r * 8192;
			Dst->Angle_DEG = ((float)Dst->Angle) * 0.0439453125f;
			Dst->Real_Angle = Dst->Angle_DEG*EXCHANGE_ANGLE_DEG_TO_INFER_REAL_ANGLE;
			Dst->Power = GetChassisMotorPower(Dst->Speed, Dst->TorqueCurrent, &Dst->PowerCOF);
			Dst->LsatAngle = Dst->MchanicalAngle;
}
