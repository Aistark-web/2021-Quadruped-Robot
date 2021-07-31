#include "my_motor.h"

MY_RM3508_Handle Motor_RM3508[8];

MY_RM3508_Handle Motor_RM3508_Front_Right_Outside_Test;
MY_RM3508_Handle Motor_RM3508_Front_Right_Inside_Test;


CAN_RxHeaderTypeDef CAN_RX;
CAN_TxHeaderTypeDef CAN1_TX;
CAN_TxHeaderTypeDef CAN2_TX;

int32_t Watch_Angle[8];
int32_t Watch_Infer_Angle[8];
void RM3508_Receive_Get_More(MY_RM3508_Handle *Motor_RM3508, uint8_t *Data,uint8_t *flag)
{
	if(!(*flag))																									//电机初始位置校准
	{
		Motor_RM3508->InitAngle = (uint16_t)(Data[0] << 8 | Data[1]);
		int16_t diff = Motor_RM3508->InitAngle;

		if (diff > 4000)
				Motor_RM3508->Init_r--;
		if (diff < -4000)
				Motor_RM3508->Init_r++;
		*flag = 1;
		return;
	}
	
	RM3508_Receive(&Motor_RM3508->Senior_RM3508_Handle,Data);
	Watch_Infer_Angle[Motor_RM3508->ESC_ID-1] = Motor_RM3508->Infer_Angle;
	Motor_RM3508->get_flag = 1;
	Motor_RM3508->Senior_RM3508_Handle.Angle 	-=  (Motor_RM3508->InitAngle + Motor_RM3508->Init_r * 8192);
	Watch_Angle[Motor_RM3508->ESC_ID-1] = Motor_RM3508->Senior_RM3508_Handle.Angle;
	Motor_RM3508->Senior_RM3508_Handle.Angle_DEG = ((float)Motor_RM3508->Senior_RM3508_Handle.Angle) * 0.0439453125f;
	Motor_RM3508->Real_Angle = Motor_RM3508->Senior_RM3508_Handle.Angle_DEG * EXCHANGE_ANGLE_DEG_TO_INFER_REAL_ANGLE;
}
