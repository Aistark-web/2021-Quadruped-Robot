#include "pid_control.h"

Ramp_Typedef Ramp_Time = {0,0,5000};
extern uint8_t Return_Data[8][8];		//!<@brief 八个电调传回数据
extern uint8_t Input_Data[2][8];			//!<@brief CAN1、CAN2发送给电调的数据
extern uint8_t Imu_Data[30];

#if 0
extern RM3508_TypeDef Motor_RM3508_Left_Front_Leg_Outside;
extern RM3508_TypeDef Motor_RM3508_Left_Front_Leg_Inside;		
extern RM3508_TypeDef Motor_RM3508_Right_Front_Leg_Outside;	
extern RM3508_TypeDef Motor_RM3508_Right_Front_Leg_Inside;		
extern RM3508_TypeDef Motor_RM3508_Left_Hind_Leg_Outside;		
extern RM3508_TypeDef Motor_RM3508_Left_Hind_Leg_Inside;		
extern RM3508_TypeDef Motor_RM3508_Right_Hind_Leg_Outside;	
extern RM3508_TypeDef Motor_RM3508_Right_Hind_Leg_Inside;		
#endif

extern CAN_RxHeaderTypeDef CAN_RX;
extern CAN_TxHeaderTypeDef CAN1_TX;
extern CAN_TxHeaderTypeDef CAN2_TX;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern Remote_State_enum_Handle Remote_State_enum;
extern Remote_State_Handle Remote_State;

uint32_t time;
uint8_t Motor_Init_State;
void Motor_Auto_Init()
{
	static uint32_t Txmailbox0 = 0;
	static uint32_t Txmailbox1 = 1;
	static uint8_t data[2][8];
	static int16_t pid_out;
	static int16_t pid_out2 = 500;
	static uint8_t i,j;
	time++;
	if(pid_out < 1500)
			pid_out++;
	if(time <=3000)
	{
		for(i=0;i<2;i++)
		{
			for(j=0;j<8;j+=2)
			{
				data[i][j] 	 = (j>=4) ? pid_out >> 8 	 : (-pid_out) >> 8;
				data[i][j+1] = (j>=4) ? pid_out & 0xff : (-pid_out) & 0xff;
			}
		}
	}
	if(time > 3000)
	{
		data[0][0] = -pid_out >> 8	;
		data[0][1] = -pid_out & 0xff;
		data[0][2] = -pid_out2	>> 8	;
		data[0][3] = -pid_out2 & 0xff;
		data[0][4] = pid_out	>> 8	;
		data[0][5] = pid_out	& 0xff;
		data[0][6] = pid_out2 >> 8	;
		data[0][7] = pid_out2	& 0xff;
		
		data[1][0] = -pid_out2 >> 8	;
		data[1][1] = -pid_out2 & 0xff;
		data[1][2] = -pid_out	>> 8	;
		data[1][3] = -pid_out 	& 0xff;
		data[1][4] = pid_out2	>> 8	;
		data[1][5] = pid_out2	& 0xff;
		data[1][6] = pid_out >> 8	;
		data[1][7] = pid_out	& 0xff;
	}
//	if(time > 3000)
//	{
//		data[0][0] = -pid_out >> 8	;
//		data[0][1] = -pid_out & 0xff;
//		data[0][2] = pid_out	>> 8	;
//		data[0][3] = pid_out 	& 0xff;
//		data[0][4] = pid_out	>> 8	;
//		data[0][5] = pid_out	& 0xff;
//		data[0][6] = -pid_out >> 8	;
//		data[0][7] = -pid_out	& 0xff;
//		
//		data[1][0] = pid_out >> 8	;
//		data[1][1] = pid_out & 0xff;
//		data[1][2] = -pid_out	>> 8	;
//		data[1][3] = -pid_out 	& 0xff;
//		data[1][4] = -pid_out	>> 8	;
//		data[1][5] = -pid_out	& 0xff;
//		data[1][6] = pid_out >> 8	;
//		data[1][7] = pid_out	& 0xff;
//	}
	HAL_CAN_AddTxMessage(&hcan1,&CAN1_TX,data[0],&Txmailbox0);
	HAL_CAN_AddTxMessage(&hcan2,&CAN2_TX,data[1],&Txmailbox1);
	if(time > 9000)
	{
		Motor_Init_State = Motor_Init_Finish;
		Remote_State_Last_Current_Update(Remote_Foot_Init_Set);
	}
}
/**
	*@brief PID位置环控制
	*/
void PID_Standard_Place_Control()
{
	static uint32_t Txmailbox = 0;
	
#if 1
	for(int i=0;i<8;i++)
	{
		PID_Control_Place_Limit(Motor_RM3508[i].Senior_RM3508_Handle.Angle,
											Motor_RM3508[i].Infer_Real_Angle*EXCHANGE_REAL_ANGLE_TO_ANGLE,
											&PID_Place_RM3508[i],RM3508_LIMIT);
	}
	
	for(int i=0;i<2;i++)
	{
		for(int j=0;j<8;j+=2)
		{
			Input_Data[i][j] = ((int16_t)PID_Place_RM3508[j/2+4*i].pid_out) >> 8;
			Input_Data[i][j+1] = ((int16_t)PID_Place_RM3508[j/2+4*i].pid_out) & 0xff;
		}
	}
	
	HAL_CAN_AddTxMessage(&hcan1,&CAN1_TX,Input_Data[0],&Txmailbox);
	HAL_CAN_AddTxMessage(&hcan2,&CAN2_TX,Input_Data[1],&Txmailbox);
	
#else
	/* 左前腿外侧 */
	PID_Control_Place(Motor_RM3508_Left_Front_Leg_Outside.Angle,			
										Motor_RM3508_Left_Front_Leg_Outside.Infer_Real_Angle*EXCHANGE_REAL_ANGLE_TO_ANGLE,
										&PID_Place_RM3508_Left_Front_Leg_Outside);
	
	/* 左前腿内侧 */
	PID_Control_Place(Motor_RM3508_Left_Front_Leg_Inside.Angle,
										Motor_RM3508_Left_Front_Leg_Inside.Infer_Real_Angle*EXCHANGE_REAL_ANGLE_TO_ANGLE,
										&PID_Place_RM3508_Left_Front_Leg_Inside);
	
	/* 右前腿外侧 */
	PID_Control_Place(Motor_RM3508_Right_Front_Leg_Outside.Angle,
										Motor_RM3508_Right_Front_Leg_Outside.Infer_Real_Angle*EXCHANGE_REAL_ANGLE_TO_ANGLE,
										&PID_Place_RM3508_Right_Front_Leg_Outside);
	
	/* 右前腿内侧 */
	PID_Control_Place(Motor_RM3508_Right_Front_Leg_Inside.Angle,
										Motor_RM3508_Right_Front_Leg_Inside.Infer_Real_Angle*EXCHANGE_REAL_ANGLE_TO_ANGLE,
										&PID_Place_RM3508_Right_Front_Leg_Inside);
										
	/* 左后腿外侧 */
	PID_Control_Place(Motor_RM3508_Left_Hind_Leg_Outside.Angle,
										Motor_RM3508_Left_Hind_Leg_Outside.Infer_Real_Angle*EXCHANGE_REAL_ANGLE_TO_ANGLE,
										&PID_Place_RM3508_Left_Hind_Leg_Outside);
										
	/* 左后腿内侧 */
	PID_Control_Place(Motor_RM3508_Left_Hind_Leg_Inside.Angle,
										Motor_RM3508_Left_Hind_Leg_Inside.Infer_Real_Angle*EXCHANGE_REAL_ANGLE_TO_ANGLE,
										&PID_Place_RM3508_Left_Hind_Leg_Inside);
	
	/* 右后腿外侧 */
	PID_Control_Place(Motor_RM3508_Right_Hind_Leg_Outside.Angle,
										Motor_RM3508_Right_Hind_Leg_Outside.Infer_Real_Angle*EXCHANGE_REAL_ANGLE_TO_ANGLE,
										&PID_Place_RM3508_Right_Hind_Leg_Outside);
										
	/* 右后腿内侧 */
	PID_Control_Place(Motor_RM3508_Right_Hind_Leg_Inside.Angle,
										Motor_RM3508_Right_Hind_Leg_Inside.Infer_Real_Angle*EXCHANGE_REAL_ANGLE_TO_ANGLE,
										&PID_Place_RM3508_Right_Hind_Leg_Inside);
	
	Input_Data[0][0] = ((int16_t)PID_Place_RM3508_Left_Front_Leg_Outside.PID_Out) >> 8;
	Input_Data[0][1] = ((int16_t)PID_Place_RM3508_Left_Front_Leg_Outside.PID_Out) & 0xff;
	Input_Data[0][2] = ((int16_t)PID_Place_RM3508_Left_Front_Leg_Inside.PID_Out) >> 8;
	Input_Data[0][3] = ((int16_t)PID_Place_RM3508_Left_Front_Leg_Inside.PID_Out) & 0xff;
	Input_Data[0][4] = ((int16_t)PID_Place_RM3508_Right_Front_Leg_Outside.PID_Out) >> 8;
	Input_Data[0][5] = ((int16_t)PID_Place_RM3508_Right_Front_Leg_Outside.PID_Out) & 0xff;
	Input_Data[0][6] = ((int16_t)PID_Place_RM3508_Right_Front_Leg_Inside.PID_Out) >> 8;
	Input_Data[0][7] = ((int16_t)PID_Place_RM3508_Right_Front_Leg_Inside.PID_Out) & 0xff;
	Input_Data[1][0] = ((int16_t)PID_Place_RM3508_Left_Hind_Leg_Outside.PID_Out) >> 8;
	Input_Data[1][1] = ((int16_t)PID_Place_RM3508_Left_Hind_Leg_Outside.PID_Out) & 0xff;
	Input_Data[1][2] = ((int16_t)PID_Place_RM3508_Left_Hind_Leg_Inside.PID_Out) >> 8;
	Input_Data[1][3] = ((int16_t)PID_Place_RM3508_Left_Hind_Leg_Inside.PID_Out) & 0xff;
	Input_Data[1][4] = ((int16_t)PID_Place_RM3508_Right_Hind_Leg_Outside.PID_Out) >> 8;
	Input_Data[1][5] = ((int16_t)PID_Place_RM3508_Right_Hind_Leg_Outside.PID_Out) & 0xff;
	Input_Data[1][6] = ((int16_t)PID_Place_RM3508_Right_Hind_Leg_Inside.PID_Out) >> 8;
	Input_Data[1][7] = ((int16_t)PID_Place_RM3508_Right_Hind_Leg_Inside.PID_Out) & 0xff;
	
	HAL_CAN_AddTxMessage(&hcan1,&CAN1_TX,Input_Data[0],&Txmailbox);
	HAL_CAN_AddTxMessage(&hcan2,&CAN2_TX,Input_Data[1],&Txmailbox);
#endif 
}

/* 位置速度双环PID控制 */
void PID_Standard_Place_Speed_Control()
{
	static uint32_t Txmailbox0 = 0;
	static uint32_t Txmailbox1 = 1;
#if 1
	
	for(uint8_t i=0;i<8;i++)
	{
		Motor_RM3508[i].Infer_Angle = (int32_t)(Motor_RM3508[i].Infer_Real_Angle*EXCHANGE_REAL_ANGLE_TO_ANGLE);
		PID_Control_Place_Limit((float)Motor_RM3508[i].Senior_RM3508_Handle.Angle,
											Motor_RM3508[i].Infer_Real_Angle*EXCHANGE_REAL_ANGLE_TO_ANGLE,
											&PID_Place_RM3508[i],
											RM3508_LIMIT);
		PID_Control_Speed_Limit((float)Motor_RM3508[i].Senior_RM3508_Handle.Speed,
											PID_Place_RM3508[i].pid_out,
											&PID_Speed_RM3508[i],
											RM3508_LIMIT);
	}
	
	for(uint8_t i=0;i<2;i++)
	{
		for(uint8_t j=0;j<8;j+=2)
		{
			Input_Data[i][j] = ((int16_t)PID_Speed_RM3508[j/2+4*i].pid_out) >> 8;
			Input_Data[i][j+1] = ((int16_t)PID_Speed_RM3508[j/2+4*i].pid_out) & 0xff;
		}
	}
	
	HAL_CAN_AddTxMessage(&hcan1,&CAN1_TX,Input_Data[0],&Txmailbox0);
	HAL_CAN_AddTxMessage(&hcan2,&CAN2_TX,Input_Data[1],&Txmailbox1);
	
#else
	/* 左前腿外侧 */
	PID_Control_Place(Motor_RM3508_Left_Front_Leg_Outside.Angle,			
										Motor_RM3508_Left_Front_Leg_Outside.Infer_Real_Angle*EXCHANGE_REAL_ANGLE_TO_ANGLE,
										&PID_Place_RM3508_Left_Front_Leg_Outside);
	PID_Control_Speed(Motor_RM3508_Left_Front_Leg_Outside.Speed,
										PID_Place_RM3508_Left_Front_Leg_Outside.PID_Out,
										&PID_Speed_RM3508_Left_Front_Leg_Outside);
	/* 左前腿内侧 */
	PID_Control_Place(Motor_RM3508_Left_Front_Leg_Inside.Angle,
										Motor_RM3508_Left_Front_Leg_Inside.Infer_Real_Angle*EXCHANGE_REAL_ANGLE_TO_ANGLE,
										&PID_Place_RM3508_Left_Front_Leg_Inside);
	PID_Control_Speed(Motor_RM3508_Left_Front_Leg_Inside.Speed,
										PID_Place_RM3508_Left_Front_Leg_Inside.PID_Out,
										&PID_Speed_RM3508_Left_Front_Leg_Inside);
	/* 右前腿外侧 */
	PID_Control_Place(Motor_RM3508_Right_Front_Leg_Outside.Angle,
										Motor_RM3508_Right_Front_Leg_Outside.Infer_Real_Angle*EXCHANGE_REAL_ANGLE_TO_ANGLE,
										&PID_Place_RM3508_Right_Front_Leg_Outside);
	PID_Control_Speed(Motor_RM3508_Right_Front_Leg_Outside.Speed,
										PID_Place_RM3508_Right_Front_Leg_Outside.PID_Out,
										&PID_Place_RM3508_Right_Front_Leg_Outside);
	/* 右前腿内侧 */
	PID_Control_Place(Motor_RM3508_Right_Front_Leg_Inside.Angle,
										Motor_RM3508_Right_Front_Leg_Inside.Infer_Real_Angle*EXCHANGE_REAL_ANGLE_TO_ANGLE,
										&PID_Place_RM3508_Right_Front_Leg_Inside);
										
	/* 左后腿外侧 */
	PID_Control_Place(Motor_RM3508_Left_Hind_Leg_Outside.Angle,
										Motor_RM3508_Left_Hind_Leg_Outside.Infer_Real_Angle*EXCHANGE_REAL_ANGLE_TO_ANGLE,
										&PID_Place_RM3508_Left_Hind_Leg_Outside);
										
	/* 左后腿内侧 */
	PID_Control_Place(Motor_RM3508_Left_Hind_Leg_Inside.Angle,
										Motor_RM3508_Left_Hind_Leg_Inside.Infer_Real_Angle*EXCHANGE_REAL_ANGLE_TO_ANGLE,
										&PID_Place_RM3508_Left_Hind_Leg_Inside);
	
	/* 右后腿外侧 */
	PID_Control_Place(Motor_RM3508_Right_Hind_Leg_Outside.Angle,
										Motor_RM3508_Right_Hind_Leg_Outside.Infer_Real_Angle*EXCHANGE_REAL_ANGLE_TO_ANGLE,
										&PID_Place_RM3508_Right_Hind_Leg_Outside);
										
	/* 右后腿内侧 */
	PID_Control_Place(Motor_RM3508_Right_Hind_Leg_Inside.Angle,
										Motor_RM3508_Right_Hind_Leg_Inside.Infer_Real_Angle*EXCHANGE_REAL_ANGLE_TO_ANGLE,
										&PID_Place_RM3508_Right_Hind_Leg_Inside);

#endif

}
/**
	*@brief PID控制测试 仅用于前右腿
	*/
void PID_Standard_Control_Place_Speed_Test()
{
	static uint32_t Txmailbox;
	Motor_RM3508_Front_Right_Outside_Test.Infer_Angle = (int32_t)(Motor_RM3508_Front_Right_Outside_Test.Infer_Real_Angle*EXCHANGE_REAL_ANGLE_TO_ANGLE);
	Motor_RM3508_Front_Right_Inside_Test.Infer_Angle	= (int32_t)(Motor_RM3508_Front_Right_Inside_Test.Infer_Real_Angle*EXCHANGE_REAL_ANGLE_TO_ANGLE);
	PID_Control_Place_Limit((float)Motor_RM3508_Front_Right_Outside_Test.Senior_RM3508_Handle.Angle,
											Motor_RM3508_Front_Right_Outside_Test.Infer_Real_Angle*EXCHANGE_REAL_ANGLE_TO_ANGLE,
											&PID_Place_RM3508[2],
											RM3508_LIMIT);
	PID_Control_Speed_Limit((float)Motor_RM3508_Front_Right_Outside_Test.Senior_RM3508_Handle.Speed,
											PID_Place_RM3508[2].pid_out,
											&PID_Speed_RM3508[2],
											RM3508_LIMIT);
	PID_Control_Place_Limit((float)Motor_RM3508_Front_Right_Inside_Test.Senior_RM3508_Handle.Angle,
											Motor_RM3508_Front_Right_Inside_Test.Infer_Real_Angle*EXCHANGE_REAL_ANGLE_TO_ANGLE,
											&PID_Place_RM3508[3],
											RM3508_LIMIT);
	PID_Control_Speed_Limit((float)Motor_RM3508_Front_Right_Inside_Test.Senior_RM3508_Handle.Speed,
											PID_Place_RM3508[3].pid_out,
											&PID_Speed_RM3508[3],
											RM3508_LIMIT);
	Input_Data[0][4] = ((int16_t)PID_Speed_RM3508[2].pid_out) >> 8;
	Input_Data[0][5] = ((int16_t)PID_Speed_RM3508[2].pid_out) & 0xff;
	Input_Data[0][6] = ((int16_t)PID_Speed_RM3508[3].pid_out) >> 8;
	Input_Data[0][7] = ((int16_t)PID_Speed_RM3508[3].pid_out) & 0xff;
	HAL_CAN_AddTxMessage(&hcan1,&CAN1_TX,Input_Data[0],&Txmailbox);
}
/**
	*@brief 检查电机反馈数据是否更新完成 
	*@return 1 更新完成 
	*/
_Bool Motor_Check(void)
{
	static uint8_t i;
	for(i=0;i<8 && Motor_RM3508[i].get_flag == 1;i++)
	{
		if(i==7)
		{
			for(i=0;i<8;i++)
				Motor_RM3508[i].get_flag = 0;
			return 1;
		}
	}
	return 0;
}

/**
	*@brief 输入期望角度
	*/
CSYS_Handle Foot_Angle_Control(CSYS_Handle *CSYS)
{
	switch(CSYS->ID)
	{
		case 1:
		{
			CSYS->RM3508_1->Infer_Real_Angle = 148.9f	+	(79.8f-CSYS_Foot[0].Infer_theta1);
			CSYS->RM3508_2->Infer_Real_Angle = 8.9f		+	(79.8f-CSYS_Foot[0].Infer_theta2);
			break;
		}
		case 2:
		{
			CSYS->RM3508_1->Infer_Real_Angle = -148.9f - (79.8f-CSYS_Foot[1].Infer_theta1);
			CSYS->RM3508_2->Infer_Real_Angle = -8.9f	 - (79.8f-CSYS_Foot[1].Infer_theta2);
			break;
		}
		case 3:
		{
			CSYS->RM3508_1->Infer_Real_Angle = 10.9f 	+ (79.8f-CSYS_Foot[2].Infer_theta1);
			CSYS->RM3508_2->Infer_Real_Angle = 146.9f + (79.8f-CSYS_Foot[2].Infer_theta2);
			break;
		}
		case 4:
		{
			CSYS->RM3508_1->Infer_Real_Angle = -10.9f	 - (79.8f-CSYS_Foot[3].Infer_theta1);
			CSYS->RM3508_2->Infer_Real_Angle = -146.9f - (79.8f-CSYS_Foot[3].Infer_theta2);
			break;
		}
	}
	return *CSYS;
}
