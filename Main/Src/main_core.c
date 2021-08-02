#include "main_core.h"


uint8_t Return_Data[8][8];		//!<@brief 八个电调传回数据
uint8_t Input_Data[2][8];			//!<@brief CAN1、CAN2发送给电调的数据
uint8_t Imu_Data[30];
uint8_t Remote_Data[21];			//!<@breif 经过处理后遥控器数据

/* BEGIN EV */

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
extern Remote_State_Handle Remote_State;			//遥控器控制四足状态位
extern uint8_t Remote_Receive_Data[42];						//串口接收接收机数据
extern UART_DataPack DataPack;
extern uint8_t Motor_Init_State;
extern uint8_t test_flag;
extern kalman_filter_t Imu_Pitch_kalman_filter;
extern kalman_filter_t Imu_Roll_kalman_filter;
extern kalman_filter_t Imu_Yaw_kalman_filter;
extern IMU_Typedef IMU;
extern CAN_RxHeaderTypeDef CAN_RX;
extern CAN_TxHeaderTypeDef CAN1_TX;
extern CAN_TxHeaderTypeDef CAN2_TX;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;

/* END EV */


/* 使用CANdrive.h中的CanFilter_Init函数 无法开启CAN2滤波器时，可用本函数替代 */
void CanFilter_Init_Replace(CAN_HandleTypeDef *hcan) {
    CAN_FilterTypeDef canfilter;
    canfilter.FilterBank = 0;
    canfilter.FilterMode = CAN_FILTERMODE_IDMASK;					//过滤器模式为掩码模式
    canfilter.FilterScale = CAN_FILTERSCALE_32BIT;	 			

    canfilter.FilterIdHigh = 0x0000;
    canfilter.FilterIdLow = 0x0000;
    canfilter.FilterMaskIdHigh = 0x0000;
    canfilter.FilterMaskIdLow = 0x0000;							
    /* CAN_FilterFIFO0 */
    canfilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;			//用FIFO0
		/* 激活过滤器 */
    canfilter.FilterActivation = ENABLE;												
    HAL_CAN_ConfigFilter(hcan, &canfilter);
}


/**
	*@brief CAN发送结构体初始化
	*/
void CAN_TX_Init()
{
	CAN1_TX.StdId = 0x200;
	CAN1_TX.IDE		= CAN_ID_STD;
	CAN1_TX.RTR		= CAN_RTR_DATA;
	CAN1_TX.DLC		= 8;
	CAN1_TX.TransmitGlobalTime	=	DISABLE;
	CAN2_TX.StdId	= 0x1ff;
	CAN2_TX.IDE		=	CAN_ID_STD;
	CAN2_TX.RTR		= CAN_RTR_DATA;
	CAN2_TX.DLC		=	8;
	CAN2_TX.TransmitGlobalTime	=	DISABLE;
}

/**
	*@brief CAN_RX_FIFO0接收报文回调 
	*/
uint8_t flag[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
		static uint8_t i;
		static uint8_t mid_data[8];
		static uint8_t ID;
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&CAN_RX,mid_data);
		ID = CAN_RX.StdId & 0xf;
		for(i=0;i<8;i++)
		{
			Return_Data[ID-1][i] = mid_data[i];
		}
		
		
//		if(ID == 3)
//			RM3508_Receive_Get_More(&Motor_RM3508_Front_Right_Outside_Test,Return_Data[2],&flag[2]);
//		else if(ID == 4)
//			RM3508_Receive_Get_More(&Motor_RM3508_Front_Right_Inside_Test,Return_Data[3],&flag[3]);
		if(Motor_Init_State == Motor_Init_Finish)
		{
			RM3508_Receive_Get_More(&Motor_RM3508[ID-1],Return_Data[ID-1],&flag[ID-1]);
		}
}

/**
	*@brief CAN_RX_FIFO1接收报文回调 
	*/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	static uint8_t i;
	static uint8_t mid_data[8];
	static uint8_t ID;
	HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO1,&CAN_RX,mid_data);
	ID = CAN_RX.StdId & 0xf;
	
	for(i=0;i<8;i++)
	{
		Return_Data[ID-1][i] = mid_data[i];
	}
	if(Motor_Init_State == Motor_Init_Finish)
	{
		RM3508_Receive_Get_More(&Motor_RM3508[ID-1],Return_Data[ID-1],&flag[ID-1]);
	}
}

/**
	*@brief 定时器更新事件回调
	*/
uint32_t i;
uint32_t j;
uint32_t k;
uint32_t save_k;
uint32_t first_save_k;
uint8_t first_flag;
float test_data;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	
	if(htim->Instance == TIM4)
	{
//		static uint16_t mid;
//		if(mid < 500)
//			mid++;
//		else
//			if(!test_flag)
//				Foot_Test_Ramp();
		if(Motor_Init_State == Motor_Init_Finish)
		{
			
			for(uint8_t j=0;j<4;j++)
				Foot_Get_CSYS(&CSYS_Foot[j]);
			Pose_Master();
			
			
		}
		static uint32_t Reset_time;
		if(DataPack.Key.Left_Rocker == 1 && DataPack.Key.Right_Rocker == 1)
		{
			
			if(++Reset_time>200)
			{
				Software_Reset();
			}
		}
		else
			Reset_time = 0;
	}
	
	else if(htim->Instance == TIM5)
	{
//		static uint32_t mid;
		if(Remote_State.Remote_Current_State == Remote_Foot_Init_Start && Remote_State.Remote_Last_State == Remote_Foot_Init_None)
		{
			Motor_Auto_Init();
		}
		else
		{
			PID_Standard_Place_Speed_Control();
		}
		
//		if(mid < 2000)
//			mid++;
//		else		//Motor_Check()
//		{
			
//			if(Motor_Init_State == Motor_Init_Nnoe)
//			{
//				Motor_Auto_Init();
//			}
//			else
//			{
//				PID_Standard_Place_Speed_Control();
//			PID_Standard_Control_Place_Speed_Test();
//			}
//			if(Motor_RM3508[4].Real_Angle < (Motor_RM3508[4].Infer_Real_Angle + 0.05f) &&
//				 Motor_RM3508[4].Real_Angle > (Motor_RM3508[4].Infer_Real_Angle - 0.05f))
//			{
//				save_k = k;
//				
//				if(j++ > 100)
//					first_flag = 1;
//			}
//			else
//			{
//				
//				if(first_flag)
//				{
//					j=0;
//					k=0;
//					first_flag = 0;
//				}
//				k++;
//			}
			
//		}
	}
}


#if 0
void UART_IT()
{
	if(__HAL_UART_GET_IT_SOURCE(&huart2,UART_IT_IDLE)&&__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE))
	{
		Remote_Receive();
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);
		/* 关闭DMA接收，置位 */
		HAL_DMA_Abort(huart2.hdmarx);
		huart2.RxState = HAL_UART_STATE_READY;
		
		/* 开启DMA接收，等待数据接收 */
		HAL_UART_Receive_DMA(&huart2,&Remote_Receive_Data[21],21);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,(DMA_IT_TC | DMA_IT_TE | DMA_IT_DME));
		__HAL_UART_DISABLE_IT(&huart2,(UART_IT_PE | UART_IT_ERR));
		
	}
}

#endif

//uint32_t uart_t;
/**
	*@brief UART完全接收回调(适用于UART_DMA_RX传输完全中断)
	*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
//		uart_t++;
		Remote_Receive();
		HAL_UART_Receive_DMA(&huart2,&Remote_Receive_Data[21],21);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
		__HAL_UART_DISABLE_IT(&huart2,(UART_IT_PE | UART_IT_ERR));
	}
}

/**
	*@brief UART中断
	*/
void UART_IT()
{
	if(__HAL_UART_GET_IT_SOURCE(&huart5,UART_IT_IDLE)&&__HAL_UART_GET_FLAG(&huart5,UART_FLAG_IDLE))
	{
		static uint8_t first_flag;
		__HAL_UART_CLEAR_IDLEFLAG(&huart5);
		HAL_DMA_Abort(huart5.hdmarx);
		huart5.RxState = HAL_UART_STATE_READY;
		IMU_Receive(&IMU,Imu_Data);
		if(!first_flag)											//获取第一次IMU数据
		{
			first_flag = 1;
			Imu_Pitch_kalman_filter.X_last = IMU.EulerAngler.Pitch;
			Imu_Roll_kalman_filter.X_last = IMU.EulerAngler.Roll;
			Imu_Yaw_kalman_filter.X_last = IMU.EulerAngler.Yaw;
		}
		else
		{
			IMU.EulerAngler.Pitch = Kalman_Filter(&Imu_Pitch_kalman_filter,IMU.EulerAngler.Pitch);
			IMU.EulerAngler.Roll 	= Kalman_Filter(&Imu_Roll_kalman_filter,IMU.EulerAngler.Roll);
			IMU.EulerAngler.Yaw 	= Kalman_Filter(&Imu_Yaw_kalman_filter,IMU.EulerAngler.Yaw);
		}
		
		HAL_UART_Receive_DMA(&huart5,Imu_Data,30);
		__HAL_DMA_DISABLE_IT(&hdma_uart5_rx,DMA_IT_TC | DMA_IT_TE | DMA_IT_DME);
		__HAL_UART_DISABLE_IT(&huart5,(UART_IT_PE | UART_IT_ERR));
	}
}

/**
	*@brief 复位操作
	*/
void Software_Reset()
{
	__disable_irq();
	__disable_fiq();
	NVIC_SystemReset();
}
