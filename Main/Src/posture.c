#include "posture.h"

static uint8_t i;
static uint8_t j;
Posture_Handle Posture;
IMU_Typedef IMU;
Ramp_Typedef Posture_Steady_State_Ramp[4];
Ramp_Typedef Posture_Steady_State_Buffer_Ramp;
/* BEGIN EV */

/* END EV */

void Posture_Handle_Init(Posture_Handle *Posture)
{
	Posture->imu = &IMU;
}

void Posture_Ramp_Init()
{
	for(i=0;i<4;i++)
	{
		Posture_Steady_State_Ramp[i].RampTime = 100;
	}
	Posture_Steady_State_Buffer_Ramp.RampTime = 50;
}

/**
	*@brief 单条腿变换姿态
	*/
void Posture_One_Foot_Change(CSYS_Global_Handle *CSYS,Posture_Handle *Posture,float Roll,float Pitch,float Yaw)
{
	Posture->Infer_Pitch = Pitch;
	Posture->Infer_Roll  = Roll;
	Posture->Infer_Yaw	 = Yaw;
	static float R[3][3];
	static float Mid[3][3];
	static float O_A[3][1];
	static float AB[3][1];
	float O_A_body[3][1]= {{CSYS->CSYS_Global_Motor->x-CSYS->CSYS_Global_Body_Center->x},
												{CSYS->CSYS_Global_Motor->y-CSYS->CSYS_Global_Body_Center->y},
												{CSYS->CSYS_Global_Motor->z-CSYS->CSYS_Global_Body_Center->z}
												};
	/* OO' */
	float OO_[3][1]		 = {{CSYS->CSYS_Global_Body_Center->x},
												{CSYS->CSYS_Global_Body_Center->y},
												{CSYS->CSYS_Global_Body_Center->z}};
	float OB[3][1]		 = {{CSYS->CSYS_Global_Foot->x},
												{CSYS->CSYS_Global_Foot->y},
												{CSYS->CSYS_Global_Foot->z}};
	float rotx_R[3][3] = {{1,                         0											 ,                                    					 0},
												{0, cosf(Posture->Infer_Roll/EXCHANGE_RAD_TO_ANGLE),-cosf(Posture->Infer_Roll/EXCHANGE_RAD_TO_ANGLE)},
												{0, sinf(Posture->Infer_Roll/EXCHANGE_RAD_TO_ANGLE), cosf(Posture->Infer_Roll/EXCHANGE_RAD_TO_ANGLE)}
											 };
	float rotx_P[3][3] = {{cosf(Posture->Infer_Pitch/EXCHANGE_RAD_TO_ANGLE), 0 ,sinf(Posture->Infer_Pitch/EXCHANGE_RAD_TO_ANGLE)},
												{0																		,	 1 ,								0											},
												{-sinf(Posture->Infer_Pitch/EXCHANGE_RAD_TO_ANGLE),0 , cosf(Posture->Infer_Pitch/EXCHANGE_RAD_TO_ANGLE)}
											 };
	float rotx_Y[3][3] = {{cosf(Posture->Infer_Yaw/EXCHANGE_RAD_TO_ANGLE), -sinf(Posture->Infer_Yaw/EXCHANGE_RAD_TO_ANGLE), 0	},
												{sinf(Posture->Infer_Yaw/EXCHANGE_RAD_TO_ANGLE), cosf(Posture->Infer_Yaw/EXCHANGE_RAD_TO_ANGLE), 0	},
												{									0									, 0										, 									1	}};
	
	/* 矩阵相乘运算 R=rotx_R*rotx_P*rotx_Y */
	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			Mid[i][j] = rotx_R[i][0]*rotx_P[0][j] + rotx_R[i][1]*rotx_P[1][j] + rotx_R[i][2]*rotx_P[2][j];
		}
	}
	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			R[i][j] = Mid[i][0]*rotx_Y[0][j]+Mid[i][1]*rotx_Y[1][j]+Mid[i][2]*rotx_Y[2][j];
		}
	}
	/*	O'A=R*O'A_body */
	for(i=0;i<3;i++)
		O_A[i][0] = R[i][0]*O_A_body[0][0] + R[i][1]*O_A_body[1][0] + R[i][2]*O_A_body[2][0];
	/* AB = -OO' + O'A + OB */
	for(i=0;i<3;i++)
		AB[i][0] = -OO_[i][0] - O_A[i][0] + OB[i][0];
	CSYS->CSYS_Global_Vector_Motor_To_Foot->Vector_x = AB[0][0];
	CSYS->CSYS_Global_Vector_Motor_To_Foot->Vector_y = AB[1][0];
	CSYS->CSYS_Global_Vector_Motor_To_Foot->Vector_z = AB[2][0];
}

/**
	*@brief 平衡稳态
	*/
void Posture_Steady_State()
{
	static float Roll;
	static float Pitch;
	static float Yaw;
	for(i=0;i<4;i++)
	{
		Posture_One_Foot_Change(&CSYS_Global[i],&Posture,Roll,Pitch,Yaw);
		CSYS_Global_Vector_Analysis(&CSYS_Global[i]);
		Foot_Point_RTO_Ramp(CSYS_Foot[i].Infer_x,CSYS_Foot[i].Infer_y,&CSYS_Foot[i],&Posture_Steady_State_Ramp[i]);
	}
}
                                    
/**
	*@breif 陡峭稳态
	*/
Posture_Handle Posture_Steep_Steady_State()
{
	static float Roll;
	static float Pitch;
	static float Yaw;
	static uint8_t first_flag;
	static uint8_t buffer_flag;
	if(!buffer_flag)				
	{
		if(!first_flag)
		{
			Roll 	+= IMU.EulerAngler.Roll;
			Pitch += IMU.EulerAngler.Pitch;
			Posture_One_Foot_Change(&CSYS_Global[i],&Posture,-Roll,-Pitch,Yaw);
			CSYS_Global_Vector_Analysis(&CSYS_Global[i]);
			first_flag = 1;
		}
		for(i=0;i<4;i++)
		{
			Foot_Point_RTO_Ramp(CSYS_Global[i].CSYS_Infer_x,CSYS_Global[i].CSYS_Infer_y,&CSYS_Foot[i],&Posture_Steady_State_Ramp[i]);
			Foot_Angle_Control(&CSYS_Foot[i]);
		}
		if(Slope(&Posture_Steady_State_Ramp[0]) == 1.0f && Slope(&Posture_Steady_State_Ramp[1]) == 1.0f &&
			 Slope(&Posture_Steady_State_Ramp[2]) == 1.0f && Slope(&Posture_Steady_State_Ramp[3]) == 1.0f)
		{
			for(i=0;i<4;i++)
			{
				Foot_Point_RTO_Ramp(CSYS_Global[i].CSYS_Infer_x,CSYS_Global[i].CSYS_Infer_y,&CSYS_Foot[i],&Posture_Steady_State_Ramp[i]);
				Foot_Angle_Control(&CSYS_Foot[i]);
				ResetSlope(&Posture_Steady_State_Ramp[i]);
			}
			buffer_flag = 1;
		}         
	}
	else                              //提供缓冲时间给电机
	{
		if(Slope(&Posture_Steady_State_Buffer_Ramp) == 1.0f)
		{
			ResetSlope(&Posture_Steady_State_Buffer_Ramp);
			buffer_flag = 0;
		}
	}
	return Posture;
}

/**
	*@biref 行走位姿
	*@param[in] CSYS_Global
	*@param[in] CSYS
	*@param[in] imu
	*@return		Posture
	*/
Posture_Handle Foot_Walk_Posture(CSYS_Global_Handle *CSYS_Global,CSYS_Handle *CSYS,IMU_Typedef *imu,uint8_t *flag)
{
	static float initial_Roll;
	static float initial_Yaw;
	static float initial_Pitch;
	static uint8_t first_flag;
	if(!(*flag))
	{
		initial_Roll 	= imu->EulerAngler.Roll;
		initial_Yaw	 	= imu->EulerAngler.Yaw;
		initial_Pitch =	imu->EulerAngler.Pitch;
		*flag = 1;
	}
	CSYS->Infer_theta1 = CSYS->Infer_theta1 + initial_Pitch;
	CSYS->Infer_theta2 = CSYS->Infer_theta2 - initial_Pitch;
	return Posture;
}

float Foot_Orient_Calibration_Direction(IMU_Typedef *imu,uint8_t *flag)
{
	static float initial_Roll;
	static float initial_Pitch;
	static float initial_Yaw;
	static float Rotate_Yaw;
	if(!(*flag))
	{
		initial_Yaw = imu->EulerAngler.Yaw;
	}
	Posture.Rotate_Yaw = imu->EulerAngler.Yaw - initial_Yaw;
	return Posture.Rotate_Yaw;
}

Posture_Handle Foot_Calibration_Direction(CSYS_Handle *CSYS,CSYS_Global_Handle *CSYS_Global,IMU_Typedef *imu,uint8_t *flag)
{
	static float initial_Roll;
	static float initial_Pitch;
	static float initial_Yaw;
	
	if(!(*flag))
	{
		initial_Yaw  	= imu->EulerAngler.Yaw;
		initial_Roll 	= imu->EulerAngler.Roll;
		initial_Pitch	=	imu->EulerAngler.Pitch;
		*flag = 1;
	}
	return Posture;
}

void ResetFlag(uint8_t *flag)
{
	*flag = 0;
}
