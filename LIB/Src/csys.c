#include "csys.h"
/**
	*@explain 工程内涉及腿的结构体数组逻辑为
						数组下标 		对应
						0					左前腿
						1					右前腿
						2					左后腿
						3					右后腿
	*@example
					CSYS[0] 左前腿 
					CSYS[1] 右前腿 
					CSYS[2] 左后腿 
					CSYS[3] 右后腿 
	*/

/**
	*@brief 四只腿坐标系
					CSYS[0] 左前腿 ID = 1
					CSYS[1] 右前腿 ID = 2
					CSYS[2] 左后腿 ID = 3
					CSYS[3] 右后腿 ID = 4
	*/
CSYS_Handle CSYS_Foot[4];

CSYS_Global_Handle CSYS_Global[4];
CSYS_Global_Body_Center_Handle CSYS_Global_Body_Center[4];
CSYS_Global_Foot_Handle CSYS_Global_Foot[4];
CSYS_Global_Motor_Handle CSYS_Global_Motor[4];
CSYS_Global_Vector_Motor_To_Foot_Handle CSYS_Global_Vector_Motor_To_Foot[4];

/**
	*@brief 坐标系初始化
	*/
void CSYS_Init()
{
	for(uint8_t i =0;i<4;i++)
	{
		CSYS_Foot[i].ID = i+1;
		CSYS_Global[i].ID = i+1;
		CSYS_Global[i].CSYS_Global_Body_Center = &CSYS_Global_Body_Center[i];
		CSYS_Global[i].CSYS_Global_Foot = &CSYS_Global_Foot[i];
		CSYS_Global[i].CSYS_Global_Motor = &CSYS_Global_Motor[i];
		CSYS_Global[i].CSYS_Global_Vector_Motor_To_Foot = &CSYS_Global_Vector_Motor_To_Foot[i];
		CSYS_Global[i].CSYS_Global_Body_Center->x = 0;
		CSYS_Global[i].CSYS_Global_Body_Center->y = 0;
		CSYS_Global[i].CSYS_Global_Body_Center->z = Motor_y;
		CSYS_Global[i].CSYS_Global_Foot->x = (i<2) ? -Body_Center_To_Motor_Distance_x+Motor_x 
																							 : 	Body_Center_To_Motor_Distance_x+Motor_x;
		CSYS_Global[i].CSYS_Global_Foot->y = (i%2) ? 	Body_Center_To_Motor_Distance_y 
																							 : -Body_Center_To_Motor_Distance_y;
		CSYS_Global[i].CSYS_Global_Foot->z = 0; 
		CSYS_Global[i].CSYS_Global_Motor->x = (i<2) ? -Body_Center_To_Motor_Distance_x+Motor_x 
																								:  Body_Center_To_Motor_Distance_x+Motor_x;
		CSYS_Global[i].CSYS_Global_Motor->y = (i%2) ?  Body_Center_To_Motor_Distance_y 
																								: -Body_Center_To_Motor_Distance_y;
		CSYS_Global[i].CSYS_Global_Motor->z = Motor_y + Body_Center_To_Motor_Distance_z;
		
	}
	/*	腿外侧电机对应	RM3508_1 
			腿内侧电机对应	RM3508_2	*/
	CSYS_Foot[0].RM3508_1 = &Motor_RM3508[0];
	CSYS_Foot[0].RM3508_2 = &Motor_RM3508[1];
	CSYS_Foot[1].RM3508_1 = &Motor_RM3508[2];
	CSYS_Foot[1].RM3508_2 = &Motor_RM3508[3];
	CSYS_Foot[2].RM3508_1 = &Motor_RM3508[4];
	CSYS_Foot[2].RM3508_2 = &Motor_RM3508[5];
	CSYS_Foot[3].RM3508_1 = &Motor_RM3508[6];
	CSYS_Foot[3].RM3508_2 = &Motor_RM3508[7];
	
}

/**
	*@breif 		利用一元二次方程求解当前腿长L(L>0)
	*@solve 		Ax^2+Bx+C=0
	*@param[in] A 系数A
	*@param[in] B	系数B
	*@param[in] C	系数C
	*@return 		L 腿长
	*/
inline float Solve_One_Quadratic_Equation(float A,float B,float C)
{
	return (-B + sqrtf(B*B-4.0f*A*C))/(2.0f*A);
}

/**
	*@brief 	获取腿坐标及其角度腿与两个小腿与y轴的角度
	*@CSYS 		坐标系句柄
	*@return 	坐标系句柄
	*/
CSYS_Handle Foot_Get_CSYS(CSYS_Handle *CSYS)
{
	switch(CSYS->ID)
	{
		case 1:
		{
			CSYS->theta1 = 148.9f + (79.8f-CSYS->RM3508_1->Real_Angle);
			CSYS->theta2 = 8.9f 	+ (79.8f - CSYS->RM3508_2->Real_Angle);
//			CSYS->theta1 = 180.0f + (50.0f-CSYS->RM3508_1->Real_Angle);
//			CSYS->theta2 = (50.0f - CSYS->RM3508_2->Real_Angle);
			break;
		}
		
		case 2:
		{
			CSYS->theta1 = 148.9f +	(79.8f + CSYS->RM3508_1->Real_Angle);
			CSYS->theta2 = 8.9f		+	(79.8f + CSYS->RM3508_2->Real_Angle);
//			CSYS->theta1 = 180.0f +(50.0f + CSYS->RM3508_1->Real_Angle);
//			CSYS->theta2 = 50.0f + CSYS->RM3508_2->Real_Angle;
			break;
		}
		
		case 3:
		{
			CSYS->theta1 = 10.9f	+	(79.8f - CSYS->RM3508_1->Real_Angle);
			CSYS->theta2 = 146.9f + (79.8f - CSYS->RM3508_2->Real_Angle);
//			CSYS->theta1 = 50.0f - CSYS->RM3508_1->Real_Angle;
//			CSYS->theta2 = 180.0f + (50.0f - CSYS->RM3508_2->Real_Angle);			
			break;
		}
		
		case 4:
		{
			CSYS->theta1 = 10.9f	+	(79.8f + CSYS->RM3508_1->Real_Angle);
			CSYS->theta2 = 146.9f + (79.8f + CSYS->RM3508_2->Real_Angle);
//			CSYS->theta1 = 50.0f + CSYS->RM3508_1->Real_Angle;
//			CSYS->theta2 = 180.0f + 50.0f + CSYS->RM3508_2->Real_Angle;
			break;
		}
		
	}
	CSYS->Phi = (CSYS->theta1/180.0f*PI + CSYS->theta2/180.0f*PI) / 2.0f;
	CSYS->Psi = (CSYS->theta2/180.0f*PI - CSYS->theta1/180.0f*PI) / 2.0f;
	CSYS->L = Solve_One_Quadratic_Equation(1.0f,-2.0f*L1*cosf(CSYS->Phi),L1*L1-L2*L2);
	CSYS->x = Motor_x + CSYS->L * sinf(CSYS->Psi);
	CSYS->y = Motor_y - CSYS->L * cosf(CSYS->Psi);
	return *CSYS;
}

/**
	*@brief 分析电机与足尖的位置向量，并转换成电机与足尖所构成坐标系内期望x，y
	*/
CSYS_Global_Handle CSYS_Global_Vector_Analysis(CSYS_Global_Handle *CSYS_Global)
{
	static float L;
	static float theta;
	switch(CSYS_Global->ID)
	{
		case 1:
		{
			
			CSYS_Global->CSYS_Infer_x = CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_x;
			L = sqrtf(CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_x*CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_x+
								CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_y*CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_y+
								CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_z*CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_z);
			theta = asinf(CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_x/L);
			CSYS_Global->CSYS_Infer_y = L * cosf(theta);
			break;
		}
		case 2:
		{
			CSYS_Global->CSYS_Infer_x = CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_x;
			L = sqrtf(CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_x*CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_x+
								CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_y*CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_y+
								CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_z*CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_z);
			theta = asinf(CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_x/L);
			CSYS_Global->CSYS_Infer_y = L * cosf(theta);
			break;
		}
		case 3:
		{
			CSYS_Global->CSYS_Infer_x = CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_x;
			L = sqrtf(CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_x*CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_x+
								CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_y*CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_y+
								CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_z*CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_z);
			theta = asinf(CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_x/L);
			CSYS_Global->CSYS_Infer_y = L * cosf(theta);
			break;
		}
		case 4:
		{
			CSYS_Global->CSYS_Infer_x = CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_x;
			L = sqrtf(CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_x*CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_x+
								CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_y*CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_y+
								CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_z*CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_z);
			theta = asinf(CSYS_Global->CSYS_Global_Vector_Motor_To_Foot->Vector_x/L);
			CSYS_Global->CSYS_Infer_y = L * cosf(theta);
			break;
		}
	}
	return *CSYS_Global;
}

/**
	*@brief 全局坐标系获取
	*/
CSYS_Global_Handle CSYS_Global_Get(IMU_Typedef *imu,CSYS_Global_Handle *CSYS)
{
	static float R[3][3];
	static float Mid[3][3];
	static uint8_t i;
	static uint8_t j;
	static float O_A[3][1];
	static float AB[3][1];
	float O_A_body[3][1]= {{CSYS->CSYS_Global_Motor->x-CSYS->CSYS_Global_Body_Center->x},
												 {CSYS->CSYS_Global_Motor->y-CSYS->CSYS_Global_Body_Center->y},
												 {CSYS->CSYS_Global_Motor->z-CSYS->CSYS_Global_Body_Center->z}
												};
	/* OO' */
	float OO_[3][1]		 = {{CSYS->CSYS_Global_Body_Center->x},
												{CSYS->CSYS_Global_Body_Center->y},
												{CSYS->CSYS_Global_Body_Center->z}
											 };
	float OB[3][1]		 = {{CSYS->CSYS_Global_Foot->x},
												{CSYS->CSYS_Global_Foot->y},
												{CSYS->CSYS_Global_Foot->z}
											 };
	float rotx_R[3][3] = {{1,                         0						,                                    0},
												{0, cosf(imu->EulerAngler.Roll/EXCHANGE_RAD_TO_ANGLE),-cosf(imu->EulerAngler.Roll/EXCHANGE_RAD_TO_ANGLE)},
												{0, sinf(imu->EulerAngler.Roll/EXCHANGE_RAD_TO_ANGLE), cosf(imu->EulerAngler.Roll/EXCHANGE_RAD_TO_ANGLE)}
											 };
	float rotx_P[3][3] = {{cosf(imu->EulerAngler.Pitch/EXCHANGE_RAD_TO_ANGLE), 0 ,sinf(imu->EulerAngler.Pitch/EXCHANGE_RAD_TO_ANGLE)},
												{0																		,	 1 ,								0											},
												{-sinf(imu->EulerAngler.Pitch/EXCHANGE_RAD_TO_ANGLE),0 , cosf(imu->EulerAngler.Pitch/EXCHANGE_RAD_TO_ANGLE)}
											 };
	float rotx_Y[3][3] = {{cosf(imu->EulerAngler.Yaw/EXCHANGE_RAD_TO_ANGLE), -sinf(imu->EulerAngler.Yaw/EXCHANGE_RAD_TO_ANGLE), 0	},
												{sinf(imu->EulerAngler.Yaw/EXCHANGE_RAD_TO_ANGLE), cosf(imu->EulerAngler.Yaw/EXCHANGE_RAD_TO_ANGLE), 0	},
												{									0									, 0										, 									1	}
											 };
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
	return *CSYS_Global;
}
/**
	*@brief 运动至坐标系（x,y)点
	*@param[in] x 坐标点x值
	*@param[in] y 坐标点y值
	*@param[in] CSYS 坐标系
	*@return 		CSYS 坐标系
	*/
CSYS_Handle Foot_Point_RTO(float x,float y,CSYS_Handle *CSYS)
{
	static float Motor_y_distance;					//坐标系中电机与足尖y的差值
	static float Motor_x_distance;					//坐标系中电机与足尖x的差值
	static float Psi;
	static float Phi;
	CSYS->Infer_x = x;
	CSYS->Infer_y = y;
	Motor_y_distance = Motor_y - CSYS->Infer_y;
	Motor_x_distance = Motor_x + CSYS->Infer_x;
	CSYS->Infer_L = sqrtf(Motor_x_distance*Motor_x_distance+Motor_y_distance*Motor_y_distance);
	Psi = asinf(Motor_x_distance/CSYS->Infer_L);
	Phi = acosf((L1*L1 + CSYS->Infer_L*CSYS->Infer_L - L2*L2)/(2.0f*L1*CSYS->Infer_L));
	CSYS->Infer_theta1 = (Phi-Psi)*EXCHANGE_RAD_TO_ANGLE;
	CSYS->Infer_theta2 = (Phi+Psi)*EXCHANGE_RAD_TO_ANGLE;
	return *CSYS;
}

/**
	*@brief 运动至坐标系（x,y)点
	*@param[in] x 坐标点x值
	*@param[in] y 坐标点y值
	*@param[in] Ramp 斜坡函数句柄
	*@param[in] CSYS 坐标系
	*@return 	 *CSYS 坐标系
	*/
CSYS_Handle Foot_Point_RTO_Ramp(float x,float y,CSYS_Handle *CSYS,Ramp_Typedef *Ramp)
{
	static float Motor_y_distance;					//坐标系中电机与足尖y的差值
	static float Motor_x_distance;					//坐标系中电机与足尖x的差值
	static float initial_x[4];									//足尖初始x坐标
	static float initial_y[4];									//足尖初始y坐标
	static float Psi;
	static float Phi;
	static float ramp_t;
	if(!Ramp->flag)													//获取初始坐标
	{
		initial_x[CSYS->ID-1] = CSYS->x;
		initial_y[CSYS->ID-1] = CSYS->y;
	}
	ramp_t = Slope(Ramp);
	CSYS->Infer_x = initial_x[CSYS->ID-1] + (x - initial_x[CSYS->ID-1])*ramp_t;
	CSYS->Infer_y = initial_y[CSYS->ID-1] + (y - initial_y[CSYS->ID-1])*ramp_t;
	Motor_x_distance = Motor_x + CSYS->Infer_x;
	Motor_y_distance = Motor_y - CSYS->Infer_y;
	CSYS->Infer_L = sqrtf(Motor_x_distance*Motor_x_distance+Motor_y_distance*Motor_y_distance);
	Psi = asinf(Motor_x_distance/CSYS->Infer_L);
	Phi = acosf((L1*L1 + CSYS->Infer_L*CSYS->Infer_L - L2*L2)/(2.0f*L1*CSYS->Infer_L));
	CSYS->Infer_theta1 = (Phi-Psi)*EXCHANGE_RAD_TO_ANGLE;
	CSYS->Infer_theta2 = (Phi+Psi)*EXCHANGE_RAD_TO_ANGLE;
	return *CSYS;
}

/**
	*@brief 								腿从当前坐标点运动至相对于当前坐标点的另一点（时间由Ramp决定）
	*@param[in] CSYS 				坐标系
	*@param[in] Vector_x 		另一坐标点相对于当前坐标点的x坐标值
	*@param[in] Vextor_y 		另一坐标点相对于当前坐标点的y坐标值
	*@param[in] Ramp  			斜坡函数句柄
	*@return 	 *CSYS 				坐标系
	*/

CSYS_Handle Foot_Line_NRTO(float x,float y,CSYS_Handle *CSYS)
{
	static float Motor_y_distance;					//坐标系中电机与足尖y的差值
	static float Motor_x_distance;					//坐标系中电机与足尖x的差值
	static float Phi;												
	static float Psi;
	CSYS->Infer_y = CSYS->y + y;
	CSYS->Infer_x = CSYS->x + x;
	Motor_y_distance = Motor_y - CSYS->Infer_y;
	Motor_x_distance = Motor_x + CSYS->Infer_x;
	CSYS->Infer_L = sqrtf(Motor_x_distance*Motor_x_distance + Motor_y_distance*Motor_y_distance);
	Psi = asinf(Motor_x_distance/CSYS->Infer_L);
	Phi = acosf((L1*L1 + CSYS->Infer_L*CSYS->Infer_L - L2*L2)/(2.0f*L1*CSYS->Infer_L));
	CSYS->Infer_theta1 = (Phi-Psi)*EXCHANGE_RAD_TO_ANGLE;
	CSYS->Infer_theta2 = (Phi+Psi)*EXCHANGE_RAD_TO_ANGLE;
	return *CSYS;
}

/**
	*@brief 								腿从当前坐标点运动至相对于当前坐标点的另一点（时间由Ramp决定）
	*@param[in] CSYS 				坐标系
	*@param[in] Vector_x 		另一坐标点相对于当前坐标点的x坐标值
	*@param[in] Vector_y 		另一坐标点相对于当前坐标点的y坐标值
	*@param[in] Ramp  			斜坡函数句柄
	*@return 		*CSYS 			坐标系
	*/
CSYS_Handle Foot_Line_NRTO_Ramp(float Vector_x,float Vector_y,CSYS_Handle *CSYS,Ramp_Typedef *Ramp)
{
	static float Motor_y_distance;					//坐标系中电机与足尖y的差值
	static float Motor_x_distance;					//坐标系中电机与足尖x的差值
	static float initial_x[4];									//足尖初始x坐标
	static float initial_y[4];									//足尖初始y坐标
	static float Phi;												
	static float Psi;
	static float ramp_t;
	if(!Ramp->flag)																//获取初始位置坐标
	{
		initial_y[CSYS->ID-1] = CSYS->y;
		initial_x[CSYS->ID-1] = CSYS->x;
	}
	ramp_t = Slope(Ramp);
	CSYS->Infer_y = initial_y[CSYS->ID-1] + (Vector_y*ramp_t);
	CSYS->Infer_x = initial_x[CSYS->ID-1] + (Vector_x*ramp_t);
	Motor_y_distance = Motor_y - CSYS->Infer_y;
	Motor_x_distance = Motor_x + CSYS->Infer_x;
	CSYS->Infer_L = sqrtf(Motor_x_distance*Motor_x_distance + Motor_y_distance*Motor_y_distance);
	Psi = asinf(Motor_x_distance/CSYS->Infer_L);
	Phi = acosf((L1*L1 + CSYS->Infer_L*CSYS->Infer_L - L2*L2)/(2.0f*L1*CSYS->Infer_L));
	CSYS->Infer_theta1 = (Phi-Psi)*EXCHANGE_RAD_TO_ANGLE;
	CSYS->Infer_theta2 = (Phi+Psi)*EXCHANGE_RAD_TO_ANGLE;
	return *CSYS;
}


/**
	*@brief 根据以某点为圆心正向画上半椭圆
	*@param[in] x 圆心x值
	*@param[in] y 圆心y值
	*@param[in] a 椭圆半长轴
	*@param[in] b 椭圆半短轴
	*@param[in] Ramp 	斜坡句柄
	*@return 	  *CSYS	坐标系句柄
	*/
CSYS_Handle Foot_Front_Up_Oval_RTO_Ramp(float x,float y,float a,float b,CSYS_Handle *CSYS,Ramp_Typedef *Ramp)
{
	static float Motor_x_distance;							//电机与足尖x距离
	static float Motor_y_distance;							//电机与足尖y距离
	static float O_x[4];												//椭圆圆心x值
	static float O_y[4];												//椭圆圆心y值
  static float oval_x;										
	static float oval_y;
	static float Phi;												
	static float Psi;
	static float real_theta;
	static float ramp_t;
	if(!Ramp->flag)													
	{
		O_x[CSYS->ID-1] = x;
		O_y[CSYS->ID-1] = y;
	}
	ramp_t = Slope(Ramp);
	real_theta = PI*(1.0f-sinf(PI/2*ramp_t));			//PI*(1.0f-ramp_t)
	oval_x = real_theta < (PI/2.0f) ? sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2))) : real_theta == (PI/2.0f) 
											? 0 : -sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2)));
	oval_y = real_theta != (PI/2.0f) ? oval_x * tanf(real_theta) : b;
	CSYS->Infer_x = O_x[CSYS->ID-1] + oval_x;
	CSYS->Infer_y = O_y[CSYS->ID-1] + oval_y;
	Motor_x_distance = Motor_x + CSYS->Infer_x;
	Motor_y_distance = Motor_y - CSYS->Infer_y;
	CSYS->Infer_L = sqrtf(Motor_x_distance*Motor_x_distance+Motor_y_distance*Motor_y_distance);
	Psi = asinf(Motor_x_distance/CSYS->Infer_L);
	Phi = acosf((L1*L1 + CSYS->Infer_L*CSYS->Infer_L - L2*L2)/(2*L1*CSYS->Infer_L));
	CSYS->Infer_theta1 = (Phi - Psi)*EXCHANGE_RAD_TO_ANGLE;
	CSYS->Infer_theta2 = (Phi + Psi)*EXCHANGE_RAD_TO_ANGLE;
	return *CSYS;
}

/**
	*@brief 以某点为圆心反向画上半椭圆
	*@param[in] x 圆心x值
	*@param[in] y 圆心y值
	*@param[in] a 椭圆半长轴
	*@param[in] b 椭圆半短轴
	*@param[in] Ramp 	斜坡句柄
	*@return 	  *CSYS	坐标系句柄
	*/
CSYS_Handle Foot_Back_Up_Oval_RTO_Ramp(float x,float y,float a,float b,CSYS_Handle *CSYS,Ramp_Typedef *Ramp)
{
	static float Motor_x_distance;							//电机与足尖x距离
	static float Motor_y_distance;							//电机与足尖y距离
	static float O_x[4];									//圆心x值
	static float O_y[4];									//圆心y值
  static float oval_x;										
	static float oval_y;
	static float Phi;												
	static float Psi;
	static float real_theta;
	static float ramp_t;
	if(!Ramp->flag)													//获取初始位置坐标
	{
		O_x[CSYS->ID-1] = x;
		O_y[CSYS->ID-1] = y;
	}
	ramp_t = Slope(Ramp);
	real_theta = PI*(1.0f - sinf(PI/2*ramp_t));		//PI*(1.0f-ramp_t)
	oval_x = real_theta < (PI/2.0f) ? sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2))) : real_theta == (PI/2.0f) 
											? 0 : -sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2)));
	oval_y = real_theta != (PI/2.0f) ? oval_x * tanf(real_theta) : b;
	CSYS->Infer_x = O_x[CSYS->ID-1] - oval_x;
	CSYS->Infer_y = O_y[CSYS->ID-1] + oval_y;
	Motor_x_distance = Motor_x + CSYS->Infer_x;
	Motor_y_distance = Motor_y - CSYS->Infer_y;
	CSYS->Infer_L = sqrtf(Motor_x_distance*Motor_x_distance+Motor_y_distance*Motor_y_distance);
	Psi = asinf(Motor_x_distance/CSYS->Infer_L);
	Phi = acosf((L1*L1 + CSYS->Infer_L*CSYS->Infer_L - L2*L2)/(2*L1*CSYS->Infer_L));
	CSYS->Infer_theta1 = (Phi - Psi)*EXCHANGE_RAD_TO_ANGLE;
	CSYS->Infer_theta2 = (Phi + Psi)*EXCHANGE_RAD_TO_ANGLE;
	return *CSYS;
}


CSYS_Handle Foot_Back_Down_Oval_RTO_Ramp(float x,float y,float a,float b,CSYS_Handle *CSYS,Ramp_Typedef *Ramp)
{
	static float Motor_x_distance;							//电机与足尖x距离
	static float Motor_y_distance;							//电机与足尖y距离
	static float O_x[4];									
	static float O_y[4];									
  static float oval_x;										
	static float oval_y;
	static float Phi;												
	static float Psi;
	static float real_theta;
	static float ramp_t;
	if(!Ramp->flag)													//获取初始位置坐标
	{
		O_x[CSYS->ID-1] = x;
		O_y[CSYS->ID-1] = y;
	}
	ramp_t = Slope(Ramp);
	real_theta = PI*(1.0f - ramp_t);		//PI*(1.0f-ramp_t)
	oval_x = real_theta < (PI/2.0f) ? sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2))) : real_theta == (PI/2.0f) 
											? 0 : -sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2)));
	oval_y = real_theta != (PI/2.0f) ? oval_x * tanf(real_theta) : b;
	CSYS->Infer_x = O_x[CSYS->ID-1] - oval_x;
	CSYS->Infer_y = O_y[CSYS->ID-1] - oval_y;
	Motor_x_distance = Motor_x + CSYS->Infer_x;
	Motor_y_distance = Motor_y - CSYS->Infer_y;
	CSYS->Infer_L = sqrtf(Motor_x_distance*Motor_x_distance+Motor_y_distance*Motor_y_distance);
	Psi = asinf(Motor_x_distance/CSYS->Infer_L);
	Phi = acosf((L1*L1 + CSYS->Infer_L*CSYS->Infer_L - L2*L2)/(2*L1*CSYS->Infer_L));
	CSYS->Infer_theta1 = (Phi - Psi)*EXCHANGE_RAD_TO_ANGLE;
	CSYS->Infer_theta2 = (Phi + Psi)*EXCHANGE_RAD_TO_ANGLE;
	return *CSYS;
}

/**
	*@brief 以某点为圆心画椭圆
	*@param[in] x 圆心x值
	*@param[in] y 圆心y值
	*@param[in] a 椭圆半长轴
	*@param[in] b 椭圆半短轴
	*@param[in] Distance_x 距离
	*@param[in] CSYS 坐标系句柄
	*@param[in] Ramp 斜坡句柄
	*@return CSYS
	*/
CSYS_Handle Foot_Front_Up_Over_Half_Oval_RTO_Ramp(float x,float y,float a,float b,float Distance_x,CSYS_Handle *CSYS,Ramp_Typedef *Ramp)
{
	static float O_x[4];
	static float O_y[4];
	static float oval_initial_x[4];
	static float oval_initial_y[4];
	static float oval_x;
	static float oval_y;
	static float Phi;
	static float Psi;
	static float real_theta;
	static float total_theta[4];
	static float initial_theta[4];
	static float ramp_t;
	if(!Ramp->flag)
	{
		O_x[CSYS->ID-1] = x;
		O_y[CSYS->ID-1] = y;
		oval_initial_x[CSYS->ID-1] = -(Distance_x/2.0f);
		oval_initial_y[CSYS->ID-1] = -sqrtf(powf(b,2) - powf(b*oval_initial_x[CSYS->ID-1]/a,2));
		initial_theta[CSYS->ID-1]  = atanf(tanf(oval_initial_y[CSYS->ID-1]/oval_initial_x[CSYS->ID-1]))+PI;
		total_theta[CSYS->ID-1] 	 = 2.0f*PI-(PI-2.0f*atanf(tanf(oval_initial_y[CSYS->ID-1]/oval_initial_x[CSYS->ID-1])));
	}
	ramp_t = Slope(Ramp);
	real_theta = initial_theta[CSYS->ID-1] - (total_theta[CSYS->ID-1]*ramp_t);
	oval_x = real_theta > (PI/2.0f) ? -sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2))) : real_theta == (PI/2.0f)
				 ? 0 :	sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2)));
	oval_y = real_theta != (PI/2.0f) ? oval_x * tanf(real_theta) : b;
	CSYS->Infer_x = Motor_x + O_x[CSYS->ID-1] + oval_x;
	CSYS->Infer_y = Motor_y - O_y[CSYS->ID-1] - oval_y;
	CSYS->Infer_L = sqrtf(CSYS->Infer_x*CSYS->Infer_x+CSYS->Infer_y*CSYS->Infer_y);
	Psi = asinf(CSYS->Infer_x/CSYS->Infer_L);
	Phi = acosf((L1*L1 + CSYS->Infer_L*CSYS->Infer_L - L2*L2)/(2*L1*CSYS->Infer_L));
	CSYS->Infer_theta1 = (Phi - Psi)*EXCHANGE_RAD_TO_ANGLE;
	CSYS->Infer_theta2 = (Phi + Psi)*EXCHANGE_RAD_TO_ANGLE;
	return *CSYS;
}

/**
	*@brief 当前点正向画上半椭圆
	*@param[in] a 椭圆半长轴
	*@param[in] b 椭圆半短轴
	*@param[in] Ramp 	斜坡句柄
	*@return 	  *CSYS	坐标系句柄
	*/
CSYS_Handle	Foot_Front_Up_Oval_NRTO_Ramp(float a,float b,CSYS_Handle *CSYS,Ramp_Typedef *Ramp)
{
	static float Motor_x_distance;							//电机与足尖之间x的距离
	static float Motor_y_distance;							//电机与足尖之间y的距离
	static float O_x[4];												//椭圆圆心x值
	static float O_y[4];												//椭圆圆心y值
  static float oval_x;										
	static float oval_y;
	static float Phi;												
	static float Psi;
	static float real_theta;
	static float ramp_t;
	if(!Ramp->flag)													//获取初始位置坐标
	{
		O_x[CSYS->ID-1] = CSYS->x + a;
		O_y[CSYS->ID-1] = CSYS->y;
	}
	ramp_t = Slope(Ramp);
	real_theta = PI*(1.0f-sinf(PI/2*ramp_t));		//PI*(1.0f-ramp_t)
	oval_x = real_theta < (PI/2.0f) ? sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2))) : real_theta == (PI/2.0f) 
											? 0 : -sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2)));
	oval_y = real_theta != (PI/2.0f) ? oval_x * tanf(real_theta) : b;
	CSYS->Infer_x = O_x[CSYS->ID-1] + oval_x;
	CSYS->Infer_y = O_y[CSYS->ID-1] + oval_y;
	Motor_x_distance = Motor_x + CSYS->Infer_x;
	Motor_y_distance = Motor_y - CSYS->Infer_y;
	CSYS->Infer_L = sqrtf(Motor_x_distance*Motor_x_distance+Motor_y_distance*Motor_y_distance);
	Psi = asinf(Motor_x_distance/CSYS->Infer_L);
	Phi = acosf((L1*L1 + CSYS->Infer_L*CSYS->Infer_L - L2*L2)/(2*L1*CSYS->Infer_L));
	CSYS->Infer_theta1 = (Phi - Psi)*EXCHANGE_RAD_TO_ANGLE;
	CSYS->Infer_theta2 = (Phi + Psi)*EXCHANGE_RAD_TO_ANGLE;
	return *CSYS;
}

/**
	*@brief 当前点反向画上半椭圆
	*@param[in] a 椭圆半长轴‘
	*@param[in] b 椭圆半短轴
	*@param[in] Ramp 	斜坡句柄
	*@return 	  *CSYS	坐标系句柄
	*/
CSYS_Handle Foot_Back_Up_Oval_NRTO_Ramp(float a,float b,CSYS_Handle *CSYS,Ramp_Typedef *Ramp)
{
	static float Motor_x_distance;							//电机与足尖x距离
	static float Motor_y_distance;							//电机与足尖y距离
	static float O_x[4];
	static float O_y[4];
  static float oval_x;										
	static float oval_y;
	static float Phi;												
	static float Psi;
	static float real_theta;
	static float ramp_t;
	if(!Ramp->flag)													//获取初始位置坐标
	{
		O_x[CSYS->ID-1] = CSYS->x - a;
		O_y[CSYS->ID-1] = CSYS->y;
	}
	ramp_t = Slope(Ramp);
	real_theta = PI*(1.0f - sinf(PI/2*ramp_t));		//PI*(1.0f-ramp_t)
	oval_x = real_theta < (PI/2.0f) ? sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2))) : real_theta == (PI/2.0f) 
											? 0 : -sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2)));
	oval_y = real_theta != (PI/2.0f) ? oval_x * tanf(real_theta) : b;
	CSYS->Infer_x = O_x[CSYS->ID-1] - oval_x;
	CSYS->Infer_y = O_y[CSYS->ID-1] + oval_y;
	Motor_x_distance = Motor_x + CSYS->Infer_x;
	Motor_y_distance = Motor_y - CSYS->Infer_y;
	CSYS->Infer_L = sqrtf(Motor_x_distance*Motor_x_distance+Motor_y_distance*Motor_y_distance);
	Psi = asinf(Motor_x_distance/CSYS->Infer_L);
	Phi = acosf((L1*L1 + CSYS->Infer_L*CSYS->Infer_L - L2*L2)/(2*L1*CSYS->Infer_L));
	CSYS->Infer_theta1 = (Phi - Psi)*EXCHANGE_RAD_TO_ANGLE;
	CSYS->Infer_theta2 = (Phi + Psi)*EXCHANGE_RAD_TO_ANGLE;
	return *CSYS;
}

/**
	*@brief 当前点反向画上半椭圆
	*@param[in] a 椭圆半长轴
	*@param[in] b 椭圆半短轴
	*@param[in] Ramp 	斜坡句柄
	*@return 	  *CSYS	坐标系句柄
	*/
CSYS_Handle Foot_Back_Down_Oval_NRTO_Ramp(float a,float b,CSYS_Handle *CSYS,Ramp_Typedef *Ramp)
{
	static float Motor_x_distance;							//电机与足尖之间x的距离
	static float Motor_y_distance;							//电机与足尖之间y的距离
	static float O_x[4];												//圆心x值
	static float O_y[4];												//圆心y值
  static float oval_x;										
	static float oval_y;
	static float Phi;												
	static float Psi;
	static float real_theta;
	static float ramp_t;
	if(!Ramp->flag)													//获取初始位置坐标
	{
		O_x[CSYS->ID-1] = CSYS->x - a;
		O_y[CSYS->ID-1] = CSYS->y;
	}
	ramp_t = Slope(Ramp);
	real_theta = PI*(1.0f - ramp_t);		
	oval_x = real_theta < (PI/2.0f) ? sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2))) : real_theta == (PI/2.0f) 
											? 0 : -sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2)));
	oval_y = real_theta != (PI/2.0f) ? oval_x * tanf(real_theta) : b;
	CSYS->Infer_x = O_x[CSYS->ID-1] - oval_x;
	CSYS->Infer_y = O_y[CSYS->ID-1] - oval_y;
	Motor_x_distance = Motor_x + CSYS->Infer_x;
	Motor_y_distance = Motor_y - CSYS->Infer_y;
	CSYS->Infer_L = sqrtf(Motor_x_distance*Motor_x_distance+Motor_y_distance*Motor_y_distance);
	Psi = asinf(Motor_x_distance/CSYS->Infer_L);
	Phi = acosf((L1*L1 + CSYS->Infer_L*CSYS->Infer_L - L2*L2)/(2*L1*CSYS->Infer_L));
	CSYS->Infer_theta1 = (Phi - Psi)*EXCHANGE_RAD_TO_ANGLE;
	CSYS->Infer_theta2 = (Phi + Psi)*EXCHANGE_RAD_TO_ANGLE;
	return *CSYS;
}

/**
	*@brief 以某点为圆心画椭圆
	*@param[in] x 圆心x值
	*@param[in] y 圆心y值
	*@param[in] a 椭圆半长轴
	*@param[in] b 椭圆半短轴
	*@param[in] Distance_x 距离
	*@param[in] CSYS 坐标系句柄
	*@param[in] Ramp 斜坡句柄
	*@return CSYS
	*/
CSYS_Handle Foot_Front_Up_Over_Half_Oval_NRTO_Ramp(float a,float b,float Distance_x,CSYS_Handle *CSYS,Ramp_Typedef *Ramp)
{
	static float O_x[4];
	static float O_y[4];
	static float oval_initial_x[4];
	static float oval_initial_y[4];
	static float oval_x;
	static float oval_y;
	static float Phi;
	static float Psi;
	static float real_theta;
	static float total_theta[4];
	static float initial_theta[4];
	static float ramp_t;
	if(!Ramp->flag)
	{
		oval_initial_x[CSYS->ID-1] = -(Distance_x/2.0f);
		oval_initial_y[CSYS->ID-1] = -sqrtf(powf(b,2) - powf(b*oval_initial_x[CSYS->ID-1]/a,2));
		initial_theta[CSYS->ID-1]  = atanf(tanf(oval_initial_y[CSYS->ID-1]/oval_initial_x[CSYS->ID-1]))+PI;
		O_x[CSYS->ID-1] = CSYS->x + (Distance_x/2.0f);
		O_y[CSYS->ID-1] = CSYS->y + (Distance_x/2.0f)*tanf(initial_theta[CSYS->ID-1]);
		total_theta[CSYS->ID-1] 	 = 2.0f*PI-(PI-2.0f*atanf(tanf(oval_initial_y[CSYS->ID-1]/oval_initial_x[CSYS->ID-1])));
	}
	ramp_t = Slope(Ramp);
	real_theta = initial_theta[CSYS->ID-1] - (total_theta[CSYS->ID-1]*ramp_t);
	oval_x = real_theta > (PI/2.0f) ? -sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2))) : real_theta == (PI/2.0f)
				 ? 0 :	sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2)));
	oval_y = real_theta != (PI/2.0f) ? oval_x * tanf(real_theta) : b;
	CSYS->Infer_x = Motor_x + O_x[CSYS->ID-1] + oval_x;
	CSYS->Infer_y = Motor_y - O_y[CSYS->ID-1] - oval_y;
	CSYS->Infer_L = sqrtf(CSYS->Infer_x*CSYS->Infer_x+CSYS->Infer_y*CSYS->Infer_y);
	Psi = asinf(CSYS->Infer_x/CSYS->Infer_L);
	Phi = acosf((L1*L1 + CSYS->Infer_L*CSYS->Infer_L - L2*L2)/(2*L1*CSYS->Infer_L));
	CSYS->Infer_theta1 = (Phi - Psi)*EXCHANGE_RAD_TO_ANGLE;
	CSYS->Infer_theta2 = (Phi + Psi)*EXCHANGE_RAD_TO_ANGLE;
	return *CSYS;
}

/**
	*@brief 以(x,y)点为圆心,正向画椭圆，终点坐标可控
	*/
CSYS_Handle Foot_Front_Up_Oval_Angle_RTO_Ramp(float x,float y,float a,float b,float Angle,CSYS_Handle *CSYS,Ramp_Typedef *Ramp)
{
	static float O_x[4];									//圆心x值
	static float O_y[4];									//圆心y值
  static float oval_x;										
	static float oval_y;
	static float Phi;												
	static float Psi;
	static float real_theta;
	static float ramp_t;
	if(!Ramp->flag)													//获取初始位置坐标
	{
		O_x[CSYS->ID-1] = x;
		O_y[CSYS->ID-1] = y;
	}
	ramp_t = Slope(Ramp);
	real_theta = (Angle/EXCHANGE_RAD_TO_ANGLE)*(1.0f-ramp_t);
	oval_x = real_theta < (PI/2.0f) ? sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2))) : real_theta == (PI/2.0f) 
											? 0 : -sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2)));
	oval_y = real_theta != (PI/2.0f) ? oval_x * tanf(real_theta) : b;
	CSYS->Infer_x = Motor_x + O_x[CSYS->ID-1] + oval_x;
	CSYS->Infer_y = Motor_y - O_y[CSYS->ID-1] - oval_y;
	CSYS->Infer_L = sqrtf(CSYS->Infer_x*CSYS->Infer_x+CSYS->Infer_y*CSYS->Infer_y);
	Psi = asinf(CSYS->Infer_x/CSYS->Infer_L);
	Phi = acosf((L1*L1 + CSYS->Infer_L*CSYS->Infer_L - L2*L2)/(2*L1*CSYS->Infer_L));
	CSYS->Infer_theta1 = (Phi - Psi)*EXCHANGE_RAD_TO_ANGLE;
	CSYS->Infer_theta2 = (Phi + Psi)*EXCHANGE_RAD_TO_ANGLE;
	return *CSYS;
}

/**
	*@brief 以(x,y)点为圆心,反向画椭圆，终点坐标可控
	*/
CSYS_Handle Foot_Back_Up_Oval_Angle_RTO_Ramp(float x,float y,float a,float b,float Angle,CSYS_Handle *CSYS,Ramp_Typedef *Ramp)
{
	static float O_x[4];									//圆心x值
	static float O_y[4];									//圆心y值
  static float oval_x;										
	static float oval_y;
	static float Phi;												
	static float Psi;
	static float real_theta;
	static float ramp_t;
	if(!Ramp->flag)										
	{
		O_x[CSYS->ID-1] = x;
		O_y[CSYS->ID-1] = y;
	}
	ramp_t = Slope(Ramp);
	real_theta = (Angle/EXCHANGE_RAD_TO_ANGLE)*(1.0f - ramp_t);
	oval_x = real_theta < (PI/2.0f) ? sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2))) : real_theta == (PI/2.0f) 
											? 0 : -sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2)));
	oval_y = real_theta != (PI/2.0f) ? oval_x * tanf(real_theta) : b;
	CSYS->Infer_x = Motor_x + O_x[CSYS->ID-1] - oval_x;
	CSYS->Infer_y = Motor_y - O_y[CSYS->ID-1] - oval_y;
	CSYS->Infer_L = sqrtf(CSYS->Infer_x*CSYS->Infer_x+CSYS->Infer_y*CSYS->Infer_y);
	Psi = asinf(CSYS->Infer_x/CSYS->Infer_L);
	Phi = acosf((L1*L1 + CSYS->Infer_L*CSYS->Infer_L - L2*L2)/(2*L1*CSYS->Infer_L));
	CSYS->Infer_theta1 = (Phi - Psi)*EXCHANGE_RAD_TO_ANGLE;
	CSYS->Infer_theta2 = (Phi + Psi)*EXCHANGE_RAD_TO_ANGLE;
	return *CSYS;
}

/**
	*@brief 当前点正向画椭圆，终点坐标可控
	*/
CSYS_Handle Foot_Front_Up_Oval_Angle_NRTO_ramp(float a,float b,float Angle,CSYS_Handle *CSYS,Ramp_Typedef *Ramp)
{
	static float O_x[4];									//圆心x值
	static float O_y[4];									//圆心y值
  static float oval_x;										
	static float oval_y;
	static float Phi;												
	static float Psi;
	static float real_theta;
	static float ramp_t;
	if(!Ramp->flag)													//获取初始位置坐标
	{
		O_x[CSYS->ID-1] = CSYS->x + a;
		O_y[CSYS->ID-1] = CSYS->y;
	}
	ramp_t = Slope(Ramp);
	real_theta = (Angle/EXCHANGE_RAD_TO_ANGLE)*(1.0f - ramp_t);
	oval_x = real_theta < (PI/2.0f) ? sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2))) : real_theta == (PI/2.0f) 
											? 0 : -sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2)));
	oval_y = real_theta != (PI/2.0f) ? oval_x * tanf(real_theta) : b;
	CSYS->Infer_x = Motor_x + O_x[CSYS->ID-1] + oval_x;
	CSYS->Infer_y = Motor_y - O_y[CSYS->ID-1] - oval_y;
	CSYS->Infer_L = sqrtf(CSYS->Infer_x*CSYS->Infer_x+CSYS->Infer_y*CSYS->Infer_y);
	Psi = asinf(CSYS->Infer_x/CSYS->Infer_L);
	Phi = acosf((L1*L1 + CSYS->Infer_L*CSYS->Infer_L - L2*L2)/(2*L1*CSYS->Infer_L));
	CSYS->Infer_theta1 = (Phi - Psi)*EXCHANGE_RAD_TO_ANGLE;
	CSYS->Infer_theta2 = (Phi + Psi)*EXCHANGE_RAD_TO_ANGLE;
	return *CSYS;
}

/**
	*@brief 当前点反向画椭圆，终点坐标可控
	*/
CSYS_Handle Foot_Back_Up_Oval_Angle_NRTO_ramp(float a,float b,float Angle,CSYS_Handle *CSYS,Ramp_Typedef *Ramp)
{
	static float O_x[4];									//圆心x值
	static float O_y[4];									//圆心y值
  static float oval_x;										
	static float oval_y;
	static float Phi;												
	static float Psi;
	static float real_theta;
	static float ramp_t;
	if(!Ramp->flag)													//获取初始位置坐标
	{
		O_x[CSYS->ID-1] = CSYS->x - a;
		O_y[CSYS->ID-1] = CSYS->y;
	}
	ramp_t = Slope(Ramp);
	real_theta = (Angle/EXCHANGE_RAD_TO_ANGLE)*(1.0f -ramp_t);
	oval_x = real_theta < (PI/2.0f) ? sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2))) : real_theta == (PI/2.0f) 
											? 0 : -sqrtf(1.0f/(powf(a,-2)+powf(tanf(real_theta)/b,2)));
	oval_y = real_theta != (PI/2.0f) ? oval_x * tanf(real_theta) : b;
	CSYS->Infer_x = Motor_x + O_x[CSYS->ID-1] - oval_x;
	CSYS->Infer_y = Motor_y - O_y[CSYS->ID-1] - oval_y;
	CSYS->Infer_L = sqrtf(CSYS->Infer_x*CSYS->Infer_x+CSYS->Infer_y*CSYS->Infer_y);
	Psi = asinf(CSYS->Infer_x/CSYS->Infer_L);
	Phi = acosf((L1*L1 + CSYS->Infer_L*CSYS->Infer_L - L2*L2)/(2*L1*CSYS->Infer_L));
	CSYS->Infer_theta1 = (Phi - Psi)*EXCHANGE_RAD_TO_ANGLE;
	CSYS->Infer_theta2 = (Phi + Psi)*EXCHANGE_RAD_TO_ANGLE;
	return *CSYS;
}

/**
	*@brief  绕足尖旋转，腿长固定不动
	*@param[in] Vector_x 目标点与当前点的x坐标向量
	*@param[in] CSYS		 坐标系句柄
	*@param[in] Ramp 		 斜坡句柄
	*@return 		CSYS		 坐标系
	*/

CSYS_Handle Foot_Round_X_NRTO_ramp(float Vector_x,CSYS_Handle *CSYS,Ramp_Typedef *Ramp)
{
	static float initial_x[4];									//足尖初始x坐标
	static float initial_L[4];
	static float initial_theta_1[4];
	static float initial_theta_2[4];
	static float initial_Psi[4];
	static float D_Value_Psi[4];								//差值
	static float Infer_Psi;
	static float ramp_t;
	if(!Ramp->flag)
	{
		initial_x[CSYS->ID-1] = CSYS->x;
		initial_L[CSYS->ID-1] = CSYS->L;
		initial_Psi[CSYS->ID-1] = CSYS->Psi;
		initial_theta_1[CSYS->ID-1] = CSYS->theta1;
		initial_theta_2[CSYS->ID-1] = CSYS->theta2;
		D_Value_Psi[CSYS->ID-1] = asinf((initial_x[CSYS->ID-1] + Vector_x)/CSYS->L) - initial_Psi[CSYS->ID-1];
	}
	ramp_t = Slope(Ramp);
	Infer_Psi = initial_Psi[CSYS->ID-1] + ramp_t*D_Value_Psi[CSYS->ID-1];
	CSYS->Infer_x = Motor_x + initial_L[CSYS->ID-1]*sinf(Infer_Psi);
	CSYS->Infer_y = Motor_y - initial_L[CSYS->ID-1]*cosf(Infer_Psi);
	
	CSYS->Infer_theta1 = initial_theta_1[CSYS->ID-1] - D_Value_Psi[CSYS->ID-1]*EXCHANGE_RAD_TO_ANGLE*ramp_t;
	CSYS->Infer_theta2 = initial_theta_2[CSYS->ID-1] + D_Value_Psi[CSYS->ID-1]*EXCHANGE_RAD_TO_ANGLE*ramp_t;
	return *CSYS;
}

CSYS_Handle Foot_New_Walk_RTO(float x,float y,float s,float h,CSYS_Handle *CSYS,Ramp_Typedef *Ramp)
{
	static float Motor_x_distance;
	static float Motor_y_distance;
	static float initial_x[4];
	static float initial_y[4];
	static float Psi,Phi;
	static float ramp_t;
	if(!Ramp->flag)
	{
		initial_x[CSYS->ID-1] = x;
		initial_y[CSYS->ID-1] = y;
	}
	ramp_t = Slope(Ramp);
	if(ramp_t < 0.5f)
	{
		CSYS->Infer_y = initial_y[CSYS->ID-1] + ((2.0f*h*(ramp_t - (sinf(4.0f*PI*ramp_t)/(4.0f*PI)))));
	}
	else
	{
		CSYS->Infer_y = initial_y[CSYS->ID-1] + ((2.0f*h*(1.0f-ramp_t + (sinf(4.0f*PI*ramp_t)/(4.0f*PI)))));
	}
	CSYS->Infer_x = initial_x[CSYS->ID-1] + (s*(ramp_t - (sinf(2.0f*PI*ramp_t)/(2.0f*PI))));
	Motor_y_distance = Motor_y - CSYS->Infer_y;
	Motor_x_distance = Motor_x + CSYS->Infer_x;
	CSYS->Infer_L = sqrtf(Motor_x_distance*Motor_x_distance+Motor_y_distance*Motor_y_distance);
	Psi = asinf(Motor_x_distance/CSYS->Infer_L);
	Phi = acosf((L1*L1 + CSYS->Infer_L*CSYS->Infer_L - L2*L2)/(2*L1*CSYS->Infer_L));
	CSYS->Infer_theta1 = (Phi - Psi)*EXCHANGE_RAD_TO_ANGLE;
	CSYS->Infer_theta2 = (Phi + Psi)*EXCHANGE_RAD_TO_ANGLE;
	return *CSYS;
}


CSYS_Handle Foot_New_Walk(float s,float h,CSYS_Handle *CSYS,Ramp_Typedef *Ramp)
{
	static float Motor_x_distance;
	static float Motor_y_distance;
	static float initial_x[4];
	static float initial_y[4];
	static float Psi,Phi;
	static float ramp_t;
	if(!Ramp->flag)
	{
		initial_x[CSYS->ID-1] = CSYS->x;
		initial_y[CSYS->ID-1] = CSYS->y;
	}
	ramp_t = Slope(Ramp);
	
	if(ramp_t < 0.5f)
	{
		CSYS->Infer_y = initial_y[CSYS->ID-1] + ((2.0f*h*(ramp_t - (sinf(4.0f*PI*ramp_t)/(4.0f*PI)))));
	}
	else
	{
		CSYS->Infer_y = initial_y[CSYS->ID-1] + ((2.0f*h*(1.0f-ramp_t + (sinf(4.0f*PI*ramp_t)/(4.0f*PI)))));
	}
	CSYS->Infer_x = initial_x[CSYS->ID-1] + (s*(ramp_t - (sinf(2.0f*PI*ramp_t)/(2.0f*PI))));
	Motor_y_distance = Motor_y - CSYS->Infer_y;
	Motor_x_distance = Motor_x + CSYS->Infer_x;
	CSYS->Infer_L = sqrtf(Motor_x_distance*Motor_x_distance+Motor_y_distance*Motor_y_distance);
	Psi = asinf(Motor_x_distance/CSYS->Infer_L);
	Phi = acosf((L1*L1 + CSYS->Infer_L*CSYS->Infer_L - L2*L2)/(2*L1*CSYS->Infer_L));
	CSYS->Infer_theta1 = (Phi - Psi)*EXCHANGE_RAD_TO_ANGLE;
	CSYS->Infer_theta2 = (Phi + Psi)*EXCHANGE_RAD_TO_ANGLE;
	return *CSYS;
}

CSYS_Handle Foot_New_Walk_Target_RTO(float x,float y,CSYS_Handle *CSYS,Ramp_Typedef *Ramp)
{
	static float Motor_x_distance;
	static float Motor_y_distance;
	static float initial_x[4];
	static float initial_y[4];
	static float s[4];
	static float h[4];
	static float Psi,Phi;
	static float ramp_t;
	if(!Ramp->flag)
	{
		initial_x[CSYS->ID-1] = CSYS->x;
		initial_y[CSYS->ID-1] = CSYS->y;
		s[CSYS->ID-1] = x - initial_x[CSYS->ID-1];
		h[CSYS->ID-1] = y - initial_y[CSYS->ID-1];
	}
	ramp_t = Slope(Ramp);
	if(ramp_t < 0.5f)
	{
		CSYS->Infer_y = initial_y[CSYS->ID-1] + ((2.0f*h[CSYS->ID-1]*(ramp_t - (sinf(4.0f*PI*ramp_t)/(4.0f*PI)))));
	}
	else
	{
		CSYS->Infer_y = initial_y[CSYS->ID-1] + ((2.0f*h[CSYS->ID-1]*(1.0f-ramp_t + (sinf(4.0f*PI*ramp_t)/(4.0f*PI)))));
	}
	CSYS->Infer_x = initial_x[CSYS->ID-1] + (s[CSYS->ID-1]*(ramp_t - (sinf(2.0f*PI*ramp_t)/(2.0f*PI))));
	Motor_y_distance = Motor_y - CSYS->Infer_y;
	Motor_x_distance = Motor_x + CSYS->Infer_x;
	CSYS->Infer_L = sqrtf(Motor_x_distance*Motor_x_distance+Motor_y_distance*Motor_y_distance);
	Psi = asinf(Motor_x_distance/CSYS->Infer_L);
	Phi = acosf((L1*L1 + CSYS->Infer_L*CSYS->Infer_L - L2*L2)/(2*L1*CSYS->Infer_L));
	CSYS->Infer_theta1 = (Phi - Psi)*EXCHANGE_RAD_TO_ANGLE;
	CSYS->Infer_theta2 = (Phi + Psi)*EXCHANGE_RAD_TO_ANGLE;
	return *CSYS;
}

CSYS_Global_Handle Posture_Pitch(float Pitch,CSYS_Handle *CSYS,CSYS_Global_Handle *CSYS_global,float x,float z)
{
	static float OA_[3][1];						//变换后的坐标
	static uint8_t i;
	float OA[3][1] = {{x},{0},{z}};		//变换前的坐标 
	float rotx_P[3][3] = {{	cosf(Pitch/EXCHANGE_RAD_TO_ANGLE),0,sinf(Pitch/EXCHANGE_RAD_TO_ANGLE)},
												{								0		 							 ,1,							0									 },
												{-sinf(Pitch/EXCHANGE_RAD_TO_ANGLE),0,cosf(Pitch/EXCHANGE_RAD_TO_ANGLE)}
											 };
	for(i=0;i<3;i++)
		OA_[i][0] = rotx_P[i][0]*OA[0][0] + rotx_P[i][1]*OA[1][0] + rotx_P[i][2]*OA[2][0];
	CSYS_global->CSYS_Infer_x = OA_[0][0];
	CSYS_global->CSYS_Infer_y = OA_[2][0];
	return *CSYS_global;
}

CSYS_Handle Foot_Direcion_Walk_NRTO(float s,float h,CSYS_Handle *CSYS,Ramp_Typedef *Ramp)
{
	static float Motor_x_distance;
	static float Motor_y_distance;
	static float initial_x[4];
	static float initial_y[4];
	static float Psi,Phi;
	static float ramp_t;
	if(!Ramp->flag)
	{
		initial_x[CSYS->ID-1] = CSYS->x;
		initial_y[CSYS->ID-1] = CSYS->y;
	}
	ramp_t = Slope(Ramp);
	if(ramp_t < 0.5f)
	{
		CSYS->Infer_y = initial_y[CSYS->ID-1] + ((2.0f*h*(ramp_t - (sinf(4.0f*PI*ramp_t)/(4.0f*PI)))));
	}
	else
	{
		CSYS->Infer_y = initial_y[CSYS->ID-1] + ((2.0f*h*(1.0f-ramp_t + (sinf(4.0f*PI*ramp_t)/(4.0f*PI)))));
	}
	
	CSYS->Infer_x = initial_x[CSYS->ID-1] + (s*(ramp_t - (sinf(2.0f*PI*ramp_t)/(2.0f*PI))));
	Motor_y_distance = Motor_y - CSYS->Infer_y;
	Motor_x_distance = Motor_x + CSYS->Infer_x;
	CSYS->Infer_L = sqrtf(Motor_x_distance*Motor_x_distance+Motor_y_distance*Motor_y_distance);
	Psi = asinf(Motor_x_distance/CSYS->Infer_L);
	Phi = acosf((L1*L1 + CSYS->Infer_L*CSYS->Infer_L - L2*L2)/(2*L1*CSYS->Infer_L));
	CSYS->Infer_theta1 = (Phi - Psi)*EXCHANGE_RAD_TO_ANGLE;
	CSYS->Infer_theta2 = (Phi + Psi)*EXCHANGE_RAD_TO_ANGLE;
	return *CSYS;
}

CSYS_Handle Foot_IMU_New_Walk_NRTO(float s,float h,CSYS_Handle *CSYS,IMU_Typedef *imu,CSYS_Global_Handle *CSYS_global,Ramp_Typedef *Ramp)
{
	static float Motor_x_distance;
	static float Motor_y_distance;
	static float initial_x[4];
	static float initial_y[4];
	static float round_y;
	static float round_x;
	static float Psi,Phi;
	static float ramp_t;
	static float initial_Pitch;
	if(!Ramp->flag)
	{
		initial_x[CSYS->ID-1] = CSYS->x;
		initial_y[CSYS->ID-1] = CSYS->y;
		initial_Pitch = imu->EulerAngler.Pitch;
	}
	ramp_t = Slope(Ramp);
	if(ramp_t < 0.5f)
		round_y = 2.0f*h*(ramp_t - (sinf(4.0f*PI*ramp_t)/(4.0f*PI)));
	else
		round_y = 2.0f*h*(1.0f-ramp_t + (sinf(4.0f*PI*ramp_t)/(4.0f*PI)));
	round_x = s*(ramp_t - (sinf(2.0f*PI*ramp_t)/(2.0f*PI)));
	Posture_Pitch(initial_Pitch,CSYS,CSYS_global,round_x,round_y);
	CSYS->Infer_x = initial_x[CSYS->ID-1] + CSYS_global->CSYS_Infer_x;
	CSYS->Infer_y = initial_y[CSYS->ID-1] + CSYS_global->CSYS_Infer_y;
	Motor_y_distance = Motor_y - CSYS->Infer_y;
	Motor_x_distance = Motor_x + CSYS->Infer_x;
	CSYS->Infer_L = sqrtf(Motor_x_distance*Motor_x_distance+Motor_y_distance*Motor_y_distance);
	Psi = asinf(Motor_x_distance/CSYS->Infer_L);
	Phi = acosf((L1*L1 + CSYS->Infer_L*CSYS->Infer_L - L2*L2)/(2*L1*CSYS->Infer_L));
	CSYS->Infer_theta1 = (Phi - Psi)*EXCHANGE_RAD_TO_ANGLE;
	CSYS->Infer_theta2 = (Phi + Psi)*EXCHANGE_RAD_TO_ANGLE;
	return *CSYS;
}