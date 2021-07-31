#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "stm32f4xx_hal.h"

#define RM3508_LIMIT 16000.0f  //!<@brief RM3508的输出限幅
#define EXCHANGE_ANGLE_DEG_TO_INFER_REAL_ANGLE (187.0f/3591.0f)		//连续化角度制角度 转换成 小轴连续化角度
#define EXCHANGE_REAL_ANGLE_TO_ANGLE 436.9796791443f							//小轴连续化角度 转换成 连续化角度

/**
 * @brief RM3508电机数据结构体
 */
typedef struct {
		uint8_t ESC_ID;							//!<@brief 电调ID
    uint16_t MchanicalAngle;    //!<@brief 机械角度
    int16_t Speed;              //!<@brief 转速
    int16_t TorqueCurrent;      //!<@brief 转矩电流
    uint8_t temp;               //!<@brief 温度
    float Power;                //!<@brief 功率
    uint16_t LsatAngle;         //!<@brief 上一次的机械角度
		uint16_t InitAngle;					//!<@brief 初始机械角度			/* 消除
		int16_t Init_r;							//!<@brief 初始圈数						 误差	*/
    int16_t r;                  //!<@brief 圈数
    int32_t Angle;              //!<@brief 连续化机械角度
    float Angle_DEG;            //!<@brief 连续化角度制角度
		float Real_Angle;						//!<@brief 小轴连续化角度			(单位：度)
		float Infer_Real_Angle;			//!<@brief 小轴连续化期望角度	(单位：度)
    struct PowerCOF_s {
        float ss;               //!<@brief 速度平方项系数
        float sc;               //!<@brief 速度,转矩电流乘积项系数
        float cc;               //!<@brief 转矩电流平方项系数
        float constant;         //!<@brief 常量
    } PowerCOF;                 //!<@brief 计算功率所用的系数,由MATLAB拟合
} RM3508_TypeDef;

extern RM3508_TypeDef Motor_RM3508[8];
/**
 * @brief RM3508数据接收
 * @param[out] Dst RM3508电机数据结构体指针
 * @param[in] Data CAN数据帧指针
 */
void RM3508_Receive(RM3508_TypeDef *Dst, uint8_t *Data,uint8_t *flag);


#endif
