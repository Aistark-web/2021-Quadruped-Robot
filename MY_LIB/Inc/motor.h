#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "stm32f4xx_hal.h"

#define RM3508_LIMIT 16000.0f  //!<@brief RM3508������޷�
#define EXCHANGE_ANGLE_DEG_TO_INFER_REAL_ANGLE (187.0f/3591.0f)		//�������Ƕ��ƽǶ� ת���� С���������Ƕ�
#define EXCHANGE_REAL_ANGLE_TO_ANGLE 436.9796791443f							//С���������Ƕ� ת���� �������Ƕ�

/**
 * @brief RM3508������ݽṹ��
 */
typedef struct {
		uint8_t ESC_ID;							//!<@brief ���ID
    uint16_t MchanicalAngle;    //!<@brief ��е�Ƕ�
    int16_t Speed;              //!<@brief ת��
    int16_t TorqueCurrent;      //!<@brief ת�ص���
    uint8_t temp;               //!<@brief �¶�
    float Power;                //!<@brief ����
    uint16_t LsatAngle;         //!<@brief ��һ�εĻ�е�Ƕ�
		uint16_t InitAngle;					//!<@brief ��ʼ��е�Ƕ�			/* ����
		int16_t Init_r;							//!<@brief ��ʼȦ��						 ���	*/
    int16_t r;                  //!<@brief Ȧ��
    int32_t Angle;              //!<@brief ��������е�Ƕ�
    float Angle_DEG;            //!<@brief �������Ƕ��ƽǶ�
		float Real_Angle;						//!<@brief С���������Ƕ�			(��λ����)
		float Infer_Real_Angle;			//!<@brief С�������������Ƕ�	(��λ����)
    struct PowerCOF_s {
        float ss;               //!<@brief �ٶ�ƽ����ϵ��
        float sc;               //!<@brief �ٶ�,ת�ص����˻���ϵ��
        float cc;               //!<@brief ת�ص���ƽ����ϵ��
        float constant;         //!<@brief ����
    } PowerCOF;                 //!<@brief ���㹦�����õ�ϵ��,��MATLAB���
} RM3508_TypeDef;

extern RM3508_TypeDef Motor_RM3508[8];
/**
 * @brief RM3508���ݽ���
 * @param[out] Dst RM3508������ݽṹ��ָ��
 * @param[in] Data CAN����ָ֡��
 */
void RM3508_Receive(RM3508_TypeDef *Dst, uint8_t *Data,uint8_t *flag);


#endif
