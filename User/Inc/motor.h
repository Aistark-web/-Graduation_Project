#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "main.h"
#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
	float round;		// @brief Ȧ��(r/s)
	float a_vel;		// @brief ���ٶ��ٶ�(rad/s)   
	float l_vel;		// @brief ���ٶ�(m/s)
	float radius;		// @brief ���ӽṹ�뾶(m)
}MG513_Motor_Handle;
/**
 * @brief ��ʼ�����
 * @param Motor ���
 * @param radius ���ӽṹ�뾶
 */
void MG513_Motor_Init(MG513_Motor_Handle *Motor,float radius);

/**
 * @brief ��ȡ�������(���ٶȣ����ٶȵ�)
 * @brief Motor ���
 * @brief get_fre ��ȡƵ��
 */
void MG513_Motor_Get(MG513_Motor_Handle *Motor,TIM_HandleTypeDef *htim,uint16_t get_fre);
#ifdef __cplusplus
}
#endif

#endif