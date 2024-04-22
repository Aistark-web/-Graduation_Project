#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "main.h"
#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
	float round;		// @brief 圈数(r/s)
	float a_vel;		// @brief 角速度速度(rad/s)   
	float l_vel;		// @brief 线速度(m/s)
	float radius;		// @brief 连接结构半径(m)
}MG513_Motor_Handle;
/**
 * @brief 初始化电机
 * @param Motor 电机
 * @param radius 连接结构半径
 */
void MG513_Motor_Init(MG513_Motor_Handle *Motor,float radius);

/**
 * @brief 获取电机参数(角速度，线速度等)
 * @brief Motor 电机
 * @brief get_fre 获取频率
 */
void MG513_Motor_Get(MG513_Motor_Handle *Motor,TIM_HandleTypeDef *htim,uint16_t get_fre);
#ifdef __cplusplus
}
#endif

#endif