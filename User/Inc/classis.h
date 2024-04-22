/**
	* @file classis.h
	* @author Luo
	* @date 2024-4-12
	* @brief 麦轮O型底盘模型解算
	*/

#ifndef _CLASSIS_H_
#define	_CLASSIS_H_

#ifdef __cplusplus
extern "C" {
#endif	
#include "main.h"
#include "arm_math.h"
typedef struct _Classis{
	float a;										// @brief	底盘中心到左轮或右轮连接线的垂直距离
	float b;										// @brief	底盘中心到前轮或后轮连接线的垂直距离
	// @brief 速度
	struct{
		// @brief 当前速度
		struct{
			float linear_x;					// @brief 当前x方向线速度
			float linear_y;					// @brief 当前y方向线速度
			float angular_z;				// @brief 当前z方向角速度
			float RF_Linear_V;			// @brief 当前右前轮线速度
			float LF_Linear_V;			// @brief 当前左前轮线速度
			float LB_Linear_V;			// @brief 当前左后轮线速度
			float RB_Linear_V;			// @brief 当前左后轮线速度
		}Current;
		// @brief 期望速度
		struct{
			float linear_x;					// @brief 期望x方向线速度
			float linear_y;					// @brief 期望y方向线速度
			float angular_z;				// @brief 期望z方向角速度
			float RF_Linear_V;			// @brief 期望右前轮线速度
			float LF_Linear_V;			// @brief 期望左前轮线速度
			float LB_Linear_V;			// @brief 期望左后轮线速度
			float RB_Linear_V;			// @brief 期望左后轮线速度
		}Expect;									  
	}Velocity;
}Classis_Handle;

/**
	* @brief 初始化底盘(初始化底盘)
	* @param[in] Classis 底盘
	* @param[in] a 底盘中心到左轮或右轮连接线的垂直距离
	* @param[in] b 底盘中心到前轮或后轮连接线的垂直距离
	*/
void Classis_Init(Classis_Handle *Classis,float a,float b);
/**
	* @brief 底盘正解算(对速度正解算，输入四轮线速度，解算出当前xy方向线速度、z方向角速度)
	* @param[in] 	RF_Linear_V 右前轮线速度
	* @param[in] 	LF_Linear_V 左前轮线速度
	* @param[in] 	LB_Linear_V 左后轮线速度
	* @param[in] 	RB_Linear_V 右后轮线速度
	* @param[out] linear_x 		x方向线速度
	* @param[out] linear_y 		y方向线速度
	* @param[out] angular_z 	z方向角速度
	*/
void Classis_Forward_Calculation(float RF_Linear_V,float LF_Linear_V,float LB_Linear_V,float RB_Linear_V,
																 float *linear_x,float *linear_y,float *angular_z);

/**
	* @brief 底盘反解算(对速度反解算，输入底盘xy方向线速度、z方向角速度，反解算出四轮线速度) 
	* @param[in] 	linear_x 		x方向线速度
	* @param[in] 	linear_y 		y方向线速度
	* @param[in] 	angular_z 	z方向角速度
	* @param[out] RF_Linear_V 右前轮线速度
	* @param[out] LF_Linear_V 左前轮线速度
	* @param[out] LB_Linear_V 左后轮线速度
	* @param[out] RB_Linear_V 右后轮线速度
	*/
void Classis_Inverse_Claculation(float linear_x,float linear_y,float angular_z,float *RF_Linear_V,
																 float *LF_Linear_V,float *LB_Linear_V,float *RB_Linear_V);
#ifdef __cplusplus
}
#endif

#endif