/**
	* @file classis.h
	* @author Luo
	* @date 2024-4-12
	* @brief ����O�͵���ģ�ͽ���
	*/

#ifndef _CLASSIS_H_
#define	_CLASSIS_H_

#ifdef __cplusplus
extern "C" {
#endif	
#include "main.h"
#include "arm_math.h"
typedef struct _Classis{
	float a;										// @brief	�������ĵ����ֻ����������ߵĴ�ֱ����
	float b;										// @brief	�������ĵ�ǰ�ֻ���������ߵĴ�ֱ����
	// @brief �ٶ�
	struct{
		// @brief ��ǰ�ٶ�
		struct{
			float linear_x;					// @brief ��ǰx�������ٶ�
			float linear_y;					// @brief ��ǰy�������ٶ�
			float angular_z;				// @brief ��ǰz������ٶ�
			float RF_Linear_V;			// @brief ��ǰ��ǰ�����ٶ�
			float LF_Linear_V;			// @brief ��ǰ��ǰ�����ٶ�
			float LB_Linear_V;			// @brief ��ǰ��������ٶ�
			float RB_Linear_V;			// @brief ��ǰ��������ٶ�
		}Current;
		// @brief �����ٶ�
		struct{
			float linear_x;					// @brief ����x�������ٶ�
			float linear_y;					// @brief ����y�������ٶ�
			float angular_z;				// @brief ����z������ٶ�
			float RF_Linear_V;			// @brief ������ǰ�����ٶ�
			float LF_Linear_V;			// @brief ������ǰ�����ٶ�
			float LB_Linear_V;			// @brief ������������ٶ�
			float RB_Linear_V;			// @brief ������������ٶ�
		}Expect;									  
	}Velocity;
}Classis_Handle;

/**
	* @brief ��ʼ������(��ʼ������)
	* @param[in] Classis ����
	* @param[in] a �������ĵ����ֻ����������ߵĴ�ֱ����
	* @param[in] b �������ĵ�ǰ�ֻ���������ߵĴ�ֱ����
	*/
void Classis_Init(Classis_Handle *Classis,float a,float b);
/**
	* @brief ����������(���ٶ������㣬�����������ٶȣ��������ǰxy�������ٶȡ�z������ٶ�)
	* @param[in] 	RF_Linear_V ��ǰ�����ٶ�
	* @param[in] 	LF_Linear_V ��ǰ�����ٶ�
	* @param[in] 	LB_Linear_V ��������ٶ�
	* @param[in] 	RB_Linear_V �Һ������ٶ�
	* @param[out] linear_x 		x�������ٶ�
	* @param[out] linear_y 		y�������ٶ�
	* @param[out] angular_z 	z������ٶ�
	*/
void Classis_Forward_Calculation(float RF_Linear_V,float LF_Linear_V,float LB_Linear_V,float RB_Linear_V,
																 float *linear_x,float *linear_y,float *angular_z);

/**
	* @brief ���̷�����(���ٶȷ����㣬�������xy�������ٶȡ�z������ٶȣ���������������ٶ�) 
	* @param[in] 	linear_x 		x�������ٶ�
	* @param[in] 	linear_y 		y�������ٶ�
	* @param[in] 	angular_z 	z������ٶ�
	* @param[out] RF_Linear_V ��ǰ�����ٶ�
	* @param[out] LF_Linear_V ��ǰ�����ٶ�
	* @param[out] LB_Linear_V ��������ٶ�
	* @param[out] RB_Linear_V �Һ������ٶ�
	*/
void Classis_Inverse_Claculation(float linear_x,float linear_y,float angular_z,float *RF_Linear_V,
																 float *LF_Linear_V,float *LB_Linear_V,float *RB_Linear_V);
#ifdef __cplusplus
}
#endif

#endif