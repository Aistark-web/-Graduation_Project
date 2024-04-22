#include "motor.h"

#define line_num 13						// @brief 编码器线数
#define reduction_ratio 60		// @brief 减速比
#define fm 4									// @breif 定时器脉冲测量倍频
void MG513_Motor_Init(MG513_Motor_Handle *Motor,float radius)
{
	Motor->radius = radius;
}

void MG513_Motor_Get(MG513_Motor_Handle *Motor,TIM_HandleTypeDef *htim,uint16_t get_fre)
{
	int16_t Encoder_TIM;
	Encoder_TIM = __HAL_TIM_GET_COUNTER(htim);
	__HAL_TIM_SET_COUNTER(htim,0);
	Motor->round = (float)Encoder_TIM*(float)get_fre/(float)(reduction_ratio*line_num*fm);
	Motor->a_vel = 2.0f*PI*Motor->round;
	Motor->l_vel = Motor->radius * Motor->a_vel;
}

