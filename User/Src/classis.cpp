#include "classis.h"

arm_matrix_instance_f32 forward_matrix_instance;	// @brief µ×ÅÌÕý½âËã¾ØÕó
arm_matrix_instance_f32 inverse_matrix_instance;	// @brief µ×ÅÌ·´½âËã¾ØÕó

float32_t forward_matrix[12] = 
{
	0.25f,  -0.25f,	-0.25f,	0.25f,
	0.25f,	0.25f,	-0.25f,	-0.25f,
	0,			0,			0,			0
};
float32_t inverse_matrix[12] = 
{
	1.0f,		1.0f,		0,
	-1.0f,	1.0f,		0,
	-1.0f,	-1.0f,	0,
	1.0f,		-1.0f,	0
};
float32_t classis_v[3];
float32_t wheel_linear_v[4];

void Classis_Init(Classis_Handle *Classis,float a,float b)
{
	Classis->a = a;
	Classis->b = b;
	// ³õÊ¼»¯Êý×ª»»¾ØÕó
	arm_fill_f32(1.0f/(a+b),&forward_matrix[8],4);
	inverse_matrix[2]		= a+b;
	inverse_matrix[5]		= a+b;
	inverse_matrix[8] 	= a+b;
	inverse_matrix[11]	= a+b;
	arm_mat_init_f32(&forward_matrix_instance,3,4,forward_matrix);
	arm_mat_init_f32(&inverse_matrix_instance,4,3,inverse_matrix);
}

void Classis_Forward_Calculation(float RF_Linear_V,float LF_Linear_V,float LB_Linear_V,float RB_Linear_V,
																 float *linear_x,float *linear_y,float *angular_z)
{
	arm_matrix_instance_f32 wheel_linear_v_instance;
	arm_matrix_instance_f32 classis_v_instance;
	arm_mat_init_f32(&wheel_linear_v_instance,4,1,wheel_linear_v);
	arm_mat_init_f32(&classis_v_instance,3,1,classis_v);
	wheel_linear_v_instance.pData[0] = RF_Linear_V;
	wheel_linear_v_instance.pData[1] = LF_Linear_V;
	wheel_linear_v_instance.pData[2] = LB_Linear_V;
	wheel_linear_v_instance.pData[3] = RB_Linear_V;
	arm_mat_mult_f32(&forward_matrix_instance,&wheel_linear_v_instance,&classis_v_instance);
	*linear_x		= classis_v_instance.pData[0];
	*linear_y		= classis_v_instance.pData[1];
	*angular_z	= classis_v_instance.pData[2];
}

void Classis_Inverse_Claculation(float linear_x,float linear_y,float angular_z,float *RF_Linear_V,
																 float *LF_Linear_V,float *LB_Linear_V,float *RB_Linear_V)
{
	arm_matrix_instance_f32 wheel_linear_v_instance;
	arm_matrix_instance_f32 classis_v_instance;
	arm_mat_init_f32(&wheel_linear_v_instance,4,1,wheel_linear_v);
	arm_mat_init_f32(&classis_v_instance,3,1,classis_v);
	classis_v_instance.pData[0] = linear_x;
	classis_v_instance.pData[1] = linear_y;
	classis_v_instance.pData[2] = angular_z;
	arm_mat_mult_f32(&inverse_matrix_instance,&classis_v_instance,&wheel_linear_v_instance);
	*RF_Linear_V = wheel_linear_v_instance.pData[0];
	*LF_Linear_V = wheel_linear_v_instance.pData[1];
	*LB_Linear_V = wheel_linear_v_instance.pData[2];
	*RB_Linear_V = wheel_linear_v_instance.pData[3];
}