#include "master.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"
/**
	* @brief ӳ��
	*	TIM8 CH2N --> PWMA	--> RF_Motor��ǰ�� PWM
	* TIM8 CH3N --> PWMB	--> LF_Motor��ǰ�� 
	* TIM2 CH3	--> PWMC	-->	LB_Motor����� 
	* TIM2 CH4	--> PWMD	-->	RB_Motor����� 
	* TIM1 E1A E1B	--> RF_Motor��ǰ�� ��������
	* TIM3 E2A E2B	--> LF_Motor��ǰ�� 
	* TIM4 E3A E3B	--> LB_Motor����� 
	* TIM4 E4A E4B	--> RB_Motor����� 
	* AIN1 AIN2	--> RF_Motor��ǰ�� ���Ʒ������
	*	BIN1 BIN2	--> LF_Motor��ǰ�� 
	* CIN1 CIN2	--> LB_Motor�����
	* DIN1 DIN2	--> RB_Motor�����
	*/

/* BEGIN �궨�� */
#define Motor_Detect_Frq 200	// @brief ����Ƶ��(hz)
#define Motor_Radius 0.030f		// @brief �뾶(m)
#define Motor_PID_OUT_LIMIT 8399.0f	// @brief pid������
#define Classis_a		0.16f			// @brief	�������ĵ����ֻ����������ߵĴ�ֱ����
#define Classis_b		0.1025f			// @brief	�������ĵ�ǰ�ֻ���������ߵĴ�ֱ����
/* END �궨�� */
/* BEGIN �����ⲿ���� */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
/* END �����ⲿ���� */

/* BEGIN �ڲ����� */
/* FreeRTOS */

TaskHandle_t Motor_Detect_Task_Handle;	// @brief �����������������
TaskHandle_t Classis_Detect_Task_Handle;// @brief ���̲���������
TaskHandle_t ROS_Loop_Task_Handle;			// @brief ROSѭ��������
TaskHandle_t ROS_Deal_Task_Handle;			// @brief ROS����������
void Motor_Detect_Control_Task();				// @brief ���������������
void Classis_Detect_Task();							// @brief ���̲�������
void ROS_Loop_Task();										// @brief ROSѭ������
void ROS_Deal_Task();										// @brief ROS��������

/* ��� */
MG513_Motor_Handle RF_Motor;
MG513_Motor_Handle LF_Motor;
MG513_Motor_Handle LB_Motor;
MG513_Motor_Handle RB_Motor;

PID_ADD RF_Motor_PID_ADD;
PID_ADD LF_Motor_PID_ADD;
PID_ADD LB_Motor_PID_ADD;
PID_ADD RB_Motor_PID_ADD;
float RF_Motor_pid_out;
float LF_Motor_pid_out;
float LB_Motor_pid_out;
float RB_Motor_pid_out;
float RF_Motor_Exp_l_vel;

/* ���� */
Classis_Handle Classis;
/* ros */
ros::NodeHandle nh;
void cmd_vel_sub_callback(const geometry_msgs::Twist& msg);	// ���ջ���cmd_vel�ص�����
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel",&cmd_vel_sub_callback);	// ����cmd_vel�Ľ�����
geometry_msgs::Twist current_cmd_vel_msg;
ros::Publisher current_cmd_vel_pub("current_cmd_vel",&current_cmd_vel_msg);
/* END �ڲ����� */

/**
	* @brief ϵͳ��ʼ��
	*/
void master_init(){
	hardware_init();
	ros_init();
	motor_init();
	Classis_Init(&Classis,Classis_a,Classis_b);
	task_init();
}
/**
	* @brief Ӳ����ʼ��  
	*/
void hardware_init(){
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
	HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET);
	HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET);
	HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);
	HAL_GPIO_WritePin(CIN1_GPIO_Port,CIN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CIN2_GPIO_Port,CIN2_Pin,GPIO_PIN_RESET);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,0);
	HAL_GPIO_WritePin(DIN1_GPIO_Port,DIN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DIN2_GPIO_Port,DIN2_Pin,GPIO_PIN_RESET);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);
}
/**
	* @brief �����ʼ��
	*/
void motor_init(){
	MG513_Motor_Init(&RF_Motor,Motor_Radius);
	MG513_Motor_Init(&LF_Motor,Motor_Radius);
	MG513_Motor_Init(&LB_Motor,Motor_Radius);
	MG513_Motor_Init(&RB_Motor,Motor_Radius);
	RF_Motor_PID_ADD.Kp = 1000.0f;
	RF_Motor_PID_ADD.Ki = 100.f;
	RF_Motor_PID_ADD.Kd = 0.0f;
	LF_Motor_PID_ADD.Kp = 1000.0f;
	LF_Motor_PID_ADD.Ki = 100.f;
	LF_Motor_PID_ADD.Kd = 0.0f;
	LB_Motor_PID_ADD.Kp = 1000.0f;
	LB_Motor_PID_ADD.Ki = 100.f;
	LB_Motor_PID_ADD.Kd = 0.0f;
	RB_Motor_PID_ADD.Kp = 1000.0f;
	RB_Motor_PID_ADD.Ki = 100.f;
	RB_Motor_PID_ADD.Kd = 0.0f;
}

/**
	* @brief �����ʼ��
	*/
void task_init(){
	xTaskCreate((TaskFunction_t)Motor_Detect_Control_Task,"Motor_Detect_Control_Task",128,NULL,1,&Motor_Detect_Task_Handle);
	xTaskCreate((TaskFunction_t)Classis_Detect_Task,"Classis_Detect_Task",128,NULL,1,&Classis_Detect_Task_Handle);
	xTaskCreate((TaskFunction_t)ROS_Loop_Task,"ROS_Loop_Task",128,NULL,1,&ROS_Loop_Task_Handle);
	xTaskCreate((TaskFunction_t)ROS_Deal_Task,"ROS_Deal_Task",128,NULL,1,&ROS_Deal_Task_Handle);
}

/**
	* @brief �����ʼ��
	*/
void ros_init(){
	nh.initNode();											// ��ʼ���ڵ�
	nh.advertise(current_cmd_vel_pub);	// ע�ᷢ����
	nh.subscribe(cmd_vel_sub);					// ע�ᶩ����
}

/* BEGIN ���� */
/**
	* @brief ���������������
	*/
void Motor_Detect_Control_Task(){
	TickType_t PreviousWakeTime;
	const TickType_t TimeIncrement = pdMS_TO_TICKS(1000/Motor_Detect_Frq);
	PreviousWakeTime = xTaskGetTickCount();
	for(;;){
		motor_detect();
		motor_control();
		vTaskDelayUntil(&PreviousWakeTime,TimeIncrement);
	}
}
/**
	* @brief ���̲�������
	*/
void Classis_Detect_Task(){
	for(;;){
		Classis.Velocity.Current.RF_Linear_V = RF_Motor.l_vel;
		Classis.Velocity.Current.LF_Linear_V = LF_Motor.l_vel;
		Classis.Velocity.Current.LB_Linear_V = LB_Motor.l_vel;
		Classis.Velocity.Current.RB_Linear_V = RB_Motor.l_vel;
		Classis_Forward_Calculation(Classis.Velocity.Current.RF_Linear_V,Classis.Velocity.Current.LF_Linear_V,
																Classis.Velocity.Current.LB_Linear_V,Classis.Velocity.Current.RB_Linear_V,
																&Classis.Velocity.Current.linear_x,&Classis.Velocity.Current.linear_y,
																&Classis.Velocity.Current.angular_z);
		current_cmd_vel_msg.linear.x = Classis.Velocity.Current.linear_x;
		current_cmd_vel_msg.linear.y = Classis.Velocity.Current.linear_y;
		current_cmd_vel_msg.angular.z = Classis.Velocity.Current.angular_z;
		if(nh.connected()){
			current_cmd_vel_pub.publish(&current_cmd_vel_msg);
		}
		vTaskDelay(20);
	}
}

/**
	* @brief ROS���ջ���cmd_vel�ص�����
	* @param msg ���յ��Ļ�����Ϣ
	*/
void cmd_vel_sub_callback(const geometry_msgs::Twist& msg)
{
	Classis.Velocity.Expect.linear_x = msg.linear.x;
	Classis.Velocity.Expect.linear_y = msg.linear.y;
	Classis.Velocity.Expect.angular_z = msg.angular.z;
	Classis_Inverse_Claculation(Classis.Velocity.Expect.linear_x,Classis.Velocity.Expect.linear_y,Classis.Velocity.Expect.angular_z,
															&Classis.Velocity.Expect.RF_Linear_V,&Classis.Velocity.Expect.LF_Linear_V,
															&Classis.Velocity.Expect.LB_Linear_V,&Classis.Velocity.Expect.RB_Linear_V);
}
/**
	* @brief ROSͬ������
	*/
void ROS_Loop_Task(){
	for(;;){
		nh.spinOnce();
		vTaskDelay(300);
	}
}

/**
	* @brief ROS��������
	*/
void ROS_Deal_Task(){
	for(;;){
		// �ȴ�����֪ͨ
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		// ���յ�֪ͨ��������
		nh.spinOnce();
	}
}

/* END ���� */

/* BEGIN ���� */
/**
  * @brief �������
	*/
void motor_detect(){
	MG513_Motor_Get(&RF_Motor,&htim1,Motor_Detect_Frq);
	MG513_Motor_Get(&LF_Motor,&htim3,Motor_Detect_Frq);
	MG513_Motor_Get(&LB_Motor,&htim4,Motor_Detect_Frq);
	MG513_Motor_Get(&RB_Motor,&htim5,Motor_Detect_Frq);
}
/**
	* @brief �������
	*/
void motor_control(){
	static int32_t load_RF_Motor_pid_out,load_LF_Motor_pid_out,load_LB_Motor_pid_out,load_RB_Motor_pid_out;
	/* ��ǰ�� */
	RF_Motor_pid_out += PID_Increment(RF_Motor.l_vel,Classis.Velocity.Expect.RF_Linear_V,&RF_Motor_PID_ADD);
	limit(RF_Motor_pid_out,Motor_PID_OUT_LIMIT,-Motor_PID_OUT_LIMIT);
	load_RF_Motor_pid_out = (int32_t)RF_Motor_pid_out;
	/* ��ǰ�� */
	LF_Motor_pid_out += PID_Increment(LF_Motor.l_vel,Classis.Velocity.Expect.LF_Linear_V,&LF_Motor_PID_ADD);
	limit(LF_Motor_pid_out,Motor_PID_OUT_LIMIT,-Motor_PID_OUT_LIMIT);
	load_LF_Motor_pid_out = (int32_t)LF_Motor_pid_out;
	/* ����� */
	LB_Motor_pid_out += PID_Increment(LB_Motor.l_vel,Classis.Velocity.Expect.LB_Linear_V,&LB_Motor_PID_ADD);
	limit(LB_Motor_pid_out,Motor_PID_OUT_LIMIT,-Motor_PID_OUT_LIMIT);
	load_LB_Motor_pid_out = (int32_t)LB_Motor_pid_out;
	/* �Һ��� */
	RB_Motor_pid_out += PID_Increment(RB_Motor.l_vel,Classis.Velocity.Expect.RB_Linear_V,&RB_Motor_PID_ADD);
	limit(RB_Motor_pid_out,Motor_PID_OUT_LIMIT,-Motor_PID_OUT_LIMIT);
	load_RB_Motor_pid_out = (int32_t)RB_Motor_pid_out;
	/* ��ǰ�� */
	if(load_RF_Motor_pid_out > 0){
		HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,load_RF_Motor_pid_out);
	}
	else if(load_RF_Motor_pid_out < 0){
		HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,-load_RF_Motor_pid_out);
	}
	else{
		HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,load_RF_Motor_pid_out);
	}
	/* ��ǰ�� */
	if(load_LF_Motor_pid_out > 0){
		HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,load_LF_Motor_pid_out);
	}
	else if(load_LF_Motor_pid_out < 0){
		HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,-load_LF_Motor_pid_out);
	}
	else{
		HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,load_LF_Motor_pid_out);
	}
	/* ����� */
	if(load_LB_Motor_pid_out > 0){
		HAL_GPIO_WritePin(CIN1_GPIO_Port,CIN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(CIN2_GPIO_Port,CIN2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,load_LB_Motor_pid_out);
	}
	else if(load_LB_Motor_pid_out < 0){
		HAL_GPIO_WritePin(CIN1_GPIO_Port,CIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CIN2_GPIO_Port,CIN2_Pin,GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,-load_LB_Motor_pid_out);
	}
	else{
		HAL_GPIO_WritePin(CIN1_GPIO_Port,CIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CIN2_GPIO_Port,CIN2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,load_LB_Motor_pid_out);
	}
	/* �Һ��� */
	if(load_RB_Motor_pid_out > 0){
		HAL_GPIO_WritePin(DIN1_GPIO_Port,DIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIN2_GPIO_Port,DIN2_Pin,GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,load_RB_Motor_pid_out);
	}
	else if(load_RB_Motor_pid_out < 0){
		HAL_GPIO_WritePin(DIN1_GPIO_Port,DIN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIN2_GPIO_Port,DIN2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,-load_RB_Motor_pid_out);
	}
	else{
		HAL_GPIO_WritePin(DIN1_GPIO_Port,DIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIN2_GPIO_Port,DIN2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,load_RB_Motor_pid_out);
	}
}
/* END ���� */

/* BEGIN �жϴ��� */
// ���ջص�
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	// ������STM32Hardware.h��Ĭ��rosserial��ش���Ϊhuart2��������������ж�
	if(huart->Instance == USART2){
		//������ȫ�ص�
		if(huart->RxXferSize == Size){
			nh.getHardware()->reset_rbuf(nh.getHardware()->rbuflen);
			BaseType_t xHigherPriorityTaskWoken;
			// ����֪ͨ   
			vTaskNotifyGiveFromISR(ROS_Deal_Task_Handle,&xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		// �ж�UART IDLE�ж��Ƿ�رգ��Ӷ��ж��Ƿ���IDLE������ж�
		else if(!__HAL_UART_GET_IT_SOURCE(huart,UART_IT_IDLE))
		{
			nh.getHardware()->reset_rbuf(huart->RxXferSize - Size);
			BaseType_t xHigherPriorityTaskWoken;
			// ����֪ͨ
			vTaskNotifyGiveFromISR(ROS_Deal_Task_Handle,&xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}

//���ͻص�
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	// ������STM32Hardware.h��Ĭ��rosserial��ش���Ϊhuart2��������������ж�
	if(huart->Instance == USART2){
		nh.getHardware()->flush();
	}
}
/* END �жϴ��� */