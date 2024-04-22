#ifndef _MASTER_H_
#define _MASTER_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "freertos.h"
#include "task.h"
#include "motor.h"
#include "PID.h"
#include "classis.h"

void task_init();
void motor_init();
void ros_init();
void hardware_init();
void master_init();
void motor_detect();
void motor_control();


#ifdef __cplusplus
}
#endif

#endif