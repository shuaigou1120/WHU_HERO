#ifndef SHOOT_H
#define SHOOT_H
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#define FRICTION_SPEED_PID_KP        25.0f   //25.0f
#define FRICTION_SPEED_PID_KI        1.0f     //1.0f
#define FRICTION_SPEED_PID_KD        5.0f       //5.0f
#define FRICTION_SPEED_PID_MAX_OUT   30000.0f   //30000.0f
#define FRICTION_SPEED_PID_MAX_IOUT  10000.f  //10000.0f
#define FRICTION_CURRENT_PID_KP        0.7f
#define FRICTION_CURRENT_PID_KI        0.2f
#define FRICTION_CURRENT_PID_KD        0.0f
#define FRICTION_CURRENT_PID_MAX_OUT   30000.0f     //30000.0f
#define FRICTION_CURRENT_PID_MAX_IOUT  10000.0f   //10000.0f
typedef enum
{
	SHOOT_STOP=0,
	SHOOT_BULLET,
}shoot_mode_e;
 typedef struct
{
	shoot_mode_e shoot_mode;
	const motor_measure_t *friction_motor_measure[4];
	const RC_ctrl_t *shoot_rc;
	pid_type_def friction_motor_speed_pid; 
	pid_type_def friction_motor_current_pid; 
    fp32 friction_1_speed;
	fp32 friction_2_speed;
	fp32 friction_3_speed;
	fp32 friction_4_speed;
    fp32 friction_speed_set;
	int16_t shoot_1_given_current;
	int16_t shoot_2_given_current;
	int16_t shoot_3_given_current;
	int16_t shoot_4_given_current;
	bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
}shoot_control_t;
const shoot_control_t *shoot_control_loop();
extern shoot_control_t shoot_control;
extern float data1[2];
#endif