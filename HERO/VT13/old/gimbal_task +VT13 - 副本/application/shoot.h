#ifndef SHOOT_H
#define SHOOT_H
#include "CAN_receive.h"
#include "pid.h"
#include "VT13_control.h"
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

// 1ºÅÄ¦²ÁÂÖ
#define motor_1_SPEED_PID_KP        25.0f   //25.0f
#define motor_1_SPEED_PID_KI        0.5f     //1.0f
#define motor_1_SPEED_PID_KD        0.0f       //5.0f
#define motor_1_SPEED_PID_MAX_OUT   30000.0f   //30000.0f
#define motor_1_SPEED_PID_MAX_IOUT  10000.f  //10000.0f
#define motor_1_CURRENT_PID_KP        0.7f
#define motor_1_CURRENT_PID_KI        0.2f
#define motor_1_CURRENT_PID_KD        0.0f
#define motor_1_CURRENT_PID_MAX_OUT   30000.0f     //30000.0f
#define motor_1_CURRENT_PID_MAX_IOUT  10000.0f   //10000.0f

//2ºÅÄ¦²ÁÂÖ
#define motor_2_SPEED_PID_KP        40.0f   //25.0f
#define motor_2_SPEED_PID_KI        1.5f     //1.0f
#define motor_2_SPEED_PID_KD        4.0f       //5.0f
#define motor_2_SPEED_PID_MAX_OUT   30000.0f   //30000.0f
#define motor_2_SPEED_PID_MAX_IOUT  10000.f  //10000.0f
#define motor_2_CURRENT_PID_KP        0.7f
#define motor_2_CURRENT_PID_KI        0.2f
#define motor_2_CURRENT_PID_KD        0.0f
#define motor_2_CURRENT_PID_MAX_OUT   30000.0f     //30000.0f
#define motor_2_CURRENT_PID_MAX_IOUT  10000.0f   //10000.0f

//3ºÅÄ¦²ÁÂÖ
#define motor_3_SPEED_PID_KP        50.0f   //25.0f
#define motor_3_SPEED_PID_KI        0.0f     //1.0f
#define motor_3_SPEED_PID_KD        2.0f       //5.0f
#define motor_3_SPEED_PID_MAX_OUT   30000.0f   //30000.0f
#define motor_3_SPEED_PID_MAX_IOUT  10000.f  //10000.0f
#define motor_3_CURRENT_PID_KP        0.7f
#define motor_3_CURRENT_PID_KI        0.2f
#define motor_3_CURRENT_PID_KD        0.0f
#define motor_3_CURRENT_PID_MAX_OUT   30000.0f     //30000.0f
#define motor_3_CURRENT_PID_MAX_IOUT  10000.0f   //10000.0f

//4ºÅÄ¦²ÁÂÖ
#define motor_4_SPEED_PID_KP        45.0f   //25.0f
#define motor_4_SPEED_PID_KI        3.0f     //1.0f
#define motor_4_SPEED_PID_KD        5.0f       //5.0f
#define motor_4_SPEED_PID_MAX_OUT   30000.0f   //30000.0f
#define motor_4_SPEED_PID_MAX_IOUT  10000.f  //10000.0f
#define motor_4_CURRENT_PID_KP        0.7f
#define motor_4_CURRENT_PID_KI        0.2f
#define motor_4_CURRENT_PID_KD        0.0f
#define motor_4_CURRENT_PID_MAX_OUT   30000.0f     //30000.0f
#define motor_4_CURRENT_PID_MAX_IOUT  10000.0f   //10000.0f


typedef enum
{
	SHOOT_STOP=0,
	SHOOT_BULLET,
}shoot_mode_e;
 typedef struct
{
	shoot_mode_e shoot_mode;
	const motor_measure_t *friction_motor_measure[4];
	const remote_data_t *shoot_rc;
	pid_type_def friction_motor_speed_pid; 
	pid_type_def friction_motor_current_pid; 
	pid_type_def motor_1_speed_pid; 
	pid_type_def motor_1_current_pid;
	pid_type_def motor_2_speed_pid; 
	pid_type_def motor_2_current_pid;
	pid_type_def motor_3_speed_pid; 
	pid_type_def motor_3_current_pid;
	pid_type_def motor_4_speed_pid; 
	pid_type_def motor_4_current_pid;
    fp32 friction_1_speed;
	fp32 friction_2_speed;
	fp32 friction_3_speed;
	fp32 friction_4_speed;
    fp32 friction_speed_set;
	fp32 friction_speed_set_1;
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
extern float data1[6];
#endif