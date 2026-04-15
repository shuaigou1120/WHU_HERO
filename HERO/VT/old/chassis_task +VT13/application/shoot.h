#ifndef SHOOT_H
#define SHOOT_H
#include "CAN_receive.h"
#include "pid.h"
#define MOTOR_ECD_TO_ANGLE          0.00007666
#define FULL_COUNT                  9.5
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
//·¢µ¯PID
#define TRIGGER_SPEED_PID_KP        25.0f//25
#define TRIGGER_SPEED_PID_KI        0.15f//0.15
#define TRIGGER_SPEED_PID_KD        3.0f
#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f
#define TRIGGER_ANGLE_PID_KP        4.0f
#define TRIGGER_ANGLE_PID_KI        0.0f
#define TRIGGER_ANGLE_PID_KD        0.6f
#define TRIGGER_ANGLE_PID_MAX_OUT  10000.0f
#define TRIGGER_ANGLE_PID_MAX_IOUT 9000.0f

//ÍËµ¯PID
#define tuidan_TRIGGER_SPEED_PID_KP        25.0f//25
#define tuidan_TRIGGER_SPEED_PID_KI        0.15f//0.15
#define tuidan_TRIGGER_SPEED_PID_KD        3.0f
#define tuidan_TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define tuidan_TRIGGER_BULLET_PID_MAX_IOUT 9000.0f
#define tuidan_TRIGGER_ANGLE_PID_KP        1.0f
#define tuidan_TRIGGER_ANGLE_PID_KI        0.0f
#define tuidan_TRIGGER_ANGLE_PID_KD        0.5f
#define tuidan_TRIGGER_ANGLE_PID_MAX_OUT  10000.0f
#define tuidan_TRIGGER_ANGLE_PID_MAX_IOUT 9000.0f


typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_BULLET,
} shoot_mode_e;
typedef struct
{
	shoot_mode_e shoot_mode;
	const motor_measure_t *shoot_motor_measure;
	const chassis_data_t *shoot_control_data;
	pid_type_def trigger_motor_speed_pid;
	pid_type_def trigger_motor_angle_pid;
	
	pid_type_def tuidan_trigger_motor_speed_pid;
	pid_type_def tuidan_trigger_motor_angle_pid;
	
	uint16_t heat_limit_42mm;
	uint16_t heat_42mm;
	fp32 trigger_speed_set;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int32_t ecd_count;
	bool_t last_press_l;
	bool_t press_l;
	uint8_t shoot_friction_mode;
	uint8_t shooter_output;
	bool_t last_press_tuidan;
	bool_t press_tuidan;
	bool_t last_selfaim_mode;
	bool_t selfaim_mode;
}shoot_control_t;
extern shoot_control_t *shoot();
extern void shoot_init();
extern float shoot_data[4];
#endif