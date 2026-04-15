#include "main.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "pid.h"
#include "gimbal_task.h"
#include "shoot.h"
#include "arm_math.h"
#include "user_lib.h"
#include "tim.h"
#include "vofa.h"
#include "usart.h"
gimbal_control_t gimbal_control;

float gimbal_data[4];

float yaw_angle_limit_func(float input) 
{
    if (input >= 180) {
        return input - 360;
    } else if (input <= -180) {
        return input + 360;
    } else {
        return input;
    }
}	





/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     gimbal_init:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd, fp32 kf)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kf = kf;
	
    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

 fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta, fp32 setgryo)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
	
    pid->err = err;
	//pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
	pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout+pid->kf*setgryo;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}
void gimbal_Init(gimbal_control_t *gimbal_control)
{
	static const fp32 Yaw_gyro_pid[3] = {YAW_GYRO_PID_KP, YAW_GYRO_PID_KI, YAW_GYRO_PID_KD};
	static const fp32 Yaw_angle_pid[3]	={YAW_ANGLE_PID_KP, YAW_ANGLE_PID_KI, YAW_ANGLE_PID_KD};
	gimbal_control->yaw_ctrl_data=get_Chassisdata_point();
	gimbal_control->gimbal_yaw_motor.gimbal_motor_mode=GIMBAL_MOTOR_OFF;
	gimbal_control->gimbal_yaw_motor.gimbal_motor_measure=get_yaw_gimbal_motor_measure_point();
	gimbal_control->gimbal_yaw_motor.absolute_angle_set=gimbal_control->gimbal_yaw_motor.absolute_angle; 
	gimbal_control->gimbal_yaw_motor.absolute_angle_set1=rad_format(gimbal_control->gimbal_yaw_motor.absolute_angle_set);
//	PID_init(&gimbal_control->gimbal_yaw_motor.gimbal_motor_angle_pid,PID_POSITION,Yaw_angle_pid,YAW_ANGLE_PID_MAX_OUT,YAW_ANGLE_PID_MAX_IOUT);
//	PID_init(&gimbal_control->gimbal_yaw_motor.gimbal_motor_gyro_pid,PID_POSITION,Yaw_gyro_pid,YAW_GYRO_PID_MAX_OUT,YAW_GYRO_PID_MAX_IOUT);
	PID_init(&gimbal_control->gimbal_yaw_motor.gimbal_motor_gyro_pid,PID_POSITION,Yaw_gyro_pid,YAW_GYRO_PID_MAX_OUT,YAW_GYRO_PID_MAX_IOUT);
	gimbal_PID_init(&gimbal_control->gimbal_yaw_motor.gimbal_motor_angle1_pid,YAW_ANGLE_PID_MAX_OUT,YAW_ANGLE_PID_MAX_IOUT,YAW_ANGLE_PID_KP,YAW_ANGLE_PID_KI, YAW_ANGLE_PID_KD,YAW_KF);
}
void gimbal_set_mode(gimbal_control_t *gimbal_control)
{
	if(gimbal_control->yaw_ctrl_data->chassis_mode==0)
	{
		gimbal_control->gimbal_yaw_motor.gimbal_motor_mode=GIMBAL_MOTOR_OFF;
	}
	else if(gimbal_control->yaw_ctrl_data->chassis_mode==1)
	{
		gimbal_control->gimbal_yaw_motor.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
	}
	else if(gimbal_control->yaw_ctrl_data->chassis_mode==2)
	{
		gimbal_control->gimbal_yaw_motor.gimbal_motor_mode=GIMBAL_MOTOR_ROTATE;
	}
}
void gimbal_feedback_update(gimbal_control_t *gimbal_control)
{
	gimbal_control->gimbal_yaw_motor.absolute_angle=gimbal_control->yaw_ctrl_data->yaw_angle;
	gimbal_control->gimbal_yaw_motor.motor_gyro=gimbal_control->yaw_ctrl_data->yaw_gyro;

	
}
void gimbal_set_control(gimbal_control_t *gimbal_control)
{
	const static fp32 yaw_angle_set_order_filter[1] = {YAW_SET_NUM};
	if(gimbal_control->gimbal_yaw_motor.gimbal_motor_mode==GIMBAL_MOTOR_GYRO)
	{
		gimbal_control->gimbal_yaw_motor.absolute_angle_set=gimbal_control->yaw_ctrl_data->yaw_angle_set;
		
	}
	else if(gimbal_control->gimbal_yaw_motor.gimbal_motor_mode==GIMBAL_MOTOR_OFF)
	{
//		gimbal_control->gimbal_yaw_motor.absolute_angle_set=gimbal_control->gimbal_yaw_motor.absolute_angle;
	}
	else if(gimbal_control->gimbal_yaw_motor.gimbal_motor_mode==GIMBAL_MOTOR_ROTATE)
	{
		gimbal_control->gimbal_yaw_motor.absolute_angle_set=gimbal_control->yaw_ctrl_data->yaw_angle_set;
	}
	
}
void gimbal_control_loop(gimbal_control_t *gimbal_control)
{
	if(gimbal_control->gimbal_yaw_motor.gimbal_motor_mode==GIMBAL_MOTOR_OFF)
	{
		gimbal_control->gimbal_yaw_motor.yaw_given_current=0;
	}
	else if(gimbal_control->gimbal_yaw_motor.gimbal_motor_mode==GIMBAL_MOTOR_GYRO)
	{
		gimbal_PID_calc(&gimbal_control->gimbal_yaw_motor.gimbal_motor_angle1_pid,gimbal_control->gimbal_yaw_motor.absolute_angle,gimbal_control->gimbal_yaw_motor.absolute_angle_set,gimbal_control->gimbal_yaw_motor.motor_gyro,YAW_SETGYRO);
//		PID_calc(&gimbal_control->gimbal_yaw_motor.gimbal_motor_angle_pid,gimbal_control->gimbal_yaw_motor.absolute_angle,gimbal_control->gimbal_yaw_motor.absolute_angle_set);
//		PID_calc(&gimbal_control->gimbal_yaw_motor.gimbal_motor_gyro_pid,gimbal_control->gimbal_yaw_motor.motor_gyro,gimbal_control->gimbal_yaw_motor.gimbal_motor_angle1_pid.out);
        PID_calc(&gimbal_control->gimbal_yaw_motor.gimbal_motor_gyro_pid,gimbal_control->gimbal_yaw_motor.motor_gyro,gimbal_control->gimbal_yaw_motor.gimbal_motor_angle1_pid.out);
		gimbal_control->gimbal_yaw_motor.yaw_given_current=gimbal_control->gimbal_yaw_motor.gimbal_motor_gyro_pid.out;
	}
	else if(gimbal_control->gimbal_yaw_motor.gimbal_motor_mode==GIMBAL_MOTOR_ROTATE)
	{
////		PID_calc(&gimbal_control->gimbal_yaw_motor.gimbal_motor_gyro_pid,gimbal_control->gimbal_yaw_motor.motor_gyro,gimbal_control->gimbal_yaw_motor.motor_gyro_set);
////		gimbal_control->gimbal_yaw_motor.yaw_given_current=gimbal_control->gimbal_yaw_motor.gimbal_motor_gyro_pid.out;
//		gimbal_PID_calc(&gimbal_control->gimbal_yaw_motor.gimbal_motor_angle1_pid,gimbal_control->gimbal_yaw_motor.absolute_angle,gimbal_control->gimbal_yaw_motor.absolute_angle_set,gimbal_control->gimbal_yaw_motor.motor_gyro,YAW_SETGYRO);
////		PID_calc(&gimbal_control->gimbal_yaw_motor.gimbal_motor_angle_pid,gimbal_control->gimbal_yaw_motor.absolute_angle,gimbal_control->gimbal_yaw_motor.absolute_angle_set);
//		PID_calc(&gimbal_control->gimbal_yaw_motor.gimbal_motor_gyro_pid,gimbal_control->gimbal_yaw_motor.motor_gyro,gimbal_control->gimbal_yaw_motor.gimbal_motor_angle1_pid.out);
//		gimbal_control->gimbal_yaw_motor.yaw_given_current=gimbal_control->gimbal_yaw_motor.gimbal_motor_gyro_pid.out;
		
		 gimbal_PID_calc(&gimbal_control->gimbal_yaw_motor.gimbal_motor_angle1_pid,gimbal_control->gimbal_yaw_motor.absolute_angle,gimbal_control->gimbal_yaw_motor.absolute_angle_set,gimbal_control->gimbal_yaw_motor.motor_gyro,YAW_SETGYRO);
//		PID_calc(&gimbal_control->gimbal_yaw_motor.gimbal_motor_angle_pid,gimbal_control->gimbal_yaw_motor.absolute_angle,gimbal_control->gimbal_yaw_motor.absolute_angle_set);
//		PID_calc(&gimbal_control->gimbal_yaw_motor.gimbal_motor_gyro_pid,gimbal_control->gimbal_yaw_motor.motor_gyro,gimbal_control->gimbal_yaw_motor.gimbal_motor_angle1_pid.out);
        PID_calc(&gimbal_control->gimbal_yaw_motor.gimbal_motor_gyro_pid,gimbal_control->gimbal_yaw_motor.motor_gyro,gimbal_control->gimbal_yaw_motor.gimbal_motor_angle1_pid.out);
		gimbal_control->gimbal_yaw_motor.yaw_given_current=gimbal_control->gimbal_yaw_motor.gimbal_motor_gyro_pid.out;
	}
}
void gimbal_task()
{
	
	gimbal_Init(&gimbal_control);
	shoot_init();
	while(1)
	{
//		data[0]=-4;
//		data[1]=gimbal_control.gimbal_yaw_motor.motor_gyro;
//		data[0]=203;
//		data[1]=gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
		gimbal_data[0]=gimbal_control.gimbal_yaw_motor.absolute_angle;
		gimbal_data[1]=gimbal_control.gimbal_yaw_motor.absolute_angle_set;
	    gimbal_data[2]=gimbal_control.gimbal_yaw_motor.motor_gyro;
	    gimbal_data[3]=gimbal_control.gimbal_yaw_motor.gimbal_motor_angle1_pid.out;
		gimbal_set_mode(&gimbal_control);
		gimbal_feedback_update(&gimbal_control);
		gimbal_set_control(&gimbal_control);
		gimbal_control_loop(&gimbal_control);
		gimbal_control.shoot=shoot();
		CAN_cmd_gimbal(gimbal_control.gimbal_yaw_motor.yaw_given_current,0,gimbal_control.shoot->given_current,0);
//		CAN_cmd_gimbal(0,0,gimbal_control.shoot->given_current,0);
//		CAN_cmd_gimbal(gimbal_control.gimbal_yaw_motor.yaw_given_current,0,0,0);
		//CAN_cmd_gimbal(0,0,0,0);
		vTaskDelay(GIMBAL_CONTROL_TIME);

//	    sendfloatdata(gimbal_data,4);  
	}
}


