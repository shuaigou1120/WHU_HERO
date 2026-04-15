#include "main.h"
#include "shoot.h"
#include "bsp_laser.h"
float data1[2];
shoot_control_t shoot_control;
int shoot_flag=0;
void shoot_Init()
{
	static const fp32 friction_speed_pid[3]={FRICTION_SPEED_PID_KP ,FRICTION_SPEED_PID_KI,FRICTION_SPEED_PID_KD};
	static const fp32 friction_current_pid[3]={FRICTION_CURRENT_PID_KP,FRICTION_CURRENT_PID_KI,FRICTION_CURRENT_PID_KD};
	PID_init(&shoot_control.friction_motor_speed_pid,PID_POSITION,friction_speed_pid,FRICTION_SPEED_PID_MAX_OUT,FRICTION_SPEED_PID_MAX_IOUT);
	PID_init(&shoot_control.friction_motor_current_pid,PID_POSITION,friction_current_pid,FRICTION_CURRENT_PID_MAX_OUT,FRICTION_CURRENT_PID_MAX_IOUT);
	shoot_control.shoot_mode=SHOOT_STOP;
	shoot_control.shoot_rc = get_remote_control_point();
//	shoot_control.friction_motor_measure[0]=get_left_friction_motor_measure_point();
//	shoot_control.friction_motor_measure[1]=get_right_friction_motor_measure_point();
	shoot_control.friction_motor_measure[0]=get_chassis_motor_measure_point(1);
	shoot_control.friction_motor_measure[1]=get_chassis_motor_measure_point(0);
}
void shoot_feedback_update()
{
	shoot_control.last_press_l=shoot_control.press_l;
	shoot_control.last_press_r=shoot_control.press_r;
	shoot_control.press_l=shoot_control.shoot_rc->mouse.press_l;
	shoot_control.press_r=shoot_control.shoot_rc->mouse.press_r;
}
void shoot_set_mode()
{
//	if(shoot_control.shoot_rc->rc.s[1]==3||shoot_control.shoot_rc->rc.s[1]==1)
//	{
//		shoot_control.shoot_mode=SHOOT_BULLET;
//	}
//	else if(shoot_control.shoot_rc->rc.s[1]==2)
//	{
//		shoot_control.shoot_mode=SHOOT_STOP;
//	}
	if(shoot_control.press_r&&shoot_control.last_press_r==0)
	{
		shoot_flag=!shoot_flag;
	}
	if(shoot_flag==1||shoot_control.shoot_rc->rc.s[0]==1)
	{
		shoot_control.shoot_mode=SHOOT_BULLET;
	}
	else if(shoot_flag==0||shoot_control.shoot_rc->rc.s[0]==2)
	{
		shoot_control.shoot_mode=SHOOT_STOP;
	}

	
}
void friction_control_set()
{
	if(shoot_control.shoot_mode==SHOOT_BULLET)
	{
		shoot_control.friction_speed_set=8500;
		laser_on();
		
	}
	else 
	{
		shoot_control.friction_speed_set=0;
		laser_off();
	}
}
//void shoot_calc()
//{
//	if(shoot_control.shoot_mode==SHOOT_BULLET)
//	{
//		shoot_control.shoot_left_friction_given_current=PID_calc(&shoot_control.friction_left_motor_speed_pid,shoot_control.friction_motor_measure[0]->speed_rpm,shoot_control.friction_speed_set);
//		shoot_control.shoot_right_friction_given_current=PID_calc(&shoot_control.friction_right_motor_speed_pid,shoot_control.friction_motor_measure[1]->speed_rpm,-shoot_control.friction_speed_set);
//	}
//	else if(shoot_control.shoot_mode==SHOOT_STOP)
//	{
//		shoot_control.shoot_left_friction_given_current=0;
//		shoot_control.shoot_right_friction_given_current=0;
//	}
//}
void shoot_speed_filter()
{
//	static fp32 speed_fliter_1 = 0.0f;
//    static fp32 speed_fliter_2 = 0.0f;
//	static fp32 speed_fliter_3 = 0.0f;

//    //拨弹轮电机速度滤波一下
//    static const fp32 fliter_num[2] = {0.7,0.3};

//    //二阶低通滤波
//    speed_fliter_1 = speed_fliter_2;
//    speed_fliter_2 = shoot_control.friction_motor_measure[0]->speed_rpm;
//    speed_fliter_3 = fliter_num[0]*speed_fliter_2+fliter_num[1]*speed_fliter_1;
//    shoot_control.friction_left_speed = speed_fliter_3;
//	speed_fliter_1 = speed_fliter_2;
//    speed_fliter_2 = shoot_control.friction_motor_measure[1]->speed_rpm;
//    speed_fliter_3 = fliter_num[0]*speed_fliter_2+fliter_num[1]*speed_fliter_1;
//    shoot_control.friction_right_speed = speed_fliter_3;
	static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
	speed_fliter_1=speed_fliter_2;
	speed_fliter_2=shoot_control.friction_motor_measure[1]->speed_rpm;
	if(speed_fliter_2==511)
	{
		shoot_control.friction_right_speed=speed_fliter_1;
	}
	else
	{
		shoot_control.friction_right_speed=speed_fliter_2;
	}
	shoot_control.friction_left_speed=shoot_control.friction_motor_measure[0]->speed_rpm;
		
	
	
}
const shoot_control_t *shoot_control_loop()
{
		shoot_Init();
		shoot_feedback_update();
		shoot_set_mode();
		friction_control_set();
		shoot_speed_filter();
//	PID_calc(&shoot_control.friction_motor_speed_pid,shoot_control.friction_motor_measure[0]->speed_rpm,shoot_control.friction_speed_set);
	PID_calc(&shoot_control.friction_motor_speed_pid,shoot_control.friction_left_speed,shoot_control.friction_speed_set);
	shoot_control.shoot_left_given_current=PID_calc(&shoot_control.friction_motor_current_pid,shoot_control.friction_motor_measure[0]->given_current,shoot_control.friction_motor_speed_pid.out);
		data1[0]=shoot_control.friction_left_speed;
	data1[1]=shoot_control.friction_right_speed;
//	PID_calc(&shoot_control.friction_motor_speed_pid,shoot_control.friction_motor_measure[1]->speed_rpm,-shoot_control.friction_speed_set);
	PID_calc(&shoot_control.friction_motor_speed_pid,shoot_control.friction_right_speed,-shoot_control.friction_speed_set);
	shoot_control.shoot_right_given_current=PID_calc(&shoot_control.friction_motor_current_pid,shoot_control.friction_motor_measure[1]->given_current,shoot_control.friction_motor_speed_pid.out);
	return &shoot_control;
		
}