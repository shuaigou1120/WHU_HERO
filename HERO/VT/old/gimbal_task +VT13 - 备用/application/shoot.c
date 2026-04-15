#include "main.h"
#include "shoot.h"
#include "bsp_laser.h"
#include "bsp_delay.h"
 float data1[6];
shoot_control_t shoot_control;
int shoot_flag=0;
void shoot_Init()
{
//static const fp32 friction_speed_pid[3]={FRICTION_SPEED_PID_KP ,FRICTION_SPEED_PID_KI,FRICTION_SPEED_PID_KD};
//static const fp32 friction_current_pid[3]={FRICTION_CURRENT_PID_KP,FRICTION_CURRENT_PID_KI,FRICTION_CURRENT_PID_KD};
	//1ºÅÄ¦²ÁÂÖPID³õÊ¼»¯
	static const fp32 motor_1_speed_pid[3]={motor_1_SPEED_PID_KP ,motor_1_SPEED_PID_KI,motor_1_SPEED_PID_KD};
	static const fp32 motor_1_current_pid[3]={motor_1_CURRENT_PID_KP,motor_1_CURRENT_PID_KI,motor_1_CURRENT_PID_KD};
//	PID_init(&shoot_control.friction_motor_speed_pid,PID_POSITION,friction_speed_pid,FRICTION_SPEED_PID_MAX_OUT,FRICTION_SPEED_PID_MAX_IOUT);
//	PID_init(&shoot_control.friction_motor_current_pid,PID_POSITION,friction_current_pid,FRICTION_CURRENT_PID_MAX_OUT,FRICTION_CURRENT_PID_MAX_IOUT);
	PID_init(&shoot_control.motor_1_speed_pid,PID_POSITION,motor_1_speed_pid,motor_1_SPEED_PID_MAX_OUT,motor_1_SPEED_PID_MAX_IOUT);
	PID_init(&shoot_control.motor_1_current_pid,PID_POSITION,motor_1_current_pid,motor_1_CURRENT_PID_MAX_OUT,motor_1_CURRENT_PID_MAX_IOUT);
	
	//2ºÅÄ¦²ÁÂÖPID³õÊ¼»¯
	static const fp32 motor_2_speed_pid[3]={motor_2_SPEED_PID_KP ,motor_2_SPEED_PID_KI,motor_2_SPEED_PID_KD};
	static const fp32 motor_2_current_pid[3]={motor_2_CURRENT_PID_KP,motor_2_CURRENT_PID_KI,motor_2_CURRENT_PID_KD};
	PID_init(&shoot_control.motor_2_speed_pid,PID_POSITION,motor_2_speed_pid,motor_2_SPEED_PID_MAX_OUT,motor_2_SPEED_PID_MAX_IOUT);
	PID_init(&shoot_control.motor_2_current_pid,PID_POSITION,motor_2_current_pid,motor_2_CURRENT_PID_MAX_OUT,motor_2_CURRENT_PID_MAX_IOUT);
	
	//3ºÅÄ¦²ÁÂÖPID³õÊ¼»¯
	static const fp32 motor_3_speed_pid[3]={motor_3_SPEED_PID_KP ,motor_3_SPEED_PID_KI,motor_3_SPEED_PID_KD};
	static const fp32 motor_3_current_pid[3]={motor_3_CURRENT_PID_KP,motor_3_CURRENT_PID_KI,motor_3_CURRENT_PID_KD};
	PID_init(&shoot_control.motor_3_speed_pid,PID_POSITION,motor_3_speed_pid,motor_3_SPEED_PID_MAX_OUT,motor_3_SPEED_PID_MAX_IOUT);
	PID_init(&shoot_control.motor_3_current_pid,PID_POSITION,motor_3_current_pid,motor_3_CURRENT_PID_MAX_OUT,motor_3_CURRENT_PID_MAX_IOUT);
	
	//4ºÅÄ¦²ÁÂÖPID³õÊ¼»¯
	static const fp32 motor_4_speed_pid[3]={motor_4_SPEED_PID_KP ,motor_4_SPEED_PID_KI,motor_4_SPEED_PID_KD};
	static const fp32 motor_4_current_pid[3]={motor_4_CURRENT_PID_KP,motor_4_CURRENT_PID_KI,motor_4_CURRENT_PID_KD};
	PID_init(&shoot_control.motor_4_speed_pid,PID_POSITION,motor_4_speed_pid,motor_4_SPEED_PID_MAX_OUT,motor_4_SPEED_PID_MAX_IOUT);
	PID_init(&shoot_control.motor_4_current_pid,PID_POSITION,motor_4_current_pid,motor_4_CURRENT_PID_MAX_OUT,motor_4_CURRENT_PID_MAX_IOUT);
	
	shoot_control.shoot_mode=SHOOT_STOP;
	shoot_control.shoot_rc = get_remote_control_point1();
//	shoot_control.friction_motor_measure[0]=get_left_friction_motor_measure_point();
//	shoot_control.friction_motor_measure[1]=get_right_friction_motor_measure_point();
	shoot_control.friction_motor_measure[0]=get_chassis_motor_measure_point(0);
	shoot_control.friction_motor_measure[1]=get_chassis_motor_measure_point(1);
	shoot_control.friction_motor_measure[2]=get_chassis_motor_measure_point(2);
	shoot_control.friction_motor_measure[3]=get_chassis_motor_measure_point(3);
}
void shoot_feedback_update()
{
	
	//Êó±ê¿ØÖÆ
	if(shoot_control.shoot_rc->rc.mode_sw==2)
	{
	shoot_control.last_press_l=shoot_control.press_l;
	shoot_control.last_press_r=shoot_control.press_r;
	shoot_control.press_l=shoot_control.shoot_rc->mouse.mouse_left;
	shoot_control.press_r=shoot_control.shoot_rc->mouse.mouse_right;
	}
	
	if(shoot_control.shoot_rc->rc.mode_sw==1)
	{
	shoot_control.last_press_r=shoot_control.press_r;
	shoot_control.press_r=shoot_control.shoot_rc->rc.fn_2;
	}
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
/*	if(shoot_control.press_r&&shoot_control.last_press_r==0)
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
*/
if((shoot_control.last_press_r==0)&&(shoot_control.press_r==1))
{
	shoot_flag=!shoot_flag;
}
//if(shoot_control.shoot_rc->rc.s[1]==3)
if(shoot_flag==1)
{
	shoot_control.shoot_mode=SHOOT_BULLET;
}
if(shoot_flag==0)
{
	shoot_control.shoot_mode=SHOOT_STOP;
}
	
}
void friction_control_set()
{
	if(shoot_control.shoot_mode==SHOOT_BULLET)
	{
		
		shoot_control.friction_speed_set=5000;
		shoot_control.friction_speed_set_1=3750;
		laser_on();
		
	}
	else 
	{
		shoot_control.friction_speed_set=0;
		shoot_control.friction_speed_set_1=0;
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

//    //²¦µ¯ÂÖµç»úËÙ¶ÈÂË²¨Ò»ÏÂ
//    static const fp32 fliter_num[2] = {0.7,0.3};

//    //¶þ½×µÍÍ¨ÂË²¨
//    speed_fliter_1 = speed_fliter_2;
//    speed_fliter_2 = shoot_control.friction_motor_measure[0]->speed_rpm;
//    speed_fliter_3 = fliter_num[0]*speed_fliter_2+fliter_num[1]*speed_fliter_1;
//    shoot_control.friction_left_speed = speed_fliter_3;
//	  speed_fliter_1 = speed_fliter_2;
//    speed_fliter_2 = shoot_control.friction_motor_measure[1]->speed_rpm;
//    speed_fliter_3 = fliter_num[0]*speed_fliter_2+fliter_num[1]*speed_fliter_1;
//    shoot_control.friction_right_speed = speed_fliter_3;
	/*static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
	speed_fliter_1=speed_fliter_2;
	speed_fliter_2=shoot_control.friction_motor_measure[0]->speed_rpm;
	if(speed_fliter_2==511)
	{
		shoot_control.friction_1_speed=speed_fliter_1;
	}
	else
	{
		shoot_control.friction_2_speed=speed_fliter_2;
	}*/
	shoot_control.friction_1_speed=shoot_control.friction_motor_measure[0]->speed_rpm;
	shoot_control.friction_2_speed=shoot_control.friction_motor_measure[1]->speed_rpm;
	shoot_control.friction_3_speed=shoot_control.friction_motor_measure[2]->speed_rpm;
	shoot_control.friction_4_speed=shoot_control.friction_motor_measure[3]->speed_rpm;
	
}
const shoot_control_t *shoot_control_loop()
{
		shoot_Init();
		shoot_feedback_update();
		shoot_set_mode();
		friction_control_set();
		shoot_speed_filter();
//	PID_calc(&shoot_control.friction_motor_speed_pid,shoot_control.friction_motor_measure[0]->speed_rpm,shoot_control.friction_speed_set);
	PID_calc(&shoot_control.motor_1_speed_pid,shoot_control.friction_1_speed,-shoot_control.friction_speed_set);
	shoot_control.shoot_1_given_current=PID_calc(&shoot_control.motor_1_current_pid,shoot_control.friction_motor_measure[0]->given_current,shoot_control.motor_1_speed_pid.out);
	data1[0]=-3375;
	data1[1]=-4500;
//	PID_calc(&shoot_control.friction_motor_speed_pid,shoot_control.friction_motor_measure[1]->speed_rpm,-shoot_control.friction_speed_set);
	PID_calc(&shoot_control.motor_2_speed_pid,shoot_control.friction_2_speed,shoot_control.friction_speed_set);
	shoot_control.shoot_2_given_current=PID_calc(&shoot_control.motor_2_current_pid,shoot_control.friction_motor_measure[1]->given_current,shoot_control.motor_2_speed_pid.out);

	data1[2]=3375;
	data1[3]=4500;
	
	data1[4]=shoot_control.friction_3_speed;
	data1[5]=shoot_control.friction_4_speed;
	
	PID_calc(&shoot_control.motor_3_speed_pid,shoot_control.friction_3_speed,-shoot_control.friction_speed_set_1);
	shoot_control.shoot_3_given_current=PID_calc(&shoot_control.motor_3_current_pid,shoot_control.friction_motor_measure[2]->given_current,shoot_control.motor_3_speed_pid.out);
	
	
	PID_calc(&shoot_control.motor_4_speed_pid,shoot_control.friction_4_speed,shoot_control.friction_speed_set_1);
	shoot_control.shoot_4_given_current=PID_calc(&shoot_control.motor_4_current_pid,shoot_control.friction_motor_measure[3]->given_current,shoot_control.motor_4_speed_pid.out);
	

	return &shoot_control;
		
}