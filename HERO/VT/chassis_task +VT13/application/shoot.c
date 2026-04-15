#include "main.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "shoot.h"
#include "pid.h"
#include "referee.h"
#include "detect_task.h"

shoot_control_t shoot_control;
uint16_t shootflag=0;
uint16_t anti_jamming_flag=0;
uint8_t  self_shoot=0;
uint32_t shoot_Tick=0;
uint32_t Tick=0;
uint8_t  shoot_time_flag=0;
uint8_t  time_flag=0;

uint8_t  shoot_mode=0;

uint16_t flag=0;

float shoot_data[4];
static void shoot_feedback_update(void)
{
	shoot_control.angle=shoot_control.shoot_motor_measure->angle;
	
	//发弹逻辑判断
	//上一次状态为1，现在状态为0，即发弹
//	shoot_control.last_press_l=shoot_control.press_l;
//	shoot_control.press_l=(shoot_control.shoot_control_data->shoot_mode>>1) & 0x01;
	
	//发弹逻辑判断
	shoot_control.last_press_l=shoot_control.press_l;
	shoot_control.press_l=shoot_control.shoot_control_data->shoot_mode;
	
	shoot_control.shoot_friction_mode=shoot_control.shoot_control_data->shoot_mode&0x01; 
	get_power_shooter_output(&shoot_control.shooter_output);
//	shoot_control.last_press_tuidan=shoot_control.press_tuidan;
//	shoot_control.press_tuidan=shoot_control.shoot_control_data->tuidan_mode;
	
	//自瞄逻辑判断
	shoot_control.last_selfaim_mode=shoot_control.selfaim_mode;
	shoot_control.selfaim_mode=shoot_control.shoot_control_data->selfaim_shoot_mode;
	
	get_42mm_allowance(&shoot_control.allowance_42mm);
	
}
void shoot_init()
{
	static const fp32 Trigger_speed_pid[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};
	static const fp32 Trigger_angle_pid[3]={TRIGGER_ANGLE_PID_KP ,TRIGGER_ANGLE_PID_KI ,TRIGGER_ANGLE_PID_KD };
	
	static const fp32 tuidan_Trigger_speed_pid[3] = {tuidan_TRIGGER_SPEED_PID_KP, tuidan_TRIGGER_SPEED_PID_KI, tuidan_TRIGGER_SPEED_PID_KD};
	static const fp32 tuidan_Trigger_angle_pid[3]={tuidan_TRIGGER_ANGLE_PID_KP ,tuidan_TRIGGER_ANGLE_PID_KI ,tuidan_TRIGGER_ANGLE_PID_KD };
	
	shoot_control.shoot_motor_measure=get_trigger_motor_measure_point();
	shoot_control.shoot_control_data=get_Chassisdata_point();
	PID_init(&shoot_control.trigger_motor_speed_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_BULLET_PID_MAX_OUT, TRIGGER_BULLET_PID_MAX_IOUT);
	PID_init(&shoot_control.trigger_motor_angle_pid,PID_POSITION,Trigger_angle_pid,TRIGGER_ANGLE_PID_MAX_OUT ,TRIGGER_ANGLE_PID_MAX_IOUT);
	
	PID_init(&shoot_control.tuidan_trigger_motor_speed_pid, PID_POSITION, tuidan_Trigger_speed_pid, tuidan_TRIGGER_BULLET_PID_MAX_OUT, tuidan_TRIGGER_BULLET_PID_MAX_IOUT);
	PID_init(&shoot_control.tuidan_trigger_motor_angle_pid,PID_POSITION,tuidan_Trigger_angle_pid,tuidan_TRIGGER_ANGLE_PID_MAX_OUT ,tuidan_TRIGGER_ANGLE_PID_MAX_IOUT);
	
	shoot_control.angle =shoot_control.shoot_motor_measure->angle;
	shoot_control.set_angle = shoot_control.angle;
	shoot_control.speed = 0.0f;
    shoot_control.speed_set=0.0f ;
	shoot_Tick=uwTick;
	Tick=uwTick;
	
}

static void shoot_set_mode(void)
{
	if(shoot_control.shoot_control_data->shoot_mode==1)
	{
		shoot_control.shoot_mode=SHOOT_BULLET;
		
	}
//	else if(shoot_control.shoot_control_data->shoot_mode==3)
//	{
//		shoot_control.shoot_mode=SHOOT_STOP;
//		
//	}
}
void shoot_control_set()
{
//	if(shoot_control.shoot_mode==SHOOT_BULLET&&shootflag==0)
//	{
//		shoot_control.set_angle-=1140;
//		shootflag=0;

//		
//	}
//	else if(shoot_control.shoot_mode==SHOOT_STOP&&shootflag==1)
//	{
//		shoot_control.set_angle-=0;
//		shootflag=0;

//	}
	
	//防卡弹处理
	if((shoot_control.set_angle-shoot_control.angle)>2000)
	{
		anti_jamming_flag=1;
		shoot_control.set_angle=shoot_control.angle;
	}
	else
	{
		anti_jamming_flag=0;
	}
	
	//热量限制处理
	if(shoot_control.heat_42mm<shoot_control.heat_limit_42mm-100)
	{
		flag=1;
	}
	else
	{
		flag=0;
	}
	

	
	//手瞄判断
	if(shoot_control.shoot_control_data->selfaim_mode==0)
		
	{
		//判断手瞄射击时间
		if(uwTick-Tick>500)
		{
			shoot_time_flag=1;
			
			if(time_flag==1)
			{
			
				Tick=uwTick;
				time_flag=0;
			}
			
		}
		else
		{
			shoot_time_flag=0;
		}
		
		
		if((shoot_control.last_press_l==0&&shoot_control.press_l==1)&&anti_jamming_flag==0&&shoot_control.shoot_control_data->ejection_flag==0&&flag==1&&shoot_time_flag==1&&shoot_control.shoot_control_data->shoot_flag==1)
	{
		
		shoot_control.set_angle+=1140;//3508减速比1:19
		shoot_mode=2;
		time_flag=1;
		
		
	}

		else if ((shoot_control.last_press_l==0&&shoot_control.press_l==1)&&anti_jamming_flag==0&&shoot_control.shoot_control_data->ejection_flag==1)
	{
		shoot_control.set_angle-=38;//3508减速比1:19
		shoot_mode=1;
	}
	
	}
	
	//自瞄判断
	else if (shoot_control.shoot_control_data->selfaim_mode==1)
	
	{ 
		if (shoot_control.selfaim_mode==2)
//	else if (shoot_control.press_l==1)
	{
		
		shoot_mode=2;
		//判断自瞄射击时间
		if(uwTick-shoot_Tick>800)
		{
			self_shoot=1;
			
			if(time_flag==1)
			{
				shoot_Tick=uwTick;
				time_flag=0;
			}
			
		}
		else
		{
			self_shoot=0;
		}
		
		
		if(self_shoot==1&&anti_jamming_flag==0&&(shoot_control.press_l==1)&&shoot_control.shoot_control_data->shoot_flag==1)
		{
			shoot_control.set_angle+=1140;//3508减速比1:19
			time_flag=1;
		}
	
	}
	
}

	

}
void shoot_control_loop()
{
	  
//	   if(shoot_mode==2)
//	   {
//		PID_calc(&shoot_control.trigger_motor_angle_pid,shoot_control.angle,shoot_control.set_angle);
//	    PID_calc(&shoot_control.trigger_motor_speed_pid, shoot_control.shoot_motor_measure->speed_rpm, shoot_control.trigger_motor_angle_pid.out);
//		shoot_control.given_current=(int16_t)(shoot_control.trigger_motor_speed_pid.out);
//	   }
	   if(shoot_mode==1)
	   {
		PID_calc(&shoot_control.tuidan_trigger_motor_angle_pid, shoot_control.angle,shoot_control.set_angle);
	    PID_calc(&shoot_control.tuidan_trigger_motor_speed_pid, shoot_control.shoot_motor_measure->speed_rpm, shoot_control.tuidan_trigger_motor_angle_pid.out);
		shoot_control.given_current=(int16_t)(shoot_control.tuidan_trigger_motor_speed_pid.out);
	   }

	   else 
	   {
		PID_calc(&shoot_control.trigger_motor_angle_pid,shoot_control.angle,shoot_control.set_angle);
	    PID_calc(&shoot_control.trigger_motor_speed_pid, shoot_control.shoot_motor_measure->speed_rpm, shoot_control.trigger_motor_angle_pid.out);
		shoot_control.given_current=(int16_t)(shoot_control.trigger_motor_speed_pid.out);
	   }
}
shoot_control_t *shoot()
{
//		shoot_data[0]=shoot_control.shoot_motor_measure->speed_rpm;
//	    shoot_data[1]=0;
//	    shoot_data[2]=shoot_control.angle;
//	    shoot_data[3]=shoot_control.set_angle;
		shoot_feedback_update();
		shoot_set_mode();
	    get_42mm_shoot_heat_limit_and_heat(&shoot_control.heat_limit_42mm,&shoot_control.heat_42mm);
		shoot_control_set();
		shoot_control_loop();
		return &shoot_control;
	
}