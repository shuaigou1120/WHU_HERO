/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       test_task.c/h
  * @brief      buzzer warning task.蜂鸣器报警任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "test_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_buzzer.h"
#include "detect_task.h"
#include "vofa.h"
#include "chassis_task.h"
#include "CAN_receive.h"
#include "shoot.h"
#include "gimbal_task.h"
#include "ui.h"
#include "referee.h"
#include "CAN_receive.h"
static void buzzer_warn_error(uint8_t num);

const error_t *error_list_test_local;

extern __IO uint32_t uwTick;
uint32_t sendTick =0;

/**
  * @brief          test task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          test任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void test_task(void const * argument)
{

//    static uint8_t error, last_error;
//    static uint8_t error_num;
//    error_list_test_local = get_error_list_point();
	
    while(1)
    {
	
		//机器人id获取
		ui_self_id = (int)get_robot_id();

		//ui初始化
		ui_init_MODE_mode();
		ui_init_MODE_text();
		ui_init_pos_static();
		
		//底盘模式颜色改变
		if(chassis_move.chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
		{
			ui_MODE_mode_follow->color = 1;
		}
		else if(chassis_move.chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
		{
			ui_MODE_mode_top->color = 5;
		}
	
		//自瞄模式颜色改变
		if(shoot_control.shoot_control_data->selfaim_mode==1)
		{
			ui_MODE_mode_aim->color = 1;
		}
		
		//底盘相对位置改变
		if(chassis_move.chassis_relative_angle+6<0)
		{
			ui_MODE_mode_direction->start_angle=-(chassis_move.chassis_relative_angle+6)+35;
			if(chassis_move.chassis_relative_angle+6<-35)
			{
			ui_MODE_mode_direction->end_angle =-(chassis_move.chassis_relative_angle+6)-35;
			}
			else
			{
		    ui_MODE_mode_direction->end_angle =-(chassis_move.chassis_relative_angle+6)+325;
			}
		}
		
		if(chassis_move.chassis_relative_angle+6>0)
		{
			ui_MODE_mode_direction->end_angle =-(chassis_move.chassis_relative_angle+6)+325;
			if(chassis_move.chassis_relative_angle+6>35)
			{
			ui_MODE_mode_direction->start_angle=-(chassis_move.chassis_relative_angle+6)+395;
			}
			else
			{
		    ui_MODE_mode_direction->start_angle=-(chassis_move.chassis_relative_angle+6)+35;
			}
		}
		
		ui_update_MODE_mode();
		ui_update_MODE_text();
		ui_update_pos_static();
		
	     osDelay(1);
    }

}


/**
  * @brief          make the buzzer sound
  * @param[in]      num: the number of beeps 
  * @retval         none
  */
/**
  * @brief          使得蜂鸣器响
  * @param[in]      num:响声次数
  * @retval         none
  */
static void buzzer_warn_error(uint8_t num)
{
    static uint8_t show_num = 0;
    static uint8_t stop_num = 100;
    if(show_num == 0 && stop_num == 0)
    {
        show_num = num;
        stop_num = 100;
    }
    else if(show_num == 0)
    {
        stop_num--;
        buzzer_off();
    }
    else
    {
        static uint8_t tick = 0;
        tick++;
        if(tick < 50)
        {
            buzzer_off();
        }
        else if(tick < 100)
        {
            buzzer_on(1, 30000);
        }
        else
        {
            tick = 0;
            show_num--;
        }
    }
}


