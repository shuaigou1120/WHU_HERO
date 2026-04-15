/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this 
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "INS_task.h"
#include "pid.h"
#include "shoot.h"
#include "Self_aim.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "tim.h"
#include "VT13_control.h"
#include "gryo.h"

/**
  * @brief          remote control dealline solve,because the value of rocker is not zero in middle place,
  * @param          input:the raw channel value 
  * @param          output: the processed channel value
  * @param          deadline
  */
/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
  * @param          输入的遥控器值
  * @param          输出的死区处理后遥控器值
  * @param          死区值
  */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }	
//motor enconde value format, range[0-8191]
//电机编码值规整 0—8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#define gimbal_total_pid_clear(gimbal_clear)                                                   \
    {                                                                                          \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                               \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
		                                                                                       \
		gimbal_PID_clear(&(gimbal_clear)->gimbal_video_motor.gimbal_motor_absolute_angle_pid); \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_video_motor.gimbal_motor_relative_angle_pid); \
        PID_clear(&(gimbal_clear)->gimbal_video_motor.gimbal_motor_gyro_pid);                  \
    }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;

#endif
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

#define WINDOW_SIZE 10
float yaw_buffer[WINDOW_SIZE];
int data_index = 0;

float sliding_average(float new_yaw) {
    yaw_buffer[data_index] = new_yaw;
    data_index = (data_index + 1) % WINDOW_SIZE;
    float sum = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) sum += yaw_buffer[i];
    return sum / WINDOW_SIZE;
}

/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     init:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init);

void chassis_rc_to_control_vector(gimbal_control_t *gimbal_control_set,chassis_data_t *chassis_data);
/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
  * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t *set_mode);
/**
  * @brief          gimbal some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     gimbal_feedback_update: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
  * @retval         none
//  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update);
/**
  * @brief          calculate the relative angle between ecd and offset_ecd
  * @param[in]      ecd: motor now encode
  * @param[in]      offset_ecd: gimbal offset encode
  * @retval         relative angle, unit rad
  */
/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
/**
  * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".         
  * @param[out]     gimbal_set_control: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
  * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control);
/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sent to motor
  * @param[out]     gimbal_control_loop: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_control_loop(gimbal_control_t *control_loop);

/**
  * @brief          gimbal angle pid init, because angle is in range(-pi,pi),can't use PID in pid.c
  * @param[out]     pid: pid data pointer stucture
  * @param[in]      maxout: pid max out
  * @param[in]      intergral_limit: pid max iout
  * @param[in]      kp: pid kp
  * @param[in]      ki: pid ki
  * @param[in]      kd: pid kd
  * @retval         none
  */
/**
  * @brief          云台角度PID初始化, 因为角度范围在(-pi,pi)，不能用PID.c的PID
  * @param[out]     pid:云台PID指针
  * @param[in]      maxout: pid最大输出
  * @param[in]      intergral_limit: pid最大积分输出
  * @param[in]      kp: pid kp
  * @param[in]      ki: pid ki
  * @param[in]      kd: pid kd
  * @retval         none
  */
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 intergral_limit, fp32 kp, fp32 ki, fp32 kd);

/**
  * @brief          gimbal PID clear, clear pid.out, iout.
  * @param[out]     pid_clear: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          云台PID清除，清除pid的out,iout
  * @param[out]     pid_clear:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_PID_clear(gimbal_PID_t *pid_clear);
/**
  * @brief          gimbal angle pid calc, because angle is in range(-pi,pi),can't use PID in pid.c
  * @param[out]     pid: pid data pointer stucture
  * @param[in]      get: angle feeback
  * @param[in]      set: angle set-point
  * @param[in]      error_delta: rotation speed
  * @retval         pid out
  */
/**
  * @brief          云台角度PID计算, 因为角度范围在(-pi,pi)，不能用PID.c的PID
  * @param[out]     pid:云台PID指针
  * @param[in]      get: 角度反馈
  * @param[in]      set: 角度设定
  * @param[in]      error_delta: 角速度
  * @retval         pid 输出
  */
static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);

static void gimbal_PID_clear(gimbal_PID_t *pid_clear);
#if GIMBAL_TEST_MODE
//j-scope 帮助pid调参
static void J_scope_gimbal_test(void);
#endif

//gimbal control data
//云台控制所有相关数据
gimbal_control_t gimbal_control;
uint8_t chassis_flag=0,turnflag=0,videoflag=0;

//底盘控制所有相关数据
chassis_data_t chassis_data;
InputData *Self_aim_data;
const motor_measure_t *a;
first_order_filter_type_t chassis_self_aim_yaw;
const static fp32 chassis_self_aim_yaw_filter[1] = {CHASSIS_ACCEL_Y_NUM};

//void servo_control(gimbal_control_t *servo);

//motor current 
//发送的电机电流
static int16_t yaw_can_set_current = 0, pitch_can_set_current = 0;
int16_t servoflag=0;
int16_t last_press;
int16_t press;

uint8_t mode_last_press;
uint8_t mode_press;

uint8_t selfaim_last_mode;
uint8_t selfaim_mode;

uint32_t time;

////VOFA+数组发送
//float gimbal_data[4];

/**
  * @brief          gimbal task, osDelay GIMBAL_CONTROL_TIME (1ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */

void gimbal_task(void const *pvParameters)
{
	
    //等待陀螺仪任务更新陀螺仪数据
    //wait a time
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //gimbal init
    //云台初始化
    gimbal_init(&gimbal_control);

    //wait for all motor online
    //判断电机是否都上线
//    while (toe_is_error(YAW_GIMBAL_MOTOR_TOE) || toe_is_error(PITCH_GIMBAL_MOTOR_TOE))
//    {
//        vTaskDelay(GIMBAL_CONTROL_TIME);
//        gimbal_feedback_update(&gimbal_control);             //云台数据反馈
//    }

    while (1)
    {
		//gimbal_control.shoot=shoot_control_loop();
		chassis_rc_to_control_vector(&gimbal_control,&chassis_data);//设置底盘控制量
        gimbal_set_mode(&gimbal_control);                    //设置云台控制模式
//        gimbal_mode_change_control_transit(&gimbal_control); //控制模式切换 控制数据过渡
        gimbal_feedback_update(&gimbal_control);             //云台数据反馈
			
        gimbal_set_control(&gimbal_control);                 //设置云台控制量
		gimbal_control_loop(&gimbal_control);  			  //云台控制PID计算
		gimbal_control.shoot=shoot_control_loop();
		
		Self_aim_data=get_selfaim_data();
		//CAN_cmd_gimbal(0,gimbal_control.gimbal_pitch_motor.given_current,0,0);
		vTaskDelay(GIMBAL_CONTROL_TIME);
		
		CAN_cmd_gimbal(0,gimbal_control.gimbal_pitch_motor.given_current,gimbal_control.gimbal_video_motor.given_current,0);
		//CAN_cmd_gimbal(0,500,gimbal_control.gimbal_video_motor.given_current,0);
		vTaskDelay(GIMBAL_CONTROL_TIME);
		CAN_cmd_chassis(gimbal_control.shoot->shoot_1_given_current,gimbal_control.shoot->shoot_2_given_current,gimbal_control.shoot->shoot_3_given_current,gimbal_control.shoot->shoot_4_given_current);
		//CAN_cmd_gimbal(0,0,0,0);
		vTaskDelay(GIMBAL_CONTROL_TIME);

//		servo_control(&gimbal_control);
    }
}
/**
  * @brief          return yaw motor data point
  * @param[in]      none
  * @retval         yaw motor data point
  */
/**
  * @brief          返回yaw 电机数据指针
  * @param[in]      none
  * @retval         yaw电机指针
  */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

/**
  * @brief          return pitch motor data point
  * @param[in]      none
  * @retval         pitch motor data point
  */
/**
  * @brief          返回pitch 电机数据指针
  * @param[in]      none
  * @retval         pitch
  */
const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}



/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     init:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init)
{

    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
	static const fp32 Pitch_angle_pid[3] ={PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD};
	
	static const fp32 Video_speed_pid[3]={VIDE0_SPEED_PID_KP, VIDE0_SPEED_PID_KI, VIDE0_SPEED_PID_KD};
	static const fp32 Video_angle_pid[3] ={VIDE0_GYRO_ABSOLUTE_PID_KP, VIDE0_GYRO_ABSOLUTE_PID_KI, VIDE0_GYRO_ABSOLUTE_PID_KD};
	
    //电机数据指针获取
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    init->gimbal_video_motor.gimbal_motor_measure= get_video_motor_measure_point();
    //陀螺仪数据指针获取
    init->gimbal_INT_angle_point = get_INS_angle_point();
	init->INS=get_INS();
    init->gimbal_INT_gyro_point =get_gyro_data_point();
    //遥控器数据指针获取
    init->gimbal_rc_ctrl = get_remote_control_point1();
	//外部陀螺仪指针获取
	init->gryo=get_gryo();
    //初始化电机模式
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_OFF;
	chassis_data.shoot_mode=0;
	chassis_data.chassis_mode=0;
    //初始化pitch电机pid
//    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
	PID_init(&init->gimbal_pitch_motor.gimbal_motor1_angle_pid,PID_POSITION,Pitch_angle_pid,PITCH_GYRO_ABSOLUTE_PID_MAX_OUT,PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT);
	PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);
	
	//初始化pitch电机pid
    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
	PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);

//初始化video电机pid
    PID_init(&init->gimbal_video_motor.gimbal_motor1_angle_pid,PID_POSITION,Video_angle_pid,VIDE0_GYRO_ABSOLUTE_PID_MAX_OUT,VIDE0_GYRO_ABSOLUTE_PID_MAX_IOUT);
	PID_init(&init->gimbal_video_motor.gimbal_motor_gyro_pid, PID_POSITION, Video_speed_pid,VIDE0_SPEED_PID_MAX_OUT, VIDE0_SPEED_PID_MAX_IOUT);
	
	gimbal_feedback_update(init);
    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
	chassis_data.yawangle_set=init->gimbal_yaw_motor.absolute_angle;
	Self_aim_data=get_selfaim_data();


    //pitch电机初始化
//    init->gimbal_pitch_motor.motor_angle_set=init->gimbal_pitch_motor.gimbal_motor_measure->angle;
    init->gimbal_pitch_motor.absolute_angle_set=0;
	
    init->gimbal_pitch_motor.offset_ecd=5606;
    init->aimflag=0;
}

/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
  * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t *set_mode)
{
//	static uint8_t flag=0;
	
//	if(set_mode->gimbal_rc_ctrl->rc.s[0]==1||set_mode->gimbal_rc_ctrl->rc.s[0]==2)
//	{
//		set_mode->gimbal_pitch_motor.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
//	}
//	else if(set_mode->gimbal_rc_ctrl->rc.s[0]==3)
//	{
//		set_mode->gimbal_pitch_motor.gimbal_motor_mode=GIMBAL_MOTOR_OFF;
//	}
	if(chassis_flag==1||chassis_flag==2)
	{
		set_mode->gimbal_pitch_motor.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
	}
	else if(chassis_flag==0)
	{
		set_mode->gimbal_pitch_motor.gimbal_motor_mode=GIMBAL_MOTOR_OFF;
	}
}
/**
  * @brief          gimbal some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     gimbal_feedback_update: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }
    //云台数据更新

	
	feedback_update->gimbal_pitch_motor.relative_angle =-motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,feedback_update->gimbal_pitch_motor.offset_ecd);
	feedback_update->gimbal_pitch_motor.absolute_angle =feedback_update->gryo->gryo_get->pitch;
	feedback_update->gimbal_pitch_motor.motor_angle = feedback_update->gimbal_pitch_motor.gimbal_motor_measure->angle;
    feedback_update->gimbal_pitch_motor.motor_gyro = feedback_update->gryo->gryo_get->pitch_accel;
//	feedback_update->gimbal_yaw_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
	feedback_update->gimbal_yaw_motor.absolute_angle=feedback_update->gryo->all_yaw;
	feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                       feedback_update->gimbal_yaw_motor.offset_ecd);
	feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (feedback_update->gryo->gryo_get->yaw_accel)+arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (feedback_update->gryo->gryo_get->pitch_accel);
	feedback_update->last_press_l=feedback_update->press_l;
	feedback_update->press_l=feedback_update->gimbal_rc_ctrl->mouse.mouse_right;
	feedback_update->lastkeyboard=feedback_update->keyboard;
	feedback_update->keyboard=feedback_update->gimbal_rc_ctrl->key.v;
	feedback_update->gimbal_pitch_motor.last_self_aim_pitch_angle=feedback_update->gimbal_pitch_motor.self_aim_pitch_angle;
	feedback_update->gimbal_pitch_motor.self_aim_pitch_angle=Self_aim_data->pitch/3.14*180;
	feedback_update->gimbal_yaw_motor.last_self_aim_yaw_angle=feedback_update->gimbal_yaw_motor.self_aim_yaw_angle;
	feedback_update->gimbal_yaw_motor.self_aim_yaw_angle=Self_aim_data->yaw/3.14*180;
//	feedback_update->gimbal_yaw_motor.self_aim_yaw_angle=Self_aim_data->shoot_yaw/3.14*180;
	
	feedback_update->gimbal_video_motor.absolute_angle =feedback_update->gimbal_video_motor.gimbal_motor_measure->angle;
	feedback_update->gimbal_video_motor.motor_gyro =feedback_update->gimbal_video_motor.gimbal_motor_measure->speed_rpm;
	
	feedback_update->lastpress=feedback_update->press;
	feedback_update->press=feedback_update->gimbal_rc_ctrl->rc.fn_1;
}

/**
  * @brief          calculate the relative angle between ecd and offset_ecd
  * @param[in]      ecd: motor now encode
  * @param[in]      offset_ecd: gimbal offset encode
  * @retval         relative angle, unit rad
  */;

/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}


/**
  * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".         
  * @param[out]     gimbal_set_control: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
  * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control)
{
//	static  int16_t pitch_channel = 0;
	static  int16_t video_channel = 0;
	static  int16_t a=0;
	static  int16_t b=0;
	//PITCH电机获取遥控器角度值
//	rc_deadband_limit(set_control->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
//	set_control->gimbal_pitch_motor.motor_angle_set+=pitch_channel*PITCH_RC_SEN+set_control->gimbal_rc_ctrl->mouse.y*PITCH_MOUSE_SEN*0.5;
	
	
	//PITCH电机获取遥控器角度值_利用陀螺仪控制PITCH
//	set_control->gimbal_pitch_motor.absolute_angle_set+=pitch_channel*PITCH_RC_SEN_INS+set_control->gimbal_rc_ctrl->mouse.y*PITCH_MOUSE_SEN*0.5;
	
//   图传电机的遥控器接受数据	
//	rc_deadband_limit(set_control->gimbal_rc_ctrl->rc.ch[4], video_channel, RC_DEADBAND);
//	a=b;
//	b=set_control->gimbal_rc_ctrl->rc.s[0];
//	if((a==1)&&(b==3))
//	{
//		
//	set_control->gimbal_video_motor.absolute_angle_set=set_control->gimbal_video_motor.gimbal_motor_measure->angle+2160;
//	
//	}

set_control->gimbal_video_motor.absolute_angle_set=set_control->gimbal_video_motor.gimbal_motor_measure->angle+video_channel*0.01;
	
	
   /* if(set_control->gimbal_pitch_motor.gimbal_motor_mode==GIMBAL_MOTOR_GYRO&&aimflag==1)
	{
//		set_control->gimbal_pitch_motor.absolute_angle_set-=pitch_channel*PITCH_RC_SEN+set_control->gimbal_rc_ctrl->mouse.y*PITCH_MOUSE_SEN;
		//set_control->gimbal_pitch_motor.absolute_angle_set=set_control->gimbal_pitch_motor.self_aim_pitch_angle+1.5;
		set_control->gimbal_pitch_motor.absolute_angle_set-=pitch_channel*PITCH_RC_SEN+set_control->gimbal_rc_ctrl->mouse.y*PITCH_MOUSE_SEN*0.5;
		if(set_control->gimbal_pitch_motor.absolute_angle_set<-19)
		{
			set_control->gimbal_pitch_motor.absolute_angle_set=-19;
		}
		else if(set_control->gimbal_pitch_motor.absolute_angle_set>29)
		{
			set_control->gimbal_pitch_motor.absolute_angle_set=29;
		}
		else
		{
			set_control->gimbal_pitch_motor.absolute_angle_set=set_control->gimbal_pitch_motor.absolute_angle_set;
		}
	}
	else if(set_control->gimbal_pitch_motor.gimbal_motor_mode==GIMBAL_MOTOR_GYRO&&aimflag==0)
	{
		set_control->gimbal_pitch_motor.absolute_angle_set-=pitch_channel*PITCH_RC_SEN+set_control->gimbal_rc_ctrl->mouse.y*PITCH_MOUSE_SEN;
		if(set_control->gimbal_pitch_motor.absolute_angle_set<-19)
		{
			set_control->gimbal_pitch_motor.absolute_angle_set=-19;
		}
		else if(set_control->gimbal_pitch_motor.absolute_angle_set>29)
		{
			set_control->gimbal_pitch_motor.absolute_angle_set=29;
		}
		else
		{
			set_control->gimbal_pitch_motor.absolute_angle_set=set_control->gimbal_pitch_motor.absolute_angle_set;
		}
	}
	*/
}



/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sent to motor
  * @param[out]     gimbal_control_loop: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_control_loop(gimbal_control_t *control_loop)
{
    if(control_loop->gimbal_pitch_motor.gimbal_motor_mode==GIMBAL_MOTOR_OFF)
	{
		control_loop->gimbal_pitch_motor.given_current=0;
		control_loop->gimbal_video_motor.given_current=0;
	}
	else if(control_loop->gimbal_pitch_motor.gimbal_motor_mode==GIMBAL_MOTOR_GYRO)
	{
		//PITCH电机角度环控制 
//		PID_calc(&control_loop->gimbal_pitch_motor.gimbal_motor1_angle_pid,control_loop->gimbal_pitch_motor.motor_angle,control_loop->gimbal_pitch_motor.motor_angle_set);
//		PID_calc(&control_loop->gimbal_pitch_motor.gimbal_motor_gyro_pid,control_loop->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm,control_loop->gimbal_pitch_motor.gimbal_motor1_angle_pid.out);
//		
		
		gimbal_PID_calc(&control_loop->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid,control_loop->gimbal_pitch_motor.absolute_angle,control_loop->gimbal_pitch_motor.absolute_angle_set,control_loop->gimbal_pitch_motor.motor_gyro);
        PID_calc(&control_loop->gimbal_pitch_motor.gimbal_motor_gyro_pid,control_loop->gimbal_pitch_motor.motor_gyro,control_loop->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid.out);
//		PID_calc(&control_loop->gimbal_pitch_motor.gimbal_motor_gyro_pid,control_loop->gimbal_pitch_motor.motor_speed,500);
		//PITCH电流输出
		control_loop->gimbal_pitch_motor.given_current=control_loop->gimbal_pitch_motor.gimbal_motor_gyro_pid.out;
		
		//图传PID调节
		PID_calc(&control_loop->gimbal_video_motor.gimbal_motor1_angle_pid,control_loop->gimbal_video_motor.absolute_angle,control_loop->gimbal_video_motor.absolute_angle_set);
		PID_calc(&control_loop->gimbal_video_motor.gimbal_motor_gyro_pid,control_loop->gimbal_video_motor.motor_gyro,control_loop->gimbal_video_motor.gimbal_motor1_angle_pid.out);
		control_loop->gimbal_video_motor.given_current=control_loop->gimbal_video_motor.gimbal_motor_gyro_pid.out;
	}
}

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    //角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     gimbal_init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     gimbal_init:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
//    pid->err = rad_format(err);
	pid->err = err;
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(pid->out, pid->max_out);
    return pid->out;
}

/**
  * @brief          gimbal PID clear, clear pid.out, iout.
  * @param[out]     gimbal_pid_clear: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          云台PID清除，清除pid的out,iout
  * @param[out]     gimbal_pid_clear:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}
void chassis_rc_to_control_vector(gimbal_control_t *gimbal_control_set,chassis_data_t *chassis_data)
{
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
	fp32 angleset;
	static int16_t yaw_channel = 0;
	static int16_t pitch_channel = 0;
	const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    
	
	
    if(gimbal_control_set->gimbal_rc_ctrl->rc.mode_sw==1)
	{
	//死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
	rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch_0, yaw_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch_1, pitch_channel, CHASSIS_RC_DEADLINE);
	rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch_2, vx_channel, RC_DEADBAND1);
	rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch_3, vy_channel, RC_DEADBAND);
	
    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

	}
    //keyboard set speed set-point
    //键盘控制
	if(gimbal_control_set->gimbal_rc_ctrl->rc.mode_sw==2)
	{
		
	if(gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_W)
	{
		vx_set_channel=1.5;
		if(gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_A)
		{
			vy_set_channel=1.5;
		}
		if(gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_D)
		{
			vy_set_channel=-1.5;
		}
	}
	else if(gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_S)
	{
		vx_set_channel=-1.5;
		if(gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_A)
		{
			vy_set_channel=1.5;
		}
		if(gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_D)
		{
			vy_set_channel=-1.5;
		}
	}
	else if(gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_A)
	{
		vy_set_channel=1.5;
		if(gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_W)
		{
			vx_set_channel=1.5;
		}
		if(gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_S)
		{
			vx_set_channel=-1.5;
		}
	}
	else if(gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_D)
	{
		vy_set_channel=-1.5;
		if(gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_W)
		{
			vx_set_channel=1.5;
		}
		if(gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_S)
		{
			vx_set_channel=-1.5;
		}
	}
	
	
	if(gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_W&&gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_SHIFT)
	{
		vx_set_channel=3;
	}
	else if(gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_S&&gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_SHIFT)
	{
		vx_set_channel=-3;
	}
	else if(gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_A&&gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_SHIFT)
	{
		vy_set_channel=3;
	}
	else if(gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_D&&gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_SHIFT)
	{
		vy_set_channel=-3;
	}

	}
	//
	first_order_filter_init(&chassis_data->chassis_cmd_slow_set_vx, GIMBAL_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_data->chassis_cmd_slow_set_vy, GIMBAL_CONTROL_TIME, chassis_y_order_filter);
    //first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_data->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_data->chassis_cmd_slow_set_vy, vy_set_channel);
	
    //stop command, need not slow change, set zero derectly
    //停止信号，不需要缓慢加速，直接减速到零
	
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_data->vx_set = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_data->vy_set = 0.0f;
    }
	
    chassis_data->vx_set = chassis_data->chassis_cmd_slow_set_vx.out;
    chassis_data->vy_set = chassis_data->chassis_cmd_slow_set_vy.out;
	chassis_data->wz_set=-CHASSIS_WZ_RC_SEN * gimbal_control_set->gimbal_rc_ctrl->rc.wheel;
	
	//关于判断遥控器模式转换按键是否按下
	mode_last_press=mode_press;
	mode_press=gimbal_control_set->gimbal_rc_ctrl->rc.pause;
	
	selfaim_last_mode=selfaim_mode;
	selfaim_mode=gimbal_control_set->gimbal_rc_ctrl->rc.fn_1;
//遥控器判断底盘运动模式（sw=0掉电,sw=1遥控器操控）
		if(gimbal_control_set->gimbal_rc_ctrl->rc.mode_sw==0)
		{
			chassis_flag=0;
		}
		
		else if(gimbal_control_set->gimbal_rc_ctrl->rc.mode_sw==1)
		{
			
			if(mode_last_press==0&&mode_press==1)
			{
				chassis_flag++;
				if(chassis_flag>2)
			{
				chassis_flag=1;
			}
			}
			
			if(gimbal_control_set->press==1&&gimbal_control_set->lastpress==0)
	
			{
				
			gimbal_control_set->aimflag=!(gimbal_control_set->aimflag);
	
			}
			
		
			
		}

//键盘判断运动模式（sw=2键盘操控）
	else if(gimbal_control_set->gimbal_rc_ctrl->rc.mode_sw==2)
	{
		//底盘运动逻辑
		if((gimbal_control_set->keyboard&KEY_PRESSED_OFFSET_Q)&&((gimbal_control_set->lastkeyboard&KEY_PRESSED_OFFSET_Q)==0))
		{
			chassis_flag=0;
		}
		else if(gimbal_control_set->keyboard&KEY_PRESSED_OFFSET_F&&(gimbal_control_set->lastkeyboard&KEY_PRESSED_OFFSET_F)==0)
		{
			chassis_flag++;
			if(chassis_flag>2)
			{
				chassis_flag=1;
			}
		}
	
	//掉头180度
	if(gimbal_control_set->keyboard&KEY_PRESSED_OFFSET_C&&(gimbal_control_set->lastkeyboard&KEY_PRESSED_OFFSET_C)==0)
	{
		turnflag=1;
	}
	
		if(gimbal_control_set->keyboard&KEY_PRESSED_OFFSET_X&&(gimbal_control_set->lastkeyboard&KEY_PRESSED_OFFSET_X)==0)
	{
		turnflag=2;
	}
	
//	if(gimbal_control_set->keyboard&KEY_PRESSED_OFFSET_R&&(gimbal_control_set->lastkeyboard&KEY_PRESSED_OFFSET_R)==0)
//	{
//		
//		gimbal_control_set->aimflag=!(gimbal_control_set->aimflag);
//	}
	

	if(gimbal_control_set->keyboard&KEY_PRESSED_OFFSET_Z&&(gimbal_control_set->lastkeyboard&KEY_PRESSED_OFFSET_Z)==0)
	{
		chassis_data->ejection_flag=!(chassis_data->ejection_flag);
	}
	
	if(gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_B&&gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_CTRL)
	{
		HAL_Software_Reset( );
	}
	
	if(gimbal_control_set->last_press_l==0&&gimbal_control_set->press_l==1)
	{
		gimbal_control_set->aimflag=!(gimbal_control_set->aimflag);

	}
	
	
	chassis_data->keyboard=gimbal_control_set->keyboard;

	
	}
	
//给底盘数据传输
	chassis_data->chassis_mode=chassis_flag;
//	chassis_data->selfaim_shoot_mode=Self_aim_data->mode;
//	chassis_data->shoot_mode=gimbal_control_set->gimbal_rc_ctrl->mouse.press_l<<1|shoot_control.shoot_mode|aimflag<<2;
	if(gimbal_control_set->gimbal_rc_ctrl->rc.mode_sw==1)
	{
	chassis_data->shoot_mode=gimbal_control_set->gimbal_rc_ctrl->rc.trigger;
	}
	
	if(gimbal_control_set->gimbal_rc_ctrl->rc.mode_sw==2)
	{
	chassis_data->shoot_mode=gimbal_control_set->gimbal_rc_ctrl->mouse.mouse_left;
	}
	
	chassis_data->yaw_angle=(gimbal_control_set->gimbal_yaw_motor.absolute_angle);
//	chassis_data->selfaim_shoot_mode=gimbal_control_set->aimflag;
//	chassis_data->yaw_angle=1;
	chassis_data->yaw_gyro=gimbal_control_set->gimbal_yaw_motor.motor_gyro;
//	chassis_data->yawangle_set=yaw_angle_limit_func(chassis_data->yawangle_set-yaw_channel*YAW_RC_SEN);
	chassis_data->aimflag=gimbal_control_set->aimflag;
	
//yaw，pitch运动状态给值
	if(chassis_data->chassis_mode==0)
	{
		chassis_data->yawangle_set=chassis_data->yaw_angle;
		chassis_data->selfaim_shoot_mode=0;
	}

	if(chassis_data->chassis_mode!=0&&gimbal_control_set->aimflag==0&&turnflag==0)
	{

		chassis_data->yawangle_set-=yaw_channel*YAW_RC_SEN+gimbal_control_set->gimbal_rc_ctrl->mouse.mouse_x*YAW_MOUSE_SEN;
	    gimbal_control_set->gimbal_pitch_motor.absolute_angle_set-=pitch_channel*PITCH_RC_SEN_INS+gimbal_control_set->gimbal_rc_ctrl->mouse.mouse_y*PITCH_MOUSE_SEN*0.5;
		if(gimbal_control_set->gimbal_pitch_motor.absolute_angle_set<-34)
		{
			gimbal_control_set->gimbal_pitch_motor.absolute_angle_set=-34;
			
		}
		
		if(gimbal_control_set->gimbal_pitch_motor.absolute_angle_set>17)
		{
			gimbal_control_set->gimbal_pitch_motor.absolute_angle_set=17;
			
		}
		
		
		chassis_data->selfaim_shoot_mode=0;
	}
	else if(chassis_data->chassis_mode!=0&&gimbal_control_set->aimflag==1)
	{	
		
		if (gimbal_control_set->gimbal_yaw_motor.self_aim_yaw_angle==0||gimbal_control_set->gimbal_pitch_motor.self_aim_pitch_angle==0)
		{
			chassis_data->yawangle_set-=yaw_channel*YAW_RC_SEN+gimbal_control_set->gimbal_rc_ctrl->mouse.mouse_x*YAW_MOUSE_SEN;
			gimbal_control_set->gimbal_pitch_motor.absolute_angle_set-=pitch_channel*PITCH_RC_SEN_INS+gimbal_control_set->gimbal_rc_ctrl->mouse.mouse_y*PITCH_MOUSE_SEN*0.5;
			chassis_data->selfaim_shoot_mode=0;
		}
		else 
		{
			chassis_data->yawangle_set=gimbal_control_set->gimbal_yaw_motor.self_aim_yaw_angle+360.0f * QEKF_INS.YawRoundCount;
			gimbal_control_set->gimbal_pitch_motor.absolute_angle_set=gimbal_control_set->gimbal_pitch_motor.self_aim_pitch_angle;
			chassis_data->selfaim_shoot_mode=Self_aim_data->mode;
//			chassis_data->selfaim_shoot_mode=0;
		}
		
	}
	else if(chassis_data->chassis_mode!=0&&turnflag==1)
	{
		chassis_data->yawangle_set+=180;
		turnflag=0;
	}
	
		else if(chassis_data->chassis_mode!=0&&turnflag==2)
	{
		chassis_data->yawangle_set-=180;
		turnflag=0;
	}
}
/**
  * @brief          返回底盘数据指针供CAN_task任务使用
  * @param[out]     none
  * @retval         none
  */
 chassis_data_t *get_chassis_data_point()
{
	return &chassis_data;
}
gimbal_control_t *get_gimbal_data()
{
	return &gimbal_control;
}

///**
//  * @brief          倍镜舵机驱动使用
//  * @param[out]     none
//  * @retval         none
//  */
//void servo_control(gimbal_control_t *servo)
//{
//	
//	
//	last_press=press;
//	press=servo->gimbal_rc_ctrl->rc.s[0];
//	if((last_press==1)&&(press=3))
//	{
//	servoflag=!servoflag;
//	}
//	if(servoflag==1)
//	{
//		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,2000);
//	}
//	if(servoflag==0)
//	{
//		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1500);
//	}
//}

//复位程序
void HAL_Software_Reset(void)
{
    // 1. 关闭全局中断，防止复位过程被中断打断
    __disable_irq();
    
    // 2. 触发系统复位（CMSIS标准函数，所有STM32 HAL库工程通用）
    NVIC_SystemReset();
    

}



