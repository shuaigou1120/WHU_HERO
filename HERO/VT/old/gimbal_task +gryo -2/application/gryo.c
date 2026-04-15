//该程序用于陀螺仪一些数据处理

#include "gryo.h"
#include "main.h"
#include "cmsis_os.h"
#include "can_receive.h"
#include "arm_math.h"

#ifndef PI
#define PI 3.1415926535f
#endif

gryo_get_t gryo_control;
q_data_t q;

void gryo_task(void const *pvParameters)
{
	gryo_init(&gryo_control);
	
	while(1)
	{
		// 读取陀螺仪数据
		gryo_feedback(&gryo_control);
		
		// 角度 → 弧度 再转四元数
		ToQuaternion(
			gryo_control.gryo_get->yaw * PI / 180.0f,
			gryo_control.gryo_get->pitch * PI / 180.0f,
			gryo_control.gryo_get->roll * PI / 180.0f
		);
		
		vTaskDelay(5);
	}
}
//获取陀螺仪数据指针
void gryo_init( gryo_get_t *init)
{
	init->gryo_get=get_gryo_point( );
}



//统计总YAW值
void gryo_feedback(gryo_get_t *feedback)
{


	// 处理 yaw 360° 循环问题
	if(feedback->gryo_get->yaw - feedback->gryo_get->last_yaw >= 180.0f)
	{
		feedback->YawRoundNumber--;
	}
	if(feedback->gryo_get->yaw - feedback->gryo_get->last_yaw <= -180.0f)
	{
		feedback->YawRoundNumber++;
	}

	feedback->all_yaw = 360.0f * feedback->YawRoundNumber + feedback->gryo_get->yaw;
}


// 欧拉角 → 四元数
void ToQuaternion(float yaw, float pitch, float roll)
{
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
 
    q.q[0] = cy * cp * cr + sy * sp * sr;
    q.q[1] = cy * cp * sr - sy * sp * cr;
    q.q[2] = sy * cp * sr + cy * sp * cr;
    q.q[3] = sy * cp * cr - cy * sp * sr;
}

//获取gryo_control指针
const gryo_get_t *get_gryo(void)
{
    return &gryo_control;
}

//获取四元数指针
const q_data_t *get_q(void)
{
	return &q;
}