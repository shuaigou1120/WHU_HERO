/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"

#include "gimbal_task.h"

#include "detect_task.h"

#include "shoot.h"

#include "gryo.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
		if((ptr)->ecd-(ptr)->last_ecd>4096)								\
		{																\
			(ptr)->ecd_count--;											\
		}																\
		else if((ptr)->ecd-(ptr)->last_ecd<-4096)						\
		{																\
			(ptr)->ecd_count++;											\
		}																\
		(ptr)->angle=(ptr)->ecd_count*360+(ptr)->ecd*360/8192;		\
    }
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006电机*/
static motor_measure_t motor_chassis[7];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
static uint8_t              color;
void CAN_receive_referee_data(uint8_t rxdata[]);
	
chassis_to_gimbal_t chassis_to_gimbal;
gryo_data_t imu;

	
#define ACCEL_CAN_MAX 235.2f
#define ACCEL_CAN_MIN -235.2f
#define GYRO_CAN_MAX 34.88f
#define GYRO_CAN_MIN -34.88f

#define PITCH_CAN_MAX 90.0f
#define PITCH_CAN_MIN -90.0f

#define ROLL_CAN_MAX 180.0f

#define ROLL_CAN_MIN -180.0f
#define YAW_CAN_MAX 180.0f
#define YAW_CAN_MIN -180.0f
#define Quaternion_CAN_MIN -1.0f
#define Quaternion_CAN_MAX 1.0f
/**
************************************************************************
* @brief: float_to_uint: 浮点数转换为无符号整数函数
* @param[in]: x_float: 待转换的浮点数
* @param[in]: x_min: 范围最小值
* @param[in]: x_max: 范围最大值
* @param[in]: bits: 目标无符号整数的位数
* @retval: 无符号整数结果
* @details: 将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个
指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{

/* Converts a float to an unsigned int, given range and number of bits */
float span = x_max - x_min;
float offset = x_min;
return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
************************************************************************
* @brief: uint_to_float: 无符号整数转换为浮点数函数
* @param[in]: x_int: 待转换的无符号整数
* @param[in]: x_min: 范围最小值
* @param[in]: x_max: 范围最大值
* @param[in]: bits: 无符号整数的位数
* @retval: 浮点数结果
* @details: 将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结
果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
/* converts unsigned int to float, given range and number of bits */
float span = x_max - x_min;
float offset = x_min;
return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
	
/**
  * @brief          接受弹丸速度与发弹量
  * @param[in]      can接受数据
  * @retval         none
  */
void CAN_receive_data1(uint8_t rxdata[])
{
	send_float_typedef temp[1];
	temp[0].uint8_t[0]=rxdata[0];
	temp[0].uint8_t[1]=rxdata[1];
	temp[0].uint8_t[2]=rxdata[2];
	temp[0].uint8_t[3]=rxdata[3];
	
	chassis_to_gimbal.speed=temp[0].float_t;
	chassis_to_gimbal.allowance_42mm=(uint16_t)(rxdata[4]<<8|rxdata[5]);
}







/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
	

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	if(hcan==&hcan1)
	{ 	
		switch (rx_header.StdId)
		{
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			case CAN_PIT_MOTOR_ID:
			case CAN_FRICTION_LEFT_ID:
			case CAN_FRICTION_RIGHT_ID:
			{
				static uint8_t i = 0;
				//get motor id
				i = rx_header.StdId - CAN_3508_M1_ID;
				get_motor_measure(&motor_chassis[i], rx_data);
				detect_hook(CHASSIS_MOTOR1_TOE + i);
				break;
			}


			
			
			
			
			default:
			{
				break;
			}
		}
    }
	else if(hcan==&hcan2)
	{
		switch (rx_header.StdId)
		{
			case CAN_referee_data: 
			{
				CAN_receive_referee_data(rx_data);
				break;
			}
			
			case CAN_chassis_gmbial_data1: 
			{
				CAN_receive_data1(rx_data);
				break;
			}
			case gyro_out: 
			{
				IMU_UpdateData(rx_data);
				break;
			}
			

			
			
			default:
			{
				break;
			}
			
		}
	}
}



/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shootL: (0x207) 3508左摩擦轮电机控制电流, 范围 [-16384,16384]
  * @param[in]      shootR: (0x208) 3508左摩擦轮电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shootL, int16_t shootR)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shootL >> 8);
    gimbal_can_send_data[5] = shootL;
    gimbal_can_send_data[6] = (shootR >> 8);
    gimbal_can_send_data[7] = shootR;
    HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void can_gimbal_chassis_data1(chassis_data_t *chassis_data)
{
	send_float_typedef temp[2];
	temp[0].float_t=chassis_data->vx_set;
	temp[1].float_t=chassis_data->vy_set;
	uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_gmbial_chassis_data1;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] =temp[0].uint8_t[0];
    chassis_can_send_data[1] = temp[0].uint8_t[1];
    chassis_can_send_data[2] = temp[0].uint8_t[2];
    chassis_can_send_data[3] = temp[0].uint8_t[3];
    chassis_can_send_data[4] = temp[1].uint8_t[0];
    chassis_can_send_data[5] = temp[1].uint8_t[1];
    chassis_can_send_data[6] = temp[1].uint8_t[2];
    chassis_can_send_data[7] = temp[1].uint8_t[3];
 while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0)
    {};
    HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void can_gimbal_chassis_data2(chassis_data_t *chassis_data)
{
	send_float_typedef temp[2];
	temp[0].float_t=chassis_data->wz_set;
	temp[1].float_t=chassis_data->yaw_angle;
	uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_gmbial_chassis_data2;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] =temp[0].uint8_t[0];
    chassis_can_send_data[1] = temp[0].uint8_t[1];
    chassis_can_send_data[2] = temp[0].uint8_t[2];
    chassis_can_send_data[3] = temp[0].uint8_t[3];
    chassis_can_send_data[4] = temp[1].uint8_t[0];
    chassis_can_send_data[5] = temp[1].uint8_t[1];
    chassis_can_send_data[6] = temp[1].uint8_t[2];
    chassis_can_send_data[7] = temp[1].uint8_t[3];
	 while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0)
	 {};
    HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void can_gimbal_chassis_data3(chassis_data_t *chassis_data)
{
	send_float_typedef temp[1];
	uint32_t send_mail_box;
	temp[0].float_t=chassis_data->yawangle_set;
    chassis_tx_message.StdId = CAN_gmbial_chassis_data3;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] =chassis_data->chassis_mode>>8;
    chassis_can_send_data[1] =chassis_data->chassis_mode;
    chassis_can_send_data[2] =chassis_data->shoot_mode>>8;
    chassis_can_send_data[3] =chassis_data->shoot_mode;
	chassis_can_send_data[4] =temp[0].uint8_t[0];
	chassis_can_send_data[5] =temp[0].uint8_t[1];
	chassis_can_send_data[6] =temp[0].uint8_t[2];
	chassis_can_send_data[7] =temp[0].uint8_t[3];
	 while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0)
	 {} ;
    HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void can_gimbal_chassis_data4(chassis_data_t *chassis_data)
{
	send_float_typedef temp[1];
	uint32_t send_mail_box;
	temp[0].float_t=chassis_data->yaw_gyro;
    chassis_tx_message.StdId = CAN_gimbal_chassis_data4;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] =temp[0].uint8_t[0];
    chassis_can_send_data[1] =temp[0].uint8_t[1];
    chassis_can_send_data[2] =temp[0].uint8_t[2];
    chassis_can_send_data[3] =temp[0].uint8_t[3];
	chassis_can_send_data[4] =chassis_data->selfaim_shoot_mode>>8;
	chassis_can_send_data[5] =chassis_data->selfaim_shoot_mode;
	chassis_can_send_data[6] =chassis_data->ejection_flag;
	chassis_can_send_data[7] =chassis_data->aimflag;
	 while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) ==0)
	 {};
    HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void can_gimbal_chassis_data5(chassis_data_t *chassis_data)
{
	
	uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_gimbal_chassis_data5;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] =chassis_data->keyboard>>8;
    chassis_can_send_data[1] =chassis_data->keyboard;
    chassis_can_send_data[2] =shoot_flag;
    chassis_can_send_data[3] =0;
	chassis_can_send_data[4] =0;
	chassis_can_send_data[5] =0;
	chassis_can_send_data[6] =0;
	chassis_can_send_data[7] =0;
	 while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) ==0)
	 {};
    HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void IMU_RequestData(uint16_t can_id,uint8_t reg)
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t cmd[4]={(uint8_t)can_id,(uint8_t)(can_id>>8),reg,0xCC};
	uint32_t returnBox;
	tx_header.DLC=4;
	tx_header.IDE=CAN_ID_STD;
	tx_header.RTR=CAN_RTR_DATA;
	tx_header.StdId=0x6FF;
	if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)>1)
	{
	HAL_CAN_AddTxMessage(&hcan2,&tx_header,cmd,&returnBox);
	}
}

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}



/**
  * @brief          返回左摩擦轮电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
//const motor_measure_t *get_left_friction_motor_measure_point(void)
//{
//    return &motor_chassis[1];
//}

/**
  * @brief          返回右摩擦轮电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
//const motor_measure_t *get_right_friction_motor_measure_point(void)
//{
//    return &motor_chassis[2];
//}

/**
  * @brief          返回图传电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_video_motor_measure_point(void)
{
    return &motor_chassis[6];
}

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
void CAN_receive_referee_data(uint8_t rxdata[])
{
	color=rxdata[0];
	
}
void get_enemy_color(uint8_t *enemy_color)
{
	*enemy_color=color;
}

const chassis_to_gimbal_t *get_speed(void)
{
    return &chassis_to_gimbal;
}


/**
  * @brief          返回陀螺仪指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const gryo_data_t *get_gryo_point(void)
{
    return &imu;
}

void IMU_UpdateAccel(uint8_t* pData)
{
    uint16_t accel[3];
    
    accel[0]=pData[3]<<8|pData[2];
    accel[1]=pData[5]<<8|pData[4];
    accel[2]=pData[7]<<8|pData[6];
    
    imu.accel[0]=uint_to_float(accel[0],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
    imu.accel[1]=uint_to_float(accel[1],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
    imu.accel[2]=uint_to_float(accel[2],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
    
}

void IMU_UpdateGyro(uint8_t* pData)
{
    uint16_t gyro[3];
    
    gyro[0]=pData[3]<<8|pData[2];
    gyro[1]=pData[5]<<8|pData[4];
    gyro[2]=pData[7]<<8|pData[6];
    
    imu.gyro[0]=uint_to_float(gyro[0],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
    imu.gyro[1]=uint_to_float(gyro[1],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
    imu.gyro[2]=uint_to_float(gyro[2],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
}


void IMU_UpdateEuler(uint8_t* pData)
{
    int euler[3];
    
    euler[0]=pData[3]<<8|pData[2];
    euler[1]=pData[5]<<8|pData[4];
    euler[2]=pData[7]<<8|pData[6];
    
    imu.pitch=uint_to_float(euler[0],PITCH_CAN_MIN,PITCH_CAN_MAX,16);
    imu.yaw=uint_to_float(euler[1],YAW_CAN_MIN,YAW_CAN_MAX,16);
    imu.roll=uint_to_float(euler[2],ROLL_CAN_MIN,ROLL_CAN_MAX,16);
}


void IMU_UpdateQuaternion(uint8_t* pData)
{
    int w = pData[1]<<6| ((pData[2]&0xF8)>>2);
    int x = (pData[2]&0x03)<<12|(pData[3]<<4)|((pData[4]&0xF0)>>4);
    int y = (pData[4]&0x0F)<<10|(pData[5]<<2)|(pData[6]&0xC0)>>6;
    int z = (pData[6]&0x3F)<<8|pData[7];
    
    imu.q[0] = uint_to_float(w,Quaternion_CAN_MIN,Quaternion_CAN_MAX,14);
    imu.q[1] = uint_to_float(x,Quaternion_CAN_MIN,Quaternion_CAN_MAX,14);
    imu.q[2] = uint_to_float(y,Quaternion_CAN_MIN,Quaternion_CAN_MAX,14);
    imu.q[3] = uint_to_float(z,Quaternion_CAN_MIN,Quaternion_CAN_MAX,14);
}

void IMU_UpdateData(uint8_t* pData)
{

    switch(pData[0])
    {
        case 1:
            IMU_UpdateAccel(pData);
            break;
        case 2:
            IMU_UpdateGyro(pData);
            break;
        case 3:
            IMU_UpdateEuler(pData);
            break;
        case 4:
            IMU_UpdateQuaternion(pData);
            break;
        case 0xCC:
            break;
    }

}
