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


#include "detect_task.h"

#include "chassis_to_gimbal.h"

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
    }																	\
		
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006电机*/
motor_measure_t motor_chassis[7];
chassis_data_t chassis_data;
static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
static CAN_TxHeaderTypeDef  referee_tx_message;
static uint8_t              referee_can_send_data[1];
	
static CAN_TxHeaderTypeDef  chassis_tx_message1;
static uint8_t              chassis_can_send_data1[8];
/**
  * @brief          将底盘速度设定值vx vy转化为浮点数
  * @param[in]      can接受数据
  * @retval         none
  */
void CAN_receive_chassis_data1(uint8_t rxdata[])
{
	send_float_typedef temp[2];
	temp[0].uint8_t[0]=rxdata[0];
	temp[0].uint8_t[1]=rxdata[1];
	temp[0].uint8_t[2]=rxdata[2];
	temp[0].uint8_t[3]=rxdata[3];
	
	temp[1].uint8_t[0]=rxdata[4];
	temp[1].uint8_t[1]=rxdata[5];
	temp[1].uint8_t[2]=rxdata[6];
	temp[1].uint8_t[3]=rxdata[7];
	chassis_data.vx_set=temp[0].float_t;
	chassis_data.vy_set=temp[1].float_t;
}
/**
  * @brief          将底盘旋转速度wz和底盘跟随云台角度angle转化为浮点数
  * @param[in]      can接受数据
  * @retval         none
  */
void CAN_receive_chassis_data2(uint8_t rxdata[])
{
	send_float_typedef temp[2];
	temp[0].uint8_t[0]=rxdata[0];
	temp[0].uint8_t[1]=rxdata[1];
	temp[0].uint8_t[2]=rxdata[2];
	temp[0].uint8_t[3]=rxdata[3];
	
	temp[1].uint8_t[0]=rxdata[4];
	temp[1].uint8_t[1]=rxdata[5];
	temp[1].uint8_t[2]=rxdata[6];
	temp[1].uint8_t[3]=rxdata[7];
	chassis_data.wz_set=temp[0].float_t;
	chassis_data.yaw_angle=temp[1].float_t;;
}
/**
  * @brief          接受底盘模式和云台yaw轴角度
  * @param[in]      can接受数据
  * @retval         none
  */
void CAN_receive_chassis_data3(uint8_t rxdata[])
{
	send_float_typedef temp[1];
	temp[0].uint8_t[0]=rxdata[4];
	temp[0].uint8_t[1]=rxdata[5];
	temp[0].uint8_t[2]=rxdata[6];
	temp[0].uint8_t[3]=rxdata[7];
	chassis_data.chassis_mode=(uint16_t)(rxdata[0]<<8|rxdata[1]);
	chassis_data.shoot_mode=(uint16_t)(rxdata[2]<<8|rxdata[3]);
	chassis_data.yaw_angle_set=temp[0].float_t;
}
/**
  * @brief          接受云台yaw轴角速度
  * @param[in]      can接受数据
  * @retval         none
  */
void CAN_receive_chassis_data4(uint8_t rxdata[])
{
	send_float_typedef temp[1];
	temp[0].uint8_t[0]=rxdata[0];
	temp[0].uint8_t[1]=rxdata[1];
	temp[0].uint8_t[2]=rxdata[2];
	temp[0].uint8_t[3]=rxdata[3];
	chassis_data.yaw_gyro=temp[0].float_t;
//	chassis_data.tuidan_mode=(uint16_t)(rxdata[4]<<8|rxdata[5]);
	chassis_data.selfaim_shoot_mode=(uint16_t)(rxdata[4]<<8|rxdata[5]);
	chassis_data.ejection_flag=rxdata[6];
	chassis_data.selfaim_mode=rxdata[7];
}
//接受键盘数据
void CAN_receive_chassis_data5(uint8_t rxdata[])
{
	chassis_data.last_keyboard=chassis_data.keyboard;
	chassis_data.keyboard=(uint16_t)(rxdata[0]<<8|rxdata[1]);
	chassis_data.shoot_flag=rxdata[2];
	
	
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
	if(hcan==&hcan1)
	{	
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

		switch (rx_header.StdId)
		{
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			case CAN_YAW_MOTOR_ID:
			case CAN_TRIGGER_MOTOR_ID:
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
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		switch(rx_header.StdId)
		{
			case CAN_gmbial_chassis_data1:
			{
				CAN_receive_chassis_data1(rx_data);
				break;
			}
			case CAN_gmbial_chassis_data2:
			{
				CAN_receive_chassis_data2(rx_data);
				break;
			}
			case CAN_gmbial_chassis_data3:
			{
				CAN_receive_chassis_data3(rx_data);
				break;
			}
			case CAN_gmbial_chassis_data4:
			{
				CAN_receive_chassis_data4(rx_data);
				break;
			}
			case CAN_gimbal_chassis_data5:
			{
				CAN_receive_chassis_data5(rx_data);
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
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
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
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
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

    HAL_CAN_AddTxMessage(&REFEREE_CAN , &chassis_tx_message, chassis_can_send_data, &send_mail_box);
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
void CAN_cmd_referee_data(uint8_t color)
{
	uint32_t send_mail_box;
    referee_tx_message.StdId = CAN_referee_data;
    referee_tx_message.IDE = CAN_ID_STD;
    referee_tx_message.RTR = CAN_RTR_DATA;
    referee_tx_message.DLC = 0x01;
    referee_can_send_data[0] = color ;
    HAL_CAN_AddTxMessage(&REFEREE_CAN, &referee_tx_message, referee_can_send_data, &send_mail_box);
}

void can_chassis_gimbal_data1(chassis_to_gimbal_t *chassis_to_gimbal)
{
	send_float_typedef temp[1];
	uint32_t send_mail_box;
	temp[0].float_t=chassis_to_gimbal->speed;
    chassis_tx_message1.StdId = CAN_chassis_gmbial_data1;
    chassis_tx_message1.IDE = CAN_ID_STD;
    chassis_tx_message1.RTR = CAN_RTR_DATA;
    chassis_tx_message1.DLC = 0x08;
    chassis_can_send_data1[0] =temp[0].uint8_t[0];
    chassis_can_send_data1[1] =temp[0].uint8_t[1];
    chassis_can_send_data1[2] =temp[0].uint8_t[2];
    chassis_can_send_data1[3] =temp[0].uint8_t[3];
	chassis_can_send_data1[4] =chassis_to_gimbal->allowance_42mm>>8;
	chassis_can_send_data1[5] =chassis_to_gimbal->allowance_42mm;
	chassis_can_send_data1[6] =0;
	chassis_can_send_data1[7] =0;
	 while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0)
	 {} ;
    HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message1, chassis_can_send_data1, &send_mail_box);
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
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
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

/**
  * @brief          返回云台发送给底盘数据设定值指针
  * @retval         底盘数据设定值指针
  */
const chassis_data_t *get_Chassisdata_point()
{
    return &chassis_data;
}