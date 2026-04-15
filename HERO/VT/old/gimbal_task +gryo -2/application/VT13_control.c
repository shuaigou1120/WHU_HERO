#include "main.h"
#include "string.h"
#include "detect_task.h"
#include "VT13_control.h"
#include "CRC8_CRC16.h"


extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

remote_data_t rc_ctrl;
uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM1];




/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM1);
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const remote_data_t *get_remote_control_point1(void)
{
    return &rc_ctrl;
}

void USART6_IRQHandler(void)
{
    if(huart6.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
    }
    else if(USART6->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart6);

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM1 - hdma_usart6_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = SBUS_RX_BUF_NUM1;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            
				
				if(sbus_rx_buf[0][0]==0xA9&&sbus_rx_buf[0][1]==0x53)
				
			{
				
				if(this_time_rx_len == RC_FRAME_LENGTH1)
				{
                //处理遥控器数据
                rc_remote(sbus_rx_buf[0], &rc_ctrl);
                //记录数据接收时间
//                detect_hook(DBUS_TOE);
				}
			
				
			}
			
				
			
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM1 - hdma_usart6_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = SBUS_RX_BUF_NUM1;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);



			
				
				if(sbus_rx_buf[1][0]==0xA9&&sbus_rx_buf[1][1]==0x53)
			{
				
				if(this_time_rx_len == RC_FRAME_LENGTH1)
				{
                //处理遥控器数据
                rc_remote(sbus_rx_buf[1], &rc_ctrl);
                //记录数据接收时间
//                detect_hook(DBUS_TOE);
				}
			
			}
			
				
			
        }
    }

}


/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void rc_remote(volatile const uint8_t *sbus_buf, remote_data_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }
		
	//遥控器键位解析
	    rc_ctrl->sof_1=sbus_buf[0];
	    rc_ctrl->sof_2=sbus_buf[1];
		rc_ctrl->rc.ch_0 = (sbus_buf[2] | (sbus_buf[3] << 8)) & 0x07ff;        //!< Channel 0
		rc_ctrl->rc.ch_1 = ((sbus_buf[3] >> 3) | (sbus_buf[4] << 5)) & 0x07ff; //!< Channel 1
		rc_ctrl->rc.ch_2 = ((sbus_buf[4] >> 6) | (sbus_buf[5] << 2) |          //!< Channel 2
							 (sbus_buf[6] << 10)) &0x07ff;
		rc_ctrl->rc.ch_3 = ((sbus_buf[6] >> 1) | (sbus_buf[7] << 7)) & 0x07ff; //!< Channel 3
		
		rc_ctrl->rc.mode_sw=(sbus_buf[7] >> 4)&0x0003;
		rc_ctrl->rc.pause=(sbus_buf[7] >> 6)&0x0001;
		rc_ctrl->rc.fn_1=(sbus_buf[7] >> 7)&0x0001;
		
		rc_ctrl->rc.fn_2=(sbus_buf[8])&0x0001;
		rc_ctrl->rc.wheel=(sbus_buf[8]>>1|sbus_buf[9]<<7)&0x07ff;
		rc_ctrl->rc.trigger=(sbus_buf[9]>>4)&0x0001;
	//键鼠键位解析
		rc_ctrl->mouse.mouse_x=(sbus_buf[10]|sbus_buf[11]<<8)& 0xffff; 
		rc_ctrl->mouse.mouse_y=(sbus_buf[12]|sbus_buf[13]<<8)& 0xffff;
		rc_ctrl->mouse.mouse_z=(sbus_buf[14]|sbus_buf[15]<<8)& 0xffff;
		
		rc_ctrl->mouse.mouse_left=sbus_buf[16]&0x0003;
		rc_ctrl->mouse.mouse_right=(sbus_buf[16]>>2)&0x0003;
		rc_ctrl->mouse.mouse_middle=(sbus_buf[16]>>4)&0x0003;
		
		rc_ctrl->key.v=(sbus_buf[17]|sbus_buf[18]<<8)&0xffff;

	//拨杆对中值
		rc_ctrl->rc.ch_0 -= RC_CH_VALUE_OFFSET;
		rc_ctrl->rc.ch_1 -= RC_CH_VALUE_OFFSET;
		rc_ctrl->rc.ch_2 -= RC_CH_VALUE_OFFSET;
		rc_ctrl->rc.ch_3 -= RC_CH_VALUE_OFFSET;
		rc_ctrl->rc.wheel-= RC_CH_VALUE_OFFSET;

		
	
	
    
}


//取正函数
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}


//错误复位函数
void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM1);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM1);
}


//判断遥控器数据是否出错，
uint8_t RC_data_is_error(void)
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_abs(rc_ctrl.rc.ch_0) > 700)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch_1) > 700)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch_2) > 700)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch_3) > 700)
    {
        goto error;
    }
    
    return 0;

error:
    rc_ctrl.rc.ch_0 = 0;
    rc_ctrl.rc.ch_1 = 0;
    rc_ctrl.rc.ch_2 = 0;
    rc_ctrl.rc.ch_3 = 0;
    rc_ctrl.mouse.mouse_x = 0;
    rc_ctrl.mouse.mouse_y = 0;
    rc_ctrl.mouse.mouse_z = 0;
    rc_ctrl.mouse.mouse_left = 0;
    rc_ctrl.mouse.mouse_right = 0;
    rc_ctrl.key.v = 0;
    return 1;
}


