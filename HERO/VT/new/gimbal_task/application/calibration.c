#include "calibration.h"
#include "main.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "VT13_control.h"
gryo_t gryo_cali;

float gyro_offset[3]={0,0,0};
uint16_t count_time=0;
uint8_t cali_flag=0;


void calibration_task(void const *pvParameters)
{
	gryo_init(&gryo_cali);
	while(1)
	{
		memcpy(gryo_cali.gryo_get, gryo_cali.Gyro, sizeof(gryo_cali.gryo_get));
		if(gryo_cali.gimbal_rc_ctrl->rc.mode_sw==0&&gryo_cali.gimbal_rc_ctrl->rc.trigger==1&&cali_flag==0)
		{
			if(count_time<=10000)
			{
				gyro_offset_calc(gyro_offset,gryo_cali.gryo_get,&count_time);
			}

			else
			{
				count_time=0;
				cali_flag=1;
			}
		}
	}
	
	vTaskDelay(100);
	
}



void gryo_init(gryo_t *init)
{
	init->Gyro=get_gyro_data_point( );
	init->gimbal_rc_ctrl=get_remote_control_point1( );
	
	
}


/**
  * @brief          calculate gyro zero drift
  * @param[out]     gyro_offset:zero drift
  * @param[in]      gyro:gyro data
  * @param[out]     offset_time_count: +1 auto
  * @retval         none
  */
/**
  * @brief          计算陀螺仪零漂
  * @param[out]     gyro_offset:计算零漂
  * @param[in]      gyro:角速度数据
  * @param[out]     offset_time_count: 自动加1
  * @retval         none
  */
void gyro_offset_calc(float gyro_offset[3], float gyro[3], uint16_t *offset_time_count)
{
    if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
    {
        return;
    }

        gyro_offset[0] = gyro_offset[0] - 0.0003f * gyro[0];
        gyro_offset[1] = gyro_offset[1] - 0.0003f * gyro[1];
        gyro_offset[2] = gyro_offset[2] - 0.0003f * gyro[2];
        (*offset_time_count)++;
}