#include "main.h"
#include "cmsis_os.h"
#include "chassis_to_gimbal.h"
#include "can.h"
#include "CAN_receive.h"
#include "referee.h"

chassis_to_gimbal_t chassis_to_gimbal;

void chassis_to_gimbal_task(void const *pvParameters)
{
	


	while(1)
	{
		get_speed_output(&chassis_to_gimbal.speed);
	    get_42mm_allowance(&chassis_to_gimbal.allowance_42mm);
		can_chassis_gimbal_data1(&chassis_to_gimbal);
		osDelay(10);
	}
	
	
}
