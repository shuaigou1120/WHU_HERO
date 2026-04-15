#ifndef CHASSIS_TO_GIMBAL
#define CHASSIS_TO_GIMBAL

typedef  struct 
{
 float speed;
 uint16_t allowance_42mm; 
 
} chassis_to_gimbal_t;

void chassis_to_gimbal_task(void const *pvParameters);

#endif