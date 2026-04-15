#include "main.h"
#include "tim.h"
void laser_on(void)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 8399);
}
void laser_off(void)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
}