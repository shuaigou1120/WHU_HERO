#ifndef GRYO_H
#define GRYO_H

#include "CAN_receive.h"


typedef struct 
{
	const gryo_data_t *gryo_get;
	int32_t YawRoundNumber;
	float all_yaw;
	float last_yaw;
	float yaw;
	float init_offset;    // 初始yaw偏移量
    uint8_t init_flag;    // 初始化完成标志
} gryo_get_t;


typedef struct 
{
	float q[4];
	
} q_data_t;

extern q_data_t q;

void gryo_task(void const *pvParameters);
void gryo_init( gryo_get_t *init);
void gryo_feedback(gryo_get_t *feedback);
void ToQuaternion(float yaw, float pitch, float roll);
extern const  gryo_get_t *get_gryo(void);
extern const q_data_t *get_q(void);

#endif