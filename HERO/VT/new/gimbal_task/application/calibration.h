#ifndef _CALIBRATION_H
#define _CALIBRATION_H

#include "VT13_control.h"

extern float gyro_offset[3];


typedef struct
{

	const float *Gyro;
	
	const remote_data_t *gimbal_rc_ctrl;
	
	float gryo_get[3];
	
	
} gryo_t;



void calibration_task(void const *pvParameters);
void gryo_init(gryo_t *init);
void gyro_offset_calc(float gyro_offset[3], float gyro[3], uint16_t *offset_time_count);
#endif