#ifndef VT13_CONTROL_H
#define VT13_CONTROL_H

#include "struct_typedef.h"
#include "bsp_rc.h"






#define SBUS_RX_BUF_NUM1 42u
#define RC_FRAME_LENGTH1 21u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)


/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)

/* ----------------------- Data Struct ------------------------------------- */
typedef __packed struct
{
    uint8_t sof_1;
    uint8_t sof_2;
	__packed struct
	{
		int64_t ch_0:11;
		int64_t ch_1:11;
		int64_t ch_2:11;
		int64_t ch_3:11;
		uint64_t mode_sw:2;
		uint64_t pause:1;
		uint64_t fn_1:1;
		uint64_t fn_2:1;
		int64_t wheel:11;
		uint64_t trigger:1;
	}rc;
	
	__packed struct
	{
		int16_t mouse_x;
		int16_t mouse_y;
		int16_t mouse_z;
		uint8_t mouse_left:2;
		uint8_t mouse_right:2;
		uint8_t mouse_middle:2;
	}mouse;
	
	__packed struct
    {
        uint16_t v;
    } key;
	
    uint16_t crc16;
}remote_data_t;

extern void remote_control_init(void);
extern const remote_data_t *get_remote_control_point1(void);
extern uint8_t RC_data_is_error(void);
extern void slove_RC_lost(void);
extern void slove_data_error(void);
static void rc_remote(volatile const uint8_t *sbus_buf, remote_data_t *rc_ctrl);

#endif
