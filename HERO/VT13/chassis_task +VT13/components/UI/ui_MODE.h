//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_MODE_H
#define UI_MODE_H

#include "ui_interface.h"

extern ui_interface_round_t *ui_MODE_mode_aim;
extern ui_interface_round_t *ui_MODE_mode_follow;
extern ui_interface_round_t *ui_MODE_mode_top;
extern ui_interface_arc_t *ui_MODE_mode_direction;
extern ui_interface_rect_t *ui_MODE_mode_Rect;

void ui_init_MODE_mode();
void ui_update_MODE_mode();
void ui_remove_MODE_mode();

extern ui_interface_string_t *ui_MODE_text_A;
extern ui_interface_string_t *ui_MODE_text_mode_text;
extern ui_interface_string_t *ui_MODE_text_F;
extern ui_interface_string_t *ui_MODE_text_T;
extern ui_interface_string_t *ui_MODE_text_hero_text;

void ui_init_MODE_text();
void ui_update_MODE_text();
void ui_remove_MODE_text();


#endif // UI_MODE_H
