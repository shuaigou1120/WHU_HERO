//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_H
#define UI_H
#ifdef __cplusplus
extern "C" {
#endif

#include "ui_interface.h"

#include "ui_MODE.h"

void ui_init_MODE_mode();
void ui_update_MODE_mode();
void ui_remove_MODE_mode();
void ui_init_MODE_text();
void ui_update_MODE_text();
void ui_remove_MODE_text();
#include "ui_default.h"

#include "ui_pos.h"

void ui_init_pos_rect();
void ui_update_pos_rect();
void ui_remove_pos_rect();
void ui_init_pos_static();
void ui_update_pos_static();
void ui_remove_pos_static();

#ifdef __cplusplus
}
#endif

#endif // UI_H
