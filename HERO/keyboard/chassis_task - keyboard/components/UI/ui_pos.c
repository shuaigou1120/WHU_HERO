//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_7_frame_t ui_pos_static_0;

ui_interface_line_t *ui_pos_static_posline1 = (ui_interface_line_t*)&(ui_pos_static_0.data[0]);
ui_interface_line_t *ui_pos_static_posline2 = (ui_interface_line_t*)&(ui_pos_static_0.data[1]);
ui_interface_line_t *ui_pos_static_aimline1 = (ui_interface_line_t*)&(ui_pos_static_0.data[2]);
ui_interface_line_t *ui_pos_static_aimline2 = (ui_interface_line_t*)&(ui_pos_static_0.data[3]);
ui_interface_line_t *ui_pos_static_aimline3 = (ui_interface_line_t*)&(ui_pos_static_0.data[4]);
ui_interface_line_t *ui_pos_static_aimline4 = (ui_interface_line_t*)&(ui_pos_static_0.data[5]);
ui_interface_line_t *ui_pos_static_aimline5 = (ui_interface_line_t*)&(ui_pos_static_0.data[6]);

void _ui_init_pos_static_0() {
    for (int i = 0; i < 7; i++) {
        ui_pos_static_0.data[i].figure_name[0] = 2;
        ui_pos_static_0.data[i].figure_name[1] = 0;
        ui_pos_static_0.data[i].figure_name[2] = i + 0;
        ui_pos_static_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_pos_static_0.data[i].operate_type = 0;
    }

    ui_pos_static_posline1->figure_type = 0;
    ui_pos_static_posline1->operate_type = 1;
    ui_pos_static_posline1->layer = 0;
    ui_pos_static_posline1->color = 2;
    ui_pos_static_posline1->start_x = 507;
    ui_pos_static_posline1->start_y = 29;
    ui_pos_static_posline1->width = 3;
    ui_pos_static_posline1->end_x = 740;
    ui_pos_static_posline1->end_y = 370;

    ui_pos_static_posline2->figure_type = 0;
    ui_pos_static_posline2->operate_type = 1;
    ui_pos_static_posline2->layer = 0;
    ui_pos_static_posline2->color = 2;
    ui_pos_static_posline2->start_x = 1212;
    ui_pos_static_posline2->start_y = 375;
    ui_pos_static_posline2->width = 3;
    ui_pos_static_posline2->end_x = 1458;
    ui_pos_static_posline2->end_y = 20;

    ui_pos_static_aimline1->figure_type = 0;
    ui_pos_static_aimline1->operate_type = 1;
    ui_pos_static_aimline1->layer = 1;
    ui_pos_static_aimline1->color = 8;
    ui_pos_static_aimline1->start_x = 598;
    ui_pos_static_aimline1->start_y = 536;
    ui_pos_static_aimline1->width = 3;
    ui_pos_static_aimline1->end_x = 1333;
    ui_pos_static_aimline1->end_y = 536;

    ui_pos_static_aimline2->figure_type = 0;
    ui_pos_static_aimline2->operate_type = 1;
    ui_pos_static_aimline2->layer = 1;
    ui_pos_static_aimline2->color = 8;
    ui_pos_static_aimline2->start_x = 763;
    ui_pos_static_aimline2->start_y = 448;
    ui_pos_static_aimline2->width = 2;
    ui_pos_static_aimline2->end_x = 1170;
    ui_pos_static_aimline2->end_y = 448;

    ui_pos_static_aimline3->figure_type = 0;
    ui_pos_static_aimline3->operate_type = 1;
    ui_pos_static_aimline3->layer = 1;
    ui_pos_static_aimline3->color = 8;
    ui_pos_static_aimline3->start_x = 735;
    ui_pos_static_aimline3->start_y = 385;
    ui_pos_static_aimline3->width = 4;
    ui_pos_static_aimline3->end_x = 1223;
    ui_pos_static_aimline3->end_y = 385;

    ui_pos_static_aimline4->figure_type = 0;
    ui_pos_static_aimline4->operate_type = 1;
    ui_pos_static_aimline4->layer = 1;
    ui_pos_static_aimline4->color = 8;
    ui_pos_static_aimline4->start_x = 783;
    ui_pos_static_aimline4->start_y = 493;
    ui_pos_static_aimline4->width = 1;
    ui_pos_static_aimline4->end_x = 1141;
    ui_pos_static_aimline4->end_y = 493;

    ui_pos_static_aimline5->figure_type = 0;
    ui_pos_static_aimline5->operate_type = 1;
    ui_pos_static_aimline5->layer = 1;
    ui_pos_static_aimline5->color = 8;
    ui_pos_static_aimline5->start_x = 961;
    ui_pos_static_aimline5->start_y = 226;
    ui_pos_static_aimline5->width = 2;
    ui_pos_static_aimline5->end_x = 961;
    ui_pos_static_aimline5->end_y = 694;


    ui_proc_7_frame(&ui_pos_static_0);
    SEND_MESSAGE((uint8_t *) &ui_pos_static_0, sizeof(ui_pos_static_0));
}

void _ui_update_pos_static_0() {
    for (int i = 0; i < 7; i++) {
        ui_pos_static_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_pos_static_0);
    SEND_MESSAGE((uint8_t *) &ui_pos_static_0, sizeof(ui_pos_static_0));
}

void _ui_remove_pos_static_0() {
    for (int i = 0; i < 7; i++) {
        ui_pos_static_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_pos_static_0);
    SEND_MESSAGE((uint8_t *) &ui_pos_static_0, sizeof(ui_pos_static_0));
}


void ui_init_pos_static() {
    _ui_init_pos_static_0();
}

void ui_update_pos_static() {
    _ui_update_pos_static_0();
}

void ui_remove_pos_static() {
    _ui_remove_pos_static_0();
}

