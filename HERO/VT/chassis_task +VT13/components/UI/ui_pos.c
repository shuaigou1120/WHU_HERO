//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_1_frame_t ui_pos_rect_0;

ui_interface_rect_t *ui_pos_rect_auto_aim_rect = (ui_interface_rect_t*)&(ui_pos_rect_0.data[0]);

void _ui_init_pos_rect_0() {
    for (int i = 0; i < 1; i++) {
        ui_pos_rect_0.data[i].figure_name[0] = 2;
        ui_pos_rect_0.data[i].figure_name[1] = 0;
        ui_pos_rect_0.data[i].figure_name[2] = i + 0;
        ui_pos_rect_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_pos_rect_0.data[i].operate_type = 0;
    }

    ui_pos_rect_auto_aim_rect->figure_type = 1;
    ui_pos_rect_auto_aim_rect->operate_type = 1;
    ui_pos_rect_auto_aim_rect->layer = 0;
    ui_pos_rect_auto_aim_rect->color = 0;
    ui_pos_rect_auto_aim_rect->start_x = 631;
    ui_pos_rect_auto_aim_rect->start_y = 318;
    ui_pos_rect_auto_aim_rect->width = 4;
    ui_pos_rect_auto_aim_rect->end_x = 1264;
    ui_pos_rect_auto_aim_rect->end_y = 731;


    ui_proc_1_frame(&ui_pos_rect_0);
    SEND_MESSAGE((uint8_t *) &ui_pos_rect_0, sizeof(ui_pos_rect_0));
}

void _ui_update_pos_rect_0() {
    for (int i = 0; i < 1; i++) {
        ui_pos_rect_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_pos_rect_0);
    SEND_MESSAGE((uint8_t *) &ui_pos_rect_0, sizeof(ui_pos_rect_0));
}

void _ui_remove_pos_rect_0() {
    for (int i = 0; i < 1; i++) {
        ui_pos_rect_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_pos_rect_0);
    SEND_MESSAGE((uint8_t *) &ui_pos_rect_0, sizeof(ui_pos_rect_0));
}


void ui_init_pos_rect() {
    _ui_init_pos_rect_0();
}

void ui_update_pos_rect() {
    _ui_update_pos_rect_0();
}

void ui_remove_pos_rect() {
    _ui_remove_pos_rect_0();
}

ui_7_frame_t ui_pos_static_0;

ui_interface_line_t *ui_pos_static_posline1 = (ui_interface_line_t*)&(ui_pos_static_0.data[0]);
ui_interface_line_t *ui_pos_static_posline2 = (ui_interface_line_t*)&(ui_pos_static_0.data[1]);
ui_interface_line_t *ui_pos_static_aimline = (ui_interface_line_t*)&(ui_pos_static_0.data[2]);
ui_interface_line_t *ui_pos_static_aimline1 = (ui_interface_line_t*)&(ui_pos_static_0.data[3]);
ui_interface_line_t *ui_pos_static_aimline2 = (ui_interface_line_t*)&(ui_pos_static_0.data[4]);
ui_interface_line_t *ui_pos_static_aimline3 = (ui_interface_line_t*)&(ui_pos_static_0.data[5]);
ui_interface_line_t *ui_pos_static_aimline4 = (ui_interface_line_t*)&(ui_pos_static_0.data[6]);

void _ui_init_pos_static_0() {
    for (int i = 0; i < 7; i++) {
        ui_pos_static_0.data[i].figure_name[0] = 2;
        ui_pos_static_0.data[i].figure_name[1] = 1;
        ui_pos_static_0.data[i].figure_name[2] = i + 0;
        ui_pos_static_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_pos_static_0.data[i].operate_type = 0;
    }

    ui_pos_static_posline1->figure_type = 0;
    ui_pos_static_posline1->operate_type = 1;
    ui_pos_static_posline1->layer = 0;
    ui_pos_static_posline1->color = 3;
    ui_pos_static_posline1->start_x = 507;
    ui_pos_static_posline1->start_y = 29;
    ui_pos_static_posline1->width = 3;
    ui_pos_static_posline1->end_x = 740;
    ui_pos_static_posline1->end_y = 370;

    ui_pos_static_posline2->figure_type = 0;
    ui_pos_static_posline2->operate_type = 1;
    ui_pos_static_posline2->layer = 0;
    ui_pos_static_posline2->color = 3;
    ui_pos_static_posline2->start_x = 1212;
    ui_pos_static_posline2->start_y = 375;
    ui_pos_static_posline2->width = 3;
    ui_pos_static_posline2->end_x = 1458;
    ui_pos_static_posline2->end_y = 20;

    ui_pos_static_aimline->figure_type = 0;
    ui_pos_static_aimline->operate_type = 1;
    ui_pos_static_aimline->layer = 0;
    ui_pos_static_aimline->color = 3;
    ui_pos_static_aimline->start_x = 845;
    ui_pos_static_aimline->start_y = 479;
    ui_pos_static_aimline->width = 4;
    ui_pos_static_aimline->end_x = 1045;
    ui_pos_static_aimline->end_y = 479;

    ui_pos_static_aimline1->figure_type = 0;
    ui_pos_static_aimline1->operate_type = 1;
    ui_pos_static_aimline1->layer = 0;
    ui_pos_static_aimline1->color = 3;
    ui_pos_static_aimline1->start_x = 946;
    ui_pos_static_aimline1->start_y = 210;
    ui_pos_static_aimline1->width = 4;
    ui_pos_static_aimline1->end_x = 946;
    ui_pos_static_aimline1->end_y = 618;

    ui_pos_static_aimline2->figure_type = 0;
    ui_pos_static_aimline2->operate_type = 1;
    ui_pos_static_aimline2->layer = 0;
    ui_pos_static_aimline2->color = 3;
    ui_pos_static_aimline2->start_x = 801;
    ui_pos_static_aimline2->start_y = 453;
    ui_pos_static_aimline2->width = 4;
    ui_pos_static_aimline2->end_x = 1090;
    ui_pos_static_aimline2->end_y = 453;

    ui_pos_static_aimline3->figure_type = 0;
    ui_pos_static_aimline3->operate_type = 1;
    ui_pos_static_aimline3->layer = 0;
    ui_pos_static_aimline3->color = 3;
    ui_pos_static_aimline3->start_x = 886;
    ui_pos_static_aimline3->start_y = 499;
    ui_pos_static_aimline3->width = 4;
    ui_pos_static_aimline3->end_x = 1013;
    ui_pos_static_aimline3->end_y = 499;

    ui_pos_static_aimline4->figure_type = 0;
    ui_pos_static_aimline4->operate_type = 1;
    ui_pos_static_aimline4->layer = 0;
    ui_pos_static_aimline4->color = 0;
    ui_pos_static_aimline4->start_x = 908;
    ui_pos_static_aimline4->start_y = 35;
    ui_pos_static_aimline4->width = 1;
    ui_pos_static_aimline4->end_x = 906;
    ui_pos_static_aimline4->end_y = 35;


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

