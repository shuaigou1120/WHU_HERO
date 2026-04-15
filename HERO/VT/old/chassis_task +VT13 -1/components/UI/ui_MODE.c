//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_7_frame_t ui_MODE_mode_0;

ui_interface_round_t *ui_MODE_mode_aim = (ui_interface_round_t*)&(ui_MODE_mode_0.data[0]);
ui_interface_line_t *ui_MODE_mode_FIRE = (ui_interface_line_t*)&(ui_MODE_mode_0.data[1]);
ui_interface_round_t *ui_MODE_mode_follow = (ui_interface_round_t*)&(ui_MODE_mode_0.data[2]);
ui_interface_round_t *ui_MODE_mode_top = (ui_interface_round_t*)&(ui_MODE_mode_0.data[3]);
ui_interface_arc_t *ui_MODE_mode_direction = (ui_interface_arc_t*)&(ui_MODE_mode_0.data[4]);
ui_interface_rect_t *ui_MODE_mode_Rect = (ui_interface_rect_t*)&(ui_MODE_mode_0.data[5]);

void _ui_init_MODE_mode_0() {
    for (int i = 0; i < 6; i++) {
        ui_MODE_mode_0.data[i].figure_name[0] = 0;
        ui_MODE_mode_0.data[i].figure_name[1] = 0;
        ui_MODE_mode_0.data[i].figure_name[2] = i + 0;
        ui_MODE_mode_0.data[i].operate_type = 1;
    }
    for (int i = 6; i < 7; i++) {
        ui_MODE_mode_0.data[i].operate_type = 0;
    }

    ui_MODE_mode_aim->figure_type = 2;
    ui_MODE_mode_aim->operate_type = 1;
    ui_MODE_mode_aim->layer = 0;
    ui_MODE_mode_aim->color = 8;
    ui_MODE_mode_aim->start_x = 1415;
    ui_MODE_mode_aim->start_y = 749;
    ui_MODE_mode_aim->width = 10;
    ui_MODE_mode_aim->r = 35;

    ui_MODE_mode_FIRE->figure_type = 0;
    ui_MODE_mode_FIRE->operate_type = 1;
    ui_MODE_mode_FIRE->layer = 0;
    ui_MODE_mode_FIRE->color = 8;
    ui_MODE_mode_FIRE->start_x = 393;
    ui_MODE_mode_FIRE->start_y = 550;
    ui_MODE_mode_FIRE->width = 35;
    ui_MODE_mode_FIRE->end_x = 479;
    ui_MODE_mode_FIRE->end_y = 550;

    ui_MODE_mode_follow->figure_type = 2;
    ui_MODE_mode_follow->operate_type = 1;
    ui_MODE_mode_follow->layer = 0;
    ui_MODE_mode_follow->color = 8;
    ui_MODE_mode_follow->start_x = 1515;
    ui_MODE_mode_follow->start_y = 515;
    ui_MODE_mode_follow->width = 10;
    ui_MODE_mode_follow->r = 35;

    ui_MODE_mode_top->figure_type = 2;
    ui_MODE_mode_top->operate_type = 1;
    ui_MODE_mode_top->layer = 0;
    ui_MODE_mode_top->color = 8;
    ui_MODE_mode_top->start_x = 1446;
    ui_MODE_mode_top->start_y = 395;
    ui_MODE_mode_top->width = 10;
    ui_MODE_mode_top->r = 35;

    ui_MODE_mode_direction->figure_type = 4;
    ui_MODE_mode_direction->operate_type = 1;
    ui_MODE_mode_direction->layer = 0;
    ui_MODE_mode_direction->color = 5;
    ui_MODE_mode_direction->start_x = 433;
    ui_MODE_mode_direction->start_y = 729;
    ui_MODE_mode_direction->width = 20;
    ui_MODE_mode_direction->start_angle = 215;
    ui_MODE_mode_direction->end_angle = 145;
    ui_MODE_mode_direction->rx = 80;
    ui_MODE_mode_direction->ry = 80;

    ui_MODE_mode_Rect->figure_type = 1;
    ui_MODE_mode_Rect->operate_type = 1;
    ui_MODE_mode_Rect->layer = 0;
    ui_MODE_mode_Rect->color = 8;
    ui_MODE_mode_Rect->start_x = 1367;
    ui_MODE_mode_Rect->start_y = 307;
    ui_MODE_mode_Rect->width = 2;
    ui_MODE_mode_Rect->end_x = 1571;
    ui_MODE_mode_Rect->end_y = 856;


    ui_proc_7_frame(&ui_MODE_mode_0);
    SEND_MESSAGE((uint8_t *) &ui_MODE_mode_0, sizeof(ui_MODE_mode_0));
}

void _ui_update_MODE_mode_0() {
    for (int i = 0; i < 6; i++) {
        ui_MODE_mode_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_MODE_mode_0);
    SEND_MESSAGE((uint8_t *) &ui_MODE_mode_0, sizeof(ui_MODE_mode_0));
}

void _ui_remove_MODE_mode_0() {
    for (int i = 0; i < 6; i++) {
        ui_MODE_mode_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_MODE_mode_0);
    SEND_MESSAGE((uint8_t *) &ui_MODE_mode_0, sizeof(ui_MODE_mode_0));
}

ui_string_frame_t ui_MODE_mode_1;
ui_interface_string_t* ui_MODE_mode_fire_text = &(ui_MODE_mode_1.option);

void _ui_init_MODE_mode_1() {
    ui_MODE_mode_1.option.figure_name[0] = 0;
    ui_MODE_mode_1.option.figure_name[1] = 0;
    ui_MODE_mode_1.option.figure_name[2] = 6;
    ui_MODE_mode_1.option.operate_type = 1;

    ui_MODE_mode_fire_text->figure_type = 7;
    ui_MODE_mode_fire_text->operate_type = 1;
    ui_MODE_mode_fire_text->layer = 0;
    ui_MODE_mode_fire_text->color = 1;
    ui_MODE_mode_fire_text->start_x = 395;
    ui_MODE_mode_fire_text->start_y = 550;
    ui_MODE_mode_fire_text->width = 2;
    ui_MODE_mode_fire_text->font_size = 20;
    ui_MODE_mode_fire_text->str_length = 4;
    strcpy(ui_MODE_mode_fire_text->string, "FIRE");


    ui_proc_string_frame(&ui_MODE_mode_1);
    SEND_MESSAGE((uint8_t *) &ui_MODE_mode_1, sizeof(ui_MODE_mode_1));
}

void _ui_update_MODE_mode_1() {
    ui_MODE_mode_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_MODE_mode_1);
    SEND_MESSAGE((uint8_t *) &ui_MODE_mode_1, sizeof(ui_MODE_mode_1));
}

void _ui_remove_MODE_mode_1() {
    ui_MODE_mode_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_MODE_mode_1);
    SEND_MESSAGE((uint8_t *) &ui_MODE_mode_1, sizeof(ui_MODE_mode_1));
}

void ui_init_MODE_mode() {
    _ui_init_MODE_mode_0();
    _ui_init_MODE_mode_1();
}

void ui_update_MODE_mode() {
    _ui_update_MODE_mode_0();
    _ui_update_MODE_mode_1();
}

void ui_remove_MODE_mode() {
    _ui_remove_MODE_mode_0();
    _ui_remove_MODE_mode_1();
}

ui_1_frame_t ui_MODE_text_0;

ui_interface_round_t *ui_MODE_text_unload = (ui_interface_round_t*)&(ui_MODE_text_0.data[0]);

void _ui_init_MODE_text_0() {
    for (int i = 0; i < 1; i++) {
        ui_MODE_text_0.data[i].figure_name[0] = 0;
        ui_MODE_text_0.data[i].figure_name[1] = 1;
        ui_MODE_text_0.data[i].figure_name[2] = i + 0;
        ui_MODE_text_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_MODE_text_0.data[i].operate_type = 0;
    }

    ui_MODE_text_unload->figure_type = 2;
    ui_MODE_text_unload->operate_type = 1;
    ui_MODE_text_unload->layer = 0;
    ui_MODE_text_unload->color = 8;
    ui_MODE_text_unload->start_x = 1512;
    ui_MODE_text_unload->start_y = 669;
    ui_MODE_text_unload->width = 10;
    ui_MODE_text_unload->r = 45;


    ui_proc_1_frame(&ui_MODE_text_0);
    SEND_MESSAGE((uint8_t *) &ui_MODE_text_0, sizeof(ui_MODE_text_0));
}

void _ui_update_MODE_text_0() {
    for (int i = 0; i < 1; i++) {
        ui_MODE_text_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_MODE_text_0);
    SEND_MESSAGE((uint8_t *) &ui_MODE_text_0, sizeof(ui_MODE_text_0));
}

void _ui_remove_MODE_text_0() {
    for (int i = 0; i < 1; i++) {
        ui_MODE_text_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_MODE_text_0);
    SEND_MESSAGE((uint8_t *) &ui_MODE_text_0, sizeof(ui_MODE_text_0));
}

ui_string_frame_t ui_MODE_text_1;
ui_interface_string_t* ui_MODE_text_A = &(ui_MODE_text_1.option);

void _ui_init_MODE_text_1() {
    ui_MODE_text_1.option.figure_name[0] = 0;
    ui_MODE_text_1.option.figure_name[1] = 1;
    ui_MODE_text_1.option.figure_name[2] = 1;
    ui_MODE_text_1.option.operate_type = 1;

    ui_MODE_text_A->figure_type = 7;
    ui_MODE_text_A->operate_type = 1;
    ui_MODE_text_A->layer = 0;
    ui_MODE_text_A->color = 8;
    ui_MODE_text_A->start_x = 1408;
    ui_MODE_text_A->start_y = 769;
    ui_MODE_text_A->width = 2;
    ui_MODE_text_A->font_size = 24;
    ui_MODE_text_A->str_length = 1;
    strcpy(ui_MODE_text_A->string, "A");


    ui_proc_string_frame(&ui_MODE_text_1);
    SEND_MESSAGE((uint8_t *) &ui_MODE_text_1, sizeof(ui_MODE_text_1));
}

void _ui_update_MODE_text_1() {
    ui_MODE_text_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_MODE_text_1);
    SEND_MESSAGE((uint8_t *) &ui_MODE_text_1, sizeof(ui_MODE_text_1));
}

void _ui_remove_MODE_text_1() {
    ui_MODE_text_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_MODE_text_1);
    SEND_MESSAGE((uint8_t *) &ui_MODE_text_1, sizeof(ui_MODE_text_1));
}
ui_string_frame_t ui_MODE_text_2;
ui_interface_string_t* ui_MODE_text_mode_text = &(ui_MODE_text_2.option);

void _ui_init_MODE_text_2() {
    ui_MODE_text_2.option.figure_name[0] = 0;
    ui_MODE_text_2.option.figure_name[1] = 1;
    ui_MODE_text_2.option.figure_name[2] = 2;
    ui_MODE_text_2.option.operate_type = 1;

    ui_MODE_text_mode_text->figure_type = 7;
    ui_MODE_text_mode_text->operate_type = 1;
    ui_MODE_text_mode_text->layer = 0;
    ui_MODE_text_mode_text->color = 0;
    ui_MODE_text_mode_text->start_x = 1471;
    ui_MODE_text_mode_text->start_y = 820;
    ui_MODE_text_mode_text->width = 2;
    ui_MODE_text_mode_text->font_size = 20;
    ui_MODE_text_mode_text->str_length = 4;
    strcpy(ui_MODE_text_mode_text->string, "MODE");


    ui_proc_string_frame(&ui_MODE_text_2);
    SEND_MESSAGE((uint8_t *) &ui_MODE_text_2, sizeof(ui_MODE_text_2));
}

void _ui_update_MODE_text_2() {
    ui_MODE_text_2.option.operate_type = 2;

    ui_proc_string_frame(&ui_MODE_text_2);
    SEND_MESSAGE((uint8_t *) &ui_MODE_text_2, sizeof(ui_MODE_text_2));
}

void _ui_remove_MODE_text_2() {
    ui_MODE_text_2.option.operate_type = 3;

    ui_proc_string_frame(&ui_MODE_text_2);
    SEND_MESSAGE((uint8_t *) &ui_MODE_text_2, sizeof(ui_MODE_text_2));
}
ui_string_frame_t ui_MODE_text_3;
ui_interface_string_t* ui_MODE_text_unload_text = &(ui_MODE_text_3.option);

void _ui_init_MODE_text_3() {
    ui_MODE_text_3.option.figure_name[0] = 0;
    ui_MODE_text_3.option.figure_name[1] = 1;
    ui_MODE_text_3.option.figure_name[2] = 3;
    ui_MODE_text_3.option.operate_type = 1;

    ui_MODE_text_unload_text->figure_type = 7;
    ui_MODE_text_unload_text->operate_type = 1;
    ui_MODE_text_unload_text->layer = 0;
    ui_MODE_text_unload_text->color = 8;
    ui_MODE_text_unload_text->start_x = 1478;
    ui_MODE_text_unload_text->start_y = 680;
    ui_MODE_text_unload_text->width = 1;
    ui_MODE_text_unload_text->font_size = 13;
    ui_MODE_text_unload_text->str_length = 6;
    strcpy(ui_MODE_text_unload_text->string, "Unload");


    ui_proc_string_frame(&ui_MODE_text_3);
    SEND_MESSAGE((uint8_t *) &ui_MODE_text_3, sizeof(ui_MODE_text_3));
}

void _ui_update_MODE_text_3() {
    ui_MODE_text_3.option.operate_type = 2;

    ui_proc_string_frame(&ui_MODE_text_3);
    SEND_MESSAGE((uint8_t *) &ui_MODE_text_3, sizeof(ui_MODE_text_3));
}

void _ui_remove_MODE_text_3() {
    ui_MODE_text_3.option.operate_type = 3;

    ui_proc_string_frame(&ui_MODE_text_3);
    SEND_MESSAGE((uint8_t *) &ui_MODE_text_3, sizeof(ui_MODE_text_3));
}
ui_string_frame_t ui_MODE_text_4;
ui_interface_string_t* ui_MODE_text_F = &(ui_MODE_text_4.option);

void _ui_init_MODE_text_4() {
    ui_MODE_text_4.option.figure_name[0] = 0;
    ui_MODE_text_4.option.figure_name[1] = 1;
    ui_MODE_text_4.option.figure_name[2] = 4;
    ui_MODE_text_4.option.operate_type = 1;

    ui_MODE_text_F->figure_type = 7;
    ui_MODE_text_F->operate_type = 1;
    ui_MODE_text_F->layer = 0;
    ui_MODE_text_F->color = 8;
    ui_MODE_text_F->start_x = 1506;
    ui_MODE_text_F->start_y = 535;
    ui_MODE_text_F->width = 2;
    ui_MODE_text_F->font_size = 24;
    ui_MODE_text_F->str_length = 1;
    strcpy(ui_MODE_text_F->string, "F");


    ui_proc_string_frame(&ui_MODE_text_4);
    SEND_MESSAGE((uint8_t *) &ui_MODE_text_4, sizeof(ui_MODE_text_4));
}

void _ui_update_MODE_text_4() {
    ui_MODE_text_4.option.operate_type = 2;

    ui_proc_string_frame(&ui_MODE_text_4);
    SEND_MESSAGE((uint8_t *) &ui_MODE_text_4, sizeof(ui_MODE_text_4));
}

void _ui_remove_MODE_text_4() {
    ui_MODE_text_4.option.operate_type = 3;

    ui_proc_string_frame(&ui_MODE_text_4);
    SEND_MESSAGE((uint8_t *) &ui_MODE_text_4, sizeof(ui_MODE_text_4));
}
ui_string_frame_t ui_MODE_text_5;
ui_interface_string_t* ui_MODE_text_T = &(ui_MODE_text_5.option);

void _ui_init_MODE_text_5() {
    ui_MODE_text_5.option.figure_name[0] = 0;
    ui_MODE_text_5.option.figure_name[1] = 1;
    ui_MODE_text_5.option.figure_name[2] = 5;
    ui_MODE_text_5.option.operate_type = 1;

    ui_MODE_text_T->figure_type = 7;
    ui_MODE_text_T->operate_type = 1;
    ui_MODE_text_T->layer = 0;
    ui_MODE_text_T->color = 8;
    ui_MODE_text_T->start_x = 1420;
    ui_MODE_text_T->start_y = 409;
    ui_MODE_text_T->width = 2;
    ui_MODE_text_T->font_size = 20;
    ui_MODE_text_T->str_length = 3;
    strcpy(ui_MODE_text_T->string, "TOP");


    ui_proc_string_frame(&ui_MODE_text_5);
    SEND_MESSAGE((uint8_t *) &ui_MODE_text_5, sizeof(ui_MODE_text_5));
}

void _ui_update_MODE_text_5() {
    ui_MODE_text_5.option.operate_type = 2;

    ui_proc_string_frame(&ui_MODE_text_5);
    SEND_MESSAGE((uint8_t *) &ui_MODE_text_5, sizeof(ui_MODE_text_5));
}

void _ui_remove_MODE_text_5() {
    ui_MODE_text_5.option.operate_type = 3;

    ui_proc_string_frame(&ui_MODE_text_5);
    SEND_MESSAGE((uint8_t *) &ui_MODE_text_5, sizeof(ui_MODE_text_5));
}
ui_string_frame_t ui_MODE_text_6;
ui_interface_string_t* ui_MODE_text_hero_text = &(ui_MODE_text_6.option);

void _ui_init_MODE_text_6() {
    ui_MODE_text_6.option.figure_name[0] = 0;
    ui_MODE_text_6.option.figure_name[1] = 1;
    ui_MODE_text_6.option.figure_name[2] = 6;
    ui_MODE_text_6.option.operate_type = 1;

    ui_MODE_text_hero_text->figure_type = 7;
    ui_MODE_text_hero_text->operate_type = 1;
    ui_MODE_text_hero_text->layer = 0;
    ui_MODE_text_hero_text->color = 1;
    ui_MODE_text_hero_text->start_x = 400;
    ui_MODE_text_hero_text->start_y = 780;
    ui_MODE_text_hero_text->width = 2;
    ui_MODE_text_hero_text->font_size = 20;
    ui_MODE_text_hero_text->str_length = 4;
    strcpy(ui_MODE_text_hero_text->string, "HERO");


    ui_proc_string_frame(&ui_MODE_text_6);
    SEND_MESSAGE((uint8_t *) &ui_MODE_text_6, sizeof(ui_MODE_text_6));
}

void _ui_update_MODE_text_6() {
    ui_MODE_text_6.option.operate_type = 2;

    ui_proc_string_frame(&ui_MODE_text_6);
    SEND_MESSAGE((uint8_t *) &ui_MODE_text_6, sizeof(ui_MODE_text_6));
}

void _ui_remove_MODE_text_6() {
    ui_MODE_text_6.option.operate_type = 3;

    ui_proc_string_frame(&ui_MODE_text_6);
    SEND_MESSAGE((uint8_t *) &ui_MODE_text_6, sizeof(ui_MODE_text_6));
}

void ui_init_MODE_text() {
    _ui_init_MODE_text_0();
    _ui_init_MODE_text_1();
    _ui_init_MODE_text_2();
    _ui_init_MODE_text_3();
    _ui_init_MODE_text_4();
    _ui_init_MODE_text_5();
    _ui_init_MODE_text_6();
}

void ui_update_MODE_text() {
    _ui_update_MODE_text_0();
    _ui_update_MODE_text_1();
    _ui_update_MODE_text_2();
    _ui_update_MODE_text_3();
    _ui_update_MODE_text_4();
    _ui_update_MODE_text_5();
    _ui_update_MODE_text_6();
}

void ui_remove_MODE_text() {
    _ui_remove_MODE_text_0();
    _ui_remove_MODE_text_1();
    _ui_remove_MODE_text_2();
    _ui_remove_MODE_text_3();
    _ui_remove_MODE_text_4();
    _ui_remove_MODE_text_5();
    _ui_remove_MODE_text_6();
}

