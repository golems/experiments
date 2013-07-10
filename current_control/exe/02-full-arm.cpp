/**
 * @file 02-full-arm.cpp
 * @author Saul Reynolds-Haertle
 * @date 2013-07-10

 * @briefs This executable demonstrates current-controlled jointspace
 * teleoperation of both arms. The interface follows the standard
 * joystick specification.
 */

#include "helpers.h"
#include "initModules.h"
#include "motion.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <ncurses.h>
#include <iomanip>

// ################################################################################
// definitions

typedef enum {
    JOYSTICK_MODE_POSITION = 0,
    JOYSTICK_MODE_DIRECT
} joystick_mode_t;
typedef enum {
    JOYSTICK_CONTROLLING_NONE = 0,
    JOYSTICK_CONTROLLING_LEFT_SMALL,
    JOYSTICK_CONTROLLING_LEFT_LARGE,
    JOYSTICK_CONTROLLING_RIGHT_SMALL,
    JOYSTICK_CONTROLLING_RIGHT_LARGE
} joystick_controlling_t;

// ################################################################################
// global variables

somatic_d_t daemon_cx;
ach_channel_t state_chan;
ach_channel_t cmd_chan;
ach_channel_t js_chan;
somatic_motor_t rlwa;
somatic_motor_t llwa;

double last_js_time;
joystick_mode_t js_mode;
joystick_controlling_t js_controlling;
Somatic__Joystick* last_js_msg;

// ################################################################################
// constants

float K_p[] = {15.0, 15.0, 15.0, 12.0, 15.0, 7.0, 7.0};
float K_d[] = {0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 0.0};

double joint_vel_command_scale = 0.30;
double joint_cur_command_scale = 8.0;

// ################################################################################
// small helper functions

double timespec_to_double(struct timespec x) {
    return (double)x.tv_sec + (double)x.tv_nsec / 1000000000.0;
}

double gettime() {
    struct timespec temp;
    clock_gettime(ACH_DEFAULT_CLOCK, &temp);
    return timespec_to_double(temp);
}

// ################################################################################
// display

double do_init_curses() {
    initscr();
    clear();
    noecho();                   // do not echo input to the screen
    cbreak();                   // do not buffer by line (receive characters immediately)
    timeout(0);                 // non-blocking getch
}

double do_end_curses() {
    clrtoeol();
    refresh();
    endwin();
}

#define AXIS_SLIDER_WIDTH 20
char input_display_buffer[(AXIS_SLIDER_WIDTH * 2) + 1];
double do_input_display(Somatic__Joystick* js_msg) {
    // column of ten buttons in the top middle
    mvprintw(1, 20, "buttons");
    for(int i = 0; i < 10; i++) {
        mvprintw(2 + i, 21, "b%d: %c", i+1, js_msg->buttons->data[i] ? '#' : ' ');
    }

    // fill in a temp buffer
    for(int i = 0; i < 2 * AXIS_SLIDER_WIDTH; i++) input_display_buffer[i] = ' ';
    input_display_buffer[(2 * AXIS_SLIDER_WIDTH)] = 0;

    // column of joystick axis values to the right of that
    mvprintw(1, 40, "axes");
    for(int i = 0; i < 4; i++) {
        mvprintw(2 + i, 41, "axis %d: [", i+1);
        mvprintw(2 + i, 41 + 9, input_display_buffer);
        mvprintw(2 + i, 41 + 9 + AXIS_SLIDER_WIDTH + AXIS_SLIDER_WIDTH, "]");
        mvprintw(2 + i, 41 + 9 + AXIS_SLIDER_WIDTH + (int)(AXIS_SLIDER_WIDTH * js_msg->axes->data[i]), "|");
    }

    // display controlling on the left
    mvprintw(1, 1, "controlling");
    for(int i = 2; i <= 6; i++) mvprintw(i, 2, "            ");
    switch(js_controlling) {
    case JOYSTICK_CONTROLLING_NONE:
        mvprintw(2, 2, "NONE");
        break;
    case JOYSTICK_CONTROLLING_LEFT_SMALL:
        mvprintw(3, 2, "LEFT SMALL");
        break;
    case JOYSTICK_CONTROLLING_LEFT_LARGE:
        mvprintw(4, 2, "LEFT LARGE");
        break;
    case JOYSTICK_CONTROLLING_RIGHT_SMALL:
        mvprintw(5, 2, "RIGHT SMALL");
        break;
    case JOYSTICK_CONTROLLING_RIGHT_LARGE:
        mvprintw(6, 2, "RIGHT LARGE");
        break;
    }

    mvprintw(8, 1, "mode");
    for(int i = 9; i <= 10; i++) mvprintw(i, 2, "            ");
    switch(js_mode) {
    case JOYSTICK_MODE_POSITION:
        mvprintw(9, 2, "POSITION");
        break;
    case JOYSTICK_MODE_DIRECT:
        mvprintw(10, 2, "DIRECT");
        break;
    }
}

double do_control_display(somatic_motor_t* llwa, somatic_motor_t* rlwa, 
                          double* ltarget_pos, double* ltarget_vel,
                          double* rtarget_pos, double* rtarget_vel,
                          double* lmessage,
                          double* rmessage) {

    // top row is for left arm, bottom row is right arm
    // from left to right: position target, position actual, velocity target, velocity actual, and commanded current
    mvprintw(15, 1, "left pos: target, actual");
    for(int i = 0; i < 7; i++) mvprintw(16+i, 2, "%f", ltarget_pos[i]);
    for(int i = 0; i < 7; i++) mvprintw(16+i, 22, "%f", llwa->pos[i]);
    mvprintw(24, 1, "right pos: target, actual");
    for(int i = 0; i < 7; i++) mvprintw(25+i, 2, "%f", rtarget_pos[i]);
    for(int i = 0; i < 7; i++) mvprintw(25+i, 22, "%f", rlwa->pos[i]);

    mvprintw(15, 41, "left vel: target, actual");
    for(int i = 0; i < 7; i++) mvprintw(16+i, 42, "%f", ltarget_vel[i]);
    for(int i = 0; i < 7; i++) mvprintw(16+i, 62, "%f", llwa->vel[i]);
    mvprintw(24, 41, "right vel: target, actual");
    for(int i = 0; i < 7; i++) mvprintw(25+i, 42, "%f", rtarget_vel[i]);
    for(int i = 0; i < 7; i++) mvprintw(25+i, 62, "%f", rlwa->vel[i]);

    mvprintw(15, 81, "left commanded current");
    for(int i = 0; i < 7; i++) mvprintw(16+i, 82, "%f", lmessage[i]);
    mvprintw(24, 81, "right commanded current");
    for(int i = 0; i < 7; i++) mvprintw(25+i, 82, "%f", rmessage[i]);
}

// ################################################################################
// functions that actually do things

int do_set_limits(somatic_motor_t* lwa) {
    // Set the min/max values for valid and limits values
    double** limits [] = {
        &lwa->pos_valid_min, &lwa->vel_valid_min, 
        &lwa->pos_limit_min, &lwa->vel_limit_min, 
        &lwa->pos_valid_max, &lwa->vel_valid_max, 
        &lwa->pos_limit_max, &lwa->vel_limit_max};
    for(size_t i = 0; i < 4; i++) aa_fset(*limits[i], -1024.1, 7);
    for(size_t i = 4; i < 8; i++) aa_fset(*limits[i], 1024.1, 7);
}

void do_init_joystick() {
    int r = ach_open(&js_chan, "joystick-data", NULL);
    aa_hard_assert(r == ACH_OK,
                   "Ach failure %s on opening Joystick channel (%s, line %d)\n",
                   ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);
    
    last_js_msg = NULL;
    while(last_js_msg == NULL) {
        last_js_msg = SOMATIC_GET_LAST_UNPACK(r, somatic__joystick, &protobuf_c_system_allocator, 4096, &js_chan);
    }
}

int do_joystick(double* ltarget_pos, double* ltarget_vel, double* rtarget_pos, double* rtarget_vel,
                double* lmessage, double* rmessage) {
    int r;
    Somatic__Joystick* js_msg = SOMATIC_GET_LAST_UNPACK(r, somatic__joystick, &protobuf_c_system_allocator, 4096, &js_chan);
    if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) {
        return -1;
    }

    // update times
    double cur_js_time = gettime();
    double js_dT = cur_js_time - last_js_time;
    last_js_time = cur_js_time;

    // figure out what mode we're in
    js_controlling = JOYSTICK_CONTROLLING_NONE;
    if (js_msg->buttons->data[4] && !js_msg->buttons->data[5] && !js_msg->buttons->data[6] && !js_msg->buttons->data[7]) {
        js_controlling = JOYSTICK_CONTROLLING_LEFT_SMALL;
    }
    if (!js_msg->buttons->data[4] && js_msg->buttons->data[5] && !js_msg->buttons->data[6] && !js_msg->buttons->data[7]) {
        js_controlling = JOYSTICK_CONTROLLING_RIGHT_SMALL;
    }
    if (!js_msg->buttons->data[4] && !js_msg->buttons->data[5] && js_msg->buttons->data[6] && !js_msg->buttons->data[7]) {
        js_controlling = JOYSTICK_CONTROLLING_LEFT_LARGE;
    }
    if (!js_msg->buttons->data[4] && !js_msg->buttons->data[5] && !js_msg->buttons->data[6] && js_msg->buttons->data[7]) {
        js_controlling = JOYSTICK_CONTROLLING_RIGHT_LARGE;
    }
    if (js_msg->buttons->data[3] && !last_js_msg->buttons->data[3]) {
        if (js_mode == JOYSTICK_MODE_POSITION) { js_mode = JOYSTICK_MODE_DIRECT; }
        else {
            for (int i = 0; i < 7; i++) ltarget_pos[i] = llwa.pos[i];
            for (int i = 0; i < 7; i++) rtarget_pos[i] = rlwa.pos[i];
            js_mode = JOYSTICK_MODE_POSITION;
        }
    }

    // do what we want with the joystick message
    switch(js_controlling) {
    case JOYSTICK_CONTROLLING_LEFT_SMALL:
        for(int i = 0; i < 3; i++) {
            lmessage[i+4] = js_msg->axes->data[i] * joint_cur_command_scale; // will be overwritten by pid in position mode
            ltarget_vel[i+4] = js_msg->axes->data[i] * joint_vel_command_scale;
            ltarget_pos[i+4] += js_dT * js_msg->axes->data[i] * joint_vel_command_scale;
        }
        break;
    case JOYSTICK_CONTROLLING_LEFT_LARGE:
        for(int i = 0; i < 4; i++) {
            lmessage[i] = js_msg->axes->data[i] * joint_cur_command_scale; // will be overwritten by pid in position mode
            ltarget_vel[i] = js_msg->axes->data[i] * joint_vel_command_scale;
            ltarget_pos[i] += js_dT * js_msg->axes->data[i] * joint_vel_command_scale;
        }
        break;
    case JOYSTICK_CONTROLLING_RIGHT_SMALL:
        for(int i = 0; i < 3; i++) {
            rmessage[i+4] = js_msg->axes->data[i] * joint_cur_command_scale; // will be overwritten by pid in position mode
            rtarget_vel[i+4] = js_msg->axes->data[i] * joint_vel_command_scale;
            rtarget_pos[i+4] += js_dT * js_msg->axes->data[i] * joint_vel_command_scale;
        }
        break;
    case JOYSTICK_CONTROLLING_RIGHT_LARGE:
        for(int i = 0; i < 4; i++) {
            rmessage[i] = js_msg->axes->data[i] * joint_cur_command_scale; // will be overwritten by pid in position mode
            rtarget_vel[i] = js_msg->axes->data[i] * joint_vel_command_scale;
            rtarget_pos[i] += js_dT * js_msg->axes->data[i] * joint_vel_command_scale;
        }
        break;
    }

    // display input
    do_input_display(js_msg);
    
    // free the memory that somatic allocated for us
    somatic__joystick__free_unpacked(last_js_msg, &protobuf_c_system_allocator);
    last_js_msg = js_msg;

    return 0;
}

double do_pid(somatic_motor_t* lwa, double* target_pos, double* target_vel, double* message) {
    double p_value;
    double d_value;

    for (int i = 0; i < 7; i++) {
        p_value = K_p[i] * (target_pos[i] - lwa->pos[i]);
        d_value = K_d[i] * (target_vel[i] - lwa->vel[i]);
        message[i] = p_value + d_value;
    }
}

// ################################################################################
// main loop

int main( int argc, char **argv ) {
    // variables
    double lmessage[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double rmessage[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    double ltarget_pos[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double ltarget_vel[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double rtarget_pos[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double rtarget_vel[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // init curses
    do_init_curses();

    // init daemon
    somatic_d_opts_t daemon_opt;
    memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
    daemon_opt.ident = "arm-joy-current";
    somatic_d_init(&daemon_cx, &daemon_opt);

    // init motors
    somatic_motor_init(&daemon_cx, &llwa, 7, "llwa-cmd", "llwa-state");
    somatic_motor_init(&daemon_cx, &rlwa, 7, "rlwa-cmd", "rlwa-state");
    somatic_motor_update(&daemon_cx, &llwa);
    somatic_motor_update(&daemon_cx, &rlwa);
    somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
    somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
    usleep(1e5);

    // set limits
    do_set_limits(&llwa);
    do_set_limits(&rlwa);

    // init joystick
    do_init_joystick();

    // start the daemon running
    somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

    // start our targets at where the motors actually are
    somatic_motor_update(&daemon_cx, &llwa);
    somatic_motor_update(&daemon_cx, &rlwa);
    for (int i = 0; i < 7; i++) ltarget_pos[i] = llwa.pos[i];
    for (int i = 0; i < 7; i++) rtarget_pos[i] = rlwa.pos[i];

    // main loop
    while(!somatic_sig_received) {
        // do curses stuff
        int ch = getch();
        switch (ch) {
        case 'q': somatic_sig_received = true; break;
        }

        // update motor
        somatic_motor_update(&daemon_cx, &llwa);
        somatic_motor_update(&daemon_cx, &rlwa);

        // update joystick
        do_joystick(ltarget_pos, ltarget_vel, rtarget_pos, rtarget_vel, lmessage, rmessage);
        
        // do the pid control thing
        if (js_mode == JOYSTICK_MODE_POSITION) {
            do_pid(&llwa, ltarget_pos, ltarget_vel, lmessage);
            do_pid(&rlwa, rtarget_pos, rtarget_vel, rmessage);
        }

        // display it all
        do_control_display(&llwa, &rlwa, ltarget_pos, ltarget_vel, rtarget_pos, rtarget_vel, lmessage, rmessage);
        refresh();

        // and send commands
        somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, lmessage, 7, NULL);
        somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, rmessage, 7, NULL);
    }

    // Send stopping event
    somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

    // halt the motor
    somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
    somatic_motor_destroy(&daemon_cx, &rlwa);
    somatic_d_destroy(&daemon_cx);

    // end curses
    do_end_curses();
}
