/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice, this list of
 *       conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright notice, this list of
 *       conditions and the following disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived from this software without
 *       specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH RESEARCH
 * CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

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
#include <cmath>

#include "current_control.cpp"

// ################################################################################
// constants

bool do_curses = true;
bool do_logging = false;

bool use_pos[] = {true, true, true, true, true, true, true};
bool use_vel[] = {true, true, true, true, true, true, true};

double init_K_p_p[] = {15.0,  15.0, 15.0, 12.0, 15.0,  7.0,  7.0};
double init_K_p_d[] = {0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0};
double init_K_v_p[] = {1.0,   1.0,  1.0,  1.0,  1.0,  1.0,  1.0};
double init_K_v_d[] = {0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0};

double joint_vel_command_scale = 1.0;
double joint_cur_command_scale = 8.0;

double log_frequency = 100.0;

// ################################################################################
// definitions

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
joystick_controlling_t js_controlling;
Somatic__Joystick* last_js_msg;

bool halted = true;

std::ofstream log_file;
double log_time_last;
double log_time_start;

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

void do_init_curses() {
    if (!do_curses) return;
    initscr();
    clear();
    noecho();                   // do not echo input to the screen
    cbreak();                   // do not buffer by line (receive characters immediately)
    timeout(0);                 // non-blocking getch
}

void do_end_curses() {
    if (!do_curses) return;
    clrtoeol();
    refresh();
    endwin();
}

#define AXIS_SLIDER_WIDTH 20
char input_display_buffer[(AXIS_SLIDER_WIDTH * 2) + 1];
void do_input_display(Somatic__Joystick* js_msg) {
    if (!do_curses) return;

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
}

void do_control_display(somatic_motor_t* llwa, somatic_motor_t* rlwa,
                        pid_state_t* lpids, pid_state_t* rpids,
                        double* lmessage, double* rmessage) {
    if (!do_curses) return;

    // whether we're halted
    if (halted) { mvprintw(8, 2, "HALTED"); }
    else { mvprintw(8, 2, "      "); }

    // top row is for left arm, bottom row is right arm
    // from left to right: position target, position actual, velocity target, velocity actual, and commanded current

    mvprintw(15, 1, "left pos: target, actual, error");
    for(int i = 0; i < 7; i++) mvprintw(16+i, 2, "%f", lpids[i].pos_target);
    for(int i = 0; i < 7; i++) mvprintw(16+i, 22, "%f", llwa->pos[i]);
    for(int i = 0; i < 7; i++) mvprintw(16+i, 42, "%f", lpids[i].pos_target - llwa->pos[i]);
    mvprintw(24, 1, "right pos: target, actual, error");
    for(int i = 0; i < 7; i++) mvprintw(25+i, 2, "%f", rpids[i].pos_target);
    for(int i = 0; i < 7; i++) mvprintw(25+i, 22, "%f", rlwa->pos[i]);
    for(int i = 0; i < 7; i++) mvprintw(25+i, 42, "%f", rpids[i].pos_target - rlwa->pos[i]);

    mvprintw(15, 61, "left vel: target, actual, error");
    for(int i = 0; i < 7; i++) mvprintw(16+i, 62, "%f", lpids[i].vel_target);
    for(int i = 0; i < 7; i++) mvprintw(16+i, 82, "%f", llwa->vel[i]);
    for(int i = 0; i < 7; i++) mvprintw(16+i, 102, "%f", lpids[i].vel_target - llwa->vel[i]);
    mvprintw(24, 61, "right vel: target, actual, error");
    for(int i = 0; i < 7; i++) mvprintw(25+i, 62, "%f", rpids[i].vel_target);
    for(int i = 0; i < 7; i++) mvprintw(25+i, 82, "%f", rlwa->vel[i]);
    for(int i = 0; i < 7; i++) mvprintw(25+i, 102, "%f", rpids[i].vel_target - rlwa->vel[i]);

    mvprintw(15, 121, "left commanded current");
    for(int i = 0; i < 7; i++) mvprintw(16+i, 122, "%f", lmessage[i]);
    mvprintw(24, 121, "right commanded current");
    for(int i = 0; i < 7; i++) mvprintw(25+i, 122, "%f", rmessage[i]);

    // and on th eobttom, display gains
    mvprintw(33, 1, "pos gains: p, d");
    for(int i = 0; i < 7; i++) mvprintw(34+i, 2, "%f", lpids[i].K_p_p);
    for(int i = 0; i < 7; i++) mvprintw(34+i, 22, "%f", lpids[i].K_p_d);
    mvprintw(33, 41, "vel gains: p, d");
    for(int i = 0; i < 7; i++) mvprintw(34+i, 42, "%f", lpids[i].K_v_p);
    for(int i = 0; i < 7; i++) mvprintw(34+i, 62, "%f", lpids[i].K_v_d);
}

// ################################################################################
// logging

void init_log_to_file() {
    if (!do_logging) return;
    log_file.open("02-full-arm.log");
    log_time_last = gettime();
    log_time_start = gettime();
}

void end_log_to_file() {
    if (!do_logging) return;
    log_file.close();
}

void do_log_to_file(pid_state_t* lpids, pid_state_t* rpids,
                    somatic_motor_t* llwa, somatic_motor_t* rlwa,
                    double* lmessage, double* rmessage) {
    if (!do_logging) return;

    double now = gettime();
    if (now - log_time_last < 1.0/log_frequency) return;
    log_time_last = now;
    
    int log_module = 1;
    log_file
        << now - log_time_start << ","         // 1
        << (halted ? 1 : 0) << ","             // 2
        << rpids[log_module].pos_target << "," // 3
        << rlwa->pos[log_module] << ","        // 4
        << rpids[log_module].vel_target << "," // 5
        << rlwa->vel[log_module] << ","        // 6
        << rmessage[log_module] << std::endl;  // 7
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

int do_joystick(pid_state_t* lpids, pid_state_t* rpids) {
    int r;
    Somatic__Joystick* js_msg = SOMATIC_GET_LAST_UNPACK(r, somatic__joystick, &protobuf_c_system_allocator, 4096, &js_chan);
    if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) {
        return -1;
    }

    // update times
    double cur_js_time = gettime();
    double js_dT = cur_js_time - last_js_time;
    last_js_time = cur_js_time;

    // figure out what we're controlling
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

    // and do stuff
    if (js_msg->buttons->data[1] && !last_js_msg->buttons->data[1]) {
        // for (int i = 0; i < 7; i++) { lpids[i].K_v_p -= .1; rpids[i].K_v_p -= .1; }
    }
    if (js_msg->buttons->data[3] && !last_js_msg->buttons->data[3]) {
        // for (int i = 0; i < 7; i++) { lpids[i].K_v_p += .1; rpids[i].K_v_p += .1; }
    }
    if (js_msg->buttons->data[0] && !last_js_msg->buttons->data[0]) {
        // for (int i = 0; i < 7; i++) { lpids[i].K_v_d -= .01; rpids[i].K_v_d -= .01; }
        for (int i = 0; i < 7; i++) { std::cout << rlwa.pos[i] << ","; }
        std::cout << std::endl;
    }
    if (js_msg->buttons->data[2] && !last_js_msg->buttons->data[2]) {
        // for (int i = 0; i < 7; i++) { lpids[i].K_v_d += .01; rpids[i].K_v_d += .01; }
    }

    if (js_msg->buttons->data[8] && !last_js_msg->buttons->data[8]) { // brakes!
        halted = !halted;
        if (halted) {
            somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
            somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
        }
        else {
            somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
            somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
        }
    }
    if (js_msg->buttons->data[9] && !last_js_msg->buttons->data[9]) { somatic_sig_received = true; } // quit

    switch(js_controlling) {
    case JOYSTICK_CONTROLLING_LEFT_SMALL:
        for(int i = 0; i < 3; i++) {
            lpids[i+4].vel_target = js_msg->axes->data[i] * joint_vel_command_scale;
            lpids[i+4].pos_target += js_dT * js_msg->axes->data[i] * joint_vel_command_scale;
        }
        break;
    case JOYSTICK_CONTROLLING_LEFT_LARGE:
        for(int i = 0; i < 4; i++) {
            lpids[i].vel_target = js_msg->axes->data[i] * joint_vel_command_scale;
            lpids[i].pos_target += js_dT * js_msg->axes->data[i] * joint_vel_command_scale;
        }
        break;
    case JOYSTICK_CONTROLLING_RIGHT_SMALL:
        for(int i = 0; i < 3; i++) {
            rpids[i+4].vel_target = js_msg->axes->data[i] * joint_vel_command_scale;
            rpids[i+4].pos_target += js_dT * js_msg->axes->data[i] * joint_vel_command_scale;
        }
        break;
    case JOYSTICK_CONTROLLING_RIGHT_LARGE:
        for(int i = 0; i < 4; i++) {
            rpids[i].vel_target = js_msg->axes->data[i] * joint_vel_command_scale;
            rpids[i].pos_target += js_dT * js_msg->axes->data[i] * joint_vel_command_scale;
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

// ################################################################################
// main loop

int main( int argc, char **argv ) {
    // variables
    double lmessage[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double rmessage[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    pid_state_t lpids[7];
    pid_state_t rpids[7];

    // init curses
    do_init_curses();

    // init daemon
    somatic_d_opts_t daemon_opt;
    memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
    daemon_opt.ident = "arm-joy-current";
    somatic_d_init(&daemon_cx, &daemon_opt);

    // init logging
    init_log_to_file();

    // init motors
    somatic_motor_init(&daemon_cx, &llwa, 7, "llwa-cmd", "llwa-state");
    somatic_motor_init(&daemon_cx, &rlwa, 7, "rlwa-cmd", "rlwa-state");
    somatic_motor_update(&daemon_cx, &llwa);
    somatic_motor_update(&daemon_cx, &rlwa);
    // somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
    // somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
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
    do_init_pids(&llwa, lpids);
    do_init_pids(&rlwa, rpids);
    for(int i = 0; i < 7; i++) {
        lpids[i].K_p_p = init_K_p_p[i];
        lpids[i].K_p_d = init_K_p_d[i];
        lpids[i].K_v_p = init_K_v_p[i];
        lpids[i].K_v_d = init_K_v_d[i];
        lpids[i].use_pos = use_pos[i];
        lpids[i].use_vel = use_vel[i];
        rpids[i].K_p_p = init_K_p_p[i];
        rpids[i].K_p_d = init_K_p_d[i];
        rpids[i].K_v_p = init_K_v_p[i];
        rpids[i].K_v_d = init_K_v_d[i];
        rpids[i].use_pos = use_pos[i];
        rpids[i].use_vel = use_vel[i];
    }


    // main loop
    while(!somatic_sig_received) {
        // do curses stuff
        if(do_curses) {
            int ch = getch();
            switch (ch) {
            case 'q': somatic_sig_received = true; break;
            }
        }

        // update motor
        somatic_motor_update(&daemon_cx, &llwa);
        somatic_motor_update(&daemon_cx, &rlwa);

        // update joystick
        do_joystick(lpids, rpids);
        
        // do the pid control thing
        update_pids(&llwa, lpids, lmessage);
        update_pids(&rlwa, rpids, rmessage);

        // display it all
        do_control_display(&llwa, &rlwa, lpids, rpids, lmessage, rmessage);
        refresh();

        // log stuff
        do_log_to_file(lpids, rpids, &llwa, &rlwa, lmessage, rmessage);

        // and send commands
        if (!halted) {
            somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, lmessage, 7, NULL);
            somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, rmessage, 7, NULL);
        }
    }

    // Send stopping event
    somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

    // halt the motor
    somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
    somatic_motor_destroy(&daemon_cx, &rlwa);
    somatic_d_destroy(&daemon_cx);

    end_log_to_file();

    // end curses
    do_end_curses();
}
