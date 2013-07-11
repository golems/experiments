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

// ################################################################################
// constants

#define PID_ERROR_WINDOW_SIZE 30

bool do_curses = true;

bool use_pos[] = {true, true, true, true, true, true, true};
bool use_vel[] = {true, true, true, true, true, true, true};

double init_K_p_p[] = {15.0,  15.0, 15.0, 12.0, 15.0,  7.0,  7.0};
double init_K_p_d[] = {0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0};
double init_K_v_p[] = {1.0,   1.0,  1.0,  1.0,  1.0,  1.0,  1.0};
double init_K_v_d[] = {0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0};

// double init_pos_limits[] = {2*M_PI/3, 2*M_PI/3, 2*M_PI/3, 2*M_PI/3, 2*M_PI/3, 2*M_PI/3, 2*M_PI/3};
// double init_vel_limits[] = {1.0,      1.0,      1.0,      1.0,      1.0,      1.0,      1.0};

double joint_vel_command_scale = 1.0;
double joint_cur_command_scale = 8.0;

// ################################################################################
// definitions

typedef enum {
    JOYSTICK_CONTROLLING_NONE = 0,
    JOYSTICK_CONTROLLING_LEFT_SMALL,
    JOYSTICK_CONTROLLING_LEFT_LARGE,
    JOYSTICK_CONTROLLING_RIGHT_SMALL,
    JOYSTICK_CONTROLLING_RIGHT_LARGE
} joystick_controlling_t;

typedef struct {
    bool use_pos;
    bool use_vel;

    double pos_target;
    double vel_target;

    // double pos_limit;
    // double vel_limit;

    double K_p_p;
    double K_p_d;
    double K_v_p;
    double K_v_d;
    double pos_error_last;
    double vel_error_last;
    double pos_error_window[PID_ERROR_WINDOW_SIZE];
    double vel_error_window[PID_ERROR_WINDOW_SIZE];
} pid_state_t;

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

    if (js_msg->buttons->data[1] && !last_js_msg->buttons->data[1]) {
        for (int i = 0; i < 7; i++) { lpids[i].K_v_p -= .1; rpids[i].K_v_p -= .1; }
    }
    if (js_msg->buttons->data[3] && !last_js_msg->buttons->data[3]) {
        for (int i = 0; i < 7; i++) { lpids[i].K_v_p += .1; rpids[i].K_v_p += .1; }
    }
    if (js_msg->buttons->data[0] && !last_js_msg->buttons->data[0]) {
        for (int i = 0; i < 7; i++) { lpids[i].K_v_d -= .01; rpids[i].K_v_d -= .01; }
    }
    if (js_msg->buttons->data[2] && !last_js_msg->buttons->data[2]) {
        for (int i = 0; i < 7; i++) { lpids[i].K_v_d += .01; rpids[i].K_v_d += .01; }
    }

    // if (js_msg->buttons->data[1] && !last_js_msg->buttons->data[1]) { for(int i = 0; i < 7; i++) K_v_p[i] -= .1; } // minus gains
    // if (js_msg->buttons->data[3] && !last_js_msg->buttons->data[3]) { for(int i = 0; i < 7; i++) K_v_p[i] += .1; } // plus gains
    // if (js_msg->buttons->data[0] && !last_js_msg->buttons->data[0]) { for(int i = 0; i < 7; i++) K_v_d[i] -= .01; } // minus gains
    // if (js_msg->buttons->data[2] && !last_js_msg->buttons->data[2]) { for(int i = 0; i < 7; i++) K_v_d[i] += .01; } // plus gains

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

    // do what we want with the joystick message
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

void do_init_pids(somatic_motor_t* mot, pid_state_t* pids) {
    for(int i = 0; i < mot->n; i++) {
        pids[i].use_pos = use_pos[i];
        pids[i].use_vel = use_vel[i];
        pids[i].pos_target = 0.0;
        pids[i].vel_target = 0.0;
        pids[i].pos_error_last = 0.0;
        pids[i].vel_error_last = 0.0;
        for(int j = 0; j < PID_ERROR_WINDOW_SIZE; j++) pids[i].pos_error_window[j] = 0.0;
        for(int j = 0; j < PID_ERROR_WINDOW_SIZE; j++) pids[i].vel_error_window[j] = 0.0;
        pids[i].pos_target = mot->pos[i];
        
        pids[i].K_p_p = init_K_p_p[i];
        pids[i].K_p_d = init_K_p_d[i];
        pids[i].K_v_p = init_K_v_p[i];
        pids[i].K_v_d = init_K_v_d[i];

        // pids[i].pos_limit = init_pos_limits[i];
        // pids[i].vel_limit = init_vel_limits[i];
    }
}

void update_pids(somatic_motor_t* mot, pid_state_t* pids, double* result) {
    double p_p_value;
    double p_d_value;
    double v_p_value;
    double v_d_value;

    double pos_error;
    double vel_error;

    for(int i = 0; i < mot->n; i++) {
        result[i] = 0;

        // if (fabs(pids[i].pos_target) > fabs(pids[i].pos_limit)) {
        //     pids[i].pos_target = copysign(pids[i].pos_limit, pids[i].pos_target);
        // }
        // if (fabs(pids[i].vel_target) > fabs(pids[i].vel_limit)) {
        //     pids[i].vel_target = copysign(pids[i].vel_limit, pids[i].vel_target);
        // }

        if(pids[i].use_pos) {
            pos_error = pids[i].pos_target - mot->pos[i];

            p_p_value = pids[i].K_p_p * pos_error;
            p_d_value = pids[i].K_p_d * (pos_error - pids[i].pos_error_last);

            result[i] += p_p_value + p_d_value;

            pids[i].pos_error_last = pos_error;
        }
        if (pids[i].use_vel) {
            vel_error = pids[i].vel_target - mot->vel[i];

            v_p_value = pids[i].K_v_p * vel_error;
            v_d_value = pids[i].K_v_d * (vel_error - pids[i].vel_error_last);

            result[i] += v_p_value + v_d_value;

            pids[i].vel_error_last = vel_error;
        }
    }
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

        // and send commands
        if (!halted) {
            // somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, lmessage, 7, NULL);
            somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, rmessage, 7, NULL);
        }
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
