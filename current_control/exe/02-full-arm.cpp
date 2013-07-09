#include "helpers.h"
#include "initModules.h"
#include "motion.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <ncurses.h>
#include <iomanip>

somatic_d_t daemon_cx;
ach_channel_t state_chan;
ach_channel_t cmd_chan;
ach_channel_t js_chan;
somatic_motor_t rlwa;

float K_p[] = {15.0, 15.0, 15.0, 12.0, 15.0, 7.0, 7.0};
float K_d[] = {0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 0.0};

double timespec_to_double(struct timespec x) {
    return (double)x.tv_sec + (double)x.tv_nsec / 1000000000.0;
}
double gettime() {
    struct timespec temp;
    clock_gettime(ACH_DEFAULT_CLOCK, &temp);
    return timespec_to_double(temp);
}

int main( int argc, char **argv ) {
    // init curses
    initscr();
    clear();
    noecho();                   // do not echo input to the screen
    cbreak();                   // do not buffer by line (receive characters immediately)
    timeout(0);                 // non-blocking getch

    // variables
    double js_scale = .30;

    double message[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    double error[7];
    double target_pos[7];
    double target_vel[7];

    double p_value;
    double d_value;

    int r = 0;
    int iter = 0;

    double cur_js_time;
    double last_js_time = gettime();
    double js_dT;

    // init daemon
    somatic_d_opts_t daemon_opt;
    memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
    daemon_opt.ident = "rlwa-current";
    somatic_d_init(&daemon_cx, &daemon_opt);

    // init motor
    somatic_motor_init(&daemon_cx, &rlwa, 7, "rlwa-cmd", "rlwa-state");
    somatic_motor_update(&daemon_cx, &rlwa);
    somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
    usleep(1e5);

    // Set the min/max values for valid and limits values
    double** limits [] = {
        &rlwa.pos_valid_min, &rlwa.vel_valid_min, 
        &rlwa.pos_limit_min, &rlwa.vel_limit_min, 
        &rlwa.pos_valid_max, &rlwa.vel_valid_max, 
        &rlwa.pos_limit_max, &rlwa.vel_limit_max};
    for(size_t i = 0; i < 4; i++) aa_fset(*limits[i], -1024.1, 7);
    for(size_t i = 4; i < 8; i++) aa_fset(*limits[i], 1024.1, 7);

    // open joystick channel
    r  = ach_open(&js_chan, "joystick-data", NULL);
    aa_hard_assert(r == ACH_OK,
                   "Ach failure %s on opening Joystick channel (%s, line %d)\n",
                   ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

    // finalize init and start running
    somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);
    somatic_motor_update(&daemon_cx, &rlwa);
    for (int i = 0; i < 7; i++) target_pos[i] = rlwa.pos[i];

    // main loop
    while(!somatic_sig_received) {
        iter++;

        // do curses stuff
        int ch = getch();
        switch (ch) {
        case 'q': somatic_sig_received = true; break;
        }

        // update motor
        somatic_motor_update(&daemon_cx, &rlwa);

        // update joystick
	Somatic__Joystick *js_msg = SOMATIC_GET_LAST_UNPACK(r, somatic__joystick, &protobuf_c_system_allocator, 4096, &js_chan);
	if((ACH_OK == r || ACH_MISSED_FRAME == r) && (js_msg != NULL)) {
            // update times
            cur_js_time = gettime();
            js_dT = cur_js_time - last_js_time;
            last_js_time = cur_js_time;

            // figure out what we want to do with the joystick stuff
            mvprintw(9, 1, "joystick input");
            for (int i = 0; i < js_msg->axes->n_data; i++) { mvprintw(i + 10, 1, "%f", js_msg->axes->data[i]); }
            for (int i = 0; i < js_msg->buttons->n_data; i++) { mvprintw(i + 10, 20, "%d", js_msg->buttons->data[i]); }

            for (int i = 0; i < rlwa.n; i++) {
                target_vel[i] = js_msg->axes->data[i] * js_scale;
                target_pos[i] = target_pos[i] + js_dT * js_msg->axes->data[i] * js_scale;
            }
            
            // free the memory that somatic allocated for us
            somatic__joystick__free_unpacked(js_msg, &protobuf_c_system_allocator);
        }
        
        // do the pid control thing
        for (int i = 0; i < 7; i++) {
            p_value = K_p[i] * (target_pos[i] - rlwa.pos[i]);
            d_value = K_d[i] * (target_vel[i] - rlwa.vel[i]);
            message[i] = p_value + d_value;
        }
        
        if (iter % 10000 == 0) {
            mvprintw(0, 1, "commanded current");
            mvprintw(0, 20, "current pos");
            mvprintw(0, 40, "current vel");
            mvprintw(0, 60, "current cur");
            for (int i = 0; i < 7; i++) {
                // mvprintw(0, 1, "dT: %f", dT);
                mvprintw(i + 1, 1, "%f", message[i]);
                mvprintw(i + 1, 20, "%f", rlwa.pos[i]);
                mvprintw(i + 1, 40, "%f", target_pos[i]);
                mvprintw(i + 1, 60, "%f", target_vel[i]);
            }
        }

        somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, message, 7, NULL);
    }

    // Send stopping event
    somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

    // halt the motor
    somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
    somatic_motor_destroy(&daemon_cx, &rlwa);
    somatic_d_destroy(&daemon_cx);

    // end curses
    clrtoeol();
    refresh();
    endwin();
}
