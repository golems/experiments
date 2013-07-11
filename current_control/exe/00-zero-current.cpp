#include "helpers.h"
#include "initModules.h"
#include "motion.h"
#include <stdio.h>
#include <iostream>
#include <ncurses.h>

somatic_d_t daemon_cx;
ach_channel_t state_chan;
ach_channel_t cmd_chan;
somatic_motor_t module;

#define STATE_CHAN_NAME "single-module-state"
#define CMD_CHAN_NAME "single-module-cmd"

int main( int argc, char **argv ) {
    // init daemon
    somatic_d_opts_t daemon_opt;
    memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
    daemon_opt.ident = "single-module-current";
    somatic_d_init(&daemon_cx, &daemon_opt);
    
    // init motor
    somatic_motor_init(&daemon_cx, &module, 1, CMD_CHAN_NAME, STATE_CHAN_NAME);
    somatic_motor_update(&daemon_cx, &module);
    somatic_motor_cmd(&daemon_cx, &module, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 1, NULL);
    usleep(1e5);

    // Set the min/max values for valid and limits values
    double** limits [] = {
        &module.pos_valid_min, &module.vel_valid_min, 
        &module.pos_limit_min, &module.vel_limit_min, 
        &module.pos_valid_max, &module.vel_valid_max, 
        &module.pos_limit_max, &module.vel_limit_max};
    for(size_t i = 0; i < 4; i++) aa_fset(*limits[i], -1024.1, 1);
    for(size_t i = 4; i < 8; i++) aa_fset(*limits[i], 1024.1, 1);

    // done with init, start running
    somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);
    
    double derivator;
    double message[1] = {0};

    double K_p = 1;
    double K_d = 0;

    double error;

    double target_pos = 0;
    double target_vel = 0;
    
    while(!somatic_sig_received) {
        somatic_motor_update(&daemon_cx, &module);
        double p_value = K_p * (target_pos - module.pos[0]);
        double d_value = K_d * (target_vel - module.vel[0]);
        //message[0] = p_value + d_value;
        message[0] = 0;
        std::cout << "\x1b[31;1m" << "Message: " << message[0] << std::endl << "\tCur: " << module.cur[0] << "\x1b[0m" << std::endl;

	somatic_motor_cmd(&daemon_cx, &module, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, message, 1, NULL);
    }

    // Send stopping event
    somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

    // halt the motor
    somatic_motor_cmd(&daemon_cx, &module, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 1, NULL);
    somatic_motor_destroy(&daemon_cx, &module);
    somatic_d_destroy(&daemon_cx);
}
