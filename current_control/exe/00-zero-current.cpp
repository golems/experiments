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
