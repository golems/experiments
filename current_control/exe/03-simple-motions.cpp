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
#include <iomanip>
#include <cmath>

#include "current_control.cpp"

// ################################################################################
// constants

bool use_pos[] = {true, true, true, true, true, true, true};
bool use_vel[] = {true, true, true, true, true, true, true};

double init_K_p_p[] = {15.0,  15.0, 15.0, 12.0, 15.0,  7.0,  7.0};
double init_K_p_d[] = {0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0};
double init_K_v_p[] = {1.0,   1.0,  1.0,  1.0,  1.0,  1.0,  1.0};
double init_K_v_d[] = {0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0};

double joint_vel_command_scale = 1.0;

double log_frequency = 100.0;
double display_frequency = 10.0;

// ################################################################################
// configurations

double conf_arm_zero[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double conf_resting[] = {2.02892,-1.00876,0.00374119,-0.820485,-0.0655409,-1.4174,0.000450295};
double conf_manipulation[] = {1.77149,-1.66232,1.58561,-0.531645,-0.104151,-1.41763,0.000450295};

double conf_flat_sweep_back[] = {-1.4, -M_PI/2, 0.0, 0.0, 0.0, 0.0, 0.0};
double conf_flat_sweep_middle[] = {-1.4, 0, 0.0, 0.0, 0.0, 0.0, 0.0};
double conf_flat_sweep_front[] = {-1.4, M_PI/2, 0.0, 0.0, 0.0, 0.0, 0.0};

double conf_vertical_sweep_bottom[] = {0.6, 0.21, 0.0, 0.0, 0.0, 0.0, 0.0};
double conf_vertical_sweep_middle[] = {0.6, -.8, 0.0, 0.0, 0.0, 0.0, 0.0};
double conf_vertical_sweep_top[] = {0.6, -M_PI/2, 0.0, 0.0, 0.0, 0.0, 0.0};

// ################################################################################
// definitions

// ################################################################################
// global variables

somatic_d_t daemon_cx;
somatic_motor_t rlwa;

std::ofstream log_file;
double log_time_last;
double log_time_start;

double display_time_last;
double display_time_start;

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

double position_error(somatic_motor_t* lwa, double* target) {
    double err = 0.0;
    for (int i = 0; i < lwa->n; i++) { err += pow(lwa->pos[i] - target[i], 2); }
    return sqrt(err);
}

void do_motion_position(somatic_motor_t* lwa, double* configuration, char* log_name, std::ios_base::openmode logmode) {
    // init logging
    log_file.open(log_name, logmode);
    
    // send position command
    somatic_motor_cmd(&daemon_cx, lwa, SOMATIC__MOTOR_PARAM__MOTOR_POSITION, configuration, 7, NULL);

    // loop until user says we're done
    while(!somatic_sig_received) {
        // update state
        somatic_motor_update(&daemon_cx, lwa);
        double now = gettime();

        // log data to a file
        if (now - log_time_last > 1.0/log_frequency) {
            log_time_last = now;
            log_file << now - log_time_start << ",";
            log_file << position_error(lwa, configuration);
            for(int i = 0; i < 7; i++) log_file << lwa->pos[i] << ",";
            for(int i = 0; i < 7; i++) log_file << configuration[i] << ",";
            log_file << std::endl;
        }

        // // and display for the user
        // if (now - display_time_last > 1.0/display_frequency) {
        //     display_time_last = now;
        //     std::cout << now - display_time_start << ",";
        //     std::cout << position_error(lwa, configuration) << ",";
        //     std::cout << std::endl;
        // }
    }

    // clean up and done
    somatic_sig_received = false;
    log_file.close();
}

void do_motion_linear_trajectory(somatic_motor_t* lwa, double* start_conf, double* end_conf, double duration,
                                 char* log_name, std::ios_base::openmode logmode) {
    // init logging
    log_file.open(log_name, logmode);

    somatic_motor_cmd(&daemon_cx, lwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);

    // init our controller
    double message[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    pid_state_t pids[7];
    do_init_pids(lwa, pids);
    for(int i = 0; i < 7; i++) {
        pids[i].K_p_p = init_K_p_p[i];
        pids[i].K_p_d = init_K_p_d[i];
        pids[i].K_v_p = init_K_v_p[i];
        pids[i].K_v_d = init_K_v_d[i];
        pids[i].use_pos = use_pos[i];
        pids[i].use_vel = use_vel[i];
    }

    double traj_start_time = gettime();

    // go until the user says done
    while(!somatic_sig_received) {
        // update state
        somatic_motor_update(&daemon_cx, lwa);
        double now = gettime();

        double progress = (now - traj_start_time) / duration;
        if (progress > 1.0) progress = 1.0;

        // do trajectory and control
        for(int i = 0; i < 7; i++) {
            pids[i].pos_target = end_conf[i] * progress + start_conf[i] * (1.0 - progress);
            if (progress < .1 || progress > .9) { pids[i].vel_target = 0; }
            else {pids[i].vel_target = (end_conf[i] - start_conf[i]) / duration; }
        }
        update_pids(lwa, pids, message);
        somatic_motor_cmd(&daemon_cx, lwa, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, message, 7, NULL);

        // log data to a file
        if (now - log_time_last > 1.0/log_frequency) {
            log_time_last = now;
            log_file << now - log_time_start << ",";
            log_file << position_error(lwa, end_conf) << ",";
            for(int i = 0; i < 7; i++) log_file << lwa->pos[i] << ",";
            for(int i = 0; i < 7; i++) log_file << pids[i].pos_target << ",";
            log_file << std::endl;
        }

        // and display some stuff for the user
        if (now - display_time_last > 1.0/display_frequency) {
            display_time_last = now;
            std::cout << now - display_time_start << ",";
            std::cout << pids[1].pos_target << ",";
            std::cout << pids[1].vel_target << ",";
            std::cout << position_error(lwa, end_conf) << ",";
            std::cout << std::endl;
        }
    }

    // clean up and done
    somatic_sig_received = false;
    log_file.close();
}

void do_motion_current(somatic_motor_t* lwa, double* configuration, char* log_name, std::ios_base::openmode logmode) {
    // init logging
    log_file.open(log_name, logmode);

    somatic_motor_cmd(&daemon_cx, lwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);

    // init our controller
    double message[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    pid_state_t pids[7];
    do_init_pids(lwa, pids);
    for(int i = 0; i < 7; i++) {
        pids[i].K_p_p = init_K_p_p[i];
        pids[i].K_p_d = init_K_p_d[i];
        pids[i].K_v_p = init_K_v_p[i];
        pids[i].K_v_d = init_K_v_d[i];
        pids[i].use_pos = use_pos[i];
        pids[i].use_vel = use_vel[i];

        pids[i].pos_target = configuration[i];
    }

    // go until the user says done
    while(!somatic_sig_received) {
        // update state
        somatic_motor_update(&daemon_cx, lwa);
        double now = gettime();

        // do controller
        for(int i = 0; i < 7; i++) pids[i].vel_target = lwa->vel[i];
        update_pids(lwa, pids, message);
        somatic_motor_cmd(&daemon_cx, lwa, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, message, 7, NULL);

        // log data to a file
        if (now - log_time_last > 1.0/log_frequency) {
            log_time_last = now;
            log_file << now - log_time_start << ",";
            log_file << position_error(lwa, configuration) << ",";
            for(int i = 0; i < 7; i++) log_file << lwa->pos[i] << ",";
            for(int i = 0; i < 7; i++) log_file << configuration[i] << ",";
            log_file << std::endl;
        }

        // // and display some stuff for the user
        // if (now - display_time_last > 1.0/display_frequency) {
        //     display_time_last = now;
        //     std::cout << now - display_time_start << ",";
        //     for(int i = 0; i < 7; i++) std::cout << message[i] << ",";
        //     std::cout << position_error(lwa, configuration) << ",";
        //     std::cout << std::endl;
        // }
    }

    // clean up and done
    somatic_sig_received = false;
    log_file.close();
}


// ################################################################################
// main loop

int main( int argc, char **argv ) {
    // variables
    double rmessage[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    log_time_last = gettime();
    log_time_start = gettime();
    display_time_start = gettime();
    display_time_last = gettime();

    // init daemon
    somatic_d_opts_t daemon_opt;
    memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
    daemon_opt.ident = "arm-joy-current";
    somatic_d_init(&daemon_cx, &daemon_opt);

    // init motors
    somatic_motor_init(&daemon_cx, &rlwa, 7, "rlwa-cmd", "rlwa-state");
    somatic_motor_update(&daemon_cx, &rlwa);
    somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
    usleep(1e5);

    // set limits
    do_set_limits(&rlwa);

    // start the daemon running
    somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

    // start our targets at where the motors actually are
    somatic_motor_update(&daemon_cx, &rlwa);

    // ############################################################
    // DO STUFF HERE ##############################################
    // ############################################################
    do_motion_position(&rlwa, conf_flat_sweep_middle, "03-simple-motions-1.log", std::ios_base::trunc);

    do_motion_position(&rlwa, conf_flat_sweep_front, "03-simple-motions-2.log", std::ios_base::trunc);
    do_motion_position(&rlwa, conf_flat_sweep_back, "03-simple-motions-2.log", std::ios_base::app);
    do_motion_position(&rlwa, conf_flat_sweep_front, "03-simple-motions-2.log", std::ios_base::app);

    do_motion_current(&rlwa, conf_flat_sweep_front, "03-simple-motions-3.log", std::ios_base::trunc);
    do_motion_current(&rlwa, conf_flat_sweep_back, "03-simple-motions-3.log", std::ios_base::app);
    do_motion_current(&rlwa, conf_flat_sweep_front, "03-simple-motions-3.log", std::ios_base::app);

    do_motion_position(&rlwa, conf_flat_sweep_middle, "03-simple-motions-4.log", std::ios_base::trunc);
    do_motion_position(&rlwa, conf_vertical_sweep_middle, "03-simple-motions-4.log", std::ios_base::app);
    do_motion_position(&rlwa, conf_vertical_sweep_bottom, "03-simple-motions-4.log", std::ios_base::app);

    do_motion_position(&rlwa, conf_vertical_sweep_top, "03-simple-motions-5.log", std::ios_base::trunc);
    do_motion_position(&rlwa, conf_vertical_sweep_bottom, "03-simple-motions-5.log", std::ios_base::app);
    do_motion_position(&rlwa, conf_vertical_sweep_middle, "03-simple-motions-5.log", std::ios_base::app);
    do_motion_position(&rlwa, conf_vertical_sweep_bottom, "03-simple-motions-5.log", std::ios_base::app);

    do_motion_current(&rlwa, conf_vertical_sweep_top, "03-simple-motions-6.log", std::ios_base::trunc);
    do_motion_current(&rlwa, conf_vertical_sweep_bottom, "03-simple-motions-6.log", std::ios_base::app);
    do_motion_current(&rlwa, conf_vertical_sweep_middle, "03-simple-motions-6.log", std::ios_base::app);
    do_motion_current(&rlwa, conf_vertical_sweep_bottom, "03-simple-motions-6.log", std::ios_base::app);

    do_motion_linear_trajectory(&rlwa, conf_vertical_sweep_bottom, conf_vertical_sweep_top, 6.0, 
                                "03-simple-motions-7.log", std::ios_base::trunc);
    do_motion_linear_trajectory(&rlwa, conf_vertical_sweep_top, conf_vertical_sweep_bottom, 6.0, 
                                "03-simple-motions-7.log", std::ios_base::app);
    do_motion_linear_trajectory(&rlwa, conf_vertical_sweep_bottom, conf_vertical_sweep_middle, 6.0, 
                                "03-simple-motions-7.log", std::ios_base::app);
    do_motion_linear_trajectory(&rlwa, conf_vertical_sweep_middle, conf_vertical_sweep_bottom, 6.0, 
                                "03-simple-motions-7.log", std::ios_base::app);
    

    // Send stopping event
    somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

    // halt the motor
    somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
    somatic_motor_destroy(&daemon_cx, &rlwa);
    somatic_d_destroy(&daemon_cx);
}
