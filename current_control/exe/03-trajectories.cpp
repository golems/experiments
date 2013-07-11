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
#include <vector>

#include <simulation/World.h>
#include <robotics/parser/dart_parser/DartLoader.h>

// ################################################################################
// definitions


// ################################################################################
// global variables

somatic_d_t daemon_cx;
somatic_motor_t rlwa;
somatic_motor_t waist;

int larmids[] = {10, 12, 14, 16, 18, 20, 22};
int waistids[] = {5, 8};

simulation::World* world;

// ################################################################################
// constants

float K_p[] = {15.0, 15.0, 15.0, 12.0, 15.0, 7.0, 7.0};
float K_d[] = {0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 0.0};

// ################################################################################
// small helper functions

// ################################################################################
// functions that actually do things

void do_init_dart() {
    DartLoader dl;
    world = dl.parseWorld("../../common/scenes/01-World-Robot.urdf");
    assert((world != NULL) && "Could not find the world");
}

// ################################################################################
// main loop

int main() {
    // variables
    double lmessage[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double rmessage[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


    // init daemon
    somatic_d_opts_t daemon_opt;
    memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
    daemon_opt.ident = "arm-joy-current";
    somatic_d_init(&daemon_cx, &daemon_opt);

    // Initialize various krang hardware
    somatic_motor_init(&daemon_cx, &waist, 2, "waist-cmd", "waist-state");
    // initArm(daemon_cx, rlwa, "rlwa");

    // start the daemon running
    somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

    // figure out the current configuration of our robot so we don't start by flipping out
    while(!somatic_sig_received) {
        sleep(1);
        somatic_motor_update(&daemon_cx, &waist);
        printf("waist pos: %f\n", waist.pos[0]);
    }    

    // Send stopping event
    somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

    // halt the motor
    // somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
    // somatic_motor_destroy(&daemon_cx, &rlwa);
    somatic_motor_destroy(&daemon_cx, &waist);

    // destroy the daemon
    somatic_d_destroy(&daemon_cx);
}
