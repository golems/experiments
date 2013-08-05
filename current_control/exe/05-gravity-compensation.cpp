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
 * @file 05-gravity-compensation.cpp
 * @author Saul Reynolds-Haertle
 * @date 2013-07-17

 * @briefs This executable demonstrates current-controlled gravity
 * compliance using dart.
 */


#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <imud.h>
#include <ncurses.h>

#include <Eigen/Dense>

// DART crap
#include <kinematics/Joint.h>
#include <kinematics/BodyNode.h>
#include <math/UtilsRotation.h>
#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>
#include <dynamics/SkeletonDynamics.h>


// ################################################################################
// ################################################################################
// DECLARATIONS
// ################################################################################
// ################################################################################

namespace Eigen {
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<double, 7, 1> Vector7d;
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
}

double ssdmu_pitch(double x, double y, double z);
double get_imu();

void update_motor_state();
void update_dart_state();

void run();
void init();
void destroy();

#define DISPLAY_VECTOR(VEC) std::cout << std::setw(25) << std::left << #VEC; for(int i = 0; i < VEC.size(); i++) std::cout << std::setw(12) << VEC[i]; std::cout << std::endl;

// ################################################################################
// ################################################################################
// GLOBALS
// ################################################################################
// ################################################################################

somatic_d_t daemon_cx;
ach_channel_t imu_chan;

somatic_motor_t rlwa;
somatic_motor_t waist;
somatic_motor_t torso;
double imu_angle;
double waist_angle;

simulation::World* world;
dynamics::SkeletonDynamics* krang;
kinematics::BodyNode* end_effector_node;

std::ofstream log_file;
double log_time_last;
double log_time_start;

Eigen::Vector7d arm_torque_constants;
double arm_torque_constants_initializer[] = {0.00054, 0.00057, 0.00122, 0.00122, 0.0, 0.0, 0.0};

// ################################################################################
// ################################################################################
// CONSTANTS
// ################################################################################
// ################################################################################

// logging constants
char* logfile_name = "../data/05-gravity-compensation.log";

// indices of joints in DART
int rarm_ids_initializer[] = {12, 14, 16, 18, 20, 22, 24};
int torso_ids_initializer[] = {9};
int waist_ids_initializer[] = {8};
int imu_ids_initializer[] = {5};
std::vector<int> rarm_ids;
std::vector<int> torso_ids;
std::vector<int> waist_ids;
std::vector<int> imu_ids;

// number of iterations to use to gather clean data at beginning of
// run
int imu_init_iters = 500;

// position of schunk gripper's com in end effector frame of reference
// 0.065 robotiq itself, 0.026 length of ext + 2nd
static const Eigen::Vector3d schunk_gripper_com_vector(0.0, -0.008, 0.091);

// mass of the end effector
double end_effector_mass = 2.3 + 0.169 + 0.000;

// name of our end effector node
char* end_effector_node_name = "rGripper";

// how often we save logs
double log_frequency = 1000.0;

// Angle that the DMU is mounted at: 45 degrees.
double csr = -.7853981634;


// #############################################################################
// #############################################################################
// HELPERS
// #############################################################################
// #############################################################################

double ssdmu_pitch(double x, double y, double z) {
    double newX;
    newX = x*cos(csr) - y*sin(csr);
    return atan2(newX, z); 
}

// #############################################################################
// #############################################################################
// CURSES
// #############################################################################
// #############################################################################

void init_curses() {
    initscr();
    clear();
    noecho();                   // do not echo input to the screen
    cbreak();                   // do not buffer by line (receive characters immediately)
    timeout(0);                 // non-blocking getch
}

void end_curses() {
    clrtoeol();
    refresh();
    endwin();
}

void do_display(somatic_motor_t* rlwa, Eigen::VectorXd arm_gravity_torques, Eigen::VectorXd arm_current_cmd) {
    // display torque constants
    mvprintw(1, 4, "torque constants");
    for(int i = 0; i < 7; i++) { mvprintw(2+i, 5, "%f", arm_torque_constants[i]); }

    // display arm configuration
    mvprintw(1, 24, "arm configuration");
    for(int i = 0; i < 7; i++) { mvprintw(2+i, 25, "%f", rlwa->pos[i]); }

    // display arm gravity torques
    mvprintw(1, 44, "arm gravity torque");
    for(int i = 0; i < 7; i++) { mvprintw(2+i, 45, "%f", arm_gravity_torques[i]); }

    // display arm current commands
    mvprintw(1, 64, "arm current command");
    for(int i = 0; i < 7; i++) { mvprintw(2+i, 65, "%f", arm_current_cmd[i]); }
}

// #############################################################################
// #############################################################################
// LOGGING
// #############################################################################
// #############################################################################

void init_log_to_file() {
    log_file.open(logfile_name);
    log_time_last = aa_tm_timespec2sec(aa_tm_now());
    log_time_start = aa_tm_timespec2sec(aa_tm_now());
}

void end_log_to_file() {
    log_file.close();
}

void do_log_to_file(somatic_motor_t* lwa) {
    double now = aa_tm_timespec2sec(aa_tm_now());
    if (now - log_time_last < 1.0/log_frequency) return;
    log_time_last = now;

    log_file
        << now - log_time_start << ","
        << lwa->pos[0] << ","
        << lwa->vel[0] << std::endl;
}

// #############################################################################
// #############################################################################
// UPDATE STATE
// #############################################################################
// #############################################################################

double get_imu() {
    // Get a message
    int r;
    struct timespec timeout = aa_tm_future(aa_tm_sec2timespec(1.0/30.0));
    Somatic__Vector *imu_msg = SOMATIC_WAIT_LAST_UNPACK(r, somatic__vector, 
                                                        &protobuf_c_system_allocator,
                                                        IMU_CHANNEL_SIZE,
                                                        &imu_chan,
                                                        &timeout);
    assert((imu_msg != NULL) && "Didn't get IMU message!");

    // extract the data into something we can use
    double imu_sample_x  = imu_msg->data[0];
    double imu_sample_y  = imu_msg->data[1];
    double imu_sample_z  = imu_msg->data[2];

    // Free the unpacked message
    somatic__vector__free_unpacked( imu_msg, &protobuf_c_system_allocator );

    // compute our result and return it
    return -ssdmu_pitch(imu_sample_x, imu_sample_y, imu_sample_z) + M_PI/2;				 
}


void update_motor_state() {
    // Read the waist's state and update the averaged waist position
    somatic_motor_update(&daemon_cx, &waist);
    waist_angle = (waist.pos[0] - waist.pos[1]) / 2.0;

    // read the arm and the torso
    somatic_motor_update(&daemon_cx, &torso);
    somatic_motor_update(&daemon_cx, &rlwa);
}

// updates dart's kinemaitcs and dynamics based on information
// gathered from sensors
void update_dart_state() {
    // update the waist, torso, and imu in dart
    Eigen::VectorXd imu_pos(1);
    imu_pos << imu_angle;
    krang->setConfig(imu_ids, imu_pos);

    Eigen::VectorXd waist_pos(1);
    waist_pos << waist_angle;
    krang->setConfig(waist_ids, waist_pos);

    Eigen::VectorXd torso_pos(1);
    torso_pos << torso.pos[0];
    krang->setConfig(torso_ids, torso_pos);

    // update the arm
    Eigen::VectorXd arm_pos(7);
    for(int i = 0; i < 7; i++) arm_pos[i] = rlwa.pos[i];
    krang->setConfig(rarm_ids, arm_pos);

    // TODO: figure out what our qdot actaully is based on motor inforamtion
    Eigen::VectorXd qDotZero = Eigen::VectorXd::Zero(krang->getNumDofs());

    // update dart's dynamics stuff
    krang->computeDynamics(world->getGravity(), qDotZero, false);
}


// #############################################################################
// #############################################################################
// MAIN LOOP
// #############################################################################
// #############################################################################

void run()
{
    // the currents to send
    Eigen::VectorXd rcurrentcmd(7);

    // variables for user interface
    int cur_ui_joint = 0;

    // and go
    while(!somatic_sig_received) {
        // handle user input
        mvprintw(2+cur_ui_joint, 1, "   ");
        int ch = getch();
        switch (ch) {
        case '0': somatic_sig_received = true; break;
        case '7': for (int i = 0; i < 7; i++) arm_torque_constants[i] = 0.0; break;
        case '8': cur_ui_joint = std::max(0, cur_ui_joint-1); break;
        case '2': cur_ui_joint = std::min(6, cur_ui_joint+1); break;
        case '6': arm_torque_constants[cur_ui_joint] += .00001; break;
        case '4': arm_torque_constants[cur_ui_joint] -= .00001; break;
        case '1':
            std::ofstream torque_constants_file;
            torque_constants_file.open("torque constants.txt", std::ios_base::trunc);
            for(int i = 0; i < 7; i++) { torque_constants_file << arm_torque_constants[i] << ","; }
            torque_constants_file.close();
            break;
        }

        // update our motors
        update_motor_state();
        
        // update DART from those
        update_dart_state();

        // get our gravity vector
        Eigen::VectorXd krang_gravity_vector = krang->getGravityVector();
        
        // grab just the part we care about
        Eigen::Vector7d arm_gravity_torques;
        for(int i = 0; i < rarm_ids.size(); i++) {
            arm_gravity_torques[i] = krang_gravity_vector[rarm_ids[i]];
        }

        // turn that into currents
        rcurrentcmd = arm_gravity_torques.cwiseProduct(arm_torque_constants);
        
        // and send it
        somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, rcurrentcmd.data(), 7, NULL);

        // then display everything
        do_display(&rlwa, arm_torque_constants, rcurrentcmd);
        refresh();
    }
}

// #############################################################################
// #############################################################################
// INIT
// #############################################################################
// #############################################################################

void init() {
    // There are a few things we need to do _before_ we initialize our
    // daemon because somatic_d_init will change our working directory
    // to the place where somatic logs go. Specifically, logfiles and
    // loading dart scenes.

    // start logging.
    init_log_to_file();

    // init dart and figure out our mappings.
    DartLoader dl;
    world = dl.parseWorld("../../common/scenes/01-World-Robot.urdf");
    assert((world != NULL) && "Could not find the world");
    krang = world->getSkeleton(0);
    Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    world->setGravity(gravity);
    end_effector_node = krang->getNode(end_effector_node_name);

    for(int i = 0; i < 7; i++) rarm_ids.push_back(rarm_ids_initializer[i]);
    for(int i = 0; i < 1; i++) waist_ids.push_back(waist_ids_initializer[i]);
    for(int i = 0; i < 1; i++) torso_ids.push_back(torso_ids_initializer[i]);
    for(int i = 0; i < 1; i++) imu_ids.push_back(imu_ids_initializer[i]);

    // init daemon
    somatic_d_opts_t daemon_opt;
    memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
    daemon_opt.ident = "arm-gravity-compensation";
    somatic_d_init(&daemon_cx, &daemon_opt);

    // init motors
    somatic_motor_init(&daemon_cx, &waist, 2, "waist-cmd", "waist-state");
    somatic_motor_init(&daemon_cx, &torso, 1, "torso-cmd", "torso-state");
    somatic_motor_init(&daemon_cx, &rlwa, 7, "rlwa-cmd", "rlwa-state");
    somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
    usleep(1e5);                // wait 100 ms for it to get going
    

    // set arm joint limits
    double** arm_minimum_values[] = { &rlwa.pos_valid_min, &rlwa.vel_valid_min,
                                      &rlwa.pos_limit_min, &rlwa.vel_limit_min };
    double** arm_maximum_values[] = { &rlwa.pos_valid_max, &rlwa.vel_valid_max,
                                      &rlwa.pos_limit_max, &rlwa.vel_limit_max };
    for(size_t i = 0; i < 4; i++) aa_fset(*arm_minimum_values[i], -1024.1, 7);
    for(size_t i = 0; i < 4; i++) aa_fset(*arm_maximum_values[i], 1024.1, 7);
    
    // set waist joint limits
    double** waist_minimum_values[] = { &waist.pos_valid_min, &waist.vel_valid_min,
                                        &waist.pos_limit_min, &waist.vel_limit_min };
    double** waist_maximum_values[] = { &waist.pos_valid_max, &waist.vel_valid_max,
                                        &waist.pos_limit_max, &waist.vel_limit_max };
    for(size_t i = 0; i < 4; i++) aa_fset(*waist_minimum_values[i], -1024.1, 2);
    for(size_t i = 0; i < 4; i++) aa_fset(*waist_maximum_values[i], 1024.1, 2);

    // set torso joint limits
    double** torso_minimum_values[] = { &torso.pos_valid_min, &torso.vel_valid_min,
                                        &torso.pos_limit_min, &torso.vel_limit_min };
    double** torso_maximum_values[] = { &torso.pos_valid_max, &torso.vel_valid_max,
                                        &torso.pos_limit_max, &torso.vel_limit_max };
    for(size_t i = 0; i < 4; i++) aa_fset(*torso_minimum_values[i], -1024.1, 1);
    for(size_t i = 0; i < 4; i++) aa_fset(*torso_maximum_values[i], 1024.1, 1);

    // initialize the torque constant vector
    for(int i = 0; i < 7; i++) { arm_torque_constants[i] = arm_torque_constants_initializer[i]; }

    // Open the ach channels for the IMU and FT sensor
    somatic_d_channel_open(&daemon_cx, &imu_chan, "imu-data", NULL);

    // start the daemon running
    somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

    // update our motors
    update_motor_state();

    // Get a nice, clean IMU reading and hang on to it
    std::cout << "Getting initial IMU reading" << std::endl;
    imu_angle = 0.0;
    for(int i = 0; i < imu_init_iters; i++) { imu_angle += get_imu(); }
    imu_angle /= (double)imu_init_iters;

    // and finally start up the UI
    init_curses();
}

// #############################################################################
// #############################################################################
// DESTROY
// #############################################################################
// #############################################################################

void destroy() {
    // Send stopping event
    somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

    // end curses
    end_curses();

    // stop logging
    end_log_to_file();

    // halt arm
    somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);

    // close motor channels
    somatic_motor_destroy(&daemon_cx, &rlwa);
    somatic_motor_destroy(&daemon_cx, &waist);
    somatic_motor_destroy(&daemon_cx, &torso);

    // Close imu channel
    somatic_d_channel_close(&daemon_cx, &imu_chan);

    // Destroy the daemon resources
    somatic_d_destroy(&daemon_cx);
}

// #############################################################################
// #############################################################################
// ENTRY POINT
// #############################################################################
// #############################################################################

int main(int argc, char* argv[]) {
    init();
    run();
    destroy();
}
